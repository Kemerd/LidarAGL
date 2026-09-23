/**
 * @file    test_flight.c
 * @brief   FULL-SORTIE integration tests: takeoff, pattern, ILS approach, land.
 *
 * @details Every other test in this suite is unit-scale — it isolates one
 *          mechanism (the Hampel gate, the arming dwell, a single rung) and
 *          drives it with a stimulus shaped to exercise exactly that mechanism.
 *          That is the right way to pin behaviour, but it leaves a real gap:
 *          the box's failure modes have overwhelmingly been INTERACTIONS, not
 *          broken units. The taxi phantom, the silent approach, the one-way
 *          CRUISE door and the truncated sensor chirp were all cases where each
 *          part behaved exactly as specified and the COMPOSITION was wrong.
 *
 *          These tests therefore fly whole sorties through the real chain —
 *          wire samples -> range_filter -> ground reference -> state machine —
 *          at real airspeeds, real climb and sink rates, and the real per-state
 *          poll cadences the firmware would actually use. Nothing is stepped by
 *          a convenient constant; altitude is integrated from a rate, and the
 *          poll interval for each tick comes from the state the machine chose
 *          on the previous one, exactly as the logic task does it.
 *
 *          Aircraft model is the Glasair III this box is built for:
 *            Vr ~70 kt, Vy ~120 kt / 1500 fpm climb, approach 85 kt.
 *          A 3-degree ILS glideslope at 85 kt is a 451 fpm sink, and 300 ft to
 *          touchdown takes about 40 seconds — so these are minute-scale flights,
 *          thousands of polls each, not a dozen hand-written steps.
 */

#include "test_util.h"
#include "range_filter.h"
#include "state_machine.h"
#include "sensor_profile.h"
#include "config.h"

#include <math.h>
#include <string.h>

TEST_GLOBALS

/* ---------------------------------------------------------------------------
 *  A flight rig: the real chain, driven by a true altitude the test controls.
 * ------------------------------------------------------------------------- */

#define SENSOR_HZ        78.0f    /* SF30/C streaming rate                     */
#define GROUND_REF_FT    3.0f     /* learned mount offset                      */
#define MAX_SPOKEN       32

/*  The PHYSICAL delay from the laser crossing a rung to the word being heard
 *  (poll + logic tick + audio pickup + ~90 ms I2S queue + word onset). Kept
 *  separate from the firmware's CALLOUT_LEAD_S on purpose: the rig judges each
 *  callout where it is HEARD, so a missing or wrong lead shows up as error.  */
#define AUDIO_LATENCY_S  0.20f

/*  The aircraft cannot change its vertical rate instantly. Segment transitions
 *  slew the rate at up to 1 g (a hard round-out or push-over); the v1.63 rig
 *  stepped it, e.g. 4000 fpm to 900 fpm between two 25 ms polls — a ~50 g
 *  stop — which a physics-based tracker rightly refuses to believe for the
 *  ~0.1 s its hand-over takes. Terrain steps under the aircraft stay instant:
 *  those are real.                                                            */
#define RIG_MAX_ACCEL_FPS2 32.0f

typedef struct {
    range_filter_t f;
    sm_ctx_t       sm;
    const sensor_profile_t *p;

    float    dt;                  /* next poll interval, chosen by the machine */
    float    t;                   /* elapsed flight time (s)                   */
    unsigned seed;                /* deterministic jitter                      */

    /* What the box actually said, in order. */
    float    spoken[MAX_SPOKEN];
    int      n_spoken;
    int      n_posrate;

    /* Where the aircraft REALLY was when each callout fired — this is what
     * makes a phantom detectable: the number spoken versus the truth.        */
    float    spoken_true_agl[MAX_SPOKEN];

    /* Observations for the assertions. */
    bool     saw_cruise;
    bool     ever_armed;
    float    worst_callout_err;   /* largest |spoken - true| over the sortie   */

    /*  The SURFACE, not just the sensor, sets how far down the laser can see.
     *  reach_range_ft is the range beyond which the current terrain returns
     *  nothing; a sortie changes it mid-flight to model a climb-out over bright
     *  concrete and an approach over darker grass. lens_fraction is the share
     *  of no-return samples that read the housing's acrylic lens (a few cm)
     *  instead of the lost-signal sentinel.                                   */
    float    reach_range_ft;
    float    lens_fraction;
    float    junk_fraction;   /* share of no-return samples that are junk      */
    float    junk_span_cm;    /* junk is uniform over 0..junk_span_cm          */
    bool     ever_disarmed_airborne;  /* parked-detector fired above 20 ft true */
    float    prev_true_agl;           /* for the true vertical rate            */
    float    cur_rate;                /* the aircraft's vertical rate (+up)    */
    /*  Rungs at the sensor's EDGE (within RANGE_CEILING_NEAR_FT of its reach,
     *  i.e. the SF30/C's 300) are judged separately: descending INTO range the
     *  ground must first be confirmed (~0.3 s), so at a 4000 fpm approach that
     *  rung is necessarily heard ~20 ft low. That is detection physics, not
     *  lag, and it is held to a TSO-C151c-like bound instead (its 500 ft test
     *  allows up to 1 s, ~25 ft, of lateness).                                */
    float    worst_edge_err;
} flight_t;

static void flight_init(flight_t *fl, const sensor_profile_t *p, sm_state_t initial)
{
    memset(fl, 0, sizeof *fl);
    fl->p    = p;
    fl->dt   = (float)POLL_MS_GROUND / 1000.0f;
    fl->seed = 1234567u;
    fl->worst_callout_err = 0.0f;
    fl->reach_range_ft    = p->max_range_ft;   /* best case: the rated ceiling */
    fl->lens_fraction     = 0.0f;
    fl->junk_fraction     = 0.25f;             /* default mix: 75% sentinel    */
    fl->junk_span_cm      = 10000.0f;
    rf_init(&fl->f, p->max_range_ft);
    sm_init(&fl->sm, initial);
}

/* Small deterministic pseudo-random in [0,1). */
static float frand(flight_t *fl)
{
    fl->seed = fl->seed * 1103515245u + 12345u;
    return (float)((fl->seed >> 16) & 0x7fff) / 32768.0f;
}

/**
 * @brief Advance the whole chain by ONE poll at a given true AGL.
 *
 * Builds a drain of raw wire samples at the sensor's real rate for the elapsed
 * interval, including per-sample noise and (above the sensor ceiling) the
 * lost-signal sentinel the hardware actually emits. Then runs finalize, applies
 * the ground reference, honours a track break the way the logic task does
 * (including the late-rung window's announcement), and steps the state machine.
 *
 * The samples follow the aircraft CONTINUOUSLY: sample i of n reads the true
 * height interpolated from the previous poll's to this one's, at its own
 * instant, because the range tracker timestamps every sample. (The v1.63 rig
 * held the height constant across a poll and jumped between polls — a
 * staircase a per-sample tracker correctly reads as teleporting.)
 */
static void flight_poll(flight_t *fl, float true_agl_ft, float noise_ft)
{
    int n = (int)(SENSOR_HZ * fl->dt);
    if (n < 1) {
        n = 1;
    }
    if (n > RANGE_DRAIN_MEDIAN_N) {
        n = RANGE_DRAIN_MEDIAN_N;
    }

    /* The height at the previous poll (the first poll has no motion yet). */
    float from_agl = (fl->t > 0.0f) ? fl->prev_true_agl : true_agl_ft;

    for (int i = 0; i < n; ++i) {
        float agl = from_agl + (true_agl_ft - from_agl) * (float)(i + 1) / (float)n;
        if (agl + GROUND_REF_FT > fl->reach_range_ft) {
            /*  Beyond the sensor's reach. The SF30 does NOT jump straight to
             *  its sentinel — "Lost signal confirmations" means it emits real
             *  erroneous distances first — so model a mix, which is what the
             *  hardware genuinely puts on the wire near and above its ceiling. */
            if (fl->lens_fraction > 0.0f && frand(fl) < fl->lens_fraction) {
                /* The acrylic lens reflecting: 5..30 cm, i.e. "0 ft". */
                rf_push_cm(&fl->f, 5.0f + frand(fl) * 25.0f);
            } else if (frand(fl) >= fl->junk_fraction) {
                rf_push_cm(&fl->f, (float)SF30_LOST_SIGNAL_CM);
            } else {
                rf_push_cm(&fl->f, frand(fl) * fl->junk_span_cm);
            }
            continue;
        }
        /* In range: the true range plus sensor noise, quantised to 1 cm as the
         * wire protocol does (this is a real and non-negligible effect at the
         * bottom of the ladder, where 1 cm is 0.033 ft).                      */
        float rng_ft = agl + GROUND_REF_FT + (frand(fl) - 0.5f) * 2.0f * noise_ft;
        if (rng_ft < 0.0f) {
            rng_ft = 0.0f;
        }
        float cm = rng_ft / CM_TO_FT;
        rf_push_cm(&fl->f, (float)((int)(cm + 0.5f)));   /* 1 cm quantisation */
    }

    bool  fresh = false;
    float pub   = 0.0f;
    (void)rf_finalize(&fl->f, fl->dt, &pub, &fresh);

    float agl = pub - GROUND_REF_FT;
    if (agl < 0.0f) {
        agl = 0.0f;
    }

    /*  The logic task's contract: the tracker's rate drives the callout lead
     *  (set BEFORE the re-anchor, whose late-rung window judges lateness where
     *  the word will be heard).                                              */
    fl->sm.lead_rate_fps = rf_rate_fps(&fl->f);

    /*  True vertical rate, so a callout can be judged where it is HEARD: the
     *  lead fires it early by design, and the word lands CALLOUT_LEAD_S later. */
    float true_rate = (fl->t > 0.0f && fl->dt > 0.0f)
                      ? (true_agl_ft - fl->prev_true_agl) / fl->dt : 0.0f;
    fl->prev_true_agl = true_agl_ft;

    /* The logic task's real contract on a discontinuous re-acquisition: re-anchor,
     * and voice a rung the late-rung window announces (flyable re-entries only). */
    int late_fired = -1;
    if (fresh && rf_track_broken(&fl->f)) {
        late_fired = sm_reanchor(&fl->sm, agl, fl->p, rf_break_reentry(&fl->f), NULL);
    }

    bool was_armed = fl->sm.armed;
    sm_out_t out;
    sm_step(&fl->sm, agl, fl->dt, fl->p, &out);
    if (was_armed && !fl->sm.armed && true_agl_ft > 20.0f) {
        fl->ever_disarmed_airborne = true;   /* "parked" while flying: a lie   */
    }

    if (out.state == ST_CRUISE) {
        fl->saw_cruise = true;
    }
    if (fl->sm.armed) {
        fl->ever_armed = true;
    }
    if (out.fired_positive_rate) {
        fl->n_posrate++;
    }
    /*  Both sources of a callout, in the order the logic task voices them: the
     *  late-rung window's (the higher rung) first, then sm_step()'s.        */
    const int fired_list[2] = { late_fired, out.fired_callout };
    for (int k = 0; k < 2; ++k) {
    if (fired_list[k] >= 0 && fl->n_spoken < MAX_SPOKEN) {
        float h = fl->p->callouts[fired_list[k]];
        /*  Where the word is HEARD: CALLOUT_LEAD_S after it fires.            */
        float heard_agl = true_agl_ft + true_rate * AUDIO_LATENCY_S;
        fl->spoken[fl->n_spoken]          = h;
        fl->spoken_true_agl[fl->n_spoken] = heard_agl;
        fl->n_spoken++;
        float err = fabsf(h - heard_agl);
        if (h >= fl->p->max_range_ft - RANGE_CEILING_NEAR_FT) {
            if (err > fl->worst_edge_err) {
                fl->worst_edge_err = err;
            }
        } else if (err > fl->worst_callout_err) {
            fl->worst_callout_err = err;
        }
    }
    }

    /*  The NEXT poll interval, chosen exactly as the firmware's logic task
     *  chooses it (sm_poll_period_ms is shared, not re-implemented here). The
     *  rig used the bare state profile before, which is how a cadence-driven
     *  failure — a CRUISE-rate poll freezing the filter on approach — could
     *  pass every sortie while flying silent in the aircraft.                */
    fl->t += fl->dt;
    fl->dt = (float)sm_poll_period_ms(out.poll, fl->sm.armed, rf_tracking(&fl->f),
                                      rf_reacquiring(&fl->f), agl,
                                      fl->sm.tone_start_ft) / 1000.0f;
}

/**
 * @brief Fly a constant-rate segment from @p from_ft to @p to_ft.
 * @param rate_fps  Magnitude of the vertical rate (ft/s); direction is inferred.
 */
static void fly_segment(flight_t *fl, float from_ft, float to_ft,
                        float rate_fps, float noise_ft)
{
    float agl    = from_ft;
    bool  down   = (to_ft < from_ft);
    float target = (down ? -1.0f : 1.0f) * rate_fps;
    /* Generous iteration bound: a 40 s segment at the 25 ms DESCENT cadence is
     * ~1600 polls; the cap only stops a runaway if a rate is ever passed as 0. */
    for (int guard = 0; guard < 200000; ++guard) {
        flight_poll(fl, agl, noise_ft);
        /* Slew the vertical rate toward this segment's at up to 1 g. */
        float dv = target - fl->cur_rate;
        float mx = RIG_MAX_ACCEL_FPS2 * fl->dt;
        fl->cur_rate += (dv > mx) ? mx : ((dv < -mx) ? -mx : dv);
        agl += fl->cur_rate * fl->dt;
        if (agl < 0.0f) {
            agl = 0.0f;                              /* the runway stops it   */
        }
        if (down ? (agl <= to_ft) : (agl >= to_ft)) {
            break;
        }
        /* Moving the wrong way for this segment while the rate reverses is
         * fine; stalling at the ground short of to_ft is not a segment end. */
        if (down && agl <= 0.0f) {
            break;
        }
    }
}

/** @brief Hold level at @p agl_ft for @p secs. */
static void fly_level(flight_t *fl, float agl_ft, float secs, float noise_ft)
{
    /* Level (or parked): no vertical rate. Level-offs happen far from any
     * rung transition that matters, so this one stays instantaneous.       */
    fl->cur_rate = 0.0f;
    float end = fl->t + secs;
    while (fl->t < end) {
        flight_poll(fl, agl_ft, noise_ft);
    }
}

/* Did the box speak this height? */
static bool spoke(const flight_t *fl, float ft)
{
    for (int i = 0; i < fl->n_spoken; ++i) {
        if (fabsf(fl->spoken[i] - ft) < 0.5f) {
            return true;
        }
    }
    return false;
}

/* How many times? */
static int spoke_count(const flight_t *fl, float ft)
{
    int n = 0;
    for (int i = 0; i < fl->n_spoken; ++i) {
        if (fabsf(fl->spoken[i] - ft) < 0.5f) {
            ++n;
        }
    }
    return n;
}

/*  Shared expectations for any normal landing: the low rungs must all speak,
 *  exactly once each, in descending order, and near the right altitude.       */
static void assert_good_landing(const flight_t *fl, const char *label)
{
    char msg[128];
    const float rungs[] = { 100.0f, 50.0f, 40.0f, 30.0f, 20.0f, 10.0f };

    for (size_t i = 0; i < sizeof rungs / sizeof rungs[0]; ++i) {
        snprintf(msg, sizeof msg, "%s: called %.0f ft", label, (double)rungs[i]);
        ASSERT_TRUE(spoke(fl, rungs[i]), msg);
        snprintf(msg, sizeof msg, "%s: called %.0f ft exactly once",
                 label, (double)rungs[i]);
        ASSERT_TRUE(spoke_count(fl, rungs[i]) == 1, msg);
    }

    bool descending = true;
    for (int i = 1; i < fl->n_spoken; ++i) {
        if (fl->spoken[i] >= fl->spoken[i - 1]) {
            descending = false;
            break;
        }
    }
    snprintf(msg, sizeof msg, "%s: callouts in descending order", label);
    ASSERT_TRUE(descending, msg);

    /*  Every number must have been spoken NEAR the altitude it names. This is
     *  the assertion that catches a phantom: a rung fired from garbage speaks a
     *  height the aircraft is nowhere near, and no amount of ordering or
     *  uniqueness checking would notice.                                       */
    snprintf(msg, sizeof msg,
             "%s: every callout within 10 ft of its true altitude (worst %.1f)",
             label, (double)fl->worst_callout_err);
    ASSERT_TRUE(fl->worst_callout_err < 10.0f, msg);

    snprintf(msg, sizeof msg,
             "%s: the edge-of-range rung within 25 ft where heard (worst %.1f)",
             label, (double)fl->worst_edge_err);
    ASSERT_TRUE(fl->worst_edge_err < 25.0f, msg);
}

/* ===========================================================================
 *  SORTIE 1 — the full flight the box is built for.
 *
 *  Cold start on the ramp, takeoff roll, climb-out to a 1000 ft pattern (well
 *  above the SF30/C's ~328 ft reach, so the sensor goes blind for most of the
 *  flight), fly the pattern, then a 3-degree ILS glideslope at 85 kt down to
 *  the flare and rollout.
 *
 *  This is the exact shape of the flight that came back silent, and it is the
 *  composition — not any single mechanism — that has to work.
 * ========================================================================= */
static void test_sortie_full_pattern_and_ils(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);

    const float NOISE = 0.15f;   /* ~2x the SF30's quoted +/-5 cm             */

    /* --- Parked on the ramp, engine running --------------------------------- */
    fly_level(&fl, 0.0f, 20.0f, NOISE);
    ASSERT_TRUE(fl.n_spoken == 0, "sortie: silent while parked");

    /* --- Takeoff roll and rotation ------------------------------------------ */
    /*  The roll itself is level at ~0 ft; rotation is where AGL starts moving. */
    fly_level(&fl, 0.0f, 12.0f, NOISE);
    ASSERT_TRUE(fl.n_spoken == 0, "sortie: silent through the takeoff roll");

    /* --- Climb-out at Vy: 1500 fpm = 25 ft/s -------------------------------- */
    fly_segment(&fl, 0.0f, 1000.0f, 25.0f, NOISE);
    ASSERT_TRUE(fl.ever_armed,
                "sortie: the climb-out armed the callout ladder");
    ASSERT_TRUE(fl.n_spoken == 0,
                "sortie: NO altitude callouts on the way up (silent climb-out)");

    /* --- Pattern at 1000 ft: far above the sensor's reach -------------------- */
    /*  ~2 minutes of downwind/base with the LiDAR seeing nothing at all. This
     *  is the stretch that used to strand the machine in CRUISE forever.      */
    fly_level(&fl, 1000.0f, 120.0f, NOISE);
    ASSERT_TRUE(fl.n_spoken == 0,
                "sortie: NO phantom callouts during blind pattern work");

    /* --- The ILS: 3 degrees at 85 kt = 451 fpm = 7.52 ft/s ------------------ */
    fly_segment(&fl, 1000.0f, 15.0f, 7.52f, NOISE);

    /* --- Flare and touchdown: rate bleeds off -------------------------------- */
    fly_segment(&fl, 15.0f, 0.0f, 2.5f, NOISE);

    /* --- Rollout ------------------------------------------------------------- */
    fly_level(&fl, 0.0f, 15.0f, NOISE);

    assert_good_landing(&fl, "sortie");

    /*  The top of the ladder deserves its own check: 300 and 200 ft are inside
     *  the sensor's reach on the way down and must speak, but only AFTER the
     *  aircraft has genuinely descended into range.                            */
    ASSERT_TRUE(spoke(&fl, 200.0f), "sortie: called 200 ft on the glideslope");

    /*  And nothing may be spoken twice across the whole flight.               */
    bool dup = false;
    for (int i = 0; i < fl.n_spoken && !dup; ++i) {
        for (int j = i + 1; j < fl.n_spoken; ++j) {
            if (fabsf(fl.spoken[i] - fl.spoken[j]) < 0.5f) {
                dup = true;
                break;
            }
        }
    }
    ASSERT_TRUE(!dup, "sortie: no rung spoke twice in the whole flight");
}

/* ===========================================================================
 *  SORTIE 2 — touch-and-go, then a second full approach.
 *
 *  The one-shot rungs must RE-ARM for the second landing. A box that calls a
 *  perfect first approach and is then silent on the go-around is arguably worse
 *  than one that never worked, because the pilot has learned to expect it.
 * ========================================================================= */
static void test_sortie_touch_and_go(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    /* First departure and circuit. */
    fly_level(&fl, 0.0f, 15.0f, NOISE);
    fly_segment(&fl, 0.0f, 800.0f, 25.0f, NOISE);
    fly_level(&fl, 800.0f, 60.0f, NOISE);

    /* First approach to a touch. */
    fly_segment(&fl, 800.0f, 12.0f, 7.52f, NOISE);
    fly_segment(&fl, 12.0f, 0.0f, 2.5f, NOISE);

    int first_landing_calls = fl.n_spoken;
    ASSERT_TRUE(first_landing_calls >= 6,
                "touch-and-go: first approach walked the ladder");

    /*  Brief touch — deliberately shorter than GROUND_RESET_MS, so the arming
     *  is KEPT (a touch-and-go is not a taxi-back).                           */
    fly_level(&fl, 0.0f, 4.0f, NOISE);

    /* Second departure. */
    fly_segment(&fl, 0.0f, 800.0f, 25.0f, NOISE);
    fly_level(&fl, 800.0f, 60.0f, NOISE);

    /* Second approach, full stop. */
    fly_segment(&fl, 800.0f, 12.0f, 7.52f, NOISE);
    fly_segment(&fl, 12.0f, 0.0f, 2.5f, NOISE);
    fly_level(&fl, 0.0f, 10.0f, NOISE);

    int second_landing_calls = fl.n_spoken - first_landing_calls;
    ASSERT_TRUE(second_landing_calls >= 6,
                "touch-and-go: SECOND approach walked the ladder too (re-armed)");

    /*  Each rung should now have spoken about twice — once per approach. */
    ASSERT_TRUE(spoke_count(&fl, 10.0f) == 2,
                "touch-and-go: the 10 ft rung spoke once per landing");
    ASSERT_TRUE(spoke_count(&fl, 50.0f) == 2,
                "touch-and-go: the 50 ft rung spoke once per landing");
    ASSERT_TRUE(fl.worst_callout_err < 10.0f,
                "touch-and-go: every callout near its true altitude");
}

/* ===========================================================================
 *  SORTIE 3 — go-around from short final.
 *
 *  The aircraft descends the ladder, breaks off low, climbs back to pattern
 *  altitude, and comes round for a second approach. The rungs it already spoke
 *  must re-arm on the climb so the second approach is fully called.
 * ========================================================================= */
static void test_sortie_go_around(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    fly_level(&fl, 0.0f, 15.0f, NOISE);
    fly_segment(&fl, 0.0f, 800.0f, 25.0f, NOISE);
    fly_level(&fl, 800.0f, 45.0f, NOISE);

    /* Down the glideslope to 30 ft, then go around. */
    fly_segment(&fl, 800.0f, 30.0f, 7.52f, NOISE);
    ASSERT_TRUE(spoke(&fl, 50.0f), "go-around: called 50 ft before breaking off");

    /* Balked landing: full power, climb away at Vy. */
    fly_segment(&fl, 30.0f, 800.0f, 25.0f, NOISE);
    fly_level(&fl, 800.0f, 45.0f, NOISE);

    int before_second = fl.n_spoken;

    /* Second, completed approach. */
    fly_segment(&fl, 800.0f, 12.0f, 7.52f, NOISE);
    fly_segment(&fl, 12.0f, 0.0f, 2.5f, NOISE);
    fly_level(&fl, 0.0f, 10.0f, NOISE);

    int second = fl.n_spoken - before_second;
    ASSERT_TRUE(second >= 6,
                "go-around: the second approach is fully called (rungs re-armed)");
    ASSERT_TRUE(spoke_count(&fl, 10.0f) >= 1,
                "go-around: the 10 ft rung speaks on the completed landing");
    ASSERT_TRUE(fl.worst_callout_err < 10.0f,
                "go-around: every callout near its true altitude");
}

/* ===========================================================================
 *  SORTIE 4 — low pattern, entirely INSIDE the sensor's range.
 *
 *  A 250 ft circuit never blinds the sensor, so the filter tracks continuously
 *  from takeoff to landing. This is the opposite regime from sortie 1 and
 *  exercises the rungs above 100 ft, which the blind-pattern flight only ever
 *  crosses on the way down.
 * ========================================================================= */
static void test_sortie_low_pattern_in_range(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    fly_level(&fl, 0.0f, 15.0f, NOISE);
    fly_segment(&fl, 0.0f, 250.0f, 20.0f, NOISE);
    ASSERT_TRUE(fl.ever_armed, "low pattern: armed on the climb-out");
    ASSERT_TRUE(fl.n_spoken == 0, "low pattern: silent climb-out");
    ASSERT_TRUE(!fl.saw_cruise,
                "low pattern: 250 ft is below cruise_ft, never enters CRUISE");

    fly_level(&fl, 250.0f, 40.0f, NOISE);
    ASSERT_TRUE(fl.n_spoken == 0, "low pattern: silent while level in range");

    /* Approach from 250 ft — crosses 200 as well as the low rungs. */
    fly_segment(&fl, 250.0f, 12.0f, 7.52f, NOISE);
    fly_segment(&fl, 12.0f, 0.0f, 2.5f, NOISE);
    fly_level(&fl, 0.0f, 15.0f, NOISE);

    assert_good_landing(&fl, "low pattern");
    ASSERT_TRUE(spoke(&fl, 200.0f), "low pattern: called 200 ft");
}

/* ===========================================================================
 *  SORTIE 5 — a steep, fast approach.
 *
 *  Not every arrival is a stabilised 3-degree ILS. A slam-dunk descent at
 *  ~1200 fpm crosses the low rungs far faster, which is where a filter that
 *  lags or a ladder that needs several polls per rung would start dropping
 *  numbers. The callouts must survive the rate.
 * ========================================================================= */
static void test_sortie_steep_approach(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    fly_level(&fl, 0.0f, 15.0f, NOISE);
    fly_segment(&fl, 0.0f, 600.0f, 25.0f, NOISE);
    fly_level(&fl, 600.0f, 30.0f, NOISE);

    /* 1200 fpm = 20 ft/s, held right down to the flare. */
    fly_segment(&fl, 600.0f, 15.0f, 20.0f, NOISE);
    fly_segment(&fl, 15.0f, 0.0f, 4.0f, NOISE);
    fly_level(&fl, 0.0f, 12.0f, NOISE);

    assert_good_landing(&fl, "steep approach");
}

/* ===========================================================================
 *  SORTIE 6 — taxi-back after a full stop.
 *
 *  Thirty seconds parked must DISARM the ladder, so the next takeoff is silent
 *  again. This is the taxi-phantom guard, exercised at sortie scale: the box
 *  taxis in, sits, and must not speak a single number while it does.
 * ========================================================================= */
static void test_sortie_taxi_back_disarms(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    /* Fly a circuit and land. */
    fly_level(&fl, 0.0f, 12.0f, NOISE);
    fly_segment(&fl, 0.0f, 500.0f, 25.0f, NOISE);
    fly_level(&fl, 500.0f, 30.0f, NOISE);
    fly_segment(&fl, 500.0f, 12.0f, 7.52f, NOISE);
    fly_segment(&fl, 12.0f, 0.0f, 2.5f, NOISE);

    ASSERT_TRUE(fl.sm.armed, "taxi-back: still armed immediately after touchdown");

    /*  Taxi in: well past GROUND_RESET_MS, with ordinary taxi jitter. This is
     *  where the phantom "50 40 30 20 10" was once spoken on the taxiway.     */
    int calls_at_touchdown = fl.n_spoken;
    fly_level(&fl, 0.0f, 90.0f, NOISE);

    ASSERT_TRUE(!fl.sm.armed,
                "taxi-back: 30 s parked DISARMS the ladder");
    ASSERT_TRUE(fl.n_spoken == calls_at_touchdown,
                "taxi-back: ZERO callouts spoken while taxiing/parked");

    /*  And the next departure is silent again, from a genuinely disarmed box. */
    fly_segment(&fl, 0.0f, 400.0f, 25.0f, NOISE);
    ASSERT_TRUE(fl.n_spoken == calls_at_touchdown,
                "taxi-back: the next climb-out is silent too");
}

/* ===========================================================================
 *  SORTIE 7 — in-flight reboot on final.
 *
 *  A power glitch on short final seeds the machine ARMED with the whole ladder
 *  hot (the boot path's airborne rescue). The rungs BELOW the aircraft must
 *  still speak as it descends through them, and — critically — the box must not
 *  blurt every number above it on the first tick.
 * ========================================================================= */
static void test_sortie_inflight_reboot_on_final(void)
{
    flight_t fl;
    /* Seed exactly as app_main does for an airborne boot. */
    flight_init(&fl, &SF30C_PROFILE, ST_ARMED);
    const float NOISE = 0.15f;

    /*  First poll happens at 220 ft, mid-approach. Nothing may fire on it: the
     *  machine has no previous sample, so there is no crossing yet.           */
    flight_poll(&fl, 220.0f, NOISE);
    ASSERT_TRUE(fl.n_spoken == 0,
                "in-flight reboot: first tick blurts nothing");

    /* Continue the approach normally. */
    fly_segment(&fl, 220.0f, 12.0f, 7.52f, NOISE);
    fly_segment(&fl, 12.0f, 0.0f, 2.5f, NOISE);
    fly_level(&fl, 0.0f, 10.0f, NOISE);

    assert_good_landing(&fl, "in-flight reboot");
    ASSERT_TRUE(!spoke(&fl, 300.0f),
                "in-flight reboot: never speaks a rung it was already below");
}

/* ===========================================================================
 *  SORTIE 8 — the September 2026 flight: darker approach surface, two
 *  go-arounds, then a landing.
 *
 *  Climb-out is over the runway (bright concrete) and the laser sees the
 *  ground to ~325 ft, so the filter pins at the sensor ceiling once it goes
 *  blind. The approach is over grass and fields, which only return from
 *  ~250 ft. The first real reading is then a ~75 ft DOWNWARD step from the
 *  pinned value — a DESCENDING re-entry after blindness, which the tracker
 *  confirms in ~0.1 s as a break and hands to the late-rung window (the 300
 *  rung, ~50 ft late, is skipped).
 *
 *  v1.62 polled at the CRUISE rate there (the pinned value sits above
 *  cruise_ft), a 450 fpm descent moved ~4 ft between polls, the agreeing
 *  cluster could never form, and the box stayed frozen at ~325 ft the whole
 *  approach: no tone, no callouts. It unstuck only when the aircraft levelled
 *  off low, so the pilot heard the tone ONLY on each go-around climb, pitch
 *  falling. That exact sequence is flown here, and every approach must call.
 * ========================================================================= */
static void test_sortie_darker_approach_surface_go_arounds(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    /* --- Ramp, roll, climb-out over concrete (sees to ~325 ft range) ------- */
    fl.reach_range_ft = 325.0f;
    fly_level(&fl, 0.0f, 20.0f, NOISE);
    fly_segment(&fl, 0.0f, 1000.0f, 25.0f, NOISE);
    fly_level(&fl, 1000.0f, 120.0f, NOISE);

    /* --- Approaches over grass: returns only from ~250 ft of range --------- */
    fl.reach_range_ft = 250.0f;

    for (int pass = 1; pass <= 3; ++pass) {
        int before = fl.n_spoken;
        char msg[128];

        if (pass < 3) {
            /* Down the glideslope, go around at 40 ft, climb back to pattern. */
            fly_segment(&fl, 1000.0f, 40.0f, 7.52f, NOISE);
            fly_segment(&fl, 40.0f, 1000.0f, 20.0f, NOISE);
            fly_level(&fl, 1000.0f, 90.0f, NOISE);
        } else {
            /* The landing. */
            fly_segment(&fl, 1000.0f, 15.0f, 7.52f, NOISE);
            fly_segment(&fl, 15.0f, 0.0f, 2.5f, NOISE);
            fly_level(&fl, 0.0f, 15.0f, NOISE);
        }

        /* What this approach said. The 300 ft rung is out of the laser's
         * reach over grass, so it is legitimately skipped; 200 and below are
         * inside it and must speak on EVERY approach.                         */
        bool said200 = false, said100 = false, said50 = false;
        for (int i = before; i < fl.n_spoken; ++i) {
            if (fabsf(fl.spoken[i] - 200.0f) < 0.5f) said200 = true;
            if (fabsf(fl.spoken[i] - 100.0f) < 0.5f) said100 = true;
            if (fabsf(fl.spoken[i] -  50.0f) < 0.5f) said50  = true;
        }
        snprintf(msg, sizeof msg, "darker surface, approach %d: called 200 ft", pass);
        ASSERT_TRUE(said200, msg);
        snprintf(msg, sizeof msg, "darker surface, approach %d: called 100 ft", pass);
        ASSERT_TRUE(said100, msg);
        snprintf(msg, sizeof msg, "darker surface, approach %d: called 50 ft", pass);
        ASSERT_TRUE(said50, msg);
    }

    ASSERT_TRUE(spoke(&fl, 10.0f), "darker surface: 10 ft called on the landing");
    ASSERT_TRUE(fl.worst_callout_err < 10.0f,
                "darker surface: every callout within 10 ft of the truth");
}

/* ===========================================================================
 *  SORTIE 9 — lens reflections out of range, with the lens floor set.
 *
 *  Above the laser's reach, half of the no-return samples bounce off the
 *  housing's acrylic lens and read a few centimetres ("0 ft"). Unfiltered,
 *  a lens-dominated drain can re-anchor the altitude at zero in the pattern,
 *  sound the flare tone at 1000 ft, and after 30 s let the parked detector
 *  DISARM the ladder in flight — a silent approach. With the floor applied
 *  (as app_main does from the learned ground), lens readings count as no
 *  return and the flight must be indistinguishable from a clean one.
 * ========================================================================= */
static void test_sortie_lens_reflections(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    fl.lens_fraction = 0.5f;
    rf_set_min_range(&fl.f, GROUND_REF_FT - GROUND_BELOW_DEV_FT);

    fly_level(&fl, 0.0f, 20.0f, NOISE);
    fly_segment(&fl, 0.0f, 1000.0f, 25.0f, NOISE);
    fly_level(&fl, 1000.0f, 150.0f, NOISE);
    ASSERT_TRUE(!fl.ever_disarmed_airborne,
                "lens: the ladder is never disarmed while flying the pattern");
    ASSERT_TRUE(fl.n_spoken == 0, "lens: no phantom callouts in the pattern");

    fly_segment(&fl, 1000.0f, 15.0f, 7.52f, NOISE);
    fly_segment(&fl, 15.0f, 0.0f, 2.5f, NOISE);
    fly_level(&fl, 0.0f, 15.0f, NOISE);
    assert_good_landing(&fl, "lens");
}

/* ===========================================================================
 *  SORTIE 10 — a real Glasair III approach: ~2000 fpm, darker surface.
 *
 *  The earlier sorties assumed a 451 fpm, 3-degree ILS at 85 kt. The Glasair
 *  III routinely comes down at ~2000 fpm (33 ft/s) — steep enough that the
 *  panel's TAWS calls "SINK RATE" on normal approaches. At that rate the
 *  aircraft covers ~17 ft per 500 ms CRUISE poll, so every rule sized in
 *  "feet per poll" (re-acquire agreement, Hampel lag) is stressed 4x harder,
 *  and the low rungs arrive 0.3 s apart. Climb out over concrete, approach
 *  over grass at 2000 fpm, round out from 50 ft, flare, land.
 * ========================================================================= */
static void test_sortie_glasair_2000fpm(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    fl.reach_range_ft = 325.0f;                    /* concrete on the climb   */
    fly_level(&fl, 0.0f, 20.0f, NOISE);
    fly_segment(&fl, 0.0f, 1000.0f, 25.0f, NOISE);
    fly_level(&fl, 1000.0f, 120.0f, NOISE);

    fl.reach_range_ft = 250.0f;                    /* grass on the approach   */
    fly_segment(&fl, 1000.0f, 50.0f, 33.3f, NOISE);  /* 2000 fpm              */
    fly_segment(&fl, 50.0f, 15.0f, 12.0f, NOISE);    /* round-out             */
    fly_segment(&fl, 15.0f, 0.0f, 3.0f, NOISE);      /* flare                 */
    fly_level(&fl, 0.0f, 15.0f, NOISE);

    ASSERT_TRUE(spoke(&fl, 200.0f), "2000 fpm: called 200 ft over the darker surface");
    assert_good_landing(&fl, "2000 fpm");
}

/* ===========================================================================
 *  SORTIE 11 — gear-down idle: ~4000 fpm onto grass, then round-out.
 *
 *  Gear down at idle and ~120 kt the Glasair III sinks 3500-4000 fpm
 *  (58-67 ft/s), faster than the filter's old 60 ft/s "physical bound", which
 *  made it reject the real descent as impossible. Arresting 67 ft/s takes
 *  ~70 ft even at 1 g (v^2 / 2a), so the round-out starts at 130 ft and the
 *  aircraft still passes 100 ft at ~3000 fpm. Every rung the laser can reach
 *  must still speak, near its true height.
 * ========================================================================= */
static void test_sortie_gear_down_idle_4000fpm(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    fl.reach_range_ft = 325.0f;                     /* concrete on the climb  */
    fly_level(&fl, 0.0f, 20.0f, NOISE);
    fly_segment(&fl, 0.0f, 1000.0f, 25.0f, NOISE);
    fly_level(&fl, 1000.0f, 120.0f, NOISE);

    fl.reach_range_ft = 250.0f;                     /* grass on the approach  */
    fly_segment(&fl, 1000.0f, 130.0f, 66.7f, NOISE);/* 4000 fpm, gear down    */
    fly_segment(&fl, 130.0f, 15.0f, 15.0f, NOISE);  /* 1 g round-out          */
    fly_segment(&fl, 15.0f, 0.0f, 3.0f, NOISE);     /* flare                  */
    fly_level(&fl, 0.0f, 15.0f, NOISE);

    ASSERT_TRUE(spoke(&fl, 200.0f), "4000 fpm: called 200 ft over the darker surface");
    assert_good_landing(&fl, "4000 fpm");
}

/* ===========================================================================
 *  SORTIE 12 — a sensor that sprays GARBAGE whenever it is blind.
 *
 *  The worst the wire can do: every no-return sample is a uniformly random
 *  14-bit value (garbled serial, or a sensor emitting erroneous distances while
 *  blind). ~64% of them land inside the plausible 0..343 ft window, so the
 *  stream is MAJORITY "valid" and passes every lost-signal vote — but it is
 *  scattered across hundreds of feet, which no real surface is. The tracker's
 *  innovation gate and the candidate's member-fraction rule must reject it:
 *  not one phantom rung in the blind pattern, and a normal ladder once the
 *  ground is really in view. Flown at 4000 fpm so the rejection cannot be
 *  bought with slow motion.
 * ========================================================================= */
static void test_sortie_garbage_while_blind(void)
{
    flight_t fl;
    flight_init(&fl, &SF30C_PROFILE, ST_GROUND);
    const float NOISE = 0.15f;

    fl.junk_fraction = 1.0f;
    fl.junk_span_cm  = 16383.0f;

    fly_level(&fl, 0.0f, 20.0f, NOISE);
    fly_segment(&fl, 0.0f, 1000.0f, 25.0f, NOISE);
    fly_level(&fl, 1000.0f, 150.0f, NOISE);
    ASSERT_TRUE(fl.n_spoken == 0,
                "garbage: no phantom callouts on the climb or in the pattern");
    ASSERT_TRUE(!fl.ever_disarmed_airborne, "garbage: never disarmed in flight");

    fly_segment(&fl, 1000.0f, 130.0f, 66.7f, NOISE);
    fly_segment(&fl, 130.0f, 15.0f, 15.0f, NOISE);
    fly_segment(&fl, 15.0f, 0.0f, 3.0f, NOISE);
    fly_level(&fl, 0.0f, 15.0f, NOISE);
    assert_good_landing(&fl, "garbage");
}

/* ===========================================================================
 *  THE ACCEPTANCE MATRIX — every combination the box must survive, in one run.
 * ---------------------------------------------------------------------------
 *  288 physical flights through the real filter + state machine + the
 *  firmware's own cadence policy and re-anchor contract:
 *
 *    wire while blind  : clean sentinel | 25% junk | full-span junk | lens
 *    approach surface  : returns from 328 (concrete) | 250 (grass) | 180 ft
 *    approach sink     : 450 | 2000 | 4000 | 6000 fpm (arrested at 1 g)
 *    terrain under beam: flat | tree canopy + ditch + road embankment at
 *                        90 kt | the same at 150 kt
 *    procedure         : straight-in | a go-around from 60 ft first
 *
 *  Samples are taken at 78 Hz at their own instants, the aircraft's vertical
 *  rate changes at up to 1 g, terrain moves under the beam at the groundspeed,
 *  and every callout is judged where it is HEARD (0.2 s later), against the
 *  height above whatever the beam is actually on — exactly what a radar
 *  altimeter reports. The v1.64 rewrite was accepted against this matrix and
 *  A/B-compared with v1.63 on identical wire streams (v1.63: worst flat-terrain
 *  heard error 7.3 ft at 4000 fpm and 15.8 ft at 6000 fpm; v1.64: 1.9 ft).
 * ========================================================================= */

#define MX_HZ      78.0f
#define MX_G       3.0f       /* ground reference (ft of range at 0 AGL)     */

typedef struct {
    range_filter_t f; sm_ctx_t sm; const sensor_profile_t *p;
    unsigned seed;
    int   model;              /* 0 sentinel, 1 25% junk, 2 full-span, 3 lens */
    float reach;              /* current surface reach (range ft)           */
    float kt;                 /* groundspeed for terrain (0 = flat)          */
    float t, dt, h, rate, x, gs;
    int   terrain_on;
    int   was_armed, disarms;
    float said[48], heard[48]; int n;
} mx_t;

static float mx_rand(mx_t *m)
{
    m->seed = m->seed * 1103515245u + 12345u;
    return (float)((m->seed >> 16) & 0x7fff) / 32768.0f;
}

/*  Terrain height under the beam at along-track x (ft, 0 = threshold). */
static float mx_terrain(const mx_t *m, float x)
{
    if (!m->terrain_on || m->kt <= 0.0f) return 0.0f;
    if (x >= -4500.0f && x < -1500.0f) return 40.0f;   /* tree canopy        */
    if (x >= -700.0f  && x < -650.0f)  return -10.0f;  /* drainage ditch     */
    if (x >= -350.0f  && x < -300.0f)  return 15.0f;   /* road embankment    */
    return 0.0f;
}

/*  What the wire carries for a true range: a return (+-0.15 ft, 1 cm
 *  quantised) inside the reach, otherwise the model's no-return behaviour.  */
static float mx_wire(mx_t *m, float rng_ft)
{
    if (rng_ft <= m->reach) {
        float r = rng_ft + (mx_rand(m) - 0.5f) * 0.3f;
        if (r < 0.0f) r = 0.0f;
        return (float)(int)(r / CM_TO_FT + 0.5f);
    }
    switch (m->model) {
    case 0:  return (float)SF30_LOST_SIGNAL_CM;
    case 1:  return mx_rand(m) < 0.75f ? (float)SF30_LOST_SIGNAL_CM : mx_rand(m) * 10000.0f;
    case 2:  return mx_rand(m) * 16383.0f;
    default: return mx_rand(m) < 0.3f ? 20.0f + mx_rand(m) * 40.0f
                                      : (float)SF30_LOST_SIGNAL_CM;
    }
}

static void mx_said(mx_t *m, int idx, float heard)
{
    if (idx >= 0 && m->n < 48) {
        m->said[m->n]  = m->p->callouts[idx];
        m->heard[m->n] = heard;
        m->n++;
    }
}

/*  One poll of the whole chain, the aircraft steering toward @p target_rate. */
static void mx_poll(mx_t *m, float target_rate)
{
    float dv = target_rate - m->rate, mx = RIG_MAX_ACCEL_FPS2 * m->dt;
    float r1 = m->rate + (dv > mx ? mx : (dv < -mx ? -mx : dv));
    int   n  = (int)(MX_HZ * m->dt + mx_rand(m));
    if (n > RANGE_DRAIN_MEDIAN_N) n = RANGE_DRAIN_MEDIAN_N;
    for (int i = 0; i < n; ++i) {
        float u  = m->dt * (float)(i + 1) / (float)n;
        float hs = m->h + 0.5f * (m->rate + r1) * u;
        if (hs < 0.0f) hs = 0.0f;
        rf_push_cm(&m->f, mx_wire(m, hs - mx_terrain(m, m->x + m->gs * u) + MX_G));
    }
    m->h += 0.5f * (m->rate + r1) * m->dt;
    if (m->h < 0.0f) { m->h = 0.0f; r1 = 0.0f; }
    m->x   += m->gs * m->dt;
    m->rate = r1;

    float pub = 0.0f; bool fresh = false;
    bool  have = rf_finalize(&m->f, m->dt, &pub, &fresh);
    m->t += m->dt;
    if (!have) {
        return;
    }
    float agl   = (pub - MX_G > 0.0f) ? pub - MX_G : 0.0f;
    float truth = m->h - mx_terrain(m, m->x);
    float heard = truth + r1 * AUDIO_LATENCY_S;

    /* The logic task's contract, in its order. */
    m->sm.lead_rate_fps = rf_rate_fps(&m->f);
    if (fresh && rf_track_broken(&m->f)) {
        mx_said(m, sm_reanchor(&m->sm, agl, m->p, rf_break_reentry(&m->f), NULL), heard);
    }
    sm_out_t o;
    sm_step(&m->sm, agl, m->dt, m->p, &o);
    mx_said(m, o.fired_callout, heard);
    if (m->was_armed && !m->sm.armed && truth > 20.0f) m->disarms++;
    m->was_armed = m->sm.armed;

    uint32_t ms = sm_poll_period_ms(o.poll, m->sm.armed, rf_tracking(&m->f),
                                    rf_reacquiring(&m->f), agl, m->sm.tone_start_ft);
    m->dt = (float)((ms / 10u) ? (ms / 10u) : 1u) * 0.010f;   /* 100 Hz tick */
}

static void mx_to(mx_t *m, float target_h, float fps)
{
    bool down = target_h < m->h;
    for (int g = 0; g < 400000; ++g) {
        mx_poll(m, down ? -fps : fps);
        if (down ? m->h <= target_h : m->h >= target_h) break;
        if (down && m->h <= 0.0f) break;
    }
}

static void mx_level(mx_t *m, float secs)
{
    float end = m->t + secs;
    while (m->t < end) mx_poll(m, 0.0f);
}

/*  Fly to @p bottom at @p vs, arresting the sink at 1 g in time to be at
 *  15 ft/s by 15 ft (the round-out), as a real aircraft must.              */
static void mx_approach(mx_t *m, float vs, float bottom)
{
    float ro = 20.0f + (vs * vs - 225.0f) / (2.0f * RIG_MAX_ACCEL_FPS2);
    if (bottom < ro) {
        mx_to(m, ro, vs);
        mx_to(m, bottom, 15.0f);
    } else {
        mx_to(m, bottom, vs);
    }
}

static void test_matrix_acceptance(void)
{
    printf("-- acceptance matrix: 288 flights --\n");
    const float reaches[] = { 328.0f, 250.0f, 180.0f };
    const float fpms[]    = { 450.0f, 2000.0f, 4000.0f, 6000.0f };
    const float kts[]     = { 0.0f, 90.0f, 150.0f };

    int   phantom_cells = 0, disarm_cells = 0, flat_missing = 0, cells = 0;
    float worst_flat = 0.0f, worst_any = 0.0f, worst_edge = 0.0f;
    char  first_bad[160] = "";
    static mx_t m;                                /* large: keep off the stack */

    for (int mi = 0; mi < 4; ++mi)
    for (int ri = 0; ri < 3; ++ri)
    for (int vi = 0; vi < 4; ++vi)
    for (int ki = 0; ki < 3; ++ki)
    for (int ga = 0; ga < 2; ++ga) {
        memset(&m, 0, sizeof m);
        m.p     = &SF30C_PROFILE;
        m.seed  = 1000u + (unsigned)(mi * 1000 + ri * 100 + vi * 10 + ki * 2 + ga);
        m.model = mi;
        m.kt    = kts[ki];
        m.dt    = 0.75f;
        m.reach = 325.0f;                         /* concrete on the climb    */
        rf_init(&m.f, m.p->max_range_ft);
        rf_set_min_range(&m.f, MX_G - GROUND_BELOW_DEV_FT);
        sm_init(&m.sm, ST_GROUND);

        float vs = fpms[vi] / 60.0f;
        mx_level(&m, 30.0f);
        mx_to(&m, 1000.0f, 25.0f);
        mx_level(&m, 60.0f);
        m.reach      = reaches[ri];               /* the approach surface     */
        m.gs         = m.kt * 1.68781f;
        m.terrain_on = 1;
        float t_app  = (1000.0f - 60.0f) / vs + 8.0f;
        m.x          = -m.gs * t_app;
        if (ga) {
            mx_approach(&m, vs, 60.0f);
            mx_to(&m, 1000.0f, 20.0f);
            mx_level(&m, 40.0f);
            m.x = -m.gs * t_app;
        }
        mx_approach(&m, vs, 15.0f);
        mx_to(&m, 0.0f, 3.0f);
        mx_level(&m, 10.0f);
        ++cells;

        /* --- judge the cell ------------------------------------------------ */
        bool phantom = false;
        for (int i = 0; i < m.n; ++i) {
            float e = fabsf(m.said[i] - m.heard[i]);
            bool edge = m.said[i] >= m.p->max_range_ft - RANGE_CEILING_NEAR_FT;
            if (e > 25.0f) phantom = true;
            if (edge) {
                if (e > worst_edge) worst_edge = e;
            } else {
                if (e > worst_any) worst_any = e;
                if (m.kt <= 0.0f && e > worst_flat) worst_flat = e;
            }
        }
        bool missing = false;
        if (m.kt <= 0.0f) {
            const float low[] = { 50.0f, 40.0f, 30.0f, 20.0f, 10.0f };
            for (size_t j = 0; j < sizeof low / sizeof low[0]; ++j) {
                int c = 0;
                for (int i = 0; i < m.n; ++i) c += fabsf(m.said[i] - low[j]) < 0.5f;
                if (c < 1 || (!ga && c > 1)) missing = true;
            }
        }
        phantom_cells += phantom;
        disarm_cells  += (m.disarms > 0);
        flat_missing  += missing;
        if ((phantom || m.disarms || missing) && first_bad[0] == '\0') {
            snprintf(first_bad, sizeof first_bad,
                     "first bad cell: wire %d, reach %.0f, %.0f fpm, %.0f kt, ga %d",
                     mi, (double)reaches[ri], (double)fpms[vi], (double)kts[ki], ga);
        }
    }

    char msg[200];
    if (first_bad[0]) printf("  %s\n", first_bad);
    snprintf(msg, sizeof msg, "matrix: %d flights, ZERO phantom callouts (%d cells)",
             cells, phantom_cells);
    ASSERT_TRUE(phantom_cells == 0, msg);
    snprintf(msg, sizeof msg, "matrix: never disarmed in flight (%d cells)", disarm_cells);
    ASSERT_TRUE(disarm_cells == 0, msg);
    snprintf(msg, sizeof msg,
             "matrix: flat terrain speaks 50/40/30/20/10 once each, every flight (%d bad)",
             flat_missing);
    ASSERT_TRUE(flat_missing == 0, msg);
    snprintf(msg, sizeof msg,
             "matrix: flat terrain, 450-6000 fpm: every rung heard within 2.5 ft (worst %.1f)",
             (double)worst_flat);
    ASSERT_TRUE(worst_flat <= 2.5f, msg);
    snprintf(msg, sizeof msg,
             "matrix: over terrain features: within 12 ft where heard (worst %.1f)",
             (double)worst_any);
    ASSERT_TRUE(worst_any <= 12.0f, msg);
    snprintf(msg, sizeof msg,
             "matrix: the edge rung (late-rung window) within %.0f ft (worst %.1f)",
             (double)CALLOUT_LATE_TOL_HI_FT, (double)worst_edge);
    ASSERT_TRUE(worst_edge <= CALLOUT_LATE_TOL_HI_FT, msg);
}

int main(void)
{
    printf("== full-sortie integration ==\n");
    test_matrix_acceptance();
    test_sortie_garbage_while_blind();
    test_sortie_gear_down_idle_4000fpm();
    test_sortie_glasair_2000fpm();
    test_sortie_darker_approach_surface_go_arounds();
    test_sortie_lens_reflections();
    test_sortie_full_pattern_and_ils();
    test_sortie_touch_and_go();
    test_sortie_go_around();
    test_sortie_low_pattern_in_range();
    test_sortie_steep_approach();
    test_sortie_taxi_back_disarms();
    test_sortie_inflight_reboot_on_final();

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
