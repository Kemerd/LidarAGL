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
} flight_t;

static void flight_init(flight_t *fl, const sensor_profile_t *p, sm_state_t initial)
{
    memset(fl, 0, sizeof *fl);
    fl->p    = p;
    fl->dt   = (float)POLL_MS_GROUND / 1000.0f;
    fl->seed = 1234567u;
    fl->worst_callout_err = 0.0f;
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
 * the ground reference, honours a track break the way the logic task does, and
 * steps the state machine.
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

    for (int i = 0; i < n; ++i) {
        float agl = true_agl_ft;
        if (agl > fl->p->max_range_ft - GROUND_REF_FT) {
            /*  Beyond the sensor's reach. The SF30 does NOT jump straight to
             *  its sentinel — "Lost signal confirmations" means it emits real
             *  erroneous distances first — so model a mix, which is what the
             *  hardware genuinely puts on the wire near and above its ceiling. */
            if (frand(fl) < 0.75f) {
                rf_push_cm(&fl->f, (float)SF30_LOST_SIGNAL_CM);
            } else {
                rf_push_cm(&fl->f, frand(fl) * 10000.0f);
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

    /* The logic task's real contract on a discontinuous re-acquisition. */
    if (fresh && rf_track_broken(&fl->f)) {
        sm_reanchor(&fl->sm, agl);
    }

    sm_out_t out;
    sm_step(&fl->sm, agl, fl->dt, fl->p, &out);

    if (out.state == ST_CRUISE) {
        fl->saw_cruise = true;
    }
    if (fl->sm.armed) {
        fl->ever_armed = true;
    }
    if (out.fired_positive_rate) {
        fl->n_posrate++;
    }
    if (out.fired_callout >= 0 && fl->n_spoken < MAX_SPOKEN) {
        float h = fl->p->callouts[out.fired_callout];
        fl->spoken[fl->n_spoken]          = h;
        fl->spoken_true_agl[fl->n_spoken] = true_agl_ft;
        fl->n_spoken++;
        float err = fabsf(h - true_agl_ft);
        if (err > fl->worst_callout_err) {
            fl->worst_callout_err = err;
        }
    }

    fl->t += fl->dt;
    fl->dt = (float)poll_profile_to_ms(out.poll) / 1000.0f;
}

/**
 * @brief Fly a constant-rate segment from @p from_ft to @p to_ft.
 * @param rate_fps  Magnitude of the vertical rate (ft/s); direction is inferred.
 */
static void fly_segment(flight_t *fl, float from_ft, float to_ft,
                        float rate_fps, float noise_ft)
{
    float agl = from_ft;
    bool  down = (to_ft < from_ft);
    /* Generous iteration bound: a 40 s segment at the 25 ms DESCENT cadence is
     * ~1600 polls; the cap only stops a runaway if a rate is ever passed as 0. */
    for (int guard = 0; guard < 200000; ++guard) {
        flight_poll(fl, agl, noise_ft);
        agl += (down ? -1.0f : 1.0f) * rate_fps * fl->dt;
        if (down ? (agl <= to_ft) : (agl >= to_ft)) {
            break;
        }
    }
}

/** @brief Hold level at @p agl_ft for @p secs. */
static void fly_level(flight_t *fl, float agl_ft, float secs, float noise_ft)
{
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

int main(void)
{
    printf("== full-sortie integration ==\n");
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
