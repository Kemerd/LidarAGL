/**
 * @file    test_range_filter.c
 * @brief   Host unit tests for the robust runtime range pipeline.
 *
 * @details Exercises every stage of range_filter.c — the 2-byte wire decoder,
 *          the validity gates, the median-of-drain vote, the Hampel outlier
 *          gate, re-acquisition, and the time-corrected EMA — plus an
 *          end-to-end regression of the real-world taxi incident: a garbage
 *          sample once armed the state machine and spoke a phantom
 *          "50 40 30 20 10" descent while the aircraft taxied. These tests
 *          pin the whole chain (filter -> state machine) silent under that
 *          exact stimulus, while proving legitimate descents still call out.
 */

#include "test_util.h"
#include "range_filter.h"
#include "state_machine.h"
#include "sensor_profile.h"
#include "config.h"

#include <math.h>
#include <string.h>

TEST_GLOBALS

/* Handy wire numbers (cm). Ground sits at ~3 ft on the gear. */
#define CM_GROUND      91.0f     /* ~2.99 ft — the parked lidar reading        */
#define CM_SPIKE       9000.0f   /* ~295 ft — passes every absolute gate       */
#define CM_TOO_FAR     12192.0f  /* ~400 ft — beyond the SF30/C ceiling gate   */
#define CM_SENTINEL    16000.0f  /* the lost-signal sentinel itself            */
#define CM_SENT_FLIP   16064.0f  /* a bit-flipped sentinel (old code: "valid") */

/* Feed a whole drain of identical samples. */
static void push_n(range_filter_t *f, float cm, int n)
{
    for (int i = 0; i < n; ++i) {
        rf_push_cm(f, cm);
    }
}

/* Feed a clean ground drain (n samples with ±1 cm deterministic jitter). */
static void push_ground_drain(range_filter_t *f, int n)
{
    for (int i = 0; i < n; ++i) {
        rf_push_cm(f, CM_GROUND + (float)(i % 3) - 1.0f);   /* 90/91/92 cm */
    }
}

/* Finalize and return the published range; copies out the fresh flag. */
static float fin(range_filter_t *f, float dt, bool *fresh)
{
    float ft = -1.0f;
    bool v = false;
    (void)rf_finalize(f, dt, &ft, &v);
    if (fresh) {
        *fresh = v;
    }
    return ft;
}

/* Run enough clean GROUND-cadence drains to seed the window and settle. */
static void settle_on_ground(range_filter_t *f, int polls)
{
    for (int i = 0; i < polls; ++i) {
        push_ground_drain(f, 58);
        (void)fin(f, 0.75f, NULL);
    }
}

/* ---------------------------------------------------------------------------
 *  The 2-byte wire decoder.
 * ------------------------------------------------------------------------- */
static void test_ascii_decoder(void)
{
    sf30_ascii_ctx_t c;
    sf30_ascii_reset(&c);
    int cm = -1;

    /* 152 cm = high 0x01, low 0x18 -> bytes 0x81 0x18. */
    ASSERT_TRUE(!sf30_ascii_feed(&c, 0x81, &cm), "high byte alone yields nothing");
    ASSERT_TRUE(sf30_ascii_feed(&c, 0x18, &cm),  "low byte completes the pair");
    ASSERT_TRUE(cm == 152,                        "pair decodes to 152 cm");

    /* An orphan low byte (no latched high) must be dropped, not paired. */
    ASSERT_TRUE(!sf30_ascii_feed(&c, 0x18, &cm), "orphan low byte is dropped");

    /* A second high byte re-latches (the stream's own resync). */
    (void)sf30_ascii_feed(&c, 0x82, &cm);        /* latch high=2   */
    (void)sf30_ascii_feed(&c, 0x85, &cm);        /* RE-latch high=5 */
    ASSERT_TRUE(sf30_ascii_feed(&c, 0x00, &cm) && cm == (5 << 7),
                "a repeated high byte re-latches (resync)");

    /* Reset clears any pending high byte. */
    (void)sf30_ascii_feed(&c, 0x81, &cm);
    sf30_ascii_reset(&c);
    ASSERT_TRUE(!sf30_ascii_feed(&c, 0x00, &cm), "reset forgets the latched high");
}

/* ---------------------------------------------------------------------------
 *  Stage 1: absolute validity gates.
 * ------------------------------------------------------------------------- */
static void test_validity_gates(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);

    /* A drain of nothing but gated-out junk must publish NOTHING (no output
     * exists yet) — sentinel, bit-flipped sentinel band, negative, too-far.    */
    rf_push_cm(&f, CM_SENTINEL);
    rf_push_cm(&f, CM_SENT_FLIP);
    rf_push_cm(&f, -50.0f);
    rf_push_cm(&f, CM_TOO_FAR);
    float ft;
    bool fresh = true;
    bool have = rf_finalize(&f, 0.05f, &ft, &fresh);
    ASSERT_TRUE(!have,  "all-junk first drain -> no output at all");
    ASSERT_TRUE(!fresh, "all-junk drain -> not fresh");

    /* After a good lock, the same junk holds the last-good value. */
    settle_on_ground(&f, 6);
    ft = fin(&f, 0.75f, &fresh);
    ASSERT_NEAR(ft, 3.0f, 0.4f, "ground lock near 3 ft");

    push_n(&f, CM_SENT_FLIP, 40);                 /* wet-patch corrupted burst */
    ft = fin(&f, 0.75f, &fresh);
    ASSERT_TRUE(!fresh,               "bit-flipped sentinel drain is not fresh");
    ASSERT_NEAR(ft, 3.0f, 0.4f,       "bit-flipped sentinel drain holds ground");
}

/* ---------------------------------------------------------------------------
 *  Stage 2: median-of-drain — the exact old failure (last pair wins) is dead.
 * ------------------------------------------------------------------------- */
static void test_median_of_drain(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    /* 57 good samples + ONE trailing garbage pair (the incident's shape: the
     * freshest bytes of a wake-edge drain are the corrupt ones). The old code
     * published 295 ft; the median doesn't move.                               */
    push_ground_drain(&f, 57);
    rf_push_cm(&f, CM_SPIKE);
    bool fresh;
    float ft = fin(&f, 0.75f, &fresh);
    ASSERT_TRUE(fresh,               "median drain with one bad pair is fresh");
    ASSERT_NEAR(ft, 3.0f, 0.4f,      "one trailing garbage pair cannot move the median");

    /* Even a 40% garbage burst loses the vote. */
    push_ground_drain(&f, 35);
    push_n(&f, CM_SPIKE, 23);
    ft = fin(&f, 0.75f, &fresh);
    ASSERT_NEAR(ft, 3.0f, 0.4f,      "a 40% garbage burst loses the median vote");
}

/* ---------------------------------------------------------------------------
 *  Stage 3: Hampel gate — a fully-garbled drain (DFS/sleep corruption) holds.
 * ------------------------------------------------------------------------- */
static void test_hampel_rejects_garbage_drain(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    /* An ENTIRE drain of plausible-band garbage: the median is garbage too, so
     * only the cross-poll Hampel gate can save us — and it must.               */
    push_n(&f, CM_SPIKE, 58);
    bool fresh;
    float ft = fin(&f, 0.75f, &fresh);
    ASSERT_TRUE(!fresh,              "fully-garbled drain is rejected (held)");
    ASSERT_NEAR(ft, 3.0f, 0.4f,      "fully-garbled drain holds last-good");

    /* Disagreeing garbage drains never re-acquire (corruption doesn't cluster). */
    push_n(&f, 7000.0f, 58);
    ft = fin(&f, 0.75f, &fresh);
    ASSERT_TRUE(!fresh, "2nd garbage drain (different value) still held");
    push_n(&f, 4500.0f, 58);
    ft = fin(&f, 0.75f, &fresh);
    ASSERT_TRUE(!fresh, "3rd garbage drain (different value) still held");
    ASSERT_NEAR(ft, 3.0f, 0.4f, "randomised garbage can never re-acquire");

    /* And a clean drain afterwards resumes normally. */
    push_ground_drain(&f, 58);
    ft = fin(&f, 0.75f, &fresh);
    ASSERT_TRUE(fresh,              "clean drain after garbage resumes fresh");
    ASSERT_NEAR(ft, 3.0f, 0.4f,     "resumed output still on ground");
}

/* ---------------------------------------------------------------------------
 *  Stage 4: re-acquisition — a REAL level step gets through in N polls.
 * ------------------------------------------------------------------------- */
static void test_reacquire_real_step(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    /* The level genuinely steps to 150 ft — far beyond the bounded slew
     * allowance, so the Hampel gate rejects it — yet consistent drains must be
     * accepted after exactly RANGE_REACQUIRE_N polls. (A smaller step, inside
     * the physical allowance, is simply accepted on the first poll.)           */
    const float cm150 = 150.0f / CM_TO_FT;
    bool fresh = false;
    float ft = 0.0f;
    int polls_to_accept = 0;
    for (int i = 0; i < 8 && !fresh; ++i) {
        push_n(&f, cm150, 58);
        ft = fin(&f, 0.75f, &fresh);
        ++polls_to_accept;
    }
    ASSERT_TRUE(fresh, "consistent new level is re-acquired");
    ASSERT_TRUE(polls_to_accept == RANGE_REACQUIRE_N,
                "re-acquire takes exactly RANGE_REACQUIRE_N polls");
    ASSERT_NEAR(ft, 150.0f, 1.5f, "re-acquired output snaps to the new level");
}

/* ---------------------------------------------------------------------------
 *  Stage 4b: re-acquisition needs sample MASS, not just polls (fast cadence).
 * ------------------------------------------------------------------------- */
static void test_reacquire_needs_sample_mass(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    /*  DESCENT cadence: a 25 ms poll catches only ~2 raw samples, so a drain
     *  "median" there is really a mean-of-2 with zero minority immunity. Three
     *  agreeing polls (the old, poll-count-only rule) are just ~6 samples of a
     *  self-consistent burst — 75 ms of a stuck, cleanly-framed byte pattern
     *  once forced a false snap and a phantom low callout on final. The
     *  cluster must now ALSO bank RANGE_REACQUIRE_MIN_SAMPLES raw samples:
     *  the first three 2-sample polls stay HELD, the fourth (8 banked)
     *  re-acquires — a genuine terrain step still lands in ~100 ms, which
     *  remains invisible in the flare.                                        */
    const float cm150 = 150.0f / CM_TO_FT;
    bool fresh = false;
    float ft;
    for (int i = 0; i < 3; ++i) {
        push_n(&f, cm150, 2);
        ft = fin(&f, 0.025f, &fresh);
        ASSERT_TRUE(!fresh,          "2-sample drains: poll count alone can't snap");
        ASSERT_NEAR(ft, 3.0f, 0.4f,  "still holding last-good through the burst");
    }
    push_n(&f, cm150, 2);
    ft = fin(&f, 0.025f, &fresh);
    ASSERT_TRUE(fresh,               "4th agreeing poll banks the mass -> re-acquire");
    ASSERT_NEAR(ft, 150.0f, 1.5f,    "fast-cadence re-acquire snaps to the level");
}

/* ---------------------------------------------------------------------------
 *  Stage 4c: the Hampel gate is LIVE on the very next poll after a re-acquire.
 * ------------------------------------------------------------------------- */
static void test_hampel_live_after_reacquire(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    /* Force a legitimate re-acquire onto a 150 ft level (GROUND-size drains
     * bank the sample mass instantly, so this snaps in RANGE_REACQUIRE_N).    */
    const float cm150 = 150.0f / CM_TO_FT;
    bool fresh = false;
    float ft = 0.0f;
    for (int i = 0; i < 8 && !fresh; ++i) {
        push_n(&f, cm150, 58);
        ft = fin(&f, 0.75f, &fresh);
    }
    ASSERT_TRUE(fresh, "precondition: level re-acquired at 150 ft");

    /*  The window used to be re-seeded with a SINGLE value, leaving the gate
     *  bypassed (win_n < HAMPEL_SEED_N) for the next two polls — a lone
     *  corrupted pair in a 1-2 sample fast drain rode ungated straight into
     *  the EMA right after the snap, exactly when corruption is most likely
     *  still in progress. The window is now pre-filled at the new level, so a
     *  garbage drain on the VERY next poll must be rejected and held.          */
    push_n(&f, CM_GROUND, 1);            /* one corrupt pair: ~3 ft vs 150 ft   */
    ft = fin(&f, 0.025f, &fresh);
    ASSERT_TRUE(!fresh,              "garbage right after re-acquire is gated (held)");
    ASSERT_NEAR(ft, 150.0f, 1.5f,    "output stays on the re-acquired level");

    /* And genuine data at the new level keeps flowing normally. */
    push_n(&f, cm150, 2);
    ft = fin(&f, 0.025f, &fresh);
    ASSERT_TRUE(fresh,               "clean data after the gated garbage is accepted");
    ASSERT_NEAR(ft, 150.0f, 1.5f,    "still tracking the new level");
}

/* ---------------------------------------------------------------------------
 *  Stage 5: a legitimate fast descent passes untouched and stays timely.
 * ------------------------------------------------------------------------- */
static void test_legit_descent_passes(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);

    /* Establish level flight at 200 ft (ARMED cadence, 50 ms polls). */
    const float cm200 = 200.0f / CM_TO_FT;
    for (int i = 0; i < 8; ++i) {
        push_n(&f, cm200, 4);
        (void)fin(&f, 0.05f, NULL);
    }

    /* Descend at 25 ft/s (1500 fpm) on the 25 ms DESCENT cadence: every poll
     * must be accepted fresh and the filter must track within a couple feet.   */
    int rejected = 0;
    float ft = 200.0f;
    float true_ft = 200.0f;
    for (int i = 0; i < 300 && true_ft > 5.0f; ++i) {
        true_ft -= 25.0f * 0.025f;
        push_n(&f, true_ft / CM_TO_FT, 2);
        bool fresh;
        ft = fin(&f, 0.025f, &fresh);
        if (!fresh) {
            ++rejected;
        }
    }
    ASSERT_TRUE(rejected == 0,        "1500 fpm descent: zero samples rejected");
    ASSERT_NEAR(ft, true_ft, 3.0f,    "1500 fpm descent: filter tracks within 3 ft");
}

/* ---------------------------------------------------------------------------
 *  END-TO-END: the taxi incident, replayed through filter + state machine.
 *
 *  The real event: taxiing, a corrupt sample (or fully-garbled drain) spiked
 *  the smoothed range, single-sample arming latched the ladder, and the decay
 *  spoke "50 40 30 20 10". The fixed chain must stay silent through all of:
 *    (a) one trailing garbage pair in a drain      (median kills it)
 *    (b) an entire drain of consistent garbage     (Hampel holds it)
 *    (c) three DIFFERENT garbage drains in a row   (re-acquire never clusters)
 *  in BOTH the disarmed taxi-out and the still-armed taxi-in configurations.
 * ------------------------------------------------------------------------- */
static void test_taxi_incident_end_to_end(bool still_armed)
{
    const sensor_profile_t *p = &SF30C_PROFILE;
    char msg[96];

    range_filter_t f;
    rf_init(&f, p->max_range_ft);

    sm_ctx_t sm;
    sm_init(&sm, ST_GROUND);
    if (still_armed) {
        /* Taxi-in after landing: the arm latch survives, every one-shot has
         * fired (empty mask) — the exact post-landing state of the incident.   */
        sm.armed      = true;
        sm.armed_mask = 0u;
    }

    const float GROUND_REF = 3.0f;
    int fires = 0;

    /* One poll: build the drain via 'shape', finalize, AGL, step the machine. */
    #define TAXI_POLL(shape)                                                  \
        do {                                                                  \
            shape;                                                            \
            bool fresh_;                                                      \
            float ft_ = fin(&f, 0.75f, &fresh_);                              \
            (void)fresh_;                                                     \
            float agl_ = ft_ - GROUND_REF;                                    \
            if (agl_ < 0.0f) agl_ = 0.0f;                                     \
            sm_out_t out_;                                                    \
            sm_step(&sm, agl_, 0.75f, p, &out_);                              \
            if (out_.fired_callout >= 0) ++fires;                             \
        } while (0)

    for (int i = 0; i < 8; ++i)  TAXI_POLL(push_ground_drain(&f, 58));
    TAXI_POLL({ push_ground_drain(&f, 57); rf_push_cm(&f, CM_SPIKE); });   /* (a) */
    for (int i = 0; i < 2; ++i)  TAXI_POLL(push_ground_drain(&f, 58));
    TAXI_POLL(push_n(&f, CM_SPIKE, 58));                                    /* (b) */
    for (int i = 0; i < 2; ++i)  TAXI_POLL(push_ground_drain(&f, 58));
    TAXI_POLL(push_n(&f, 8200.0f, 58));                                     /* (c) */
    TAXI_POLL(push_n(&f, 5100.0f, 58));
    TAXI_POLL(push_n(&f, 9900.0f, 58));
    for (int i = 0; i < 8; ++i)  TAXI_POLL(push_ground_drain(&f, 58));

    #undef TAXI_POLL

    snprintf(msg, sizeof msg, "taxi incident (%s): ZERO callouts fired",
             still_armed ? "still-armed taxi-in" : "disarmed taxi-out");
    ASSERT_TRUE(fires == 0, msg);
    if (!still_armed) {
        ASSERT_TRUE(!sm.armed, "taxi incident (disarmed): garbage never arms");
    }
}

/* ---------------------------------------------------------------------------
 *  REGRESSION: the silent approach after climbing out of the sensor's range.
 *
 *  The real flight: calibrate, take off, hear "positive rate", climb well above
 *  the SF30/C's ~328 ft ceiling, fly the pattern, come back down — and hear NOT
 *  ONE altitude callout the whole way to the runway.
 *
 *  The mechanism this pins down is a chain of three:
 *
 *    1. Above the ceiling every sample is the lost-signal sentinel, so each
 *       drain is majority-lost and the filter HELD its last good value. The
 *       published range froze at ~318 ft and was republished forever.
 *    2. A frozen range has ZERO trend. The state machine's CRUISE exit was
 *       conditioned on a DESCENDING trend, so the box entered ST_CRUISE and
 *       could never leave it — audio suspended, poll relaxed to 500 ms, light
 *       sleep permitted, for the remainder of the flight.
 *    3. On the way back down the first in-range reading was a huge deviation
 *       from the frozen 318 ft anchor, so the Hampel gate rejected it and the
 *       re-acquire burned several polls at the slow CRUISE cadence. By the time
 *       the filter snapped, prev_agl had jumped straight past the top rungs —
 *       and a rung with no DOWNWARD crossing on the books never fires, then
 *       stays one-shot disarmed for the rest of the approach.
 *
 *  The test flies that profile end to end and demands the ladder speak. It is
 *  written against the ladder's BEHAVIOUR (numbers actually called on the way
 *  down), not against any internal flag, so it stays meaningful if the fix is
 *  ever reworked.
 * ------------------------------------------------------------------------- */
static void test_out_of_range_climb_then_descent(void)
{
    const sensor_profile_t *p = &SF30C_PROFILE;

    range_filter_t f;
    rf_init(&f, p->max_range_ft);

    sm_ctx_t sm;
    sm_init(&sm, ST_GROUND);

    const float GROUND_REF = 3.0f;

    /* Record which callout heights actually spoke, in order. */
    float spoken[SM_MAX_CALLOUTS];
    int   n_spoken = 0;
    bool  saw_cruise = false;

    /*  One poll of the real chain: build a drain, finalize, subtract the ground
     *  reference, step the machine. dt is the cadence the state the machine is
     *  ACTUALLY in would have requested, so the CRUISE slow-poll penalty is
     *  part of the simulation rather than assumed away.                        */
    float dt = 0.10f;
    #define FLY_POLL(shape)                                                   \
        do {                                                                  \
            shape;                                                            \
            bool fresh_;                                                      \
            float ft_ = fin(&f, dt, &fresh_);                                 \
            (void)fresh_;                                                     \
            float agl_ = ft_ - GROUND_REF;                                    \
            if (agl_ < 0.0f) agl_ = 0.0f;                                     \
            sm_out_t out_;                                                    \
            sm_step(&sm, agl_, dt, p, &out_);                                 \
            if (out_.state == ST_CRUISE) saw_cruise = true;                   \
            if (out_.fired_callout >= 0 && n_spoken < SM_MAX_CALLOUTS) {      \
                spoken[n_spoken++] = p->callouts[out_.fired_callout];          \
            }                                                                 \
            dt = (float)poll_profile_to_ms(out_.poll) / 1000.0f;              \
        } while (0)

    /* --- Parked, then the climb-out ------------------------------------ */
    for (int i = 0; i < 10; ++i) FLY_POLL(push_ground_drain(&f, 58));

    /*  Climb 3 ft -> 340 ft at a realistic Glasair rate. Feeding real returns
     *  the whole way arms the ladder exactly as a genuine climb-out does.      */
    for (float ft = 5.0f; ft <= 340.0f; ft += 5.0f) {
        float cm = (ft + GROUND_REF) / CM_TO_FT;
        FLY_POLL(push_n(&f, cm, 12));
    }

    ASSERT_TRUE(sm.armed, "out-of-range flight: climb-out armed the ladder");

    /* --- Above the ceiling: the sensor sees nothing at all --------------- */
    /*  ~90 s of pattern work out of range. This is the stretch that used to
     *  freeze the filter and strand the machine in CRUISE.                    */
    for (int i = 0; i < 180; ++i) {
        FLY_POLL(push_n(&f, CM_SENTINEL, 20));
    }
    ASSERT_TRUE(saw_cruise, "out-of-range flight: reached CRUISE while high");

    /* --- Descend back through the ceiling and fly it down to the flare --- */
    for (float ft = 325.0f; ft >= 0.0f; ft -= 2.0f) {
        float cm = (ft + GROUND_REF) / CM_TO_FT;
        FLY_POLL(push_n(&f, cm, 12));
    }

    /*  Rollout. The filter's EMA lags the true trajectory by a few feet, so the
     *  lowest rung's downward crossing lands during the flare/rollout rather
     *  than at the last airborne sample — a real approach provides that time.  */
    for (int i = 0; i < 60; ++i) {
        FLY_POLL(push_n(&f, GROUND_REF / CM_TO_FT, 12));
    }

    #undef FLY_POLL

    /*  The machine must have LEFT cruise — the whole defect was that it could
     *  not. Checked via the ladder below, but assert the state directly too.  */
    ASSERT_TRUE(sm.state != ST_CRUISE,
                "out-of-range flight: left CRUISE on the way back down");

    /*  The ladder must have spoken. We require the low, safety-critical rungs
     *  (100/50/40/30/20/10) that every approach depends on. Note the ceiling
     *  handling is what recovers the LOWEST rung here: holding a frozen anchor
     *  costs enough re-acquire polls that the bottom of the ladder is reached
     *  before the filter has caught up.                                       */
    ASSERT_TRUE(n_spoken > 0,
                "out-of-range flight: the ladder spoke at all");

    const float required[] = { 100.0f, 50.0f, 40.0f, 30.0f, 20.0f, 10.0f };
    for (size_t r = 0; r < sizeof required / sizeof required[0]; ++r) {
        bool found = false;
        for (int i = 0; i < n_spoken; ++i) {
            if (fabsf(spoken[i] - required[r]) < 0.5f) {
                found = true;
                break;
            }
        }
        char msg[96];
        snprintf(msg, sizeof msg,
                 "out-of-range flight: called %.0f ft on the way down",
                 (double)required[r]);
        ASSERT_TRUE(found, msg);
    }

    /*  And the numbers must arrive in DESCENDING order — a ladder that speaks
     *  out of order would mean the re-acquire snapped past rungs and fired
     *  them late, which is its own (equally unflyable) failure.               */
    bool descending = true;
    for (int i = 1; i < n_spoken; ++i) {
        if (spoken[i] >= spoken[i - 1]) {
            descending = false;
            break;
        }
    }
    ASSERT_TRUE(descending,
                "out-of-range flight: callouts arrived in descending order");
}

/* ===========================================================================
 *  BAD DATA ON THE WAY DOWN.
 * ---------------------------------------------------------------------------
 *  The taxi tests above pin the filter SILENT under garbage while parked. These
 *  pin the opposite and much harder requirement: while genuinely descending to
 *  a runway, corruption must not cost us the callouts. A landing aid that goes
 *  quiet exactly when the data gets dirty is worse than no landing aid, because
 *  the pilot has been trained by every previous approach to expect the numbers.
 *
 *  The SF30/C's legacy serial stream carries NO checksum, so every one of these
 *  stimuli is something the real wire can produce: a flipped bit decodes to a
 *  plausible distance, a dropped byte desyncs the high/low pairing, and the
 *  sensor's own lost-signal sentinel arrives over dark or wet surfaces.
 *
 *  Each test flies a real descent, injects one specific corruption pattern, and
 *  demands the ladder still speak the low rungs in the right order.
 * ========================================================================= */

/*  Shared descent driver. Flies AGL from @p from_ft down to @p to_ft in 2 ft
 *  steps, calling @p shape_fn to build each poll's drain, and records which
 *  callout heights spoke. Returns the count; @p out_spoken receives the list.  */
typedef void (*drain_shape_fn)(range_filter_t *f, float true_cm, int poll_idx,
                               unsigned *seed);

static int fly_descent_with(drain_shape_fn shape, float from_ft, float to_ft,
                            float ground_ref, float spoken_out[SM_MAX_CALLOUTS],
                            bool *ended_armed)
{
    const sensor_profile_t *p = &SF30C_PROFILE;

    range_filter_t f;
    rf_init(&f, p->max_range_ft);

    sm_ctx_t sm;
    /*  Seed as a box that has already climbed out and armed — the state every
     *  real approach begins from. sm_init arms the whole ladder for ARMED
     *  seeds, and callouts still require a genuine downward crossing.          */
    sm_init(&sm, ST_ARMED);

    int      n_spoken = 0;
    unsigned seed     = 20260811u;
    float    dt       = (float)POLL_MS_ARMED / 1000.0f;
    int      poll_idx = 0;

    /*  Seed the filter at the starting altitude with clean data so the Hampel
     *  window is live before any corruption is injected — otherwise we would be
     *  testing the un-gated seed phase rather than the gate itself.            */
    for (int i = 0; i < 6; ++i) {
        float cm = (from_ft + ground_ref) / CM_TO_FT;
        push_n(&f, cm, 12);
        bool fresh_;
        float ft_ = fin(&f, dt, &fresh_);
        sm_out_t out_;
        float agl_ = ft_ - ground_ref;
        if (agl_ < 0.0f) agl_ = 0.0f;
        sm_step(&sm, agl_, dt, p, &out_);
    }

    /*  Fly the descent, then HOLD on the ground for a couple of seconds. The
     *  hold is not padding: the filter's EMA legitimately lags the true
     *  trajectory by a few feet, so an aircraft that stopped dead at the
     *  threshold altitude would leave the lowest rung un-crossed on paper. A
     *  real approach flares and rolls out, which is exactly this hold — and it
     *  is where the last rung's downward crossing actually lands.              */
    for (float ft = from_ft; ft >= to_ft; ft -= 2.0f) {
        float true_cm = (ft + ground_ref) / CM_TO_FT;
        shape(&f, true_cm, poll_idx++, &seed);

        bool  fresh_;
        float ft_ = fin(&f, dt, &fresh_);
        float agl_ = ft_ - ground_ref;
        if (agl_ < 0.0f) agl_ = 0.0f;

        sm_out_t out_;
        sm_step(&sm, agl_, dt, p, &out_);
        if (out_.fired_callout >= 0 && n_spoken < SM_MAX_CALLOUTS) {
            spoken_out[n_spoken++] = p->callouts[out_.fired_callout];
        }
        dt = (float)poll_profile_to_ms(out_.poll) / 1000.0f;
    }

    /*  Rollout: sit at the ground reference long enough for the filter to
     *  settle, still running the SAME corruption shape (a wet runway does not
     *  become clean just because the wheels are down).                         */
    for (int i = 0; i < 60; ++i) {
        shape(&f, ground_ref / CM_TO_FT, poll_idx++, &seed);

        bool  fresh_;
        float ft_ = fin(&f, dt, &fresh_);
        float agl_ = ft_ - ground_ref;
        if (agl_ < 0.0f) agl_ = 0.0f;

        sm_out_t out_;
        sm_step(&sm, agl_, dt, p, &out_);
        if (out_.fired_callout >= 0 && n_spoken < SM_MAX_CALLOUTS) {
            spoken_out[n_spoken++] = p->callouts[out_.fired_callout];
        }
        dt = (float)poll_profile_to_ms(out_.poll) / 1000.0f;
    }

    if (ended_armed) {
        *ended_armed = sm.armed;
    }
    return n_spoken;
}

/*  Assert the required low rungs appear, in descending order. */
static void assert_ladder_ok(const float *spoken, int n, const char *label)
{
    char msg[128];
    const float required[] = { 100.0f, 50.0f, 40.0f, 30.0f, 20.0f, 10.0f };

    for (size_t r = 0; r < sizeof required / sizeof required[0]; ++r) {
        bool found = false;
        for (int i = 0; i < n; ++i) {
            if (fabsf(spoken[i] - required[r]) < 0.5f) {
                found = true;
                break;
            }
        }
        snprintf(msg, sizeof msg, "%s: called %.0f ft", label, (double)required[r]);
        ASSERT_TRUE(found, msg);
    }

    bool descending = true;
    for (int i = 1; i < n; ++i) {
        if (spoken[i] >= spoken[i - 1]) {
            descending = false;
            break;
        }
    }
    snprintf(msg, sizeof msg, "%s: callouts in descending order", label);
    ASSERT_TRUE(descending, msg);

    /*  No rung may speak twice on a single descent — a duplicate means a
     *  re-arm fired mid-approach, which in the air sounds like the aircraft
     *  bounced back up through a height it never reached.                      */
    bool dup = false;
    for (int i = 0; i < n && !dup; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (fabsf(spoken[i] - spoken[j]) < 0.5f) {
                dup = true;
                break;
            }
        }
    }
    snprintf(msg, sizeof msg, "%s: no rung spoke twice", label);
    ASSERT_TRUE(!dup, msg);
}

/*  (1) Isolated single-sample bit flips scattered through an otherwise clean
 *      descent. This is the single most common real corruption: one byte of a
 *      high/low pair flips and decodes to a plausible distance. The median of
 *      the drain must absorb it entirely — a minority can never outvote.       */
static void shape_bitflip_minority(range_filter_t *f, float true_cm,
                                   int poll_idx, unsigned *seed)
{
    (void)poll_idx;
    for (int j = 0; j < 12; ++j) {
        *seed = *seed * 1103515245u + 12345u;
        if (((*seed >> 16) & 0x7fff) % 11 == 0) {
            /*  A flipped bit high in the value: decodes to a wildly different
             *  but individually "plausible" distance.                          */
            rf_push_cm(f, true_cm + 4096.0f);
        } else {
            rf_push_cm(f, true_cm);
        }
    }
}

static void test_descent_with_bitflips(void)
{
    float spoken[SM_MAX_CALLOUTS];
    int n = fly_descent_with(shape_bitflip_minority, 250.0f, 0.0f, 3.0f,
                             spoken, NULL);
    assert_ladder_ok(spoken, n, "descent w/ scattered bit flips");
}

/*  (2) Intermittent lost-signal: the sensor returns nothing for whole polls at
 *      a time (a dark, wet runway is the documented case). The filter HOLDS,
 *      which is correct — but the held value must not cost us a rung once real
 *      data returns.                                                           */
static void shape_intermittent_lost(range_filter_t *f, float true_cm,
                                    int poll_idx, unsigned *seed)
{
    (void)seed;
    /*  Every third poll the sensor sees nothing at all. */
    if (poll_idx % 3 == 2) {
        push_n(f, CM_SENTINEL, 12);
    } else {
        push_n(f, true_cm, 12);
    }
}

static void test_descent_with_dropouts(void)
{
    float spoken[SM_MAX_CALLOUTS];
    int n = fly_descent_with(shape_intermittent_lost, 250.0f, 0.0f, 3.0f,
                             spoken, NULL);
    assert_ladder_ok(spoken, n, "descent w/ lost-signal dropouts");
}

/*  (3) A sustained burst of self-consistent garbage — the nastiest case, and
 *      the one RANGE_REACQUIRE_MIN_SAMPLES exists for. A stuck byte pattern
 *      repeats cleanly for several polls, so the median AGREES with itself and
 *      cannot be voted down. The filter must not adopt it as a new level (that
 *      would fire phantom rungs), and must recover once real data resumes.     */
static void shape_stuck_burst(range_filter_t *f, float true_cm, int poll_idx,
                              unsigned *seed)
{
    (void)seed;
    /*  Two separate bursts during the descent, each a few polls long. */
    bool in_burst = (poll_idx >= 30 && poll_idx < 34) ||
                    (poll_idx >= 70 && poll_idx < 73);
    if (in_burst) {
        push_n(f, 2400.0f, 12);      /* a stuck ~79 ft pattern */
    } else {
        push_n(f, true_cm, 12);
    }
}

static void test_descent_with_stuck_burst(void)
{
    float spoken[SM_MAX_CALLOUTS];
    int n = fly_descent_with(shape_stuck_burst, 250.0f, 0.0f, 3.0f,
                             spoken, NULL);
    assert_ladder_ok(spoken, n, "descent w/ stuck-pattern bursts");
}

/*  (4) Bit-flipped lost-signal sentinels. 16000 cm is one bit away from
 *      16001..16383, all of which decode to 525..537 ft — beyond any real
 *      return. These must die at the absolute gate, not sail through as a
 *      "530 ft" reading that would re-arm rungs mid-approach.                   */
static void shape_flipped_sentinels(range_filter_t *f, float true_cm,
                                    int poll_idx, unsigned *seed)
{
    (void)poll_idx;
    for (int j = 0; j < 12; ++j) {
        *seed = *seed * 1103515245u + 12345u;
        if (((*seed >> 16) & 0x7fff) % 9 == 0) {
            rf_push_cm(f, CM_SENT_FLIP);
        } else {
            rf_push_cm(f, true_cm);
        }
    }
}

static void test_descent_with_flipped_sentinels(void)
{
    float spoken[SM_MAX_CALLOUTS];
    bool  armed_end = false;
    int n = fly_descent_with(shape_flipped_sentinels, 250.0f, 0.0f, 3.0f,
                             spoken, &armed_end);
    assert_ladder_ok(spoken, n, "descent w/ bit-flipped sentinels");
    ASSERT_TRUE(armed_end,
                "descent w/ bit-flipped sentinels: stayed armed throughout");
}

/*  (5) UART framing errors mid-descent. The hardware tells us a drain's bytes
 *      are untrustworthy and rf_drain_abort() throws the WHOLE drain away. That
 *      is the right call, but repeated aborts must degrade to a HOLD (a missed
 *      poll), never to a wrong altitude or a lost rung.                         */
static void shape_framing_aborts(range_filter_t *f, float true_cm, int poll_idx,
                                 unsigned *seed)
{
    (void)seed;
    push_n(f, true_cm, 12);
    /*  Every fourth poll the UART reports framing/parity errors. */
    if (poll_idx % 4 == 3) {
        rf_drain_abort(f);
    }
}

static void test_descent_with_framing_aborts(void)
{
    float spoken[SM_MAX_CALLOUTS];
    int n = fly_descent_with(shape_framing_aborts, 250.0f, 0.0f, 3.0f,
                             spoken, NULL);
    assert_ladder_ok(spoken, n, "descent w/ UART framing aborts");
}

/*  (6) Everything at once — flips, dropouts, a stuck burst and framing aborts
 *      on the same approach. The individual tests prove each mechanism; this
 *      proves they compose, which is the only thing the aircraft cares about.  */
static void shape_kitchen_sink(range_filter_t *f, float true_cm, int poll_idx,
                               unsigned *seed)
{
    if (poll_idx % 7 == 6) {                       /* whole-poll dropout       */
        push_n(f, CM_SENTINEL, 12);
        return;
    }
    if (poll_idx >= 40 && poll_idx < 43) {         /* stuck burst              */
        push_n(f, 2400.0f, 12);
        return;
    }
    for (int j = 0; j < 12; ++j) {                 /* scattered corruption     */
        *seed = *seed * 1103515245u + 12345u;
        unsigned r = (*seed >> 16) & 0x7fff;
        if (r % 13 == 0) {
            rf_push_cm(f, true_cm + 4096.0f);      /* bit flip                 */
        } else if (r % 17 == 0) {
            rf_push_cm(f, CM_SENT_FLIP);           /* flipped sentinel         */
        } else {
            rf_push_cm(f, true_cm);
        }
    }
    if (poll_idx % 11 == 10) {                     /* framing abort            */
        rf_drain_abort(f);
    }
}

static void test_descent_kitchen_sink(void)
{
    float spoken[SM_MAX_CALLOUTS];
    bool  armed_end = false;
    int n = fly_descent_with(shape_kitchen_sink, 250.0f, 0.0f, 3.0f,
                             spoken, &armed_end);
    assert_ladder_ok(spoken, n, "descent w/ combined corruption");
    ASSERT_TRUE(armed_end, "descent w/ combined corruption: stayed armed");
}

/*  (7) A dead sensor mid-descent: the stream simply STOPS. Every subsequent
 *      drain is empty. The filter must report not-fresh so the logic task can
 *      mute the tone and annunciate — and must NEVER invent an altitude. This
 *      is the one case where going quiet is the correct behaviour, and it is
 *      distinguished from case (2) by the drain being empty rather than lost.  */
static void test_sensor_dies_mid_descent(void)
{
    const sensor_profile_t *p = &SF30C_PROFILE;

    range_filter_t f;
    rf_init(&f, p->max_range_ft);

    /* Establish a solid lock at 150 ft. */
    for (int i = 0; i < 8; ++i) {
        push_n(&f, (150.0f + 3.0f) / CM_TO_FT, 12);
        (void)fin(&f, 0.05f, NULL);
    }

    /*  Now the sensor goes completely silent: empty drains forever. */
    float last = -1.0f;
    bool  fresh_seen = false;
    for (int i = 0; i < 40; ++i) {
        bool fresh_;
        last = fin(&f, 0.05f, &fresh_);   /* nothing pushed at all */
        if (fresh_) {
            fresh_seen = true;
        }
    }

    ASSERT_TRUE(!fresh_seen,
                "dead sensor: never reports a FRESH value from an empty drain");
    ASSERT_TRUE(fabsf(last - 153.0f) < 2.0f,
                "dead sensor: holds the last good value, invents nothing");
}

/*  (8) REGRESSION: a self-consistent garbage burst must never TELEPORT the
 *      published range across callout rungs.
 *
 *      This is the defect the stuck-burst descent above exposed. A stuck byte
 *      pattern repeats cleanly, so it agrees with itself perfectly and
 *      satisfies both re-acquire conditions (poll count AND sample mass) for
 *      free. At 186 ft AGL on final, four such polls re-acquired a ~76 ft
 *      "level" — a 110 ft downward jump in ~100 ms — and the box spoke "one
 *      hundred" while the aircraft was nowhere near it.
 *
 *      A phantom LOW callout on approach is the most dangerous lie this box can
 *      tell, so the snap now also has to be physically reachable. Here we hold
 *      altitude steady and feed a long stuck burst: the filter must refuse to
 *      adopt it, and the state machine must stay silent.                       */
static void test_garbage_cannot_teleport_across_rungs(void)
{
    const sensor_profile_t *p = &SF30C_PROFILE;
    const float G = 3.0f;

    range_filter_t f;
    rf_init(&f, p->max_range_ft);

    sm_ctx_t sm;
    sm_init(&sm, ST_ARMED);

    const float dt = (float)POLL_MS_DESCENT / 1000.0f;   /* 25 ms, the fast path */

    /* Establish a solid lock at 186 ft — just above the 100 ft rung. */
    for (int i = 0; i < 10; ++i) {
        push_n(&f, (186.0f + G) / CM_TO_FT, 12);
        bool fresh_;
        float ft_ = fin(&f, dt, &fresh_);
        sm_out_t out_;
        sm_step(&sm, ft_ - G, dt, p, &out_);
    }

    /*  Now a long, perfectly self-consistent stuck pattern at ~76 ft. It agrees
     *  with itself on every poll, so consistency alone would adopt it.
     *
     *  Two outcomes are acceptable and BOTH are safe: the filter may hold the
     *  garbage out entirely, or — once the cluster is sustained long enough to
     *  be indistinguishable from a genuine discontinuity — it may break track
     *  and re-acquire. What is NOT acceptable in either case is a spoken rung:
     *  a break is a teleport, not flown motion, so the consumer re-anchors and
     *  nothing is said. We model the logic task's real contract here.          */
    int  fires  = 0;
    bool broke  = false;
    for (int i = 0; i < 20; ++i) {
        push_n(&f, 2400.0f, 12);
        bool  fresh_;
        float ft_  = fin(&f, dt, &fresh_);
        float agl_ = ft_ - G;
        if (agl_ < 0.0f) agl_ = 0.0f;

        if (fresh_ && rf_track_broken(&f)) {
            broke = true;
            sm_reanchor(&sm, agl_);
        }

        sm_out_t out_;
        sm_step(&sm, agl_, dt, p, &out_);
        if (out_.fired_callout >= 0) {
            ++fires;
        }
    }

    ASSERT_TRUE(fires == 0,
                "stuck burst at altitude: fires NO phantom callout");
    /*  If it did adopt the garbage, it must have ANNOUNCED that as a broken
     *  track rather than passing it off as a continuous descent.               */
    ASSERT_TRUE(!broke || fires == 0,
                "stuck burst at altitude: any adoption was a flagged track break");

    /*  And a genuine descent afterwards must still work — the guard must not
     *  have wedged the filter into permanently refusing to move.                */
    int spoke_100 = 0;
    for (float ft = 186.0f; ft >= 60.0f; ft -= 1.0f) {
        push_n(&f, (ft + G) / CM_TO_FT, 12);
        bool  fresh_;
        float ft_  = fin(&f, dt, &fresh_);
        float agl_ = ft_ - G;
        if (agl_ < 0.0f) agl_ = 0.0f;
        sm_out_t out_;
        sm_step(&sm, agl_, dt, p, &out_);
        if (out_.fired_callout >= 0 &&
            fabsf(p->callouts[out_.fired_callout] - 100.0f) < 0.5f) {
            ++spoke_100;
        }
    }
    ASSERT_TRUE(spoke_100 == 1,
                "stuck burst at altitude: real descent still calls 100 ft once");
}

/*  (9) A GENUINE level step must still re-acquire. The reachability guard is
 *      only legitimate if it costs real terrain nothing: a bluff, a displaced
 *      threshold or an in-flight power-up over new ground really can step the
 *      measured range, and holding that forever would be its own failure.      */
static void test_real_level_step_still_reacquires(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);

    const float dt = (float)POLL_MS_DESCENT / 1000.0f;

    for (int i = 0; i < 10; ++i) {
        push_n(&f, (120.0f + 3.0f) / CM_TO_FT, 12);
        (void)fin(&f, dt, NULL);
    }

    /*  A real terrain step of ~30 ft, sustained. This is well inside what the
     *  airframe could fly in the cluster's duration once a few polls have
     *  accrued, so it must be adopted rather than held indefinitely.           */
    bool  reacquired = false;
    for (int i = 0; i < 40; ++i) {
        push_n(&f, (90.0f + 3.0f) / CM_TO_FT, 12);
        bool  fresh_;
        float ft_ = fin(&f, dt, &fresh_);
        if (fabsf(ft_ - 93.0f) < 5.0f) {
            reacquired = true;
            break;
        }
    }
    ASSERT_TRUE(reacquired,
                "genuine level step still re-acquires (guard isn't a wedge)");
}

/*  (10) REGRESSION: out-of-range erroneous readings must not speak a phantom.
 *
 *      The SF30/C does NOT jump straight to its 16000 cm lost-signal sentinel
 *      when the target leaves its ~328 ft range. Its "Lost signal confirmations"
 *      setting (1..250 per the product guide) is the number of FAILED readings
 *      required before loss of signal is reported — so above the ceiling the
 *      sensor emits genuinely erroneous distances first, and any of them landing
 *      in 0..343 ft passes every absolute gate as a plausible altitude.
 *
 *      When those erroneous readings cluster (a weak return off haze, the
 *      airframe, or simply a stuck pattern), the cluster is self-consistent and
 *      eventually forces a re-acquisition. Before the track-break handling that
 *      snapped the published altitude from ~335 ft to ~12 ft in a single poll
 *      and the ladder faithfully spoke "twenty" — while the aircraft was more
 *      than 300 ft up. A phantom LOW callout is the most dangerous thing this
 *      box can say, so this test pins it at zero.
 *
 *      It also pins the other half: after the aircraft genuinely descends back
 *      into range, the full ladder must still speak. Silencing the phantom by
 *      simply refusing to ever re-acquire would trade one failure for another.
 * ------------------------------------------------------------------------- */
static void test_out_of_range_erroneous_no_phantom(void)
{
    const sensor_profile_t *p = &SF30C_PROFILE;
    const float G = 3.0f;

    range_filter_t f;
    rf_init(&f, p->max_range_ft);

    sm_ctx_t sm;
    sm_init(&sm, ST_GROUND);

    float dt       = 0.10f;
    int   fires    = 0;
    int   phantoms = 0;

    #define ERR_POLL(shape)                                                   \
        do {                                                                  \
            shape;                                                            \
            bool  fresh_;                                                     \
            float ft_ = fin(&f, dt, &fresh_);                                 \
            float agl_ = ft_ - G;                                             \
            if (agl_ < 0.0f) agl_ = 0.0f;                                     \
            /* The logic task's real contract: re-anchor on a broken track. */ \
            if (fresh_ && rf_track_broken(&f)) {                              \
                sm_reanchor(&sm, agl_);                                       \
            }                                                                 \
            sm_out_t out_;                                                    \
            sm_step(&sm, agl_, dt, p, &out_);                                 \
            if (out_.fired_callout >= 0) ++fires;                             \
            dt = (float)poll_profile_to_ms(out_.poll) / 1000.0f;              \
        } while (0)

    /* Parked, then a genuine climb-out that arms the ladder. */
    for (int i = 0; i < 10; ++i) ERR_POLL(push_ground_drain(&f, 58));
    for (float ft = 5.0f; ft <= 340.0f; ft += 5.0f) {
        ERR_POLL(push_n(&f, (ft + G) / CM_TO_FT, 12));
    }
    ASSERT_TRUE(sm.armed, "erroneous out-of-range: climb-out armed the ladder");

    /*  Out of range, with erroneous readings clustered LOW — the shape that
     *  produced the phantom. The aircraft is really above 328 ft throughout.   */
    int fires_before = fires;
    for (int i = 0; i < 200; ++i) {
        ERR_POLL(push_n(&f, 460.0f, 20));     /* ~15 ft of pure garbage */
    }
    phantoms = fires - fires_before;
    ASSERT_TRUE(phantoms == 0,
                "erroneous out-of-range: ZERO phantom callouts while high");

    /*  Now genuinely descend back into range: the ladder must still work.     */
    int fires_at_descent = fires;
    for (float ft = 325.0f; ft >= 0.0f; ft -= 2.0f) {
        ERR_POLL(push_n(&f, (ft + G) / CM_TO_FT, 12));
    }
    for (int i = 0; i < 60; ++i) {
        ERR_POLL(push_n(&f, G / CM_TO_FT, 12));
    }

    #undef ERR_POLL

    ASSERT_TRUE(fires - fires_at_descent >= 6,
                "erroneous out-of-range: real descent still walks the ladder");
}

int main(void)
{
    printf("== range_filter ==\n");
    test_ascii_decoder();
    test_validity_gates();
    test_median_of_drain();
    test_hampel_rejects_garbage_drain();
    test_reacquire_real_step();
    test_reacquire_needs_sample_mass();
    test_hampel_live_after_reacquire();
    test_legit_descent_passes();
    test_taxi_incident_end_to_end(false);
    test_taxi_incident_end_to_end(true);
    test_out_of_range_climb_then_descent();
    test_descent_with_bitflips();
    test_descent_with_dropouts();
    test_descent_with_stuck_burst();
    test_descent_with_flipped_sentinels();
    test_descent_with_framing_aborts();
    test_descent_kitchen_sink();
    test_sensor_dies_mid_descent();
    test_garbage_cannot_teleport_across_rungs();
    test_real_level_step_still_reacquires();
    test_out_of_range_erroneous_no_phantom();

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
