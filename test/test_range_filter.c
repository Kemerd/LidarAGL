/**
 * @file    test_range_filter.c
 * @brief   Host unit tests for the range tracker (range_filter.c).
 *
 * @details Exercises every stage of the per-sample tracker — the 2-byte wire
 *          decoder, the validity gates, the Kalman track's innovation gate,
 *          COAST / LOST, M-of-N candidate confirmation, track breaks, the
 *          re-entry verdict and sample-and-hold repeats — plus end-to-end
 *          regressions through the state machine: the real-world taxi
 *          incident (a garbage sample once armed the ladder and spoke a
 *          phantom "50 40 30 20 10" while taxiing), the silent approaches, and
 *          the stuck-pattern teleport.
 *
 *          INPUTS ARE PHYSICAL. The tracker timestamps every raw sample (a
 *          drain's samples are spread uniformly across its interval), so a
 *          moving aircraft must be fed as the continuous RAMP a real sensor
 *          produces (push_ramp), at a physical rate. The v1.63 tests fed each
 *          poll as identical samples followed by a jump — a staircase whose
 *          "rate" changed with the poll cadence — which a per-poll median never
 *          noticed but which a per-sample tracker correctly reads as a
 *          teleport. Tests whose assertions named a v1.63 MECHANISM (median,
 *          Hampel window, poll counts) now assert the equivalent BEHAVIOUR;
 *          each says what changed and why.
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

/*  Feed a drain that RAMPS linearly from the previous poll's value to this
 *  one's: sample j of n reads cm_from + (cm_to - cm_from) * (j+1)/n, so the
 *  last sample is "now". This is the continuous stream a moving aircraft
 *  produces — see the file header for why staircases are not.               */
static void push_ramp(range_filter_t *f, float cm_from, float cm_to, int n)
{
    for (int i = 0; i < n; ++i) {
        rf_push_cm(f, cm_from + (cm_to - cm_from) * (float)(i + 1) / (float)n);
    }
}

/*  Feet (range) -> cm, the wire unit. */
static float ft_cm(float ft)
{
    return ft / CM_TO_FT;
}

/* Feed a clean ground drain (n samples with ±1 cm deterministic jitter). */
static void push_ground_drain(range_filter_t *f, int n)
{
    for (int i = 0; i < n; ++i) {
        rf_push_cm(f, CM_GROUND + (float)(i % 3) - 1.0f);   /* 90/91/92 cm */
    }
}

/*  The logic task's contract on a broken track (app_main.c): re-anchor the
 *  state machine, with the late-rung window open only for a flyable
 *  descending re-entry. Returns the rung the window announced, or -1 — a
 *  caller counting "what the box said" must count it like a fired callout.   */
static int reanchor_like_logic_task(sm_ctx_t *sm, const range_filter_t *f,
                                    float agl, const sensor_profile_t *p)
{
    return sm_reanchor(sm, agl, p, rf_break_reentry(f), NULL);
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
 *  A minority of garbage cannot move the output — the exact old failure (the
 *  LAST pair of a drain won) is dead. v1.63 proved this with a per-drain
 *  median; the tracker proves it per sample: every garbage sample fails the
 *  innovation gate on its own.
 * ------------------------------------------------------------------------- */
static void test_minority_garbage(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    /* 57 good samples + ONE trailing garbage pair (the incident's shape: the
     * freshest bytes of a wake-edge drain are the corrupt ones). The old code
     * published 295 ft; the tracker rejects the sample.                        */
    push_ground_drain(&f, 57);
    rf_push_cm(&f, CM_SPIKE);
    bool fresh;
    float ft = fin(&f, 0.75f, &fresh);
    ASSERT_TRUE(fresh,               "drain with one bad pair is still fresh");
    ASSERT_NEAR(ft, 3.0f, 0.4f,      "one trailing garbage pair cannot move the output");

    /* 40% garbage INTERLEAVED through the drain: each junk sample fails the
     * gate, and junk can never gather the majority a candidate needs.       */
    for (int i = 0; i < 58; ++i) {
        if (i % 5 < 2) {
            rf_push_cm(&f, CM_SPIKE - (float)(i % 7) * 900.0f);
        } else {
            rf_push_cm(&f, CM_GROUND + (float)(i % 3) - 1.0f);
        }
    }
    ft = fin(&f, 0.75f, &fresh);
    ASSERT_TRUE(fresh,               "40% interleaved garbage: still fresh");
    ASSERT_NEAR(ft, 3.0f, 0.4f,      "40% interleaved garbage cannot move the output");

    /* A 23-sample (0.3 s) garbage BURST at the end of the drain: the track
     * coasts through it on its own prediction.                              */
    push_ground_drain(&f, 35);
    push_n(&f, CM_SPIKE, 23);
    ft = fin(&f, 0.75f, &fresh);
    ASSERT_NEAR(ft, 3.0f, 0.4f,      "a 0.3 s garbage burst is coasted through");
    push_ground_drain(&f, 58);
    ft = fin(&f, 0.75f, &fresh);
    ASSERT_TRUE(fresh && !rf_track_broken(&f),
                "after the burst the SAME track resumes (no break)");
    ASSERT_NEAR(ft, 3.0f, 0.4f,      "after the burst: still on the ground");
}

/* ---------------------------------------------------------------------------
 *  A fully-garbled drain (DFS/sleep corruption) holds. v1.63 needed a
 *  cross-poll Hampel window for this; here the whole drain fails the gate,
 *  the track coasts then goes LOST (held), and a stationary level that far
 *  from the hold would need RF_BREAK_S of persistence to be adopted.
 * ------------------------------------------------------------------------- */
static void test_rejects_garbage_drain(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    /* An ENTIRE drain of plausible-band garbage — here the worst case, a
     * perfectly self-consistent value: nothing in the drain can outvote it,
     * so only the track's own gate and the break rules can save us.         */
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
 *  A REAL level step far beyond physical reach is adopted — as a flagged
 *  TRACK BREAK, after RF_BREAK_S of persistence.
 *
 *  v1.63 took any step after 3 agreeing polls (2.25 s at this cadence, 75 ms at
 *  the fastest) and let UPWARD steps through as continuous motion. Now every
 *  step beyond the continuous reach needs the same wall-clock persistence at
 *  every cadence and is reported as a break, so the ladder re-anchors on it
 *  instead of reading it as flown motion.
 * ------------------------------------------------------------------------- */
static void test_reacquire_real_step(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    const float cm150 = 150.0f / CM_TO_FT;
    bool  fresh = false;
    bool  broke = false;
    float ft    = 0.0f;
    float t     = 0.0f;
    for (int i = 0; i < 8 && !fresh; ++i) {
        push_n(&f, cm150, 58);
        ft = fin(&f, 0.75f, &fresh);
        broke = rf_track_broken(&f);
        t += 0.75f;
    }
    ASSERT_TRUE(fresh, "consistent new level is re-acquired");
    ASSERT_TRUE(t >= RF_BREAK_S && t <= RF_COAST_MAX_S + RF_BREAK_S + 0.75f,
                "an unreachable step needs ~RF_BREAK_S of persistence, no more");
    ASSERT_TRUE(broke,              "...and is reported as a TRACK BREAK");
    ASSERT_TRUE(!rf_break_reentry(&f),
                "...never as a re-entry (the returns never stopped)");
    ASSERT_NEAR(ft, 150.0f, 1.5f,   "re-acquired output is the new level");
}

/* ---------------------------------------------------------------------------
 *  Re-acquisition needs TIME-SPANNED evidence, not polls (fast cadence).
 *
 *  At the 25 ms DESCENT poll a drain is ~2 raw samples. v1.63 counted polls
 *  and then had to add a raw-sample "mass" rule after 75 ms of a stuck,
 *  cleanly-framed byte pattern forced a false snap and a phantom low callout
 *  on final. The tracker's M-of-N rule is per sample by construction:
 *  RF_CONFIRM_SAMPLES members spanning RF_CONFIRM_MIN_S, however the polls
 *  fall. Here a reachable level (+27 ft of range) must stay held for the
 *  first 75 ms, then confirm continuously (no break) within ~150 ms.
 * ------------------------------------------------------------------------- */
static void test_reacquire_needs_timed_evidence(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    bool  fresh = false;
    float ft    = 0.0f;
    int   polls = 0;
    for (int i = 0; i < 3; ++i) {
        rf_push_cm(&f, ft_cm(30.0f) + 1.0f);          /* +-1 cm real jitter   */
        rf_push_cm(&f, ft_cm(30.0f) - 1.0f);
        ft = fin(&f, 0.025f, &fresh);
        ++polls;
        ASSERT_TRUE(!fresh,          "75 ms of a new level: poll count alone can't snap");
        ASSERT_NEAR(ft, 3.0f, 0.6f,  "still on the old level through it");
    }
    while (!fresh && polls < 12) {
        rf_push_cm(&f, ft_cm(30.0f) + 1.0f);
        rf_push_cm(&f, ft_cm(30.0f) - 1.0f);
        ft = fin(&f, 0.025f, &fresh);
        ++polls;
    }
    ASSERT_TRUE(fresh && polls <= 6, "confirmed once the evidence spans ~0.1 s");
    ASSERT_TRUE(!rf_track_broken(&f),"a reachable level hands over continuously");
    ASSERT_NEAR(ft, 30.0f, 1.0f,     "fast-cadence re-acquire lands on the level");
}

/* ---------------------------------------------------------------------------
 *  Stage 4c: the Hampel gate is LIVE on the very next poll after a re-acquire.
 * ------------------------------------------------------------------------- */
static void test_gate_live_after_reacquire(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 6);

    /* Force a legitimate re-acquire onto a 150 ft level (a flagged break
     * after RF_BREAK_S, see test_reacquire_real_step).                      */
    const float cm150 = 150.0f / CM_TO_FT;
    bool fresh = false;
    float ft = 0.0f;
    for (int i = 0; i < 8 && !fresh; ++i) {
        push_n(&f, cm150, 58);
        ft = fin(&f, 0.75f, &fresh);
    }
    ASSERT_TRUE(fresh, "precondition: level re-acquired at 150 ft");

    /*  v1.63 once re-seeded its Hampel window with a SINGLE value, leaving the
     *  gate bypassed for two polls right after a snap — a lone corrupted pair
     *  rode ungated into the output exactly when corruption is most likely
     *  still in progress. The adopted candidate carries its own converged
     *  covariance, so the innovation gate is live from the very next sample.  */
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
        for (int k = 0; k < 4; ++k) {
            rf_push_cm(&f, cm200 + (float)(k % 3) - 1.0f);
        }
        (void)fin(&f, 0.05f, NULL);
    }

    /* Push over into a 25 ft/s (1500 fpm) descent the way an aircraft does —
     * the sink builds over ~1 s (~0.8 g), it does not appear instantly — then
     * hold it on the 25 ms DESCENT cadence. Every poll must be accepted fresh
     * and the filter must track within a foot. (v1.63's version stepped the
     * rate instantly on a staircase; see the file header.)                    */
    int   rejected = 0;
    float ft       = 200.0f;
    float true_ft  = 200.0f;
    float rate     = 0.0f;
    float worst    = 0.0f;
    for (int i = 0; i < 400 && true_ft > 5.0f; ++i) {
        float from = true_ft;
        rate = (rate < 25.0f) ? rate + 25.0f * 0.025f : 25.0f;
        true_ft -= rate * 0.025f;
        push_ramp(&f, ft_cm(from), ft_cm(true_ft), 2);
        bool fresh;
        ft = fin(&f, 0.025f, &fresh);
        if (!fresh) {
            ++rejected;
        }
        if (fabsf(ft - true_ft) > worst) {
            worst = fabsf(ft - true_ft);
        }
    }
    ASSERT_TRUE(rejected == 0,        "1500 fpm descent: zero polls rejected");
    ASSERT_TRUE(worst < 1.0f,         "1500 fpm descent: tracked within 1 ft throughout");
    ASSERT_NEAR(rf_rate_fps(&f), -25.0f, 1.0f, "1500 fpm descent: rate ~ -25 ft/s");
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

    /*  Climb 3 ft -> 340 ft at a realistic Glasair rate (1500 fpm), as the
     *  continuous stream a real sensor sees. Feeding real returns the whole
     *  way arms the ladder exactly as a genuine climb-out does.              */
    for (float ft = 0.0f; ft < 340.0f; ) {
        float from = ft;
        ft += 25.0f * dt;
        FLY_POLL(push_ramp(&f, ft_cm(from + GROUND_REF), ft_cm(ft + GROUND_REF), 12));
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
    /*  1200 fpm, continuous. Above the reach the sensor still says nothing.  */
    for (float ft = 360.0f; ft > 0.0f; ) {
        float from = ft;
        ft -= 20.0f * dt;
        if (ft < 0.0f) ft = 0.0f;
        if (from + GROUND_REF > p->max_range_ft) {
            FLY_POLL(push_n(&f, CM_SENTINEL, 12));
        } else {
            FLY_POLL(push_ramp(&f, ft_cm(from + GROUND_REF), ft_cm(ft + GROUND_REF), 12));
        }
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

/*  Shared descent driver. Flies AGL from @p from_ft down to @p to_ft at a
 *  steady 25 ft/s (1500 fpm), calling @p shape_fn to build each poll's drain
 *  from the continuous true range (it ramps cm_from -> cm_to across the poll),
 *  and records which callout heights spoke. Returns the count; @p out_spoken
 *  receives the list. (v1.63 stepped 2 ft per POLL, so the "descent rate"
 *  changed with the cadence; see the file header.)                            */
typedef void (*drain_shape_fn)(range_filter_t *f, float cm_from, float cm_to,
                               int poll_idx, unsigned *seed);

/*  The true (uncorrupted) value of sample @p j of @p n in a ramped poll. */
static float ramp_at(float cm_from, float cm_to, int j, int n)
{
    return cm_from + (cm_to - cm_from) * (float)(j + 1) / (float)n;
}

#define DESCENT_FPS 25.0f

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
    for (float ft = from_ft; ft > to_ft; ) {
        float prev = ft;
        ft -= DESCENT_FPS * dt;
        if (ft < to_ft) ft = to_ft;
        shape(&f, (prev + ground_ref) / CM_TO_FT, (ft + ground_ref) / CM_TO_FT,
              poll_idx++, &seed);

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
        shape(&f, ground_ref / CM_TO_FT, ground_ref / CM_TO_FT, poll_idx++, &seed);

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
static void shape_bitflip_minority(range_filter_t *f, float cm_from, float cm_to,
                                   int poll_idx, unsigned *seed)
{
    (void)poll_idx;
    for (int j = 0; j < 12; ++j) {
        float true_cm = ramp_at(cm_from, cm_to, j, 12);
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
static void shape_intermittent_lost(range_filter_t *f, float cm_from, float cm_to,
                                    int poll_idx, unsigned *seed)
{
    (void)seed;
    /*  Every third poll the sensor sees nothing at all. */
    if (poll_idx % 3 == 2) {
        push_n(f, CM_SENTINEL, 12);
    } else {
        push_ramp(f, cm_from, cm_to, 12);
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
static void shape_stuck_burst(range_filter_t *f, float cm_from, float cm_to,
                              int poll_idx, unsigned *seed)
{
    (void)seed;
    /*  Two separate bursts during the descent, each a few polls long. */
    bool in_burst = (poll_idx >= 30 && poll_idx < 34) ||
                    (poll_idx >= 70 && poll_idx < 73);
    if (in_burst) {
        push_n(f, 2400.0f, 12);      /* a stuck ~79 ft pattern */
    } else {
        push_ramp(f, cm_from, cm_to, 12);
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
static void shape_flipped_sentinels(range_filter_t *f, float cm_from, float cm_to,
                                    int poll_idx, unsigned *seed)
{
    (void)poll_idx;
    for (int j = 0; j < 12; ++j) {
        *seed = *seed * 1103515245u + 12345u;
        if (((*seed >> 16) & 0x7fff) % 9 == 0) {
            rf_push_cm(f, CM_SENT_FLIP);
        } else {
            rf_push_cm(f, ramp_at(cm_from, cm_to, j, 12));
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
static void shape_framing_aborts(range_filter_t *f, float cm_from, float cm_to,
                                 int poll_idx, unsigned *seed)
{
    (void)seed;
    push_ramp(f, cm_from, cm_to, 12);
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
static void shape_kitchen_sink(range_filter_t *f, float cm_from, float cm_to,
                               int poll_idx, unsigned *seed)
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
        float true_cm = ramp_at(cm_from, cm_to, j, 12);
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
            if (reanchor_like_logic_task(&sm, &f, agl_, p) >= 0) {
                ++fires;
            }
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
        push_ramp(&f, (ft + 1.0f + G) / CM_TO_FT, (ft + G) / CM_TO_FT, 12);
        bool  fresh_;
        float ft_  = fin(&f, dt, &fresh_);
        float agl_ = ft_ - G;
        if (agl_ < 0.0f) agl_ = 0.0f;
        if (fresh_ && rf_track_broken(&f)) {
            int late = reanchor_like_logic_task(&sm, &f, agl_, p);
            if (late >= 0 && fabsf(p->callouts[late] - 100.0f) < 0.5f) {
                ++spoke_100;
            }
        }
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
            if (fresh_ && rf_track_broken(&f) &&                              \
                reanchor_like_logic_task(&sm, &f, agl_, p) >= 0) {            \
                ++fires;                                                      \
            }                                                                 \
            sm_out_t out_;                                                    \
            sm_step(&sm, agl_, dt, p, &out_);                                 \
            if (out_.fired_callout >= 0) ++fires;                             \
            dt = (float)poll_profile_to_ms(out_.poll) / 1000.0f;              \
        } while (0)

    /* Parked, then a genuine climb-out (1500 fpm, continuous) that arms. */
    for (int i = 0; i < 10; ++i) ERR_POLL(push_ground_drain(&f, 58));
    for (float ft = 0.0f; ft < 340.0f; ) {
        float from = ft;
        ft += 25.0f * dt;
        ERR_POLL(push_ramp(&f, ft_cm(from + G), ft_cm(ft + G), 12));
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
    for (float ft = 325.0f; ft > 0.0f; ) {
        float from = ft;
        ft -= 20.0f * dt;
        if (ft < 0.0f) ft = 0.0f;
        ERR_POLL(push_ramp(&f, ft_cm(from + G), ft_cm(ft + G), 12));
    }
    for (int i = 0; i < 60; ++i) {
        ERR_POLL(push_n(&f, G / CM_TO_FT, 12));
    }

    #undef ERR_POLL

    ASSERT_TRUE(fires - fires_at_descent >= 6,
                "erroneous out-of-range: real descent still walks the ladder");
}

/* ===========================================================================
 *  The TRACKING verdict — the box's real power/latency signal.
 * ---------------------------------------------------------------------------
 *  Whether the box may relax must be a statement about the SENSOR ("can we see
 *  the ground?"), not an inference from ALTITUDE ("the number is big, so the
 *  sensor probably can't see"). The inference was wrong both ways: it relaxed
 *  while the sensor still tracked fine below its ceiling, and said nothing about
 *  a sensor blind at low altitude. It also assumed a clean cutoff that the
 *  hardware does not have — 328 ft is a best-case rating, so returns degrade
 *  into a ragged mix of good and erroneous values well before it.
 * ========================================================================= */

/*  Build a drain with a given mix of lost-signal and usable returns. */
static void mixed_drain(range_filter_t *f, int n_lost, int n_good, float good_cm)
{
    for (int i = 0; i < n_lost; ++i) {
        rf_push_cm(f, CM_SENTINEL);
    }
    for (int i = 0; i < n_good; ++i) {
        rf_push_cm(f, good_cm);
    }
}

static void test_tracking_verdict(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);

    ASSERT_TRUE(rf_tracking(&f),
                "tracking: a box that has never locked is NOT reported dark");

    /* Establish a normal lock. */
    for (int i = 0; i < 8; ++i) {
        mixed_drain(&f, 0, 20, 3000.0f);
        (void)fin(&f, 0.05f, NULL);
    }
    ASSERT_TRUE(rf_tracking(&f), "tracking: a clean lock is tracking");

    /*  The headline case: a ragged near-ceiling stream that is overwhelmingly
     *  junk but carries the occasional real return. The sensor CAN still see —
     *  the box must stay awake and fast through this, not decide it is idle.  */
    bool stayed = true;
    for (int i = 0; i < 20; ++i) {
        mixed_drain(&f, 19, 1, (float)(9000 + (i % 7) * 300));
        (void)fin(&f, 0.05f, NULL);
        if (!rf_tracking(&f)) {
            stayed = false;
        }
    }
    ASSERT_TRUE(stayed,
                "tracking: 95% junk + 5% good still counts as TRACKING");

    /*  Now genuinely out of range: nothing usable at all. Going dark must take
     *  SUSTAINED evidence — never a single bad drain — but must not take so long
     *  that a real climb-out keeps the box at full rate indefinitely.
     *
     *  The exact count is deliberately not asserted: the ragged stream above may
     *  legitimately leave the no-track counter part-way advanced (its final
     *  drain can carry no usable return), and pinning the sequencing would make
     *  this a test of the fixture rather than of the behaviour. What matters is
     *  the bound — more than one drain, and inside RANGE_NOTRACK_POLLS.        */
    int polls_to_dark = 0;
    for (int i = 0; i < 30; ++i) {
        mixed_drain(&f, 20, 0, 0.0f);
        (void)fin(&f, 0.05f, NULL);
        ++polls_to_dark;
        if (!rf_tracking(&f)) {
            break;
        }
    }
    ASSERT_TRUE(!rf_tracking(&f),
                "tracking: sustained all-lost drains DO go dark");
    ASSERT_TRUE(polls_to_dark > 1,
                "tracking: one bad drain alone never goes dark");
    ASSERT_TRUE(polls_to_dark <= (int)RANGE_NOTRACK_POLLS,
                "tracking: goes dark within RANGE_NOTRACK_POLLS drains");

    /*  And the asymmetry that carries the safety argument: ONE usable return
     *  restores tracking immediately. Being slow to notice the sensor can see
     *  again would cost callouts on an approach.                              */
    mixed_drain(&f, 19, 1, 3000.0f);
    (void)fin(&f, 0.05f, NULL);
    ASSERT_TRUE(rf_tracking(&f),
                "tracking: ONE usable return wakes the box instantly");
}

/*  The exact go-dark count, on a clean fixture where nothing has part-way
 *  advanced the counter. Kept separate from the ragged-stream test above so the
 *  precise threshold is pinned without depending on that fixture's sequencing. */
static void test_tracking_dark_threshold_exact(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);

    for (int i = 0; i < 8; ++i) {
        mixed_drain(&f, 0, 20, 3000.0f);
        (void)fin(&f, 0.05f, NULL);
    }
    ASSERT_TRUE(rf_tracking(&f), "tracking (exact): clean lock is tracking");

    /*  Every drain up to the threshold must still read as tracking. */
    bool held = true;
    for (uint32_t i = 1; i < RANGE_NOTRACK_POLLS; ++i) {
        mixed_drain(&f, 20, 0, 0.0f);
        (void)fin(&f, 0.05f, NULL);
        if (!rf_tracking(&f)) {
            held = false;
        }
    }
    ASSERT_TRUE(held,
                "tracking (exact): still tracking right up to the threshold");

    /*  ...and the threshold drain itself flips it. */
    mixed_drain(&f, 20, 0, 0.0f);
    (void)fin(&f, 0.05f, NULL);
    ASSERT_TRUE(!rf_tracking(&f),
                "tracking (exact): the RANGE_NOTRACK_POLLS'th drain goes dark");
}

/*  A rejected value still proves the sensor is SEEING something. Conflating
 *  "the gate rejected it" with "the sensor is blind" would relax the box during
 *  exactly the noisy, ambiguous stretch where it most needs to be watching.   */
static void test_tracking_counts_rejected_returns(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 8);

    /*  Whole drains of plausible-band garbage: the Hampel gate rejects every
     *  one of them (proven by test_hampel_rejects_garbage_drain), but they are
     *  real returns and must keep the box awake.                              */
    bool tracked = true;
    for (int i = 0; i < 15; ++i) {
        push_n(&f, CM_SPIKE, 40);
        bool fresh_;
        (void)fin(&f, 0.05f, &fresh_);
        if (!rf_tracking(&f)) {
            tracked = false;
        }
    }
    ASSERT_TRUE(tracked,
                "tracking: gate-REJECTED returns still count as tracking");
}

/* ---------------------------------------------------------------------------
 *  A candidate track needs CONSECUTIVE members.
 *
 *  While blind, a junk stream can produce the occasional coherent-looking
 *  run with no-return stretches in between. If a candidate survived those
 *  gaps, scattered junk could collect "members" for as long as it liked and
 *  re-acquire a phantom level (a phantom "100" at 450 ft, found in the v1.63
 *  sweep). Here short bursts — each too little evidence on its own — lie on a
 *  PERFECT straight descending line (the worst case: they agree with the
 *  candidate's prediction across every gap), separated by no-return gaps
 *  longer than RF_CANDIDATE_GAP_S. They must never be believed; the same line
 *  flown as a CONTIGUOUS run must be, as a descending re-entry.
 *  (v1.63 expressed this in whole drains; the per-sample rule is the gap.)
 * ------------------------------------------------------------------------- */
static void test_track_needs_consecutive_members(void)
{
    printf("\n-- re-acquire track needs consecutive members --\n");
    range_filter_t f;
    rf_init(&f, 328.0f);
    const float hz   = 78.0f;
    const float step = 1.0f / hz;                /* one sample period         */
    const float sink = 20.0f;                    /* the line: 20 ft/s down    */

    /* Established at 100 ft of range, then blind long enough to lose track. */
    for (int i = 0; i < 6; ++i) {
        push_n(&f, 100.0f / CM_TO_FT, 30);
        (void)fin(&f, 0.5f, NULL);
    }
    for (int i = 0; i < 10; ++i) {
        push_n(&f, (float)SF30_LOST_SIGNAL_CM, 30);
        (void)fin(&f, 0.5f, NULL);
    }
    ASSERT_TRUE(rf_track_state(&f) == RF_LOST, "setup: track lost after blind drains");

    /*  Bursts of 5 samples ON the line (65 ms: short of the evidence), each
     *  followed by 16 no-return samples (0.2 s > RF_CANDIDATE_GAP_S). One
     *  poll per sample so nothing depends on drain grouping.              */
    bool  believed = false;
    float t        = 0.0f;
    for (int k = 0; k < 40; ++k) {
        for (int j = 0; j < 5; ++j) {
            bool fresh = false;
            rf_push_cm(&f, (300.0f - sink * t) / CM_TO_FT);
            (void)fin(&f, step, &fresh);
            believed |= fresh;
            t += step;
        }
        for (int g = 0; g < 16; ++g) {
            rf_push_cm(&f, (float)SF30_LOST_SIGNAL_CM);
            (void)fin(&f, step, NULL);
            t += step;
        }
    }
    ASSERT_TRUE(!believed, "gapped bursts on a straight line never re-acquire");

    /* The same line as a CONTIGUOUS run is a real (descending) re-entry. */
    believed = false;
    for (int k = 0; k < 40 && !believed; ++k) {
        bool fresh = false;
        rf_push_cm(&f, (300.0f - sink * t) / CM_TO_FT);
        (void)fin(&f, step, &fresh);
        believed |= fresh;
        t += step;
    }
    ASSERT_TRUE(believed, "a contiguous run of the same line re-acquires");
    ASSERT_TRUE(rf_track_broken(&f) && rf_break_reentry(&f),
                "...as a descending RE-ENTRY after blindness");
}

/* ===========================================================================
 *  v1.64 TRACKER BEHAVIOUR — per-sample time, track states, re-entry.
 * ---------------------------------------------------------------------------
 *  A small continuous-time rig: the true range is a function of time, the
 *  sensor samples it at SENSOR_HZ (1 cm quantised, like the wire), and each
 *  poll's samples are spread across the interval exactly as the firmware's
 *  drain is. The poll interval is whatever the caller (or the state machine)
 *  asks for, so the same physical flight can be flown at any cadence.
 * ========================================================================= */

#define SENSOR_HZ_T  78.0f

/*  A trajectory: the true range (ft) the laser sees at time @p t, or a
 *  negative value for "no return"; @p true_agl receives the aircraft's real
 *  height for scoring. @p ctx is the scenario's parameters.                   */
typedef float (*traj_fn)(float t, float *true_agl, const void *ctx);

typedef struct {
    range_filter_t f;
    sm_ctx_t       sm;
    const sensor_profile_t *p;
    float  t;                   /* time of the last poll (s)                   */
    float  dt;                  /* next poll interval (s)                      */
    float  G;                   /* ground reference (ft of range at 0 AGL)     */
    bool   fixed_dt;            /* true: never let the state machine pick dt   */
    float  spoken[SM_MAX_CALLOUTS * 2];
    float  spoken_true[SM_MAX_CALLOUTS * 2];
    int    n_spoken;
    int    n_breaks;
    int    n_reentries;
    float  last_out;            /* last published range                       */
    bool   last_fresh;
} trig_t;

static void trig_init(trig_t *r, sm_state_t st, float dt, bool fixed_dt)
{
    memset(r, 0, sizeof *r);
    r->p        = &SF30C_PROFILE;
    r->G        = 3.0f;
    r->dt       = dt;
    r->fixed_dt = fixed_dt;
    rf_init(&r->f, r->p->max_range_ft);
    sm_init(&r->sm, st);
}

/*  Record a spoken rung (from either path), scored where it is HEARD. */
static void trig_spoke(trig_t *r, int idx, float true_agl, float true_rate)
{
    if (idx < 0 || r->n_spoken >= (int)(sizeof r->spoken / sizeof r->spoken[0])) {
        return;
    }
    r->spoken[r->n_spoken]      = r->p->callouts[idx];
    r->spoken_true[r->n_spoken] = true_agl + true_rate * 0.20f;
    r->n_spoken++;
}

/*  One poll of the whole chain, exactly as the logic task runs it. */
static void trig_poll(trig_t *r, traj_fn traj, const void *ctx)
{
    int n = (int)(SENSOR_HZ_T * r->dt + 0.5f);
    if (n < 1) n = 1;
    if (n > RANGE_DRAIN_MEDIAN_N) n = RANGE_DRAIN_MEDIAN_N;

    float agl_true = 0.0f;
    for (int j = 0; j < n; ++j) {
        float ts = r->t + r->dt * (float)(j + 1) / (float)n;
        float rng = traj(ts, &agl_true, ctx);
        if (rng < 0.0f) {
            rf_push_cm(&r->f, CM_SENTINEL);
        } else {
            rf_push_cm(&r->f, (float)(int)(rng / CM_TO_FT + 0.5f));
        }
    }
    float prev_agl_true = 0.0f;
    (void)traj(r->t, &prev_agl_true, ctx);
    float true_rate = (agl_true - prev_agl_true) / r->dt;

    bool  fresh = false;
    float out   = 0.0f;
    (void)rf_finalize(&r->f, r->dt, &out, &fresh);
    r->last_out   = out;
    r->last_fresh = fresh;

    float agl = out - r->G;
    if (agl < 0.0f) agl = 0.0f;

    r->sm.lead_rate_fps = rf_rate_fps(&r->f);
    if (fresh && rf_track_broken(&r->f)) {
        r->n_breaks++;
        if (rf_break_reentry(&r->f)) r->n_reentries++;
        trig_spoke(r, reanchor_like_logic_task(&r->sm, &r->f, agl, r->p),
                   agl_true, true_rate);
    }
    sm_out_t o;
    sm_step(&r->sm, agl, r->dt, r->p, &o);
    trig_spoke(r, o.fired_callout, agl_true, true_rate);

    r->t += r->dt;
    if (!r->fixed_dt) {
        r->dt = (float)sm_poll_period_ms(o.poll, r->sm.armed, rf_tracking(&r->f),
                                         rf_reacquiring(&r->f), agl,
                                         r->sm.tone_start_ft) / 1000.0f;
    }
}

/*  Did the rig speak @p ft? Returns the count; @p heard gets the last one's
 *  true (heard) altitude.                                                    */
static int trig_count(const trig_t *r, float ft, float *heard)
{
    int n = 0;
    for (int i = 0; i < r->n_spoken; ++i) {
        if (fabsf(r->spoken[i] - ft) < 0.5f) {
            ++n;
            if (heard) *heard = r->spoken_true[i];
        }
    }
    return n;
}

/*  Deterministic surface texture: a few tenths of a foot, a function of time
 *  (so two cadences sampling different instants see different noise).        */
static float texture(float t)
{
    return 0.15f * sinf(t * 97.0f) + 0.10f * sinf(t * 263.0f + 1.3f);
}

/* ---------------------------------------------------------------------------
 *  CADENCE INVARIANCE — the property the rewrite exists for.
 *
 *  The same physical approach (level at 250 ft, a 1 s push-over into a
 *  3000 fpm descent, round-out) flown at every poll cadence the firmware uses.
 *  Every cadence must track it within a foot, never lose or break track, and
 *  agree with every other cadence at common instants. v1.63's poll-count rules
 *  (Hampel horizon, re-acquire polls, 12-poll break) could not pass this.
 * ------------------------------------------------------------------------- */
static float traj_approach(float t, float *agl, const void *ctx)
{
    (void)ctx;
    float h;
    if (t < 3.0f) {
        h = 250.0f;                                    /* level              */
    } else if (t < 4.0f) {
        float u = t - 3.0f;                            /* push-over, 50 ft/s^2 */
        h = 250.0f - 25.0f * u * u;
    } else {
        h = 225.0f - 50.0f * (t - 4.0f);               /* 3000 fpm           */
    }
    if (h < 20.0f) {
        h = 20.0f;
    }
    *agl = h;
    return h + 3.0f + texture(t);
}

static void test_cadence_invariance(void)
{
    const float cadences[] = { 0.025f, 0.05f, 0.10f, 0.50f, 0.75f };
    const int   nc = (int)(sizeof cadences / sizeof cadences[0]);
    float at_common[5][8];
    char  msg[128];

    for (int c = 0; c < nc; ++c) {
        trig_t r;
        trig_init(&r, ST_ARMED, cadences[c], true);
        float worst = 0.0f;
        bool  lost  = false;
        int   k     = 0;
        while (r.t < 7.4f) {
            trig_poll(&r, traj_approach, NULL);
            float agl_now;
            float rng_now = traj_approach(r.t, &agl_now, NULL) - texture(r.t);
            if (r.t > 1.0f) {
                float e = fabsf(r.last_out - rng_now);
                if (e > worst) worst = e;
                if (rf_track_state(&r.f) == RF_LOST) lost = true;
            }
            /* Common instants: every 1.5 s (a multiple of every cadence). */
            float m = fmodf(r.t + 1e-4f, 1.5f);
            if (m < 2e-4f && k < 8) {
                at_common[c][k++] = r.last_out;
            }
        }
        snprintf(msg, sizeof msg, "cadence %3.0f ms: tracked within 1 ft (worst %.2f)",
                 (double)(cadences[c] * 1000.0f), (double)worst);
        ASSERT_TRUE(worst < 1.0f, msg);
        snprintf(msg, sizeof msg, "cadence %3.0f ms: never LOST, never broke",
                 (double)(cadences[c] * 1000.0f));
        ASSERT_TRUE(!lost && r.n_breaks == 0, msg);
        snprintf(msg, sizeof msg, "cadence %3.0f ms: rate ~ -50 ft/s (%.1f)",
                 (double)(cadences[c] * 1000.0f), (double)rf_rate_fps(&r.f));
        ASSERT_TRUE(fabsf(rf_rate_fps(&r.f) + 50.0f) < 2.0f, msg);
    }

    /* Every cadence agrees with the fastest at the common instants. */
    float spread = 0.0f;
    for (int c = 1; c < nc; ++c) {
        for (int k = 1; k < 4; ++k) {
            float d = fabsf(at_common[c][k] - at_common[0][k]);
            if (d > spread) spread = d;
        }
    }
    snprintf(msg, sizeof msg, "all cadences agree within 0.6 ft (spread %.2f)",
             (double)spread);
    ASSERT_TRUE(spread < 0.6f, msg);
}

/* ---------------------------------------------------------------------------
 *  COAST: a short dark patch at 4000 fpm keeps the output MOVING.
 *
 *  Freezing through a 0.25 s dropout would be ~17 ft of callout lateness at a
 *  gear-down Glasair sink rate. The track coasts on its own measured rate and
 *  resumes as the SAME track (no break) when returns come back.
 * ------------------------------------------------------------------------- */
typedef struct { float gap_from, gap_to; float rate; float top; } gap_ctx_t;

static float traj_gap(float t, float *agl, const void *ctx)
{
    const gap_ctx_t *g = (const gap_ctx_t *)ctx;
    float h = g->top - g->rate * t;
    if (h < 5.0f) h = 5.0f;
    *agl = h;
    if (t > g->gap_from && t <= g->gap_to) {
        return -1.0f;                                  /* dark patch         */
    }
    return h + 3.0f + texture(t);
}

static void test_coast_bridges_short_gap(void)
{
    gap_ctx_t g = { 1.00f, 1.25f, 66.7f, 250.0f };
    trig_t r;
    trig_init(&r, ST_ARMED, 0.025f, true);
    bool  coasted = false;
    float worst_in_gap = 0.0f;
    while (r.t < 1.6f) {
        trig_poll(&r, traj_gap, &g);
        float agl;
        (void)traj_gap(r.t, &agl, &g);
        if (r.t > g.gap_from + 0.01f && r.t <= g.gap_to) {
            coasted |= (rf_track_state(&r.f) == RF_COAST);
            float e = fabsf(r.last_out - (agl + 3.0f));
            if (e > worst_in_gap) worst_in_gap = e;
        }
    }
    ASSERT_TRUE(coasted, "0.25 s dark patch: the track COASTs");
    char msg[96];
    snprintf(msg, sizeof msg, "4000 fpm coast: output keeps moving, within 1 ft (%.2f)",
             (double)worst_in_gap);
    ASSERT_TRUE(worst_in_gap < 1.0f, msg);
    ASSERT_TRUE(rf_track_state(&r.f) == RF_TRACK && r.n_breaks == 0,
                "after the patch: the SAME track resumes (no break)");
}

/*  A long dark patch: LOST after RF_COAST_MAX_S, then the output HOLDS. */
static void test_long_gap_goes_lost_and_holds(void)
{
    gap_ctx_t g = { 1.00f, 2.00f, 0.0f, 150.0f };      /* level, 1 s dark    */
    trig_t r;
    trig_init(&r, ST_ARMED, 0.025f, true);
    float held      = -1.0f;
    bool  moved     = false;
    bool  fresh_any = false;
    float t_lost    = -1.0f;
    while (r.t < 1.95f) {
        trig_poll(&r, traj_gap, &g);
        if (r.t > g.gap_from) {
            fresh_any |= r.last_fresh;
            if (rf_track_state(&r.f) == RF_LOST) {
                if (t_lost < 0.0f) {
                    t_lost = r.t;
                    held   = r.last_out;
                } else if (fabsf(r.last_out - held) > 1e-4f) {
                    moved = true;
                }
            }
        }
    }
    ASSERT_TRUE(t_lost > 0.0f, "1 s dark patch: the track goes LOST");
    ASSERT_TRUE(t_lost - g.gap_from <= RF_COAST_MAX_S + 0.03f,
                "...right after RF_COAST_MAX_S");
    ASSERT_TRUE(!moved && fabsf(held - 153.0f) < 1.0f,
                "LOST: the output HOLDS the last value, invents nothing");
    ASSERT_TRUE(!fresh_any, "LOST: never reported fresh");
    ASSERT_TRUE(rf_rate_fps(&r.f) == 0.0f, "LOST: carries no rate");
}

/* ---------------------------------------------------------------------------
 *  Covariance and outputs survive hostile inputs.
 * ------------------------------------------------------------------------- */
static bool kf_healthy(const rf_kf_t *k)
{
    return isfinite(k->x) && isfinite(k->v) && isfinite(k->p00) &&
           isfinite(k->p01) && isfinite(k->p11) && isfinite(k->r_var) &&
           k->p00 > 0.0f && k->p11 > 0.0f && k->r_var > 0.0f &&
           k->p01 * k->p01 <= k->p00 * k->p11 * 1.0001f;
}

static void test_covariance_safety(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    settle_on_ground(&f, 4);

    const float cms[] = { NAN, -5.0f, 1.0e9f, 16383.0f, INFINITY, -INFINITY,
                          15.0f, 9000.0f, 91.0f, 0.0f, 10058.0f, 91.0f };
    const float dts[] = { 1.0e-9f, -1.0f, NAN, 1.0e9f, 0.025f, 0.75f,
                          INFINITY, 0.0f, 5.0f };
    bool healthy = true, finite = true;
    unsigned s = 7u;
    for (int i = 0; i < 4000; ++i) {
        s = s * 1103515245u + 12345u;
        int npush = (int)((s >> 16) % 70u);
        for (int j = 0; j < npush; ++j) {
            s = s * 1103515245u + 12345u;
            rf_push_cm(&f, cms[(s >> 16) % (sizeof cms / sizeof cms[0])]);
        }
        s = s * 1103515245u + 12345u;
        if (((s >> 16) & 15u) == 0u) {
            rf_drain_abort(&f);
        }
        float ft; bool fr;
        (void)rf_finalize(&f, dts[(s >> 20) % (sizeof dts / sizeof dts[0])], &ft, &fr);
        finite  &= isfinite(ft) && isfinite(rf_rate_fps(&f));
        healthy &= kf_healthy(&f.trk);
        if (f.c_n > 0) {
            healthy &= kf_healthy(&f.cand);
        }
    }
    ASSERT_TRUE(finite,  "hostile inputs: outputs always finite");
    ASSERT_TRUE(healthy, "hostile inputs: covariances stay finite + positive-definite");

    /* NULL everything: no crash, sane returns. */
    float ft = 1.0f; bool fr = true;
    ASSERT_TRUE(!rf_finalize(NULL, 0.1f, &ft, &fr) && !fr, "NULL filter: finalize is inert");
    rf_push_cm(NULL, 91.0f);
    rf_drain_abort(NULL);
    ASSERT_TRUE(!rf_track_broken(NULL) && !rf_break_reentry(NULL) &&
                rf_rate_fps(NULL) == 0.0f && rf_track_state(NULL) == RF_SEARCH &&
                !rf_reacquiring(NULL) && !rf_tracking(NULL),
                "NULL filter: every query is inert");
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    ASSERT_TRUE(!rf_finalize(&f, 0.1f, NULL, NULL), "NULL outputs: finalize is safe");
}

/* ---------------------------------------------------------------------------
 *  The innovation gate against a junk-laden stream while tracking a descent.
 * ------------------------------------------------------------------------- */
static void test_gate_vs_interleaved_junk(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    unsigned s = 99u;
    float h = 250.0f, worst = 0.0f;
    bool lost = false, broke = false;
    /*  20 s at 10 ft/s (600 fpm): 250 -> 50 ft, 50 ms polls of 4 samples. */
    for (int i = 0; i < 400; ++i) {
        for (int j = 0; j < 4; ++j) {
            float hj = h - 10.0f * 0.05f * (float)(j + 1) / 4.0f;
            s = s * 1103515245u + 12345u;
            if (((s >> 16) & 3u) == 0u) {                 /* 25% junk          */
                s = s * 1103515245u + 12345u;
                rf_push_cm(&f, (float)((s >> 16) % 10000u));
            } else {
                rf_push_cm(&f, ft_cm(hj + texture((float)i + (float)j * 0.25f)));
            }
        }
        h -= 10.0f * 0.05f;
        float out; bool fr;
        (void)rf_finalize(&f, 0.05f, &out, &fr);
        if (i > 20) {
            if (fabsf(out - h) > worst) worst = fabsf(out - h);
            lost  |= rf_track_state(&f) == RF_LOST;
            broke |= rf_track_broken(&f);
        }
    }
    char msg[96];
    snprintf(msg, sizeof msg, "25%% junk while descending: within 1 ft (worst %.2f)",
             (double)worst);
    ASSERT_TRUE(worst < 1.0f, msg);
    ASSERT_TRUE(!lost && !broke, "25% junk: never lost, never broke");
}

/* ---------------------------------------------------------------------------
 *  Sample-and-hold output (SF30/C #U faster than #R): 4x repeated readings at
 *  a 4000 fpm approach must still be tracked — each new value at its true
 *  arrival time — with no loss of track and no break.
 * ------------------------------------------------------------------------- */
static void test_sample_and_hold_repeats(void)
{
    range_filter_t f;
    rf_init(&f, SF30C_PROFILE.max_range_ft);
    const float step = 1.0f / 78.0f;
    float t = 0.0f, worst = 0.0f, held_cm = 0.0f;
    bool lost = false, broke = false;
    int  k = 0;
    for (int poll = 0; poll < 160; ++poll) {             /* 25 ms polls, 4 s  */
        for (int j = 0; j < 2; ++j) {
            t += step;
            if (k++ % 4 == 0) {                           /* new reading 1-in-4 */
                float h = 300.0f - 66.7f * t;
                held_cm = (float)(int)(ft_cm(h) + 0.5f);
            }
            rf_push_cm(&f, held_cm);
        }
        float out; bool fr;
        (void)rf_finalize(&f, 2.0f * step, &out, &fr);
        if (poll > 20) {
            float e = fabsf(out - (300.0f - 66.7f * t));
            if (e > worst) worst = e;
            lost  |= rf_track_state(&f) == RF_LOST;
            broke |= rf_track_broken(&f);
        }
    }
    char msg[96];
    snprintf(msg, sizeof msg, "4x sample-and-hold at 4000 fpm: within 2.5 ft (worst %.2f)",
             (double)worst);
    ASSERT_TRUE(worst < 2.5f, msg);
    ASSERT_TRUE(!lost && !broke, "4x sample-and-hold: never lost, never broke");
}

/* ---------------------------------------------------------------------------
 *  Re-entry from LOST: DESCENDING confirms fast and is a re-entry; a
 *  STATIONARY level far from the hold needs RF_BREAK_S and is not.
 * ------------------------------------------------------------------------- */
typedef struct { float reach; float climb_to; float level_s; float sink; int stuck; } pat_ctx_t;

/*  Climb out at 1500 fpm to climb_to, hold level_s, then descend at sink. Above
 *  the reach: nothing (or, with stuck, a constant 200 ft pattern).             */
static float traj_pattern(float t, float *agl, const void *ctx)
{
    const pat_ctx_t *c = (const pat_ctx_t *)ctx;
    float t_climb = c->climb_to / 25.0f;
    float h;
    if (t < 2.0f) {
        h = 0.0f;
    } else if (t < 2.0f + t_climb) {
        h = 25.0f * (t - 2.0f);
    } else if (t < 2.0f + t_climb + c->level_s) {
        h = c->climb_to;
    } else {
        h = c->climb_to - c->sink * (t - 2.0f - t_climb - c->level_s);
    }
    if (h < 0.0f) h = 0.0f;
    *agl = h;
    if (h + 3.0f > c->reach) {
        return c->stuck ? 200.0f : -1.0f;
    }
    return h + 3.0f + texture(t) * 0.5f;
}

static void test_reentry_descending_vs_stationary(void)
{
    /* (a) Descending back into range after a pin: fast, and a RE-ENTRY. */
    pat_ctx_t c = { 328.0f, 450.0f, 20.0f, 33.3f, 0 };
    trig_t r;
    trig_init(&r, ST_GROUND, 0.75f, false);
    float t_first_return = -1.0f, t_confirm = -1.0f;
    bool  pinned = false, reentry = false;
    while (r.t < 60.0f && t_confirm < 0.0f) {
        trig_poll(&r, traj_pattern, &c);
        float agl;
        float rng = traj_pattern(r.t, &agl, &c);
        pinned |= fabsf(r.last_out - r.p->max_range_ft) < 0.01f &&
                  rf_track_state(&r.f) == RF_LOST;
        if (pinned && rng >= 0.0f && t_first_return < 0.0f && agl < 400.0f) {
            t_first_return = r.t;
        }
        if (t_first_return > 0.0f && r.last_fresh && rf_track_broken(&r.f)) {
            t_confirm = r.t;
            reentry   = rf_break_reentry(&r.f);
        }
    }
    ASSERT_TRUE(pinned, "re-entry: the climb-out pinned the output at the ceiling");
    ASSERT_TRUE(t_confirm > 0.0f && t_confirm - t_first_return < 0.35f,
                "re-entry: a descending return confirms within ~0.3 s");
    ASSERT_TRUE(reentry, "re-entry: flagged as a flyable descending RE-ENTRY");

    /* (b) A stationary stuck pattern at 200 ft while blind: held until
     *     RF_BREAK_S, then adopted silently — never a re-entry.              */
    pat_ctx_t s = { 328.0f, 450.0f, 30.0f, 33.3f, 1 };
    trig_init(&r, ST_GROUND, 0.75f, false);
    float t_stuck = -1.0f, t_adopt = -1.0f;
    bool  stuck_reentry = false;
    int   spoke_before = 0;
    while (r.t < 45.0f) {
        trig_poll(&r, traj_pattern, &s);
        float agl;
        (void)traj_pattern(r.t, &agl, &s);
        if (agl + 3.0f > s.reach && t_stuck < 0.0f) {
            /* The pattern's true onset (the climb crosses the reach), not the
             * first poll after it: at the CRUISE cadence that is 0.5 s later. */
            t_stuck = 2.0f + (s.reach - 3.0f) / 25.0f;
            spoke_before = r.n_spoken;
        }
        if (t_stuck > 0.0f && t_adopt < 0.0f && r.last_fresh && rf_track_broken(&r.f)) {
            t_adopt       = r.t;
            stuck_reentry = rf_break_reentry(&r.f);
        }
    }
    /*  The persistence clock runs from the pattern's first sample, alongside
     *  the coast; allow one 50 ms poll of granularity.                      */
    ASSERT_TRUE(t_adopt > 0.0f && t_adopt - t_stuck >= RF_BREAK_S - 0.05f,
                "stuck pattern: held for RF_BREAK_S before adoption");
    ASSERT_TRUE(!stuck_reentry, "stuck pattern: adopted as a SILENT break, never a re-entry");
    ASSERT_TRUE(r.n_spoken == spoke_before, "stuck pattern: not one word spoken");
}

/* ---------------------------------------------------------------------------
 *  Tree tops on final must never SPEND a rung.
 *
 *  Descending at 1200 fpm through 145..115 ft over a 40 ft canopy, the range
 *  reads 105..75 — "through" the 100 rung while the aircraft is 40 ft above
 *  it. The canopy is a discontinuity of REJECTED RETURNS, not blindness: a
 *  silent break, the 100 rung left armed, and "one hundred" heard once, at
 *  100 ft, after the trees.
 * ------------------------------------------------------------------------- */
static float traj_trees(float t, float *agl, const void *ctx)
{
    (void)ctx;
    float h = (t < 2.0f) ? 180.0f : 180.0f - 20.0f * (t - 2.0f);
    if (h < 0.0f) h = 0.0f;
    *agl = h;
    float canopy = (h <= 145.0f && h > 115.0f) ? 40.0f : 0.0f;
    return h + 3.0f - canopy + texture(t) * 0.5f;
}

static void test_trees_do_not_spend_rungs(void)
{
    trig_t r;
    trig_init(&r, ST_ARMED, 0.05f, false);
    while (r.t < 12.0f) {
        trig_poll(&r, traj_trees, NULL);
    }
    float heard = 0.0f;
    int   n100  = trig_count(&r, 100.0f, &heard);
    char  msg[112];
    ASSERT_TRUE(r.n_breaks >= 1 && r.n_reentries == 0,
                "trees: the canopy is a break, never a re-entry");
    snprintf(msg, sizeof msg, "trees: 100 spoken exactly once (%d)", n100);
    ASSERT_TRUE(n100 == 1, msg);
    snprintf(msg, sizeof msg, "trees: ...heard at 100 ft, not over the canopy (%.1f)",
             (double)heard);
    ASSERT_TRUE(fabsf(heard - 100.0f) < 10.0f, msg);
}

/* ---------------------------------------------------------------------------
 *  The late-rung window, end to end: an approach over a surface the laser
 *  only sees from @p reach. Over grass (250 ft) the 300 rung was passed ~55 ft
 *  before the ground came back — skipped. Seen from 295 ft it is only ~15 ft
 *  late where heard — spoken. Everything below speaks normally either way.
 * ------------------------------------------------------------------------- */
static void fly_reentry_approach(float reach, trig_t *r)
{
    pat_ctx_t c = { reach, 500.0f, 20.0f, 33.3f, 0 };
    trig_init(r, ST_GROUND, 0.75f, false);
    while (r->t < 70.0f) {
        trig_poll(r, traj_pattern, &c);
    }
}

static void test_late_rung_window_end_to_end(void)
{
    char  msg[112];
    float heard = 0.0f;
    trig_t r;

    fly_reentry_approach(250.0f, &r);
    snprintf(msg, sizeof msg, "grass (250 ft reach): 300 skipped (%d)",
             trig_count(&r, 300.0f, NULL));
    ASSERT_TRUE(trig_count(&r, 300.0f, NULL) == 0, msg);
    ASSERT_TRUE(r.n_reentries == 1, "grass: exactly one re-entry");
    const float low[] = { 200.0f, 100.0f, 50.0f, 40.0f, 30.0f, 20.0f, 10.0f };
    for (size_t i = 0; i < sizeof low / sizeof low[0]; ++i) {
        int n = trig_count(&r, low[i], &heard);
        snprintf(msg, sizeof msg, "grass: %.0f spoken once, near %.0f (%.1f)",
                 (double)low[i], (double)low[i], (double)heard);
        ASSERT_TRUE(n == 1 && fabsf(heard - low[i]) < 10.0f, msg);
    }

    fly_reentry_approach(295.0f, &r);
    int n300 = trig_count(&r, 300.0f, &heard);
    snprintf(msg, sizeof msg, "295 ft reach: 300 spoken via the window (%d, heard %.1f)",
             n300, (double)heard);
    ASSERT_TRUE(n300 == 1 && heard > 300.0f - CALLOUT_LATE_TOL_HI_FT - 1.0f &&
                heard < 300.0f, msg);
    snprintf(msg, sizeof msg, "295 ft reach: 200 still spoken once (%d)",
             trig_count(&r, 200.0f, NULL));
    ASSERT_TRUE(trig_count(&r, 200.0f, NULL) == 1, msg);
}

/* ---------------------------------------------------------------------------
 *  A re-entry must be physically FLYABLE to open the late-rung window.
 *
 *  Tracking 189 ft of range while descending, the laser goes blind for 0.4 s
 *  (LOST, and lost to blindness), then a descending track appears at ~95 ft —
 *  94 ft below the last measurement, where the aircraft could have sunk at
 *  most 100 ft/s x 0.5 s + 25 ft = 75 ft. Were it a re-entry, the late-rung
 *  window would call "one hundred" (heard ~12 ft late) with the aircraft
 *  ~180 ft up. It must re-anchor SILENTLY, leaving the rung armed.
 * ------------------------------------------------------------------------- */
static float traj_unreachable(float t, float *agl, const void *ctx)
{
    (void)ctx;
    float h = 230.0f - 20.0f * t;                      /* the aircraft          */
    *agl = h;
    if (t > 2.0f && t <= 2.4f) {
        return -1.0f;                                  /* blind                 */
    }
    if (t > 2.4f) {
        return 95.0f - 20.0f * (t - 2.4f);             /* a far-below "track"   */
    }
    return h + 3.0f + texture(t) * 0.5f;
}

static void test_reentry_must_be_reachable(void)
{
    trig_t r;
    trig_init(&r, ST_ARMED, 0.025f, true);
    bool broke = false, reentry = false;
    while (r.t < 3.0f) {
        trig_poll(&r, traj_unreachable, NULL);
        if (r.last_fresh && rf_track_broken(&r.f)) {
            broke   = true;
            reentry = rf_break_reentry(&r.f);
        }
    }
    int i100 = -1;
    for (size_t i = 0; i < r.p->n_callouts; ++i) {
        if (fabsf(r.p->callouts[i] - 100.0f) < 0.5f) i100 = (int)i;
    }
    ASSERT_TRUE(broke, "unreachable re-entry: the new level is adopted as a break");
    ASSERT_TRUE(!reentry, "unreachable re-entry: ...but NOT as a flyable re-entry");
    ASSERT_TRUE(trig_count(&r, 100.0f, NULL) == 0,
                "unreachable re-entry: \"one hundred\" is NOT spoken at ~180 ft");
    ASSERT_TRUE(i100 >= 0 && (r.sm.armed_mask & (1u << i100)) != 0u,
                "unreachable re-entry: the 100 rung stays armed for the real descent");
}

int main(void)
{
    printf("== range_filter ==\n");
    test_track_needs_consecutive_members();
    test_ascii_decoder();
    test_validity_gates();
    test_minority_garbage();
    test_rejects_garbage_drain();
    test_reacquire_real_step();
    test_reacquire_needs_timed_evidence();
    test_gate_live_after_reacquire();
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
    test_tracking_verdict();
    test_tracking_dark_threshold_exact();
    test_tracking_counts_rejected_returns();
    test_cadence_invariance();
    test_coast_bridges_short_gap();
    test_long_gap_goes_lost_and_holds();
    test_covariance_safety();
    test_gate_vs_interleaved_junk();
    test_sample_and_hold_repeats();
    test_reentry_descending_vs_stationary();
    test_trees_do_not_spend_rungs();
    test_late_rung_window_end_to_end();
    test_reentry_must_be_reachable();

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
