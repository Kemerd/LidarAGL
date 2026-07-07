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

int main(void)
{
    printf("== range_filter ==\n");
    test_ascii_decoder();
    test_validity_gates();
    test_median_of_drain();
    test_hampel_rejects_garbage_drain();
    test_reacquire_real_step();
    test_legit_descent_passes();
    test_taxi_incident_end_to_end(false);
    test_taxi_incident_end_to_end(true);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
