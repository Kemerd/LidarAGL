/**
 * @file    test_state_machine.c
 * @brief   Host unit tests for the behaviour state machine.
 *
 * @details Drives sm_step() with synthetic AGL sequences and asserts the
 *          safety-critical behaviours: silent climb-out, edge-triggered descent
 *          callouts, hysteresis / go-around re-arm, CRUISE gating, and trend
 *          dead-band. Every test runs against BOTH sensor profiles so we prove
 *          the SF30/D fires its high 600/500/400/300 numbers while the SF30/C tops
 *          out at 300 and never fires the longer-range ones.
 */

#include "test_util.h"
#include "state_machine.h"
#include "sensor_profile.h"
#include "config.h"

#include <math.h>
#include <string.h>

TEST_GLOBALS

/* Fixed tick used throughout: 25 ms (the DESCENT poll period). */
#define DT 0.025f

/* Feed one AGL sample and return the fired callout HEIGHT (ft), or -1. */
static float step_height(sm_ctx_t *c, float agl, const sensor_profile_t *p)
{
    sm_out_t out;
    sm_step(c, agl, DT, p, &out);
    if (out.fired_callout >= 0) {
        return p->callouts[out.fired_callout];
    }
    return -1.0f;
}

/* Ramp AGL from 'from' to 'to' in 'step' increments, collecting every fired
 * callout height into 'hits' (capacity 'cap'); returns the number of hits.
 * A small negative step descends; positive climbs. */
static int ramp(sm_ctx_t *c, const sensor_profile_t *p,
                float from, float to, float step,
                float *hits, int cap)
{
    int n = 0;
    float agl = from;
    /* Iterate until we pass 'to' in the direction of 'step'. */
    while ((step < 0 && agl >= to) || (step > 0 && agl <= to)) {
        float h = step_height(c, agl, p);
        if (h >= 0 && n < cap) {
            hits[n++] = h;
        }
        agl += step;
    }
    return n;
}

/* Convenience: does 'hits[0..n)' contain 'v'? */
static bool has_hit(const float *hits, int n, float v)
{
    for (int i = 0; i < n; ++i) {
        if (fabsf(hits[i] - v) < 0.01f) {
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------------- */
/*  Test bodies, parametrised by profile.                                    */
/* ------------------------------------------------------------------------- */

static void test_silent_climbout(const sensor_profile_t *p)
{
    char msg[96];
    sm_ctx_t c;
    sm_init(&c, ST_GROUND);

    /* Climb 0 -> 150 ft. Crossing callout heights on the way UP must be silent
     * (no takeoff callouts) — arming only happens AT ARM_FT and only enables
     * future DOWNWARD crossings.                                              */
    float hits[16];
    int n = ramp(&c, p, 0.0f, 150.0f, +1.0f, hits, 16);

    snprintf(msg, sizeof msg, "[%s] silent climb-out: no callouts going up", p->name);
    ASSERT_TRUE(n == 0, msg);
}

static void test_descent_sequence(const sensor_profile_t *p)
{
    char msg[96];
    sm_ctx_t c;
    sm_init(&c, ST_GROUND);

    /* First climb above the highest callout + margin so everything arms. */
    float top = p->callouts[0] + REARM_MARGIN_FT + 20.0f;
    float scratch[16];
    ramp(&c, p, 0.0f, top, +2.0f, scratch, 16);

    /* Now descend all the way to the ground, collecting fired callouts. */
    float hits[16];
    int n = ramp(&c, p, top, 0.0f, -1.0f, hits, 16);

    /* Every callout in the profile must have fired exactly once, in order. */
    snprintf(msg, sizeof msg, "[%s] descent fires all %d callouts",
             p->name, (int)p->n_callouts);
    ASSERT_TRUE(n == (int)p->n_callouts, msg);

    for (size_t i = 0; i < p->n_callouts; ++i) {
        snprintf(msg, sizeof msg, "[%s] fired %g ft", p->name, p->callouts[i]);
        ASSERT_TRUE(has_hit(hits, n, p->callouts[i]), msg);
    }

    /* Descending order check: hits should be monotonically decreasing. */
    bool ordered = true;
    for (int i = 1; i < n; ++i) {
        if (hits[i] >= hits[i - 1]) {
            ordered = false;
        }
    }
    snprintf(msg, sizeof msg, "[%s] callouts fire high-to-low", p->name);
    ASSERT_TRUE(ordered, msg);
}

static void test_high_callouts_profile_specific(void)
{
    /* SF30/D tops out at 600 ft and must fire 600/500/400/300; SF30/C tops out at
     * 300 ft (its new default top), firing 300 but never the longer-range numbers. */
    float scratch[16], hits[16];

    /* SF30/D: climb above 600 + REARM_MARGIN so the top number arms, then descend. */
    sm_ctx_t cd;
    sm_init(&cd, ST_GROUND);
    ramp(&cd, &SF30D_PROFILE, 0.0f, 640.0f, +4.0f, scratch, 16);
    int nd = ramp(&cd, &SF30D_PROFILE, 640.0f, 0.0f, -2.0f, hits, 16);
    ASSERT_TRUE(has_hit(hits, nd, 600.0f), "[SF30/D] fires 600 ft (top)");
    ASSERT_TRUE(has_hit(hits, nd, 500.0f), "[SF30/D] fires 500 ft");
    ASSERT_TRUE(has_hit(hits, nd, 400.0f), "[SF30/D] fires 400 ft");
    ASSERT_TRUE(has_hit(hits, nd, 300.0f), "[SF30/D] fires 300 ft");

    /* SF30/C: climb above 300 + REARM_MARGIN so the new top number arms, then
     * descend. It must fire 300 (top) and 200, but never 400/500/600 — those are
     * SF30/D-only and not in the SF30/C ladder. */
    sm_ctx_t cc;
    sm_init(&cc, ST_GROUND);
    ramp(&cc, &SF30C_PROFILE, 0.0f, 340.0f, +4.0f, scratch, 16);
    int nc = ramp(&cc, &SF30C_PROFILE, 340.0f, 0.0f, -2.0f, hits, 16);
    ASSERT_TRUE(has_hit(hits, nc, 300.0f),  "[SF30/C] top callout is 300 ft");
    ASSERT_TRUE(has_hit(hits, nc, 200.0f),  "[SF30/C] fires 200 ft");
    ASSERT_TRUE(!has_hit(hits, nc, 400.0f), "[SF30/C] never fires 400 ft");
    ASSERT_TRUE(!has_hit(hits, nc, 600.0f), "[SF30/C] never fires 600 ft");
}

static void test_hysteresis_no_machinegun(const sensor_profile_t *p)
{
    char msg[96];
    sm_ctx_t c;
    sm_init(&c, ST_GROUND);

    /* Arm, then descend to just below the 50 ft callout to fire it once. */
    float scratch[16];
    ramp(&c, p, 0.0f, p->callouts[0] + 40.0f, +2.0f, scratch, 16);
    ramp(&c, p, p->callouts[0] + 40.0f, 49.0f, -2.0f, scratch, 16);

    /* Now hover/jitter around 50 ft (48..52) for many ticks. The 50 ft callout
     * has already fired and we have NOT climbed REARM_MARGIN_FT above it, so it
     * must NOT fire again — no machine-gunning.                                */
    int refires = 0;
    for (int i = 0; i < 40; ++i) {
        float agl = (i & 1) ? 51.5f : 48.5f;
        float h = step_height(&c, agl, p);
        if (fabsf(h - 50.0f) < 0.01f) {
            ++refires;
        }
    }
    snprintf(msg, sizeof msg, "[%s] no machine-gun hovering at 50 ft", p->name);
    ASSERT_TRUE(refires == 0, msg);
}

static void test_goaround_rearm(const sensor_profile_t *p)
{
    char msg[96];
    sm_ctx_t c;
    sm_init(&c, ST_GROUND);

    /* Arm and descend through 50 ft (fires once). */
    float scratch[16];
    ramp(&c, p, 0.0f, p->callouts[0] + 40.0f, +2.0f, scratch, 16);
    ramp(&c, p, p->callouts[0] + 40.0f, 45.0f, -2.0f, scratch, 16);

    /* Go around: climb well above 50 + REARM_MARGIN, then descend again. */
    float hits[16];
    ramp(&c, p, 45.0f, 50.0f + REARM_MARGIN_FT + 15.0f, +2.0f, scratch, 16);
    int n = ramp(&c, p, 50.0f + REARM_MARGIN_FT + 15.0f, 5.0f, -2.0f, hits, 16);

    snprintf(msg, sizeof msg, "[%s] go-around re-fires 50 ft", p->name);
    ASSERT_TRUE(has_hit(hits, n, 50.0f), msg);
}

static void test_cruise_gating(const sensor_profile_t *p)
{
    char msg[96];
    sm_ctx_t c;
    sm_init(&c, ST_GROUND);

    /* Climb to just above cruise_ft; state must be CRUISE. */
    sm_out_t out;
    float scratch[16];
    ramp(&c, p, 0.0f, p->cruise_ft + 10.0f, +4.0f, scratch, 16);
    sm_step(&c, p->cruise_ft + 5.0f, DT, p, &out);
    snprintf(msg, sizeof msg, "[%s] >= cruise_ft -> CRUISE", p->name);
    ASSERT_TRUE(out.state == ST_CRUISE, msg);

    /* Descend just below cruise while sinking -> DESCENT (spins polling up). */
    for (int i = 0; i < 5; ++i) {
        sm_step(&c, p->cruise_ft - 5.0f - (float)i * 3.0f, DT, p, &out);
    }
    snprintf(msg, sizeof msg, "[%s] sink below cruise -> DESCENT", p->name);
    ASSERT_TRUE(out.state == ST_DESCENT, msg);
    snprintf(msg, sizeof msg, "[%s] DESCENT uses fast poll", p->name);
    ASSERT_TRUE(out.poll == POLL_DESCENT, msg);
}

static void test_initial_state(const sensor_profile_t *p)
{
    char msg[96];

    snprintf(msg, sizeof msg, "[%s] high boot estimate -> CRUISE", p->name);
    ASSERT_TRUE(sm_initial_state(p->cruise_ft + 50.0f, true, p) == ST_CRUISE, msg);

    snprintf(msg, sizeof msg, "[%s] mid boot estimate -> ARMED", p->name);
    ASSERT_TRUE(sm_initial_state(ARM_FT + 30.0f, true, p) == ST_ARMED, msg);

    snprintf(msg, sizeof msg, "[%s] low boot estimate -> GROUND", p->name);
    ASSERT_TRUE(sm_initial_state(3.0f, true, p) == ST_GROUND, msg);

    snprintf(msg, sizeof msg, "[%s] untrusted estimate -> GROUND", p->name);
    ASSERT_TRUE(sm_initial_state(400.0f, false, p) == ST_GROUND, msg);
}

static void test_trend_deadband(const sensor_profile_t *p)
{
    char msg[96];

    /* Part A — jitter sitting exactly on a callout height must not re-fire it.
     * We descend cleanly through the 50 ft callout once, then jitter ±0.4 ft
     * around 50 ft. Each tiny dip is a downward crossing, but the callout is a
     * one-shot and only re-arms after climbing REARM_MARGIN_FT above it, so it
     * must stay silent. This is the practical "noise at a threshold" case.     */
    {
        sm_ctx_t c;
        sm_init(&c, ST_GROUND);
        float scratch[16];
        ramp(&c, p, 0.0f, p->callouts[0] + 40.0f, +2.0f, scratch, 16);
        ramp(&c, p, p->callouts[0] + 40.0f, 49.5f, -2.0f, scratch, 16);

        sm_out_t out;
        int fires = 0;
        for (int i = 0; i < 40; ++i) {
            float agl = 50.0f + ((i % 2) ? 0.4f : -0.4f);
            sm_step(&c, agl, DT, p, &out);
            if (out.fired_callout >= 0) {
                ++fires;
            }
        }
        snprintf(msg, sizeof msg, "[%s] dead-band: jitter at a fired callout stays silent", p->name);
        ASSERT_TRUE(fires == 0, msg);
    }

    /* Part B — sub-dead-band drift while armed and level (no callout nearby)
     * neither fires a callout nor flips the machine into DESCENT. We sit at
     * 80 ft with ±0.005 ft jitter (~0.4 fps < TREND_DEADBAND_FPS).             */
    {
        sm_ctx_t c;
        sm_init(&c, ST_ARMED);
        sm_out_t out;
        int fires = 0, descents = 0;
        for (int i = 0; i < 30; ++i) {
            float agl = 80.0f + ((i % 2) ? 0.005f : -0.005f);
            sm_step(&c, agl, DT, p, &out);
            if (out.fired_callout >= 0) ++fires;
            if (out.state == ST_DESCENT) ++descents;
        }
        snprintf(msg, sizeof msg, "[%s] dead-band: level jitter fires nothing", p->name);
        ASSERT_TRUE(fires == 0, msg);
        snprintf(msg, sizeof msg, "[%s] dead-band: level jitter doesn't trip DESCENT", p->name);
        ASSERT_TRUE(descents == 0, msg);
    }
}

/* ---------------------------------------------------------------------------
 *  Ground-dwell disarm: after GROUND_RESET_MS continuously parked we must DISARM
 *  (as if rebooted onto the ground) so the next takeoff is silent until we climb
 *  back through ARM_FT; but a quick touch-and-go inside the window keeps the arm.
 * ------------------------------------------------------------------------- */
static void test_ground_dwell_disarm(const sensor_profile_t *p)
{
    char msg[96];

    /* --- Part A: park past the timeout -> next descent is silent until re-arm. */
    {
        sm_ctx_t c;
        sm_init(&c, ST_GROUND);

        /* Fly a full circuit so everything is armed and fired once. */
        float scratch[16];
        float top = p->callouts[0] + REARM_MARGIN_FT + 20.0f;
        ramp(&c, p, 0.0f, top, +2.0f, scratch, 16);
        ramp(&c, p, top, 0.0f, -2.0f, scratch, 16);

        /* Sit on the ground (0 ft, level) for well past GROUND_RESET_MS. With
         * DT = 25 ms that is GROUND_RESET_MS/25 ticks; add margin.               */
        int park_ticks = (int)(GROUND_RESET_MS / (DT * 1000.0f)) + 20;
        sm_out_t out;
        for (int i = 0; i < park_ticks; ++i) {
            sm_step(&c, 0.0f, DT, p, &out);
        }
        snprintf(msg, sizeof msg, "[%s] parked past timeout -> disarmed", p->name);
        ASSERT_TRUE(out.state == ST_GROUND, msg);

        /* Now take off and descend again WITHOUT first climbing through ARM_FT:
         * a low hop to just above a mid callout then back down must stay silent,
         * because the disarm means callouts only re-arm after ARM_FT.            */
        float hits[16];
        float hop = ARM_FT - 10.0f;     /* below ARM_FT: must not re-arm */
        ramp(&c, p, 0.0f, hop, +2.0f, scratch, 16);
        int n = ramp(&c, p, hop, 0.0f, -2.0f, hits, 16);
        snprintf(msg, sizeof msg, "[%s] post-park low hop stays silent", p->name);
        ASSERT_TRUE(n == 0, msg);

        /* And a proper climb back through ARM_FT re-arms it the normal way. */
        ramp(&c, p, 0.0f, top, +2.0f, scratch, 16);
        int n2 = ramp(&c, p, top, 0.0f, -2.0f, hits, 16);
        snprintf(msg, sizeof msg, "[%s] climb past ARM_FT re-arms after park", p->name);
        ASSERT_TRUE(n2 == (int)p->n_callouts, msg);
    }

    /* --- Part B: a quick touch-and-go inside the window keeps the arm. -------- */
    {
        sm_ctx_t c;
        sm_init(&c, ST_GROUND);

        float scratch[16];
        float top = p->callouts[0] + REARM_MARGIN_FT + 20.0f;
        ramp(&c, p, 0.0f, top, +2.0f, scratch, 16);
        ramp(&c, p, top, 0.0f, -2.0f, scratch, 16);

        /* Touch down only briefly — a handful of ground ticks, far under the
         * timeout — then go around. The arm must survive, so the 50 ft callout
         * re-fires on the next descent without needing a full re-climb logic
         * beyond the normal go-around re-arm.                                    */
        sm_out_t out;
        for (int i = 0; i < 5; ++i) {       /* ~125 ms on the ground */
            sm_step(&c, 0.0f, DT, p, &out);
        }
        float hits[16];
        ramp(&c, p, 0.0f, top, +2.0f, scratch, 16);
        int n = ramp(&c, p, top, 0.0f, -2.0f, hits, 16);
        snprintf(msg, sizeof msg, "[%s] brief touch-and-go keeps callouts armed", p->name);
        ASSERT_TRUE(n == (int)p->n_callouts, msg);
    }
}

/* ---------------------------------------------------------------------------
 *  "Positive rate" climb callout. A confirmed-climb detector: armed only once
 *  settled in the flare region (held at/below POSRATE_ARM_FT for the flare
 *  fade-out time), then fires one-shot after a sustained >= POSRATE_MIN_FPS climb
 *  above the gate for POSRATE_SUSTAIN_MS. Re-arms on every settled touch, so it
 *  fires on every takeoff — not just the first.
 * ------------------------------------------------------------------------- */

/* Step once and report whether the positive-rate callout fired this tick. */
static bool step_posrate(sm_ctx_t *c, float agl, const sensor_profile_t *p)
{
    sm_out_t out;
    sm_step(c, agl, DT, p, &out);
    return out.fired_positive_rate;
}

/* Ramp AGL from 'from' to 'to' in 'step' increments; return the number of ticks
 * on which the positive-rate callout fired. */
static int ramp_posrate(sm_ctx_t *c, const sensor_profile_t *p,
                        float from, float to, float step)
{
    int fires = 0;
    float agl = from;
    while ((step < 0 && agl >= to) || (step > 0 && agl <= to)) {
        if (step_posrate(c, agl, p)) {
            ++fires;
        }
        agl += step;
    }
    return fires;
}

/* Hold AGL steady for 'ticks' and return the number of positive-rate fires. */
static int hold_posrate(sm_ctx_t *c, const sensor_profile_t *p, float agl, int ticks)
{
    int fires = 0;
    for (int i = 0; i < ticks; ++i) {
        if (step_posrate(c, agl, p)) {
            ++fires;
        }
    }
    return fires;
}

static void test_positive_rate(const sensor_profile_t *p)
{
    char msg[96];

    /* A brisk but realistic climb: 0.4 ft/tick = 16 ft/s (~960 fpm), well above
     * the 100 fpm floor. Settled-low arming needs FLARE_FADE_OUT_MS continuously
     * at/below the gate; with DT this is that many ticks plus a margin.          */
    const float CLIMB_STEP   = 0.4f;
    const int   SETTLE_TICKS = (int)(FLARE_FADE_OUT_MS / (DT * 1000.0f)) + 10;

    /* --- Part A: a confirmed climb from the ground fires exactly once. -------- */
    {
        sm_ctx_t c;
        sm_init(&c, ST_GROUND);                 /* seeded on the ground -> armed */
        hold_posrate(&c, p, 0.0f, 5);           /* settle a moment on the ground */
        int fires = ramp_posrate(&c, p, 0.0f, 150.0f, CLIMB_STEP);
        snprintf(msg, sizeof msg, "[%s] positive rate fires once on a real climb", p->name);
        ASSERT_TRUE(fires == 1, msg);
    }

    /* --- Part B: it re-arms on the ground and fires again on the NEXT takeoff. */
    {
        sm_ctx_t c;
        sm_init(&c, ST_GROUND);
        ramp_posrate(&c, p, 0.0f, 150.0f, CLIMB_STEP);   /* first takeoff (fires) */
        ramp_posrate(&c, p, 150.0f, 0.0f, -CLIMB_STEP);  /* land back down        */
        hold_posrate(&c, p, 0.0f, SETTLE_TICKS);         /* settle -> re-arm      */
        int fires = ramp_posrate(&c, p, 0.0f, 150.0f, CLIMB_STEP);
        snprintf(msg, sizeof msg, "[%s] positive rate re-fires on a second takeoff", p->name);
        ASSERT_TRUE(fires == 1, msg);
    }

    /* --- Part C: a bounce (brief dip below the gate) must NOT arm it. --------- */
    {
        sm_ctx_t c;
        sm_init(&c, ST_ARMED);                  /* flying seed -> NOT armed      */
        /* Dip to 5 ft only briefly — far less than the fade-out settle time —
         * then climb away. The settle never completes, so nothing arms or fires. */
        hold_posrate(&c, p, 5.0f, SETTLE_TICKS / 4);
        int fires = ramp_posrate(&c, p, 5.0f, 150.0f, CLIMB_STEP);
        snprintf(msg, sizeof msg, "[%s] a bounce below the gate never arms positive rate", p->name);
        ASSERT_TRUE(fires == 0, msg);
    }

    /* --- Part D: a brief, non-sustained climb (window never fills) is silent. - */
    {
        sm_ctx_t c;
        sm_init(&c, ST_GROUND);                 /* armed */
        /* Hop to ~20 ft then hold level: the climb is above the gate only briefly
         * and the rate then collapses, so the 2 s sustain window never fills.    */
        ramp_posrate(&c, p, 0.0f, 20.0f, CLIMB_STEP);
        int fires = hold_posrate(&c, p, 20.0f, 200);   /* 5 s level at 20 ft */
        snprintf(msg, sizeof msg, "[%s] an unsustained hop does not fire positive rate", p->name);
        ASSERT_TRUE(fires == 0, msg);
    }
}

/* ---------------------------------------------------------------------------
 *  Vertical rate + acceleration outputs. These feed the audio engine's vario
 *  "blip": vert_fps must track the smoothed descent rate (sign + magnitude), and
 *  vert_accel_fps2 (its first derivative) must read negative while the sink is
 *  WORSENING and positive while it EASES. Both are profile-independent.
 * ------------------------------------------------------------------------- */
static void test_vertical_rate(const sensor_profile_t *p)
{
    char msg[96];

    /* --- Part A: a steady descent converges vert_fps to the true rate. -------- */
    {
        sm_ctx_t c;
        sm_init(&c, ST_ARMED);
        sm_out_t out;
        memset(&out, 0, sizeof out);

        /* Descend 0.25 ft per 25 ms tick = -10 ft/s (~600 fpm). Run long enough for
         * the one-pole rate EMA to settle, then check the reported rate + sign.    */
        float agl = 90.0f;
        for (int i = 0; i < 80; ++i) {
            sm_step(&c, agl, DT, p, &out);
            agl -= 0.25f;
        }
        snprintf(msg, sizeof msg, "[%s] steady descent -> vert_fps negative", p->name);
        ASSERT_TRUE(out.vert_fps < 0.0f, msg);
        snprintf(msg, sizeof msg, "[%s] steady descent -> vert_fps ~= -10 ft/s", p->name);
        ASSERT_TRUE(fabsf(out.vert_fps - (-10.0f)) < 1.5f, msg);
        snprintf(msg, sizeof msg, "[%s] steady descent -> accel settles ~0", p->name);
        ASSERT_TRUE(fabsf(out.vert_accel_fps2) < 3.0f, msg);
    }

    /* --- Part B: a steepening then easing sink flips the accel sign. ---------- */
    {
        sm_ctx_t c;
        sm_init(&c, ST_ARMED);
        sm_out_t out;
        memset(&out, 0, sizeof out);

        /* Settle a gentle -4 ft/s descent. */
        float agl = 90.0f;
        for (int i = 0; i < 40; ++i) { sm_step(&c, agl, DT, p, &out); agl -= 0.10f; }

        /* Now sink much faster (-16 ft/s): the rate becomes MORE negative, so the
         * acceleration (d/dt of rate) must read negative through the transient.    */
        bool saw_neg_accel = false;
        for (int i = 0; i < 20; ++i) {
            sm_step(&c, agl, DT, p, &out);
            agl -= 0.40f;
            if (out.vert_accel_fps2 < -2.0f) saw_neg_accel = true;
        }
        snprintf(msg, sizeof msg, "[%s] steepening sink -> negative accel", p->name);
        ASSERT_TRUE(saw_neg_accel, msg);

        /* Ease back toward level: the rate climbs toward 0, so accel goes positive. */
        bool saw_pos_accel = false;
        for (int i = 0; i < 30; ++i) {
            sm_step(&c, agl, DT, p, &out);
            agl -= 0.02f;                 /* nearly level now */
            if (out.vert_accel_fps2 > 2.0f) saw_pos_accel = true;
        }
        snprintf(msg, sizeof msg, "[%s] easing sink -> positive accel", p->name);
        ASSERT_TRUE(saw_pos_accel, msg);
    }
}

int main(void)
{
    const sensor_profile_t *profiles[2] = { &SF30C_PROFILE, &SF30D_PROFILE };

    printf("== state_machine ==\n");
    for (int i = 0; i < 2; ++i) {
        const sensor_profile_t *p = profiles[i];
        printf("-- profile %s --\n", p->name);
        test_silent_climbout(p);
        test_descent_sequence(p);
        test_hysteresis_no_machinegun(p);
        test_goaround_rearm(p);
        test_cruise_gating(p);
        test_initial_state(p);
        test_trend_deadband(p);
        test_ground_dwell_disarm(p);
        test_positive_rate(p);
        test_vertical_rate(p);
    }
    printf("-- profile-specific high callouts --\n");
    test_high_callouts_profile_specific();

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
