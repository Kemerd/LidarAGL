/**
 * @file    test_state_machine.c
 * @brief   Host unit tests for the behaviour state machine.
 *
 * @details Drives sm_step() with synthetic AGL sequences and asserts the
 *          safety-critical behaviours: silent climb-out, edge-triggered descent
 *          callouts, hysteresis / go-around re-arm, CRUISE gating, and trend
 *          dead-band. Every test runs against BOTH sensor profiles so we prove
 *          the SF30/D fires 500/400/300 and the SF30/C never does.
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
    /* SF30/D must fire 500/400/300; SF30/C must NOT have them at all. */
    float scratch[16], hits[16];

    sm_ctx_t cd;
    sm_init(&cd, ST_GROUND);
    ramp(&cd, &SF30D_PROFILE, 0.0f, 560.0f, +4.0f, scratch, 16);
    int nd = ramp(&cd, &SF30D_PROFILE, 560.0f, 0.0f, -2.0f, hits, 16);
    ASSERT_TRUE(has_hit(hits, nd, 500.0f), "[SF30/D] fires 500 ft");
    ASSERT_TRUE(has_hit(hits, nd, 400.0f), "[SF30/D] fires 400 ft");
    ASSERT_TRUE(has_hit(hits, nd, 300.0f), "[SF30/D] fires 300 ft");

    /* SF30/C: confirm 300 is simply not in its ladder and never fires even if
     * we (impossibly) flew that high. */
    sm_ctx_t cc;
    sm_init(&cc, ST_GROUND);
    ramp(&cc, &SF30C_PROFILE, 0.0f, 320.0f, +4.0f, scratch, 16);
    int nc = ramp(&cc, &SF30C_PROFILE, 320.0f, 0.0f, -2.0f, hits, 16);
    ASSERT_TRUE(!has_hit(hits, nc, 300.0f), "[SF30/C] never fires 300 ft");
    ASSERT_TRUE(!has_hit(hits, nc, 500.0f), "[SF30/C] never fires 500 ft");
    /* But it must still start its ladder at 200. */
    ASSERT_TRUE(has_hit(hits, nc, 200.0f), "[SF30/C] top callout is 200 ft");
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
    }
    printf("-- profile-specific high callouts --\n");
    test_high_callouts_profile_specific();

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
