/**
 * @file    state_machine.c
 * @brief   Implementation of the LidarAGL behaviour state machine.
 *
 * @details PURE: standard library + config.h pure region + sensor_profile.h.
 *          Every threshold comes from either config.h (profile-independent) or
 *          the active sensor_profile_t (per-sensor). No magic numbers here.
 */

#include "state_machine.h"
#include "config.h"

#include <math.h>

/* ---------------------------------------------------------------------------
 *  Trend smoothing factor. We low-pass the instantaneous vertical rate so a
 *  single noisy sample can't flip the perceived direction. This is separate
 *  from the AGL EMA (which lives in the sensor path); here we only need a
 *  stable SIGN, not a precise rate.
 * ------------------------------------------------------------------------- */
#define TREND_SMOOTH_ALPHA 0.25f

void sm_init(sm_ctx_t *c, sm_state_t initial)
{
    c->state      = initial;
    c->prev_agl   = 0.0f;
    c->trend_fps  = 0.0f;
    c->armed_mask = 0u;
    c->have_prev  = false;
    c->ground_ms  = 0.0f;   /* fresh ground-dwell timer */

    /*  Positive-rate detector. We pre-arm only when seeded on the ground; a
     *  flying seed (in-flight reboot) starts DISARMED so a reboot mid-climb can
     *  never blurt "positive rate". The low-dwell timer starts fresh either way. */
    c->posrate_armed  = (initial == ST_GROUND);
    c->posrate_low_ms = 0.0f;
    c->posrate_ms     = 0.0f;

    /* Default the tone gate to the compile-time start altitude; app_main overrides
     * it from the pilot's saved config after init (host tests use the default). */
    c->tone_start_ft  = TONE_START_FT;

    /* If we are seeded into a flying state, the descent callouts must already
     * be armed — we may have rebooted mid-descent and need to fire on the way
     * down without first having to climb through ARM_FT again.                 */
    if (initial == ST_ARMED || initial == ST_CRUISE || initial == ST_DESCENT) {
        c->armed = true;
        /* Arm every callout; each still edge-triggers only on its own crossing. */
        c->armed_mask = ~0u;
    } else {
        c->armed = false;
    }
}

sm_state_t sm_initial_state(float boot_agl, bool ok, const sensor_profile_t *p)
{
    /* Untrustworthy estimate -> assume parked. GROUND is the safe default: it
     * produces no audio and simply watches for a climb.                        */
    if (!ok) {
        return ST_GROUND;
    }

    /* Clearly at cruise altitude -> wake straight into the low-power state. */
    if (boot_agl >= p->cruise_ft) {
        return ST_CRUISE;
    }

    /* Above the arm height but below cruise -> we are flying and armed. */
    if (boot_agl > ARM_FT) {
        return ST_ARMED;
    }

    /* Near the ground and trustworthy -> parked. */
    return ST_GROUND;
}

/* ---------------------------------------------------------------------------
 *  Update the smoothed vertical-rate estimate and return the carried context's
 *  current trend in ft/s (positive = climbing).
 * ------------------------------------------------------------------------- */
static float update_trend(sm_ctx_t *c, float agl_ft, float dt_s)
{
    if (!c->have_prev || dt_s <= 0.0f) {
        /* First sample (or a bogus dt): seed prev, report level. */
        c->prev_agl  = agl_ft;
        c->have_prev = true;
        c->trend_fps = 0.0f;
        return 0.0f;
    }

    float inst = (agl_ft - c->prev_agl) / dt_s;            /* instantaneous fps */
    c->trend_fps += TREND_SMOOTH_ALPHA * (inst - c->trend_fps);
    c->prev_agl = agl_ft;
    return c->trend_fps;
}

/* Direction helpers built on the dead-band so noise reads as "level". */
static bool is_descending(float trend_fps)
{
    return trend_fps < -TREND_DEADBAND_FPS;
}
static bool is_climbing(float trend_fps)
{
    return trend_fps > TREND_DEADBAND_FPS;
}

uint32_t poll_profile_to_ms(poll_profile_t pp)
{
    /* Single place the power/latency policy lives. */
    switch (pp) {
        case POLL_GROUND:  return POLL_MS_GROUND;
        case POLL_CLIMB:   return POLL_MS_CLIMB;
        case POLL_ARMED:   return POLL_MS_ARMED;
        case POLL_CRUISE:  return POLL_MS_CRUISE;
        case POLL_DESCENT: return POLL_MS_DESCENT;
    }
    return POLL_MS_ARMED;   /* unreachable; safe default */
}

/* Map a state to its poll profile. */
static poll_profile_t poll_for_state(sm_state_t s)
{
    switch (s) {
        case ST_GROUND:  return POLL_GROUND;
        case ST_CLIMB:   return POLL_CLIMB;
        case ST_ARMED:   return POLL_ARMED;
        case ST_CRUISE:  return POLL_CRUISE;
        case ST_DESCENT: return POLL_DESCENT;
    }
    return POLL_ARMED;
}

/* ---------------------------------------------------------------------------
 *  Re-arm callouts whose height the aircraft has climbed back above by at least
 *  REARM_MARGIN_FT. This is the hysteresis that (a) prevents machine-gunning a
 *  callout while hovering near its threshold and (b) re-enables callouts for a
 *  go-around. Only meaningful once we are 'armed' (past ARM_FT at least once).
 * ------------------------------------------------------------------------- */
static void rearm_above(sm_ctx_t *c, float agl_ft, const sensor_profile_t *p)
{
    for (size_t i = 0; i < p->n_callouts; ++i) {
        if (agl_ft > p->callouts[i] + REARM_MARGIN_FT) {
            c->armed_mask |= (1u << i);
        }
    }
}

/* ---------------------------------------------------------------------------
 *  On a downward crossing, find the HIGHEST armed callout that the aircraft has
 *  now descended THROUGH, fire it once, and disarm it. Returns the callout
 *  index fired, or -1 if none.
 *
 *  A callout fires only on a genuine DOWNWARD CROSSING: the previous AGL was
 *  strictly above the threshold and the current AGL is at/below it. Requiring
 *  the edge (rather than merely "agl <= threshold") is essential — without it,
 *  an in-flight reboot that seeds every callout armed would instantly blurt out
 *  every number above the current altitude. We only want a callout when the
 *  aircraft actually passes down through that height.
 *
 *  We test highest-first and fire at most one per tick. Because descent is
 *  monotonic between ticks and ticks are fast (POLL_MS_DESCENT), this gives the
 *  correct "200 ... 100 ... 50 ..." sequence; if two thresholds are crossed in
 *  one tick (very steep descent / slow tick) the higher one fires this tick and
 *  the lower remains armed for the next tick — a late number is never skipped.
 * ------------------------------------------------------------------------- */
static int fire_descent_callout(sm_ctx_t *c, float prev_agl, float agl_ft,
                                const sensor_profile_t *p)
{
    for (size_t i = 0; i < p->n_callouts; ++i) {
        bool armed = (c->armed_mask & (1u << i)) != 0u;
        bool crossed_down = (prev_agl > p->callouts[i]) &&
                            (agl_ft   <= p->callouts[i]);
        if (armed && crossed_down) {
            c->armed_mask &= ~(1u << i);   /* one-shot: disarm after firing */
            return (int)i;
        }
    }
    return -1;
}

void sm_step(sm_ctx_t *c, float agl_ft, float dt_s,
             const sensor_profile_t *p, sm_out_t *out)
{
    /* Clamp negative AGL (sensor noise / over-subtracted ground ref) to 0 so we
     * never reason about "below the ground". */
    if (agl_ft < 0.0f) {
        agl_ft = 0.0f;
    }

    /* Capture the previous AGL BEFORE update_trend() overwrites it — the
     * downward-crossing test below needs "where we were last tick".            */
    float prev_agl   = c->prev_agl;
    bool  had_prev   = c->have_prev;
    float trend      = update_trend(c, agl_ft, dt_s);

    int fired = -1;

    /* --- Arming: the silent climb-out ------------------------------------- */
    /*  Until the aircraft has climbed through ARM_FT for the first time, NO
     *  callout may fire. Crossing ARM_FT latches 'armed' and arms the WHOLE
     *  ladder — every callout, not just the ones already below us.
     *
     *  Why arm the higher ones too? A callout only ever FIRES on a genuine
     *  DOWNWARD crossing (see fire_descent_callout), so arming a number we're
     *  still climbing toward is harmless: it simply waits until the aircraft
     *  actually descends through it. Arming only "below current" here forced the
     *  higher numbers to rely on rearm_above(), which needs a climb of
     *  REARM_MARGIN_FT ABOVE the callout. For a TOP callout sitting close to the
     *  sensor ceiling (e.g. SF30/C's 300 ft, only ~28 ft below the 328 ft range)
     *  that climb is impossible, so the top number could never arm and never
     *  spoke. Arming the full ladder fixes that while keeping the climb silent
     *  (firing still needs the downward crossing) and the one-shot/go-around
     *  hysteresis intact (rearm_above still gates RE-arming after a fire).        */
    if (!c->armed && agl_ft > ARM_FT) {
        c->armed = true;
        for (size_t i = 0; i < p->n_callouts; ++i) {
            c->armed_mask |= (1u << i);
        }
    }

    /* --- State transition logic ------------------------------------------- */
    sm_state_t next = c->state;

    if (!c->armed) {
        /* Pre-arm life: GROUND when low & not climbing, CLIMB while rising. */
        if (agl_ft <= ARM_FT && (is_climbing(trend))) {
            next = ST_CLIMB;
        } else if (agl_ft <= 2.0f && !is_climbing(trend)) {
            next = ST_GROUND;
        } else {
            /* Hold current pre-arm state (GROUND or CLIMB). */
            next = (c->state == ST_CLIMB) ? ST_CLIMB : ST_GROUND;
        }
    } else {
        /* Armed life. Cruise band gates the low-power state; below it we are
         * ARMED (level/climbing) or DESCENT (sinking).                         */
        if (agl_ft >= p->cruise_ft) {
            next = ST_CRUISE;
        } else if (is_descending(trend)) {
            next = ST_DESCENT;
        } else {
            /* Level or climbing below cruise: stay/return to ARMED. Leaving
             * DESCENT when we stop sinking avoids fast-polling on the runway.  */
            next = ST_ARMED;
        }

        /* Re-arm callouts we've climbed safely back above (go-around support). */
        rearm_above(c, agl_ft, p);

        /* Fire on a genuine downward crossing. We need a valid previous sample
         * (had_prev) so the very first tick after a boot — which seeds every
         * callout armed for in-flight-reboot recovery — does NOT mistake the
         * initial reading for a crossing and blurt out every number at once.   */
        if (had_prev) {
            fired = fire_descent_callout(c, prev_agl, agl_ft, p);
        }
    }

    c->state = next;

    /* --- Ground-dwell disarm (taxi-back / parked reset) ------------------- */
    /*  Accumulate continuous time spent SETTLED ON THE GROUND. Note we can't key
     *  this off ST_GROUND: once 'armed', the machine only ever picks CRUISE /
     *  DESCENT / ARMED, so a landed-but-armed box sits in ST_ARMED, never returns
     *  to ST_GROUND. So we detect "parked" directly from the motion — at/below the
     *  ground band (a hair above 0) and not descending or climbing. The instant we
     *  leave that (a bounce, a climb, a go-around) the timer resets, so a quick
     *  return to flight keeps the callouts/tone armed. Once we've simply sat parked
     *  past GROUND_RESET_MS we DISARM as if freshly rebooted onto the ground: clear
     *  the arm latch and every armed bit, so the next takeoff is silent until the
     *  aircraft climbs back through ARM_FT and re-arms naturally.                  */
    bool parked = (agl_ft <= 2.0f) && !is_descending(trend) && !is_climbing(trend);
    if (parked) {
        c->ground_ms += dt_s * 1000.0f;
        if (c->armed && c->ground_ms >= (float)GROUND_RESET_MS) {
            c->armed      = false;
            c->armed_mask = 0u;
            next          = ST_GROUND;   /* reflect the disarmed, parked reset */
            c->state      = next;
        }
    } else {
        c->ground_ms = 0.0f;
    }

    /* --- "Positive rate" climb callout (takeoff / touch-and-go) ----------- */
    /*  A confirmed-climb detector, deliberately NOT a single AGL crossing (a lone
     *  sample would fire on a bounce, a flare balloon, or sensor jitter). See the
     *  POSRATE_* block in config.h for the full rationale. In brief:
     *
     *    ARM gate (bounce guard): we arm only once the aircraft has SETTLED in the
     *    flare region — held continuously at/below POSRATE_ARM_FT long enough for
     *    the tone's flare fade-out to finish (FLARE_FADE_OUT_MS). A bounce that
     *    pops back above the gate before the fade completes resets the dwell and
     *    never arms. This re-arms on every landing/touch, so each departure (the
     *    first or a touch-and-go) gets its own one-shot.
     *
     *    FIRE: once armed, the call fires after the aircraft has climbed back ABOVE
     *    the gate AND the smoothed climb rate has held at/above POSRATE_MIN_FPS
     *    (100 fpm) CONTINUOUSLY for POSRATE_SUSTAIN_MS. Then it disarms (one-shot)
     *    until the next settled touch re-arms it.                                   */
    bool posrate_fire = false;
    if (agl_ft <= POSRATE_ARM_FT) {
        /* In the flare / touch region: a real climb can't be in progress, so the
         * sustain window is held at zero. Accrue settled-low time toward the arm. */
        c->posrate_ms     = 0.0f;
        c->posrate_low_ms += dt_s * 1000.0f;
        if (c->posrate_low_ms >= (float)FLARE_FADE_OUT_MS) {
            c->posrate_armed = true;   /* fade-out finished -> genuinely settled */
        }
    } else {
        /* Above the gate: any low-dwell is broken, so a future dip must re-settle
         * from scratch before it can arm again.                                   */
        c->posrate_low_ms = 0.0f;
        if (c->posrate_armed && trend >= POSRATE_MIN_FPS) {
            /* Armed and climbing convincingly: grow the sustain window. */
            c->posrate_ms += dt_s * 1000.0f;
            if (c->posrate_ms >= (float)POSRATE_SUSTAIN_MS) {
                posrate_fire     = true;   /* climb confirmed */
                c->posrate_armed = false;  /* one-shot until re-armed on a touch */
            }
        } else {
            /* Not armed, or the climb broke before the window filled: restart it. */
            c->posrate_ms = 0.0f;
        }
    }

    /* --- Tone gating ------------------------------------------------------ */
    /*  The audio engine owns the dB swell curve; here we only decide WHETHER a
     *  tone should sound and feed it the current AGL. The tone is active only
     *  when armed and within the swell band, and never in GROUND/CRUISE.       */
    bool tone_active = c->armed &&
                       next != ST_CRUISE &&
                       next != ST_GROUND &&
                       agl_ft <= c->tone_start_ft;

    out->state               = next;
    out->fired_callout       = fired;
    out->poll                = poll_for_state(next);
    out->tone_agl            = agl_ft;
    out->tone_active         = tone_active;
    out->fired_positive_rate = posrate_fire;
    /*  Hand the smoothed vertical rate to the audio engine so the vario blip can
     *  chop the tone by descent rate (see audio.c blip gate).                     */
    out->vert_fps            = c->trend_fps;
}
