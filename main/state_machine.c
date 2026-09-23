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
#include <string.h>

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
    c->park_ok    = true;   /* a boot on the ramp IS parked                     */
    c->lead_rate_fps = 0.0f;  /* no callout lead until the caller supplies it   */
    c->prev_lead_ft  = 0.0f;

    /* Fresh arming-persistence dwells: nothing is part-way to arming. The
     * observation counters clear alongside their timers so the first in-band
     * decision after a (re)init always opens a window rather than closing one. */
    c->arm_ms  = 0.0f;
    c->arm_obs = 0u;
    memset(c->rearm_ms,  0, sizeof c->rearm_ms);
    memset(c->rearm_obs, 0, sizeof c->rearm_obs);

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

/* ---------------------------------------------------------------------------
 *  The callout lead for the caller-supplied tracker rate (see CALLOUT_LEAD_S):
 *  how far the aircraft will descend between a rung firing and the word being
 *  heard. Descents only, capped at CALLOUT_LEAD_MAX_FT; a non-finite rate
 *  simply means no lead. Shared by sm_step() and sm_reanchor() so the crossing
 *  test and the late-rung window judge lateness identically.
 * ------------------------------------------------------------------------- */
static float callout_lead_ft(const sm_ctx_t *c)
{
    float rate = c->lead_rate_fps;
    if (!isfinite(rate) || !(rate < -TREND_DEADBAND_FPS)) {
        return 0.0f;
    }
    float lead = -rate * CALLOUT_LEAD_S;
    return (lead > CALLOUT_LEAD_MAX_FT) ? CALLOUT_LEAD_MAX_FT : lead;
}

int sm_reanchor(sm_ctx_t *c, float agl_ft, const sensor_profile_t *p,
                bool reentry, uint32_t *crossed_mask)
{
    if (crossed_mask != NULL) {
        *crossed_mask = 0u;
    }
    if (c == NULL) {
        return -1;
    }
    if (!(agl_ft >= 0.0f)) {
        agl_ft = 0.0f;                   /* negative or NaN: clamp, like sm_step */
    }

    /* --- Late-rung window: rungs genuinely passed while the laser was blind --
     *  Only for a flyable DESCENDING re-entry (the filter's verdict), only
     *  once armed, and only against a real previous anchor. Evaluated BEFORE
     *  the anchor moves: a rung was "passed" when it lies strictly below the
     *  old anchor and at/above the new level — the same edge
     *  fire_descent_callout() uses (prev > rung >= agl).                      */
    int speak = -1;
    if (reentry && p != NULL && c->armed && c->have_prev) {
        size_t n = p->n_callouts;
        if (n > SM_MAX_CALLOUTS) {
            n = SM_MAX_CALLOUTS;          /* defensive: profiles are far smaller */
        }
        uint32_t passed = 0u;
        int      lowest = -1;
        for (size_t i = 0; i < n; ++i) {
            bool armed = (c->armed_mask & (1u << i)) != 0u;
            float h    = p->callouts[i];
            if (armed && c->prev_agl > h && agl_ft <= h) {
                passed |= (1u << i);
                lowest  = (int)i;        /* ladders descend: last hit = lowest  */
            }
        }
        if (lowest >= 0) {
            /*  Every passed rung is spent — exactly as a multi-rung crossing
             *  spends them — so jitter around the new level can never speak a
             *  stale number later. Go-around re-arming works as always.     */
            c->armed_mask &= ~passed;
            if (crossed_mask != NULL) {
                *crossed_mask = passed;
            }

            /*  Where the word would be HEARD: the aircraft keeps sinking
             *  through the audio latency, the same lead sm_step() applies.   */
            float h    = p->callouts[lowest];
            float late = h - (agl_ft - callout_lead_ft(c));
            float tol  = (h >= CALLOUT_LATE_TOL_SPLIT_FT) ? CALLOUT_LATE_TOL_HI_FT
                                                          : CALLOUT_LATE_TOL_LO_FT;
            if (late <= tol) {
                speak = lowest;
            }
        }
    }

    /*  Move the crossing anchor to the new level WITHOUT letting the move look
     *  like flown motion. fire_descent_callout() compares prev_agl against the
     *  current AGL, so seeding prev_agl to the new value means the very next
     *  sm_step() sees no crossing across the gap — only genuine movement from
     *  here on can fire a rung.                                                 */
    c->prev_agl  = agl_ft;
    c->have_prev = true;

    /*  The callout lead's anchor becomes the PHYSICAL new level (no lead): the
     *  next step then compares the new level with where the aircraft will be one
     *  lead time later, so a rung the descent is about to reach is announced
     *  now, which is the point of the lead. (Re-basing the anchor on the lead
     *  instead would have hidden any rung within the lead distance below the new
     *  level, permanently: in lead-compensated terms the aircraft would already
     *  be "past" it without ever crossing it.)                                 */
    c->prev_lead_ft = 0.0f;

    /*  A discontinuity carries no velocity information: the jump did not happen
     *  at any particular rate. Leaving the old trend in place would let a stale
     *  climb/descent sign drive the state transition (and the vario blip) for
     *  several ticks after the re-anchor.                                        */
    c->trend_fps = 0.0f;

    /*  A re-anchor INTO the ground band is a broken track landing on a low
     *  level, not a landing: it must not start (or continue) the parked-disarm
     *  dwell. Flying back above the band restores it (see sm_step).           */
    if (agl_ft <= GROUND_BAND_FT) {
        c->park_ok   = false;
        c->ground_ms = 0.0f;
    }

    /*  Arming, the per-rung one-shot mask and the arm/re-arm dwells are
     *  otherwise deliberately LEFT ALONE. The rungs already spoken were
     *  genuinely passed and must stay spent; the rungs ahead must stay
     *  available; and the arming dwells are about time spent in a band, which
     *  the re-anchor does not invalidate. (The PARKED dwell is the one
     *  exception, handled above; the late-rung window above is the other.)     */
    return speak;
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

uint32_t sm_poll_period_ms(poll_profile_t pp, bool armed, bool tracking,
                           bool reacquiring, float agl_ft, float tone_start_ft)
{
    uint32_t ms = poll_profile_to_ms(pp);

    /*  Inside the armed tone band a dark sensor is a FAILURE to annunciate,
     *  not an idle period, so it keeps the state's own (fast) rate there.     */
    bool in_live_band = armed && (agl_ft <= tone_start_ft);

    /* --- Dark: relax. Nothing to respond to, however fast we ask. -------- */
    if (!tracking && !in_live_band && ms < POLL_MS_CRUISE) {
        ms = POLL_MS_CRUISE;
    }

    /* --- Reacquiring while armed: look often. ----------------------------
     *  The ground is coming back into view — almost always the approach
     *  descending into range — and the new track must be confirmed before the
     *  aircraft falls past the top rungs (4000 fpm is ~33 ft per 500 ms poll).
     *
     *  A blanket "tracking -> poll fast" rule was tried first and REJECTED: at
     *  25-50 ms a drain holds 1-2 raw samples, the per-drain median vote lost
     *  its minority immunity, and out-of-range junk walked the altitude into
     *  phantom rungs at 600 ft. This narrower rule is safe because, while the
     *  track is lost, NOTHING is accepted on a single drain: the sliding
     *  RANGE_LOST_VOTE_SAMPLES vote must first say most recent samples are
     *  real, and then the candidates must form a constant-velocity track.    */
    if (reacquiring && tracking && armed && ms > POLL_MS_ARMED) {
        ms = POLL_MS_ARMED;
    }
    return ms;
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
 *  REARM_MARGIN_FT — and HELD there for REARM_SUSTAIN_MS. This is the hysteresis
 *  that (a) prevents machine-gunning a callout while hovering near its threshold
 *  and (b) re-enables callouts for a go-around. Only meaningful once we are
 *  'armed' (past ARM_FT at least once).
 *
 *  Why a dwell and not the old instant re-arm: a garbage range spike used to
 *  re-arm every callout below its own peak DURING the spike, and the spike's
 *  smoothed decay back to ground then fired them all — the phantom
 *  "50 40 30 20 10" heard on the taxiway. A real go-around climbs through a
 *  re-arm band in a sustained way (>=400 ms above height+margin is a few feet
 *  of climb), while a spike's peak spends at most a poll or two there.
 *
 *  As with the ARM_FT dwell, time accrues only from the SECOND consecutive
 *  in-band decision: the elapsed dt handed to the first one was spent BELOW the
 *  band. Without that rule a single sample at the 500 ms CRUISE cadence already
 *  exceeded REARM_SUSTAIN_MS and re-armed a rung in one observation.
 * ------------------------------------------------------------------------- */
static void rearm_above(sm_ctx_t *c, float agl_ft, float dt_s,
                        const sensor_profile_t *p)
{
    size_t n = p->n_callouts;
    if (n > SM_MAX_CALLOUTS) {
        n = SM_MAX_CALLOUTS;    /* defensive: profiles are far smaller */
    }
    for (size_t i = 0; i < n; ++i) {
        if ((c->armed_mask & (1u << i)) != 0u) {
            c->rearm_ms[i]  = 0.0f;        /* already armed: nothing to accrue  */
            c->rearm_obs[i] = 0u;
            continue;
        }
        if (agl_ft > p->callouts[i] + REARM_MARGIN_FT) {
            /* First in-band decision opens the window but banks NO time: the
             * elapsed dt belongs to the interval BEFORE this sample, when the
             * aircraft was still below the band. Only genuinely observed
             * in-band spans accrue (see sm_ctx_t's observation-counter note). */
            if (c->rearm_obs[i] < UINT16_MAX) {
                c->rearm_obs[i]++;
            }
            if (c->rearm_obs[i] >= 2u) {
                c->rearm_ms[i] += dt_s * 1000.0f;
            }
            if (c->rearm_ms[i] >= (float)REARM_SUSTAIN_MS) {
                c->armed_mask  |= (1u << i);
                c->rearm_ms[i]  = 0.0f;
                c->rearm_obs[i] = 0u;
            }
        } else {
            c->rearm_ms[i]  = 0.0f;        /* dropped out of the band: restart  */
            c->rearm_obs[i] = 0u;
        }
    }
}

/* ---------------------------------------------------------------------------
 *  On a downward crossing, fire the LOWEST armed callout the aircraft has now
 *  descended through, and disarm EVERY callout that was crossed. Returns the
 *  callout index fired, or -1 if none.
 *
 *  A callout fires only on a genuine DOWNWARD CROSSING: the previous AGL was
 *  strictly above the threshold and the current AGL is at/below it. Requiring
 *  the edge (rather than merely "agl <= threshold") is essential — without it,
 *  an in-flight reboot that seeds every callout armed would instantly blurt out
 *  every number above the current altitude. We only want a callout when the
 *  aircraft actually passes down through that height.
 *
 *  In a normal descent one tick crosses at most one threshold, so this is the
 *  familiar "200 ... 100 ... 50 ..." ladder. When one step DOES cross several
 *  (a genuine terrain drop under final, or a re-acquired level after the range
 *  filter held out an obstruction) we speak the LOWEST — the number closest to
 *  where the aircraft actually IS. The old code fired the highest, which is
 *  precisely the stalest information, and then silently skipped the rest (its
 *  own comment claimed otherwise): the skipped numbers could never edge-fire
 *  afterwards because prev_agl was already below them. Every crossed callout
 *  is disarmed either way — it was genuinely passed — and all of them re-arm
 *  through the normal go-around hysteresis.
 * ------------------------------------------------------------------------- */
static int fire_descent_callout(sm_ctx_t *c, float prev_agl, float agl_ft,
                                const sensor_profile_t *p)
{
    int fired = -1;
    for (size_t i = 0; i < p->n_callouts; ++i) {
        bool armed = (c->armed_mask & (1u << i)) != 0u;
        bool crossed_down = (prev_agl > p->callouts[i]) &&
                            (agl_ft   <= p->callouts[i]);
        if (armed && crossed_down) {
            c->armed_mask &= ~(1u << i);   /* one-shot: disarm every crossed   */
            fired = (int)i;                /* ladders are descending, so the   */
        }                                  /* LAST hit is the LOWEST height    */
    }
    return fired;
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

    int      fired        = -1;
    uint32_t crossed_mask = 0u;   /* every rung crossed downward this tick */

    /* --- Callout lead for this step (see CALLOUT_LEAD_S) ------------------
     *  Computed every step, armed or not, so the anchor carried into the next
     *  step is always the lead that was actually in force. Descents only.    */
    float lead = callout_lead_ft(c);

    /* --- Arming: the silent climb-out ------------------------------------- */
    /*  Until the aircraft has climbed through ARM_FT for the first time, NO
     *  callout may fire. HOLDING above ARM_FT for ARM_DWELL_MS latches 'armed'
     *  and arms the WHOLE ladder — every callout, not just those below us.
     *
     *  Why a dwell instead of the old single-sample latch: one unfiltered range
     *  spike above 100 ft used to arm the entire ladder instantly, and its
     *  smoothed decay back to ground then walked the ladder downward — the
     *  phantom "50 40 30 20 10" a taxiing aircraft once spoke. A real climb-out
     *  spends the whole climb above ARM_FT, so the dwell costs nothing (the
     *  callouts are silent on the way up regardless); a spike lasts a poll or
     *  two and can never accrue it. Same sustained-confirmation philosophy as
     *  the positive-rate detector below.
     *
     *  The dwell is accrued only from the SECOND consecutive above-gate decision
     *  onward. dt_s is the gap since the previous decision — time spent below
     *  the gate — so crediting it on the first in-band sample handed that sample
     *  a whole poll period (or a dropped poll's several seconds) of dwell it
     *  never earned. At the 750 ms GROUND cadence that let two samples, or after
     *  a gap a single one, latch the entire ladder: precisely the one-spike arm
     *  this dwell exists to prevent.
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
    if (!c->armed) {
        if (agl_ft > ARM_FT) {
            /* The first above-gate decision opens the window but banks NO time.
             * dt_s measures the interval BEFORE this sample — during which the
             * aircraft was observed (or unobserved) LOW — so crediting it would
             * let one spike sample carry a dropped poll's worth of dwell and
             * latch outright. Requiring a second consecutive in-band decision
             * makes ARM_DWELL_MS a genuine sustained confirmation at every poll
             * cadence, including the 750 ms GROUND rate where the taxi-phantom
             * garbage actually occurred.                                        */
            if (c->arm_obs < UINT16_MAX) {
                c->arm_obs++;
            }
            if (c->arm_obs >= 2u) {
                c->arm_ms += dt_s * 1000.0f;
            }
            if (c->arm_ms >= (float)ARM_DWELL_MS) {
                c->armed   = true;
                c->arm_ms  = 0.0f;
                c->arm_obs = 0u;
                for (size_t i = 0; i < p->n_callouts; ++i) {
                    c->armed_mask |= (1u << i);
                }
            }
        } else {
            c->arm_ms  = 0.0f;   /* dipped back below the gate: start over */
            c->arm_obs = 0u;
        }
    }

    /* --- State transition logic ------------------------------------------- */
    sm_state_t next = c->state;

    if (!c->armed) {
        /* Pre-arm life: GROUND when low & not climbing, CLIMB while rising. */
        if (agl_ft <= ARM_FT && (is_climbing(trend))) {
            next = ST_CLIMB;
        } else if (agl_ft <= GROUND_BAND_FT && !is_climbing(trend)) {
            next = ST_GROUND;
        } else {
            /* Hold current pre-arm state (GROUND or CLIMB). */
            next = (c->state == ST_CLIMB) ? ST_CLIMB : ST_GROUND;
        }
    } else {
        /* Armed life. Cruise band gates the low-power state; below it we are
         * ARMED (level/climbing) or DESCENT (sinking).
         *
         * The CRUISE test is deliberately on ALTITUDE ALONE, never on the trend.
         * That matters because CRUISE must not be a one-way door: it suspends
         * audio, relaxes the poll and permits light-sleep, so anything that can
         * make the machine enter but not leave costs the entire approach. The
         * historical failure was exactly that — climbing out of the sensor's
         * range froze the published AGL just above cruise_ft, a frozen input
         * yields a zero trend, and an exit conditioned on "descending" could
         * therefore never fire. Keying the exit off the height means any fall
         * below cruise_ft leaves immediately, whatever the trend estimator
         * thinks; the range filter's ceiling handling supplies a height that
         * actually moves (see RANGE_CEILING_NEAR_FT).                          */
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
        rearm_above(c, agl_ft, dt_s, p);

        /* Fire on a genuine downward crossing. We need a valid previous sample
         * (had_prev) so the very first tick after a boot — which seeds every
         * callout armed for in-flight-reboot recovery — does NOT mistake the
         * initial reading for a crossing and blurt out every number at once.
         *
         * The armed bits are snapshotted around the fire so the FULL crossed
         * set can be reported, not just the lowest (spoken) rung: everything
         * fire_descent_callout disarms this tick was genuinely passed, and a
         * consumer pairing a side-effect to a specific height (the gear-check
         * reminder) would otherwise silently lose a rung that a terrain drop
         * or re-acquire snap stepped over.                                    */
        if (had_prev) {
            /*  Lead-compensated crossing (see CALLOUT_LEAD_S): compare where
             *  the aircraft is PREDICTED to be one lead time ahead, so the word
             *  is heard at the rung rather than ~0.2 s x sink below it.       */
            uint32_t mask_before = c->armed_mask;
            fired        = fire_descent_callout(c, prev_agl - c->prev_lead_ft,
                                                agl_ft - lead, p);
            crossed_mask = mask_before & ~c->armed_mask;
        }
    }

    c->state = next;
    c->prev_lead_ft = lead;          /* the anchor for the next step's crossing */

    /* --- Ground-dwell disarm (taxi-back / parked reset) ------------------- */
    /*  Accumulate continuous time spent ON THE GROUND. Note we can't key this
     *  off ST_GROUND: once 'armed', the machine only ever picks CRUISE /
     *  DESCENT / ARMED, so a landed-but-armed box sits in ST_ARMED, never returns
     *  to ST_GROUND. Instead: at/below the ground band counts as parked, full stop.
     *
     *  The test used to ALSO require a level trend — and that made the disarm
     *  unreachable in practice: with the trend computed from centimetre-level
     *  range steps, ordinary taxi jitter (oleo bounce, pavement seams, even the
     *  sensor's own quantisation) tripped the dead-band and zeroed the timer
     *  every few polls, so a landed box stayed armed from touchdown to shutdown.
     *  An aircraft that has been at/below GROUND_BAND_FT for 30 continuous
     *  seconds is on the ground by any definition; a touch-and-go leaves the
     *  band within seconds and keeps its arming, exactly as before. Once parked
     *  past GROUND_RESET_MS we DISARM as if freshly rebooted onto the ground:
     *  clear the arm latch, every armed bit, and the arming dwells, so the next
     *  takeoff is silent until a sustained climb through ARM_FT re-arms.          */
    /*  ...and only when the aircraft got there by tracked motion: a level
     *  reached through sm_reanchor() (a broken track) never counts, however
     *  long it sits there — see sm_ctx_t.park_ok.                            */
    if (agl_ft > GROUND_BAND_FT) {
        c->park_ok = true;
    }
    bool parked = (agl_ft <= GROUND_BAND_FT) && c->park_ok;
    if (parked) {
        c->ground_ms += dt_s * 1000.0f;
        if (c->armed && c->ground_ms >= (float)GROUND_RESET_MS) {
            c->armed      = false;
            c->armed_mask = 0u;
            c->arm_ms     = 0.0f;
            memset(c->rearm_ms, 0, sizeof c->rearm_ms);
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
    out->crossed_mask        = crossed_mask;
    out->poll                = poll_for_state(next);
    out->tone_agl            = agl_ft;
    out->tone_active         = tone_active;
    out->fired_positive_rate = posrate_fire;
    /*  Hand the smoothed vertical rate to the audio engine so the vario blip can
     *  chop the tone by descent rate (see audio.c blip gate).                     */
    out->vert_fps            = c->trend_fps;
}
