/**
 * @file    range_filter.c
 * @brief   Implementation of the robust runtime range pipeline (see header).
 *
 * @details PURE: C standard library + config.h's pure region + robust.h (for
 *          the shared median). Everything here is deterministic in its inputs,
 *          allocation-free, and exercised by test/test_range_filter.c — the
 *          layer that previously admitted the taxi-incident garbage lived in a
 *          hardware-only translation unit with zero coverage.
 */

#include "range_filter.h"
#include "robust.h"     /* median_inplace — one median implementation, reused  */

#include <math.h>
#include <string.h>

/* ===========================================================================
 *  SF30 legacy 2-byte pair decoder.
 * ===========================================================================*/

void sf30_ascii_reset(sf30_ascii_ctx_t *c)
{
    c->have_high = false;
    c->high      = 0;
}

bool sf30_ascii_feed(sf30_ascii_ctx_t *c, uint8_t b, int *out_cm)
{
    /* Bit 7 set -> this is a HIGH byte: latch it (a repeated high byte simply
     * re-latches, which is the stream's own resync mechanism).                 */
    if (b & 0x80) {
        c->high      = (uint8_t)(b & 0x7F);
        c->have_high = true;
        return false;
    }
    /* A LOW byte with no preceding high byte is an orphan (we joined the
     * stream mid-pair, or its high byte was lost) — drop it and stay synced.   */
    if (!c->have_high) {
        return false;
    }
    c->have_high = false;
    *out_cm = ((int)c->high << 7) | (int)(b & 0x7F);
    return true;
}

/* ===========================================================================
 *  Small internal helpers.
 * ===========================================================================*/

/*  cm -> ft. The SINGLE place this conversion happens (moved here from sf30c.c
 *  so the whole numeric path is host-testable).                                */
static inline float cm_to_ft(float cm)
{
    return cm * CM_TO_FT;
}

/*  Push one accepted value into the Hampel window ring. */
static void win_push(range_filter_t *f, float v)
{
    f->win[f->win_head] = v;
    f->win_head = (f->win_head + 1) % HAMPEL_WIN;
    if (f->win_n < HAMPEL_WIN) {
        ++f->win_n;
    }
}

/*  Median of the Hampel window (copies; the ring itself is left untouched). */
static float win_median(const range_filter_t *f)
{
    float scratch[HAMPEL_WIN];
    memcpy(scratch, f->win, f->win_n * sizeof(float));
    return median_inplace(scratch, f->win_n);
}

/*  Scaled MAD of the Hampel window about @p med. The 1.4826 factor makes the
 *  MAD a consistent estimator of a Gaussian sigma (standard robust practice). */
static float win_mad_sigma(const range_filter_t *f, float med)
{
    float devs[HAMPEL_WIN];
    for (size_t i = 0; i < f->win_n; ++i) {
        devs[i] = fabsf(f->win[i] - med);
    }
    return 1.4826f * median_inplace(devs, f->win_n);
}

/*  Accept a value: seed/advance the EMA with the time-corrected alpha and
 *  record it in the Hampel window. Clears any pending re-acquisition.          */
static void accept_value(range_filter_t *f, float v, float dt_s)
{
    if (!f->have_out) {
        f->ema_ft   = v;      /* first lock: seed, don't lag */
        f->have_out = true;
    } else {
        /* alpha = 1 - exp(-dt/tau): one wall-clock bandwidth at EVERY poll
         * cadence, unlike the retired fixed-alpha-per-poll EMA.               */
        float alpha = 1.0f - expf(-dt_s / RANGE_EMA_TAU_S);
        f->ema_ft += alpha * (v - f->ema_ft);
    }
    win_push(f, v);
    f->pend_n       = 0;
    f->pend_samples = 0;
}

/* ===========================================================================
 *  Public API.
 * ===========================================================================*/

void rf_init(range_filter_t *f, float max_range_ft)
{
    memset(f, 0, sizeof *f);
    f->max_range_ft = max_range_ft;
}

void rf_set_max_range(range_filter_t *f, float max_range_ft)
{
    f->max_range_ft = max_range_ft;
}

bool rf_track_broken(const range_filter_t *f)
{
    return f->track_break;
}

void rf_push_cm(range_filter_t *f, float cm)
{
    /* --- Stage 1: absolute validity gates ---------------------------------- */
    /*  The lost-signal check is a BAND, not an equality: the sensor's 16000 cm
     *  sentinel is one bit-flip away from 16001..16383 cm (525..537 ft), all of
     *  which are beyond any real return and used to pass as "valid".           */
    if (!(cm >= 0.0f) || cm >= (float)SF30_LOST_SIGNAL_CM) {
        ++f->drain_lost;                 /* NaN, negative, or sentinel band     */
        return;
    }
    float ft = cm_to_ft(cm);
    /*  Beyond the fitted sensor's ceiling (+margin) is physically impossible —
     *  the SF30/C cannot see 400 ft, so such a value is corruption by
     *  definition. (Inert for the SF30/D, whose ceiling exceeds the wire max.) */
    if (f->max_range_ft > 0.0f && ft > f->max_range_ft + RANGE_MAX_MARGIN_FT) {
        ++f->drain_lost;
        return;
    }

    /* --- Accumulate into the drain ring (newest samples win on overflow) --- */
    f->drain[f->drain_head] = ft;
    f->drain_head = (f->drain_head + 1) % RANGE_DRAIN_MEDIAN_N;
    if (f->drain_n < RANGE_DRAIN_MEDIAN_N) {
        ++f->drain_n;
    }
}

void rf_drain_abort(range_filter_t *f)
{
    /* Hardware said these bytes are untrustworthy (framing/parity/overflow):
     * throw the WHOLE drain away. Cross-poll state is deliberately kept — the
     * next clean drain continues from the last good value.                     */
    f->drain_n    = 0;
    f->drain_head = 0;
    f->drain_lost = 0;
}

bool rf_finalize(range_filter_t *f, float dt_s, float *range_ft, bool *fresh_valid)
{
    size_t n_valid = f->drain_n;
    size_t n_lost  = f->drain_lost;

    /* The drain accumulator is consumed by this call no matter what. */
    float scratch[RANGE_DRAIN_MEDIAN_N];
    memcpy(scratch, f->drain, n_valid * sizeof(float));
    f->drain_n    = 0;
    f->drain_head = 0;
    f->drain_lost = 0;

    /* Guard the dt used for the slew allowance / EMA against nonsense. */
    if (!(dt_s > 0.0f)) {
        dt_s = 0.001f;
    }

    /*  The track-break flag describes THIS finalize only; clear it up front so
     *  it is never left set from a previous call.                              */
    f->track_break = false;

    /* --- No usable fresh data: out-of-range ABOVE, or hold last-good -------- */
    /*  A drain that is EMPTY (sensor silent) or MAJORITY lost-signal is not
     *  trusted even if a few stray "returns" survived the gates — when the
     *  sensor itself says "no return" 40 times in a row, the two samples that
     *  disagree are more likely corruption than ground.
     *
     *  But "no return" has two opposite physical causes, and holding is only
     *  right for ONE of them. If the last GOOD reading had already climbed to
     *  within RANGE_CEILING_NEAR_FT of the sensor's ceiling, the aircraft flew
     *  out the TOP of the sensor's range — the reading walked up to the limit
     *  and off the end of it, which a non-reflective surface cannot imitate
     *  (that begins from wherever the aircraft happens to be, typically low).
     *  In that case the honest output is the CEILING, not a frozen mid-air
     *  number: it keeps the published range monotonically consistent with the
     *  climb, and — critically — it means the descent back through the ceiling
     *  is a genuine DOWNWARD movement the state machine can see and the
     *  callout ladder can edge-trigger on. Freezing instead produced a
     *  zero-trend "level at 318 ft" that pinned the box in ST_CRUISE for the
     *  rest of the flight (see RANGE_CEILING_NEAR_FT in config.h).
     *
     *  The verdict needs RANGE_CEILING_CONFIRM_POLLS consecutive lost drains so
     *  one ragged drain in the noisy top of the range cannot flip it, and it is
     *  reported as fresh_valid == false either way: this is an INFERENCE about
     *  where we are, not a measurement, so the tone's stale-data mute and the
     *  no-data annunciation upstream still behave exactly as before.           */
    if (n_valid == 0 || n_lost > n_valid) {
        bool near_ceiling = f->have_out && f->max_range_ft > 0.0f &&
                            f->ema_ft >= f->max_range_ft - RANGE_CEILING_NEAR_FT;

        if (near_ceiling && n_lost > 0) {
            /* Only a drain that actually SAW lost-signal returns is evidence of
             * flying out of range; a wholly EMPTY drain (n_lost == 0) means the
             * sensor said nothing at all — dead, unplugged, or wrong baud — and
             * must never be read as an altitude claim.                          */
            if (f->ceiling_polls < UINT32_MAX) {
                ++f->ceiling_polls;
            }
            if (f->ceiling_polls >= RANGE_CEILING_CONFIRM_POLLS) {
                f->above_ceiling = true;
            }
        } else {
            f->ceiling_polls = 0;
            f->above_ceiling = false;
        }

        if (f->above_ceiling) {
            /* Pin the output AT the ceiling and drag the Hampel window with it,
             * so the descent back into range is gated against "we were at the
             * ceiling" rather than against a stale mid-air anchor. Without the
             * re-anchor the first in-range reading would look like a huge
             * outlier and burn RANGE_REACQUIRE_N polls before it was believed —
             * several hundred feet of descent at the CRUISE cadence.            */
            f->ema_ft = f->max_range_ft;
            f->win_n    = 0;
            f->win_head = 0;
            while (f->win_n < HAMPEL_SEED_N) {
                win_push(f, f->max_range_ft);
            }
            f->pend_n       = 0;
            f->pend_samples = 0;
        }

        *fresh_valid = false;
        *range_ft    = f->ema_ft;
        return f->have_out;
    }

    /* A usable drain arrived: we are back inside the sensor's range. */
    f->ceiling_polls = 0;
    f->above_ceiling = false;

    /* --- Stage 2: the drain votes; the median wins -------------------------- */
    float med = median_inplace(scratch, n_valid);

    /* --- Stage 3: Hampel gate (once the window is seeded) ------------------- */
    if (f->win_n >= HAMPEL_SEED_N) {
        float wmed  = win_median(f);
        float sigma = win_mad_sigma(f, wmed);
        if (sigma < HAMPEL_MAD_FLOOR_FT) {
            sigma = HAMPEL_MAD_FLOOR_FT;    /* flat window -> keep test sane    */
        }
        /*  Threshold = statistical band + bounded physical motion allowance.
         *  The cap is the essential part: at the 750 ms GROUND poll an
         *  uncapped 60 ft/s * 4.5 * 0.75 s = 202 ft allowance would re-open
         *  the exact arming-spike hole this filter exists to close.            */
        float slew = RANGE_MAX_SLEW_FPS * dt_s * RANGE_SLEW_HORIZON;
        if (slew > RANGE_GATE_CAP_FT) {
            slew = RANGE_GATE_CAP_FT;
        }
        float thr = HAMPEL_K * sigma + slew;
        float dev = fabsf(med - wmed);

        if (dev > thr) {
            /* --- Stage 4: outlier. Hold, and track re-acquisition. ---------- */
            if (f->pend_n > 0 &&
                fabsf(med - f->pend_mean) <= RANGE_REACQUIRE_BAND_FT) {
                /* Consecutive reject AGREEING with the previous ones: running
                 * mean so the accepted level is the cluster's centre. The raw
                 * sample count rides along — see the MIN_SAMPLES test below.   */
                f->pend_mean += (med - f->pend_mean) / (float)(f->pend_n + 1u);
                ++f->pend_n;
                f->pend_samples += (uint32_t)n_valid;
            } else {
                /* First reject, or it disagrees with the pending cluster:
                 * start a fresh cluster on this value.                         */
                f->pend_mean    = med;
                f->pend_n       = 1;
                f->pend_samples = (uint32_t)n_valid;
            }

            /*  How far this snap would MOVE the published range, and how far the
             *  aircraft could physically have moved while the cluster was being
             *  collected. A re-acquire is an admission that our anchor is wrong,
             *  so it deliberately bypasses the Hampel gate — but "the anchor is
             *  wrong" must not become "any self-consistent garbage may teleport
             *  us anywhere". A stuck byte pattern repeats cleanly and therefore
             *  agrees with itself perfectly, satisfying the poll count and the
             *  sample mass for free; the only thing it cannot fake is being
             *  physically reachable in the time it took to observe.
             *
             *  Concretely, this is what a 4-poll stuck burst on final used to
             *  do: at 186 ft AGL it re-acquired a garbage level of ~76 ft — a
             *  110 ft downward teleport in ~100 ms — and the state machine
             *  faithfully spoke "one hundred" through it. That is the taxi
             *  phantom's exact failure class, except in the air on approach,
             *  where a spurious low callout is at its most dangerous. Requiring
             *  the jump to be reachable at RANGE_MAX_SLEW_FPS costs a genuine
             *  terrain step nothing (real ground moves at aircraft speeds) and
             *  makes a fabricated one need corruption that lasts long enough to
             *  be physically plausible — by which point it is indistinguishable
             *  from, and as slow as, a real level change.
             *
             *  NOTE the allowance is bounded by RANGE_GATE_CAP_FT and does NOT
             *  grow without limit with the cluster's duration. An unbounded
             *  version was worse than useless: simply waiting long enough
             *  bought an arbitrarily large teleport, so an out-of-range stretch
             *  that eventually settled on a low erroneous cluster snapped
             *  335 ft -> 12 ft in a single poll and spoke "twenty" at altitude.
             *  Past the cap the jump is not judged reachable at all — it is a
             *  BROKEN TRACK, handled below, which is a different thing from a
             *  measurement and must be reported as such.                        */
            /*  Direction matters. An UPWARD jump (the new level reads FARTHER
             *  than our anchor) cannot fabricate a low callout — the callout
             *  ladder only ever fires on a DOWNWARD crossing, so adopting a
             *  higher level can at worst delay a number, never invent one. The
             *  in-flight power-up case this filter must support is exactly that
             *  shape: the box wakes anchored near the ground and the true range
             *  is hundreds of feet farther away. Those stay governed by the
             *  poll-count and sample-mass rules alone.
             *
             *  A DOWNWARD jump is the dangerous direction, and it is the one a
             *  stuck pattern exploited: adopting a lower level walks the ladder
             *  and speaks numbers the aircraft never reached. So only downward
             *  snaps must additionally be physically reachable.                 */
            float jump_ft    = f->ema_ft - f->pend_mean;   /* >0 == downward     */
            float cluster_s  = dt_s * (float)f->pend_n;
            float reach_ft   = RANGE_MAX_SLEW_FPS * cluster_s +
                               RANGE_REACQUIRE_JUMP_SLACK_FT;
            /*  Bounded exactly like the Hampel slew allowance, and for the same
             *  reason: an allowance that grows with elapsed time lets patience
             *  substitute for evidence.                                         */
            if (reach_ft > RANGE_GATE_CAP_FT) {
                reach_ft = RANGE_GATE_CAP_FT;
            }
            bool  reachable  = !f->have_out ||
                               jump_ft <= 0.0f ||          /* upward: unrestricted */
                               jump_ft <= reach_ft;

            /*  A downward jump too large to have been flown is still something
             *  we must eventually accept — refusing forever would strand the
             *  filter on a stale anchor after a genuine discontinuity (an
             *  in-flight power-up, a recovery from a long out-of-range stretch,
             *  a real cliff edge). We accept it, but we do NOT pretend the
             *  aircraft flew there: the track is BROKEN and re-established, and
             *  rf_track_broken() tells the consumer so. The callout ladder
             *  re-anchors on the new level instead of speaking the rungs in
             *  between — which is exactly the phantom this prevents.
             *
             *  The extra evidence required scales with how implausible the jump
             *  is, so ordinary noise can never trigger a break.                 */
            bool  breaks_track = !reachable &&
                                 f->pend_n >= (uint32_t)RANGE_TRACK_BREAK_POLLS;

            if (f->pend_n >= (uint32_t)RANGE_REACQUIRE_N &&
                f->pend_samples >= (uint32_t)RANGE_REACQUIRE_MIN_SAMPLES &&
                (reachable || breaks_track)) {
                /*  Annunciate a discontinuous snap so downstream can re-anchor
                 *  rather than interpret the gap as flown motion.               */
                f->track_break = breaks_track;
                /* A REAL level step: N consecutive rejects agreed AND enough
                 *  raw samples backed them. The second condition is what stops
                 *  a fast-cadence false snap: a DESCENT drain is only ~2 raw
                 *  samples, so its "median" has no minority immunity and poll
                 *  count alone let ~75 ms of self-consistent corruption (a
                 *  stuck, cleanly-framed byte pattern) re-acquire. Sample MASS
                 *  makes the evidence cadence-independent: a GROUND drain
                 *  (~58 samples) satisfies it in one poll as before, while at
                 *  DESCENT a genuine step needs ~4 polls (~100 ms — still
                 *  invisible in the flare).                                    */
                f->win_n    = 0;
                f->win_head = 0;
                float level = f->pend_mean;
                f->have_out = false;         /* seed the EMA at the new level  */
                accept_value(f, level, dt_s);
                /* Pre-fill the Hampel window to HAMPEL_SEED_N with the new
                 *  level so the gate is LIVE again from the very next poll.
                 *  accept_value() pushed one copy; left there, the seed phase
                 *  (win_n < HAMPEL_SEED_N) would bypass the gate for the next
                 *  TWO polls — and at the fast cadences a drain is 1-2 raw
                 *  samples, so a single corrupted pair could ride ungated into
                 *  the EMA exactly when corruption is most likely still in
                 *  progress. Identical seeds give MAD == 0, so the
                 *  HAMPEL_MAD_FLOOR_FT floor keeps the fresh gate meaningful,
                 *  anchored at the newly accepted level.                       */
                while (f->win_n < HAMPEL_SEED_N) {
                    win_push(f, level);
                }
                *fresh_valid = true;
                *range_ft    = f->ema_ft;
                return true;
            }

            /* Rejected and not (yet) re-acquired: republish the held value.   */
            *fresh_valid = false;
            *range_ft    = f->ema_ft;
            return f->have_out;
        }
    }

    /* --- Stage 5: accepted -> time-corrected EMA ---------------------------- */
    accept_value(f, med, dt_s);
    *fresh_valid = true;
    *range_ft    = f->ema_ft;
    return true;
}
