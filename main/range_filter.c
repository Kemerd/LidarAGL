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
    f->pend_n = 0;
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

    /* --- No usable fresh data: hold last-good ------------------------------ */
    /*  A drain that is EMPTY (sensor silent) or MAJORITY lost-signal is not
     *  trusted even if a few stray "returns" survived the gates — when the
     *  sensor itself says "no return" 40 times in a row, the two samples that
     *  disagree are more likely corruption than ground.                        */
    if (n_valid == 0 || n_lost > n_valid) {
        *fresh_valid = false;
        *range_ft    = f->ema_ft;
        return f->have_out;
    }

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
                 * mean so the accepted level is the cluster's centre.          */
                f->pend_mean += (med - f->pend_mean) / (float)(f->pend_n + 1u);
                ++f->pend_n;
            } else {
                /* First reject, or it disagrees with the pending cluster:
                 * start a fresh cluster on this value.                         */
                f->pend_mean = med;
                f->pend_n    = 1;
            }

            if (f->pend_n >= (uint32_t)RANGE_REACQUIRE_N) {
                /* A REAL level step: N consecutive rejects agreed. Snap to the
                 * cluster (decisive — three polls of consistent evidence) and
                 * re-seed the window on the new level.                         */
                f->win_n    = 0;
                f->win_head = 0;
                float level = f->pend_mean;
                f->have_out = false;         /* seed the EMA at the new level  */
                accept_value(f, level, dt_s);
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
