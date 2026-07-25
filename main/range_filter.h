/**
 * @file    range_filter.h
 * @brief   Robust runtime range conditioning: decode -> vote -> gate -> smooth.
 *
 * @details PURE module (no ESP-IDF headers, host-testable) that turns the raw
 *          per-drain sample stream from the SF30 into ONE trustworthy range per
 *          poll. It exists because the legacy SF30/C "Distance over Serial"
 *          protocol carries NO checksum: a single corrupted byte can decode to
 *          a perfectly plausible-looking distance anywhere in 0..537 ft, and a
 *          bare EMA happily swallows it. One such sample once armed the whole
 *          callout ladder on a taxiing aircraft and spoke a phantom
 *          "50 40 30 20 10" descent — this module is the fix at the cause.
 *
 *          The pipeline is the textbook robust-statistics stack, applied in
 *          order of increasing memory:
 *
 *            1. VALIDITY GATES (per sample) — reject the lost-signal sentinel
 *               BAND (>= 16000 cm, not just == 16000, so a bit-flipped sentinel
 *               can't masquerade as a 530 ft return), negatives, and anything
 *               beyond the active sensor's physical ceiling. Free and absolute.
 *
 *            2. MEDIAN-OF-DRAIN (within one poll) — every decoded sample of the
 *               drain votes and the MEDIAN wins (Tukey's median smoother). The
 *               old code kept only the LAST pair of the drain, so one trailing
 *               garbage pair outvoted ~57 good samples at the GROUND cadence.
 *               A median is immune to any minority (<50%) of arbitrarily wrong
 *               samples — isolated flips AND short bursts die here.
 *
 *            3. HAMPEL IDENTIFIER (across polls) — the classic moving-window
 *               median/MAD outlier detector (Hampel 1974; Pearson, "Outliers in
 *               process modelling and identification", IEEE Trans. Contr. Syst.
 *               Tech. 2002): a new value is an outlier when it deviates from
 *               the window median by more than K x scaled-MAD plus a physical
 *               slew allowance (an aircraft cannot teleport; the allowance is
 *               bounded so slow polls can't open the gate to arming-sized
 *               spikes). Outliers are HELD (last good value republished) —
 *               never propagated.
 *
 *            4. RE-ACQUISITION — a genuine level step (terrain edge on final,
 *               in-flight power-up) must not be held forever: when
 *               RANGE_REACQUIRE_N consecutive rejected values AGREE with each
 *               other AND the agreeing cluster was backed by at least
 *               RANGE_REACQUIRE_MIN_SAMPLES raw samples, the filter accepts
 *               the new level and re-seeds (the window is pre-filled to
 *               HAMPEL_SEED_N at the new level so the gate is live again on
 *               the very next poll). Random corruption doesn't cluster, and a
 *               short self-consistent burst can't muster the sample mass at
 *               the fast cadences, so garbage can never re-acquire.
 *
 *            5. TIME-CORRECTED EMA — the final smoothing uses
 *               alpha = 1 - exp(-dt / RANGE_EMA_TAU_S), so the filter has ONE
 *               bandwidth in wall-clock terms across every poll cadence
 *               (the old fixed-alpha-per-poll EMA was 30x wider in DESCENT
 *               than GROUND, and a spike's decay swept the callout ladder).
 *
 *          The SF30 legacy 2-byte pair decoder also lives here (moved out of
 *          the hardware TU) so the wire protocol's desync behaviour is finally
 *          unit-testable on the host.
 */
#ifndef LIDARAGL_RANGE_FILTER_H
#define LIDARAGL_RANGE_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "config.h"

/* ===========================================================================
 *  SF30 legacy "Distance over Serial" 2-byte pair decoder (was sf30c.c-static).
 * ---------------------------------------------------------------------------
 *  The stream is pairs: first byte has bit 7 SET and carries the upper 7 bits;
 *  the second byte (bit 7 CLEAR) carries the lower 7 bits.
 *  distance_cm = (high << 7) | low. There is no checksum — validity judgement
 *  is deliberately NOT done here; the range filter's gates own that.
 * ===========================================================================*/

/** Carried pairing state of the 2-byte decoder. */
typedef struct {
    bool    have_high;   /**< A high byte is latched, awaiting its low byte.   */
    uint8_t high;        /**< The latched high byte's 7 value bits.            */
} sf30_ascii_ctx_t;

/** @brief Reset the pair decoder (call after any flush / framing error). */
void sf30_ascii_reset(sf30_ascii_ctx_t *c);

/**
 * @brief Feed one wire byte to the pair decoder.
 *
 * @param c        Carried pairing state.
 * @param b        The received byte.
 * @param[out] out_cm  Decoded distance in cm when a pair completes.
 * @return         True when @p b completed a pair and @p *out_cm is valid.
 */
bool sf30_ascii_feed(sf30_ascii_ctx_t *c, uint8_t b, int *out_cm);

/* ===========================================================================
 *  The robust range filter.
 * ===========================================================================*/

/** Carried state of the robust range filter. One instance per sensor stream. */
typedef struct {
    /* --- Per-drain accumulator (stage 2). Ring so an extra-long drain keeps
     *     the NEWEST samples rather than silently truncating.                 */
    float    drain[RANGE_DRAIN_MEDIAN_N]; /**< Valid samples of this drain (ft).*/
    size_t   drain_n;      /**< Samples stored (capped at the array size).     */
    size_t   drain_head;   /**< Ring write index.                              */
    size_t   drain_lost;   /**< Lost-signal / gated-out samples this drain.    */

    /* --- Hampel window of the last accepted per-drain medians (stage 3).     */
    float    win[HAMPEL_WIN];  /**< Accepted values, ring-ordered.             */
    size_t   win_n;        /**< Fill count (rejection starts once seeded).     */
    size_t   win_head;     /**< Ring write index.                              */

    /* --- Re-acquisition tracker (stage 4).                                   */
    float    pend_mean;    /**< Running mean of the agreeing rejected values.  */
    uint32_t pend_n;       /**< How many consecutive rejects agreed so far.    */
    uint32_t pend_samples; /**< RAW samples backing those agreeing rejects —   */
                           /**< the snap needs sample MASS, not just polls, so */
                           /**< 1-2-sample fast-cadence drains can't fake it.  */

    /* --- Output (stage 5).                                                   */
    float    ema_ft;       /**< Published smoothed range (ft).                 */
    bool     have_out;     /**< False until the first accepted value seeds it. */

    /* --- Validity gate configuration.                                        */
    float    max_range_ft; /**< Active sensor ceiling; beyond +margin = junk.  */
} range_filter_t;

/**
 * @brief Initialise / reset the filter.
 * @param f             Filter state.
 * @param max_range_ft  The active sensor's nominal ceiling (profile value).
 */
void rf_init(range_filter_t *f, float max_range_ft);

/**
 * @brief Update the sensor-ceiling gate (e.g. after profile autodetect).
 */
void rf_set_max_range(range_filter_t *f, float max_range_ft);

/**
 * @brief Push ONE decoded sample (in cm, as it comes off the wire) into the
 *        current drain. Applies the stage-1 validity gates; gated-out samples
 *        are counted so a mostly-lost drain can be judged untrustworthy.
 *
 * @details This is the single place the cm -> feet conversion happens.
 */
void rf_push_cm(range_filter_t *f, float cm);

/**
 * @brief Discard everything accumulated in the CURRENT drain (hardware saw
 *        framing/parity/overflow errors, so none of these bytes can be
 *        trusted). The cross-poll state (window, EMA, re-acquire) is kept.
 */
void rf_drain_abort(range_filter_t *f);

/**
 * @brief Close out the current drain: vote, gate, smooth, publish.
 *
 * @param f                Filter state.
 * @param dt_s             Wall-clock seconds since the previous finalize
 *                         (drives the slew allowance and the EMA bandwidth).
 * @param[out] range_ft    The published range (fresh, or held last-good).
 * @param[out] fresh_valid True when THIS drain produced an accepted ranging
 *                         value; false when the output is a held value
 *                         (empty drain, majority-lost drain, Hampel reject).
 * @return                 True once any output exists at all (mirrors the old
 *                         "have last-good" contract); false before first lock.
 */
bool rf_finalize(range_filter_t *f, float dt_s, float *range_ft, bool *fresh_valid);

#endif /* LIDARAGL_RANGE_FILTER_H */
