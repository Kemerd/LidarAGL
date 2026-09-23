/**
 * @file    range_filter.h
 * @brief   Range tracking: decode -> per-sample Kalman tracker with explicit
 *          track states (SEARCH / TRACK / COAST / LOST).
 *
 * @details PURE module (no ESP-IDF headers, host-testable) that turns the raw
 *          SF30 sample stream into ONE trustworthy range per poll, plus the
 *          range RATE and a track state. It exists because the legacy SF30/C
 *          "Distance over Serial" protocol carries NO checksum: a corrupted byte
 *          can decode to any plausible distance in 0..537 ft, the protective
 *          acrylic window can reflect a "0 ft" return, and the sensor emits
 *          erroneous distances before it reports lost signal. One bad sample
 *          once armed the whole callout ladder on a taxiing aircraft.
 *
 *          ------------------------------------------------------------------
 *          ARCHITECTURE (v1.64 rewrite)
 *          ------------------------------------------------------------------
 *          The previous design was built around per-POLL batches ("drains"):
 *          median-of-drain, a Hampel gate, poll-count re-acquisition, a
 *          coherence test, a lost-signal vote, reachability rules and an
 *          alpha-beta smoother — eleven layers, most of them patched because a
 *          rule correct at one poll cadence broke at another (a 1-2 sample drain
 *          has no median immunity; "12 polls" is 0.3 s at one cadence and 6 s
 *          at another). This version processes every raw SAMPLE with its own
 *          timestamp, so the poll cadence changes latency and nothing else:
 *
 *            1. VALIDITY GATES (per sample): lost-signal sentinel BAND, NaN and
 *               negatives, beyond the sensor ceiling, and the LENS FLOOR
 *               (closer than the ground can physically be). A gated sample is a
 *               NO-RETURN sample — evidence of "nothing seen", never a range.
 *
 *            2. MAIN TRACK: a two-state Kalman filter [range, rate] with a
 *               constant-velocity model and white-noise acceleration (sigma_a),
 *               updated per sample. Each sample must pass an INNOVATION GATE —
 *               nu^2 <= g^2 * S with g = RF_GATE_SIGMA, the PX4 EKF2 practice —
 *               so junk is rejected one sample at a time, statistically, and
 *               the gate widens by itself across a gap as the covariance grows.
 *               The measurement noise is estimated online from the innovations
 *               (bounded), so the gate adapts to the real surface texture.
 *               A steady descent is tracked with no lag at any rate.
 *
 *            3. CANDIDATE TRACK (radar M-of-N track initiation): samples the
 *               main gate rejects build a separate constant-velocity candidate.
 *               It is confirmed only after RF_CONFIRM_SAMPLES members spanning
 *               RF_CONFIRM_MIN_S that make up at least RF_CONFIRM_MEMBER_FRAC
 *               of EVERY sample in its span (returns, junk and no-returns).
 *               This is how the filter locks on first, re-enters range on an
 *               approach, and follows a genuine terrain step — and why
 *               scattered junk can never lock on. A level the aircraft could
 *               not have flown to (a stuck byte pattern, a cliff) needs
 *               RF_BREAK_S of persistence and is reported as a track break.
 *
 *            4. TRACK STATES: TRACK (updating), COAST (no accepted sample for
 *               <= RF_COAST_MAX_S: keep predicting so the tone and callouts keep
 *               moving through a dark patch), LOST (hold, or pin at the sensor
 *               ceiling after flying out the top), SEARCH (never locked yet).
 *               A track re-established from LOST is a NEW track and reports a
 *               track break, so the state machine re-anchors instead of reading
 *               the gap as flown motion (the radar-altimeter convention).
 *
 *          The public API is unchanged from the previous design; only
 *          rf_track_state() is new.
 *
 *          The SF30 legacy 2-byte pair decoder also lives here so the wire
 *          protocol's desync behaviour is unit-testable on the host.
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
 *  The range tracker.
 * ===========================================================================*/

/** Track state of the range filter (two bits in the flight recorder). */
typedef enum {
    RF_SEARCH = 0,  /**< Never locked yet: no output exists.                   */
    RF_TRACK  = 1,  /**< Locked and updating from accepted samples.            */
    RF_COAST  = 2,  /**< Locked, no accepted sample for <= RF_COAST_MAX_S:     */
                    /**< the output keeps following the predicted descent.     */
    RF_LOST   = 3,  /**< Track lost: output held (or pinned at the ceiling).   */
} rf_state_t;

/**
 * @brief A two-state [range, rate] Kalman estimate with its covariance and its
 *        own adaptive measurement-noise estimate.
 *
 * @details Shared by the main track and the candidate so one set of predict /
 *          gate / update routines (range_filter.c) serves both.
 */
typedef struct {
    float x;                /**< Range (ft).                                   */
    float v;                /**< Range rate (ft/s, + means range increasing).  */
    float p00;              /**< Range variance (ft^2).                        */
    float p01;              /**< Range/rate covariance (ft^2/s).               */
    float p11;              /**< Rate variance (ft^2/s^2).                     */
    float r_var;            /**< Measurement noise variance (ft^2), adaptive.  */
} rf_kf_t;

/** Carried state of the range tracker. One instance per sensor stream. */
typedef struct {
    /* --- This poll's drain, IN ARRIVAL ORDER (a ring keeping the newest). ---
     *     Every sample occupies a slot, returns and no-returns alike, so each
     *     one can be given its own timestamp. NaN marks a no-return slot.     */
    float    drain[RANGE_DRAIN_MEDIAN_N]; /**< ft, or NaN for a no-return.     */
    size_t   drain_n;       /**< Slots stored (capped at the ring size).       */
    size_t   drain_head;    /**< Ring write index.                             */
    uint32_t drain_total;   /**< Samples pushed this drain, incl. overflowed.  */

    /* --- Validity gate configuration. -------------------------------------- */
    float    max_range_ft;  /**< Active sensor ceiling; beyond +margin = junk. */
    float    min_range_ft;  /**< Lens floor: closer is not ground (0 = off).   */

    /* --- Main track: two-state Kalman filter [x = range, v = rate]. --------
     *     The state is kept AT THE TIME OF THE LAST ACCEPTED SAMPLE; predictions
     *     to "now" are computed on demand, never committed, so a rejected or
     *     missing sample costs nothing and the covariance only ever grows by
     *     the process noise of time that genuinely passed.                   */
    rf_state_t state;       /**< SEARCH / TRACK / COAST / LOST.                */
    rf_kf_t  trk;           /**< The main track's estimate.                    */
    float    since_accept_s;/**< Time since the last accepted sample (s).      */
    uint32_t unacc_void;    /**< No-return samples since the last accept.      */
    uint32_t unacc_real;    /**< Real returns since the last accept (rejected). */
    bool     lost_blind;    /**< The track was LOST to no-returns (blindness), */
                            /**< not to rejected returns (see rf_break_reentry).*/

    /* --- Sample-and-hold repeat detection (see RF_REPEAT_HOLD_S). ----------- */
    float    last_z;        /**< Previous raw sample (ft), NaN after a no-return.*/
    float    repeat_s;      /**< How long the current value has repeated (s).  */

    /* --- Published output. -------------------------------------------------- */
    bool     have_out;      /**< False until the first lock.                   */
    float    out_ft;        /**< Published range (ft).                         */
    float    out_rate_fps;  /**< Published rate (0 while LOST / SEARCH).       */
    bool     track_break;   /**< True for the ONE finalize that switched onto a */
                            /**< discontinuous level (re-anchor, don't walk).  */
    bool     break_reentry; /**< ...and that break is a flyable DESCENDING     */
                            /**< re-entry (see rf_break_reentry()).            */

    /* --- Candidate track (M-of-N initiation), a Kalman track of its own. --- */
    rf_kf_t  cand;          /**< Candidate estimate, at its last member.       */
    uint32_t c_n;           /**< Members so far (0 = no candidate).            */
    uint32_t c_miss;        /**< Real samples since its first member that did  */
                            /**< not join it (junk, or main-track accepts).    */
    uint32_t c_void;        /**< No-return samples since its first member.     */
    float    c_age_s;       /**< Time since its last member (s).               */
    float    c_span_s;      /**< Time since its first member (s).              */

    /* --- Verdicts. ------------------------------------------------------------ */
    bool     tracking;      /**< The sensor sees something (see rf_tracking).  */
    uint32_t notrack_polls; /**< Consecutive drains with nothing usable.       */
    uint32_t lost_bits;     /**< Last RANGE_LOST_VOTE_SAMPLES samples, newest  */
                            /**< in bit 0: 1 = no return, 0 = a return.        */
    uint32_t lost_bits_n;   /**< How many samples the history holds.           */
    bool     stream_real;   /**< Last finalize: most recent samples are real.  */
    bool     above_ceiling; /**< LOST because we flew out the TOP of range.     */
    uint32_t ceiling_polls; /**< Consecutive drains supporting that verdict.   */
} range_filter_t;

/**
 * @brief Initialise a filter.
 * @param f             Filter to initialise (fully zeroed first).
 * @param max_range_ft  The active sensor's nominal ceiling (profile value).
 */
void rf_init(range_filter_t *f, float max_range_ft);

/** @brief Update the ceiling gate (profile resolved after init). */
void rf_set_max_range(range_filter_t *f, float max_range_ft);

/**
 * @brief Set the physical floor below which a reading cannot be the ground.
 *
 * @details The laser looks through the housing's acrylic lens. When the ground
 *          return is weak or absent, the lens itself (a few centimetres away)
 *          can reflect enough to be reported as a distance — a "0 ft" reading
 *          at any altitude. Nothing real can be closer to the sensor than the
 *          ground beneath the parked wheels, minus strut compression and flare
 *          pitch, so readings under this floor are NO RETURN samples.
 *
 * @param f             Filter.
 * @param min_range_ft  Floor in feet of RANGE (0 disables). Negative -> 0.
 */
void rf_set_min_range(range_filter_t *f, float min_range_ft);

/**
 * @brief Add one decoded wire sample (cm) to the current drain.
 * @details Validity gates are applied here; a gated sample occupies its time
 *          slot as a no-return, so the per-sample timestamps stay correct.
 */
void rf_push_cm(range_filter_t *f, float cm);

/**
 * @brief Discard the current drain (UART framing/parity/overflow reported).
 * @details The whole drain is dropped; its interval still elapses at the next
 *          rf_finalize(), so the tracker's clocks stay honest.
 */
void rf_drain_abort(range_filter_t *f);

/**
 * @brief Close the drain: process every sample in order, update the track
 *        state, and publish.
 *
 * @param f            Filter.
 * @param dt_s         Wall-clock time since the previous finalize; the drain's
 *                     samples are spread uniformly across it.
 * @param[out] range_ft     Published range (ft) — valid when the return is true.
 * @param[out] fresh_valid  True when this drain produced accepted data (or a
 *                          confirmed new track); false while coasting/holding.
 * @return True once any output exists (after the first lock).
 */
bool rf_finalize(range_filter_t *f, float dt_s, float *range_ft, bool *fresh_valid);

/**
 * @brief Whether the last finalize switched onto a DISCONTINUOUS level.
 *
 * @details True for exactly the finalize that re-established a track from
 *          LOST, or confirmed a downward level jump too large to have been
 *          flown. The value is trustworthy; the jump is not motion, so the
 *          state machine must re-anchor (see sm_reanchor) instead of speaking
 *          every rung across the gap.
 */
bool rf_track_broken(const range_filter_t *f);

/**
 * @brief Whether the current track break is a genuine RE-ENTRY: a DESCENDING
 *        track re-established after the old one was lost, at a level the
 *        aircraft could physically have flown to from its last tracked
 *        position in the time that passed.
 *
 * @details Only then were the callout rungs in the gap genuinely passed while
 *          the laser was blind, so only then may the state machine apply the
 *          late-rung window (sm_reanchor's @p reentry). A stationary break (the
 *          shape of a stuck byte pattern), a break out of a live track, or a
 *          jump too large for the elapsed time all re-anchor silently.
 *          Meaningful only while rf_track_broken() is true.
 */
bool rf_break_reentry(const range_filter_t *f);

/**
 * @brief Whether the SENSOR currently has any picture of the ground.
 *
 * @details False only after RANGE_NOTRACK_POLLS consecutive drains produced
 *          nothing usable; true again on the FIRST drain that does. A stream of
 *          junk with the occasional real return counts as tracking, so the box
 *          stays awake through it. Drives the power/poll policy.
 */
bool rf_tracking(const range_filter_t *f);

/**
 * @brief Whether a lost track is waiting to be re-established while real
 *        returns are arriving — the ground coming back into view.
 * @details Drives the fast poll in sm_poll_period_ms() so the re-entry is
 *          confirmed in a fraction of a second.
 */
bool rf_reacquiring(const range_filter_t *f);

/**
 * @brief The tracked range rate in ft/s (+ climbing / - descending).
 * @details Lag-free at a steady sink; 0 while LOST or SEARCH. Drives the
 *          callout lead (sm_ctx_t.lead_rate_fps).
 */
float rf_rate_fps(const range_filter_t *f);

/** @brief The current track state (SEARCH / TRACK / COAST / LOST). */
rf_state_t rf_track_state(const range_filter_t *f);

#endif /* LIDARAGL_RANGE_FILTER_H */
