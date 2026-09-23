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
 *               RANGE_REACQUIRE_N consecutive rejected values form a consistent
 *               constant-velocity TRACK (any descent rate the airframe can fly)
 *               AND that track was backed by at least
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
    float    pend_mean;    /**< Running mean of the agreeing rejected values — */
                           /**< the level a re-acquire snaps to (its lag keeps */
                           /**< each snap small enough to cross rungs singly).  */
    float    pend_vel_fps; /**< Velocity of the candidate track (ft/s), from  */
                           /**< its last two members: the next member must    */
                           /**< land where it PREDICTS (any descent rate).     */
    float    pend_last;    /**< Most recent member of the candidate track.    */
                           /**< Members must follow a consistent constant-    */
                           /**< velocity track, not sit near their own mean — */
                           /**< see the stage-4 note in range_filter.c.       */
    uint32_t pend_n;       /**< How many consecutive rejects agreed so far.    */
    uint32_t pend_samples; /**< RAW samples backing those agreeing rejects —   */
                           /**< the snap needs sample MASS, not just polls, so */
                           /**< 1-2-sample fast-cadence drains can't fake it.  */

    /* --- Output (stage 5).                                                   */
    float    ema_ft;       /**< Published smoothed range (ft).                 */
    bool     have_out;     /**< False until the first accepted value seeds it. */

    /* --- Validity gate configuration.                                        */
    float    max_range_ft; /**< Active sensor ceiling; beyond +margin = junk.  */
    float    min_range_ft; /**< Physical floor: anything CLOSER is not ground  */
                           /**< (lens reflection) and counts as no return.    */
                           /**< 0 until the ground reference is known.        */

    /* --- Out-of-range (above the ceiling) verdict, stage 1b.                 *
     *  A lost-signal drain is ambiguous on its own: the SF30 reports "no
     *  return" both when the target is BEYOND its range and when the surface
     *  simply does not reflect (water, a dark wet runway). The two demand
     *  OPPOSITE handling — the first means "we are higher than the sensor can
     *  see", the second means "we know nothing" — so the filter records which
     *  case the evidence supports rather than collapsing both into a hold.
     *  See rf_finalize()'s ceiling test for how the verdict is reached.        */
    uint32_t nouse_run;       /**< Consecutive drains with nothing usable.      */
    bool     stream_real;     /**< Last finalize: the sliding lost-signal vote */
                              /**< said most recent samples were REAL returns. */
    bool     track_lost;      /**< Output is a hold/pin INFERENCE: the next     */
                              /**< level must be confirmed by re-acquire, never */
                              /**< accepted on a single drain by the gate.      */
    uint32_t lost_bits;       /**< Last RANGE_LOST_VOTE_SAMPLES raw samples,    */
                              /**< newest in bit 0: 1 = no return, 0 = usable.  */
    uint32_t lost_bits_n;     /**< How many samples the history holds so far.   */
    bool     above_ceiling;   /**< Last drain read as out-of-range ABOVE.       */
    uint32_t ceiling_polls;   /**< Consecutive drains supporting that verdict.  */

    /* --- Track-break annunciation (stage 4b).                                *
     *  Set for exactly ONE finalize when the filter RE-ACQUIRES onto a level
     *  that is discontinuous with the one it was tracking. The published value
     *  is trustworthy again from that moment, but the JUMP itself is not a
     *  flown trajectory — nothing crossed the intervening heights. Consumers
     *  that edge-trigger on movement (the callout ladder) must treat it as a
     *  teleport and re-anchor rather than walking the rungs between the two
     *  levels. See rf_finalize() and the logic task's handling.               */
    bool     track_break;

    /* --- Tracking verdict (stage 1c): "is the sensor telling us anything?"    *
     *  The single, directly-observed answer to whether this box has a usable
     *  picture of the ground — as opposed to inferring it from ALTITUDE, which
     *  is a guess about the sensor dressed up as a fact about the aircraft.
     *
     *  ASYMMETRIC by design: losing tracking requires sustained evidence
     *  (RANGE_NOTRACK_POLLS consecutive useless drains), while REGAINING it
     *  takes a single usable drain. That asymmetry is the whole safety
     *  argument — we may be slow to conclude the sensor is blind, but we are
     *  instant to notice it can see again, because the cost of those two
     *  mistakes is wildly different on an approach.                            */
    bool     tracking;        /**< False once the sensor has gone useless.      */
    uint32_t notrack_polls;   /**< Consecutive drains with nothing usable.      */
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
 * @brief Set the physical floor below which a reading cannot be the ground.
 *
 * @details The laser looks through the housing's acrylic lens. When the ground
 *          return is weak or absent, the lens itself (a few centimetres away)
 *          can reflect enough to be reported as a distance — a "0 ft" reading
 *          at any altitude. But nothing real can be closer to the sensor than
 *          the ground beneath the parked wheels, minus strut compression and
 *          flare pitch. Readings under this floor are therefore classified as
 *          NO RETURN (counted with the lost-signal sentinels), never as an
 *          altitude, so they can neither be accepted nor cluster into a
 *          re-acquired "we are on the ground" level in flight.
 *
 * @param f             Filter.
 * @param min_range_ft  Floor in feet of RANGE (0 disables). Negative -> 0.
 */
void rf_set_min_range(range_filter_t *f, float min_range_ft);

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

/**
 * @brief Did the LAST rf_finalize() re-acquire onto a DISCONTINUOUS level?
 *
 * @details True for exactly one finalize after the filter breaks track and
 *          snaps to a new level (a genuine terrain step, an in-flight power-up,
 *          or a recovery from a stretch of untrustworthy data). The published
 *          range is trustworthy again — but the aircraft did NOT fly through
 *          the heights between the old level and the new one, so a consumer
 *          that edge-triggers on downward movement must NOT walk the callout
 *          rungs across the gap. It should re-anchor to the new level instead.
 *
 *          This mirrors the "break track / re-acquire" distinction a radar
 *          altimeter draws: a track that has been broken and re-established is
 *          a NEW track, not a continuation of the old one, and the discontinuity
 *          must never be interpreted as motion.
 *
 * @param f  Filter state.
 * @return   True if the last finalize was a discontinuous re-acquisition.
 */
bool rf_track_broken(const range_filter_t *f);

/**
 * @brief Is the sensor currently giving us a usable picture of the ground?
 *
 * @details This is the box's power/latency signal, and it is deliberately a
 *          statement about the SENSOR, not about the aircraft. The older policy
 *          slept whenever the ALTITUDE was high (ST_CRUISE), which is an
 *          inference — "high means the sensor probably can't see" — and it was
 *          wrong in both directions: it slept while the sensor was still
 *          tracking fine just under its ceiling, and (worse) it could not tell
 *          "high" from "sensor blind at any altitude".
 *
 *          Tracking is FALSE only after RANGE_NOTRACK_POLLS consecutive drains
 *          produced nothing usable, and returns TRUE on the FIRST drain that
 *          does. So a stream that is mostly junk with an occasional real return
 *          — the ragged 350/400/325/375 mess near the ceiling — counts as
 *          TRACKING, and the box stays awake and responsive through it. Only a
 *          sensor that has genuinely gone quiet lets us relax.
 *
 * @param f  Filter state.
 * @return   True while the sensor is usable; false once it has gone dark.
 */
bool rf_tracking(const range_filter_t *f);

/**
 * @brief Whether the filter has LOST its track and is waiting to confirm a new
 *        one (its output is a hold/pin inference, not a measurement).
 *
 * @details While this is true, returns that do arrive are the ground coming
 *          back into view — typically the descent into range on an approach —
 *          and they must be confirmed QUICKLY: at 4000 fpm the aircraft falls
 *          ~33 ft per 500 ms poll. The poll policy (sm_poll_period_ms) uses it
 *          to poll fast in exactly that window. Safe because nothing is
 *          accepted on a single drain while the track is lost: the sliding
 *          lost-signal vote and the constant-velocity track gate must both pass.
 *
 *          "Returns are arriving" means the sliding RANGE_LOST_VOTE_SAMPLES
 *          vote says MOST recent samples are real — not merely that one sample
 *          got through. A sensor spraying junk while blind (a garbage-level
 *          stream still passes ~1/3 of samples as plausible distances) would
 *          otherwise hold the box at the fast rate, where 1-4 sample drains
 *          give the junk far more chances to line up.
 *
 * @param f  Filter state.
 * @return   True while a lost track awaits confirmation and the stream is real.
 */
bool rf_reacquiring(const range_filter_t *f);

#endif /* LIDARAGL_RANGE_FILTER_H */
