/**
 * @file    state_machine.h
 * @brief   The behaviour brain: states, arming, callouts, hysteresis, polling.
 *
 * @details This is a PURE module. sm_step() is a deterministic function of its
 *          inputs (current AGL, elapsed time, active profile) and the carried
 *          context — it performs no I/O, calls no clock, and includes no
 *          ESP-IDF headers. That makes the entire callout/arming/hysteresis
 *          behaviour exhaustively unit-testable on the host, which is exactly
 *          where we want the box's most safety-relevant logic exercised.
 *
 *          Key behaviours (spec §6.3):
 *            - NO takeoff callouts: callouts stay disarmed until the aircraft
 *              has climbed through ARM_FT, guaranteeing a silent climb-out.
 *            - Each callout is an edge-trigger on the way DOWN, fired once.
 *            - Hysteresis: a callout re-arms only after climbing back above
 *              callout_height + REARM_MARGIN_FT, so hovering near a threshold
 *              doesn't machine-gun and a go-around re-enables the callouts.
 *            - CRUISE (low-power) is entered at/above the PROFILE's cruise_ft.
 *            - Climb/descent direction comes from the smoothed range trend with
 *              a dead-band, since there is no IMU or baro.
 */
#ifndef LIDARAGL_STATE_MACHINE_H
#define LIDARAGL_STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_profile.h"

/** Power-aware behaviour states (spec §6.2). */
typedef enum {
    ST_GROUND = 0, /**< On the ground, stable low. Slow poll, no audio.        */
    ST_CLIMB,      /**< Rising from ground; callouts not yet armed.            */
    ST_ARMED,      /**< Climbed through ARM_FT; descent callouts enabled.      */
    ST_CRUISE,     /**< At/above cruise_ft; slowest poll, sleep-friendly.      */
    ST_DESCENT     /**< Descending while armed; fast poll, callouts + tone.    */
} sm_state_t;

/** Poll/power profile the sensor task should apply for the current state. */
typedef enum {
    POLL_GROUND = 0,
    POLL_CLIMB,
    POLL_ARMED,
    POLL_CRUISE,
    POLL_DESCENT
} poll_profile_t;

/**
 * @brief Carried state between sm_step() calls.
 *
 * @note  armed_mask uses one bit per callout index (LSB = callouts[0]). A set
 *        bit means "this callout is armed and will fire on the next downward
 *        crossing". This bounds us to 32 callouts, far more than either profile.
 */
typedef struct {
    sm_state_t state;        /**< Current behaviour state.                     */
    float      prev_agl;     /**< Previous AGL, for the trend estimate.        */
    float      trend_fps;    /**< Smoothed vertical rate (ft/s, +up / -down).  */
    uint32_t   armed_mask;   /**< Per-callout armed bits.                      */
    bool       armed;        /**< True once climbed through ARM_FT.            */
    bool       have_prev;    /**< False until the first step seeds prev_agl.   */
} sm_ctx_t;

/** Result of one sm_step() evaluation. */
typedef struct {
    sm_state_t     state;          /**< New (or unchanged) state.             */
    int            fired_callout;  /**< Index into profile->callouts, or -1.  */
    poll_profile_t poll;           /**< Poll profile to apply.                */
    float          tone_agl;       /**< AGL to hand the audio engine.         */
    bool           tone_active;    /**< Whether the presence tone should play. */
} sm_out_t;

/**
 * @brief Initialise a context into a given starting state.
 * @param c        Context to initialise.
 * @param initial  Starting state (typically from sm_initial_state()).
 */
void sm_init(sm_ctx_t *c, sm_state_t initial);

/**
 * @brief Decide the boot-time starting state from the reconstructed AGL.
 *
 * @details After an in-flight reboot the box must NOT wake up in GROUND and do
 *          something dumb. If the boot estimate says we are clearly up (above
 *          ARM_FT) we boot into a flying state; if it says we are near the
 *          ground and the estimate is trustworthy we boot into GROUND.
 *
 * @param boot_agl  Reconstructed AGL from the boot sanity filter.
 * @param ok        Whether @p boot_agl is trustworthy (from robust_estimate).
 * @param p         Active sensor profile (for cruise_ft).
 * @return          The state to seed the machine with.
 */
sm_state_t sm_initial_state(float boot_agl, bool ok, const sensor_profile_t *p);

/**
 * @brief Advance the state machine by one decision tick.
 *
 * @param c    Carried context (updated in place).
 * @param agl_ft  Current AGL in feet (already ground-referenced & EMA'd).
 * @param dt_s    Seconds since the previous step (for the trend estimate).
 * @param p       Active sensor profile (callout list + cruise_ft).
 * @param[out] out  Decision outputs for this tick.
 */
void sm_step(sm_ctx_t *c, float agl_ft, float dt_s,
             const sensor_profile_t *p, sm_out_t *out);

/**
 * @brief Map a poll profile to a concrete period in milliseconds (config.h).
 * @param pp  Poll profile.
 * @return    Milliseconds between sensor reads for that profile.
 */
uint32_t poll_profile_to_ms(poll_profile_t pp);

#endif /* LIDARAGL_STATE_MACHINE_H */
