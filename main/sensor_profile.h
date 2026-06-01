/**
 * @file    sensor_profile.h
 * @brief   Per-sensor behaviour profile (callout list, cruise altitude, range).
 *
 * @details The box supports two LightWare micro-lidars whose usable range
 *          differs by 2x. Rather than scatter `#ifdef`s through the logic, every
 *          value that depends on WHICH sensor is fitted is collected into a
 *          single immutable `sensor_profile_t`. Exactly one profile is selected
 *          at boot (by hardware autodetect — see sf30c_detect_profile()) and a
 *          pointer to it is threaded into the state machine.
 *
 *          Why per-sensor callouts (the headline requirement):
 *            - SF30/C tops out near 328 ft, so its highest callout is 200 ft.
 *              A 300 ft callout would sit too close to the sensor's ceiling to
 *              be trustworthy — we give the sensor room to breathe.
 *            - SF30/D reaches ~656 ft, so it adds 500/400/300 ft high-altitude
 *              callouts on top of the shared flare-band set.
 *
 *          This header is PURE: it pulls in no ESP-IDF headers and is compiled
 *          unchanged by the host unit-test build.
 */
#ifndef LIDARAGL_SENSOR_PROFILE_H
#define LIDARAGL_SENSOR_PROFILE_H

#include <stddef.h>

/** Which physical LightWare unit the firmware is driving. */
typedef enum {
    SENSOR_SF30C = 0,   /**< 100 m / ~328 ft — the default unit.               */
    SENSOR_SF30D = 1    /**< 200 m / ~656 ft — long-range variant.             */
} sensor_model_t;

/**
 * @brief Immutable description of one sensor's operating envelope.
 *
 * Callout heights are stored in DESCENDING order (highest first). The state
 * machine indexes them by position and edge-triggers each one once on descent.
 */
typedef struct {
    sensor_model_t model;          /**< Which unit this profile describes.     */
    const char    *name;           /**< Human/log name, e.g. "SF30/C".         */
    float          max_range_ft;   /**< Nominal usable ceiling in feet.        */
    const float   *callouts;       /**< Descending callout heights (ft AGL).   */
    size_t         n_callouts;     /**< Number of entries in @ref callouts.    */
    float          cruise_ft;      /**< At/above this -> low-power CRUISE.      */
} sensor_profile_t;

/* The two built-in profiles. Defined once in sensor_profile.c. */
extern const sensor_profile_t SF30C_PROFILE;   /**< default; callouts 200..10 */
extern const sensor_profile_t SF30D_PROFILE;   /**< callouts 500..10           */

/**
 * @brief Look up a profile by model enum.
 * @param m  Sensor model.
 * @return   Pointer to the matching profile (never NULL; SF30/C for unknown).
 */
const sensor_profile_t *sensor_profile_for_model(sensor_model_t m);

/**
 * @brief Choose a profile from a LightWare product-name string.
 *
 * @details Used by the boot-time autodetect. If @p product_name contains the
 *          substring "SF30/D" (case-insensitive) the SF30/D profile is
 *          returned; otherwise it falls back to the SF30/C profile. A NULL or
 *          empty name also yields SF30/C, so the box always has a valid profile.
 *
 * @param product_name  Name reported by the sensor (may be NULL).
 * @return              Pointer to the selected profile (never NULL).
 */
const sensor_profile_t *sensor_profile_from_name(const char *product_name);

#endif /* LIDARAGL_SENSOR_PROFILE_H */
