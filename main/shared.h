/**
 * @file    shared.h
 * @brief   Cross-task shared state and the inter-task handoff primitives.
 *
 * @details Defined once in app_main.c, declared extern here. The concurrency
 *          contract (spec §9):
 *
 *            - g_profile          : the active sensor profile. Written ONCE at
 *                                   boot (before any task starts) and read-only
 *                                   thereafter, so no lock is needed.
 *            - g_poll_period_ms   : the sensor poll cadence. Written by the
 *                                   logic task, read by the sensor task. A
 *                                   32-bit aligned scalar — atomic on the S3.
 *            - q_range_latest     : length-1 OVERWRITE queue carrying the freshest
 *                                   range sample; the consumer never sees stale
 *                                   backlog, which is critical for on-time callouts.
 *            - q_callouts         : small queue of callout ids from logic->audio.
 *            - g_audio_params (+ mutex) : current tone AGL / active flag.
 *
 *          This header is HARDWARE-side (pulls in FreeRTOS) and is excluded from
 *          the host test build.
 */
#ifndef LIDARAGL_SHARED_H
#define LIDARAGL_SHARED_H

#ifndef UNIT_TEST

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "sensor_profile.h"

/** One range sample handed from the sensor task to the logic task. */
typedef struct {
    float    range_ft;   /**< Filtered range in feet (mount offset NOT applied). */
    bool     valid;      /**< False on a lost-signal / no-return sample.         */
    uint32_t seq;        /**< Monotonic sample counter (for dt sanity).          */
} range_sample_t;

/** Tone parameters the logic task publishes for the audio task. */
typedef struct {
    float tone_agl;      /**< AGL the tone should track (already smoothed).      */
    bool  tone_active;   /**< Whether the presence tone should sound.            */
} audio_params_t;

/* ---- Globals (defined in app_main.c) ------------------------------------ */
extern const sensor_profile_t *g_profile;       /**< boot-selected, read-only.  */
extern volatile uint32_t       g_poll_period_ms;/**< logic->sensor cadence.     */

extern QueueHandle_t     q_range_latest;        /**< len-1 overwrite queue.     */
extern QueueHandle_t     q_callouts;            /**< logic->audio callout ids.  */

extern audio_params_t    g_audio_params;        /**< guarded by g_audio_mutex.  */
extern SemaphoreHandle_t g_audio_mutex;

#endif /* !UNIT_TEST */

#endif /* LIDARAGL_SHARED_H */
