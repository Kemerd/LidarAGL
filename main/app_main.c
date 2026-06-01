/**
 * @file    app_main.c
 * @brief   LidarAGL entry point: boot sequence, task spawning, and logic loop.
 *
 * @details Boot order (spec §5, §6, §8 + the builder's ground-reference model):
 *            1. NVS up; radios stay off (compiled out + never started).
 *            2. Reset button held? -> wipe ground buffer + reboot.
 *            3. UART up; autodetect the sensor -> g_profile; start streaming.
 *            4. I2S up.
 *            5. Capture a ground-fill (10 readings spread over ~1 s) + one
 *               current reading; reconstruct the ground reference and boot AGL.
 *            6. If no real reference existed -> calibration-error chirp.
 *            7. One NVS write: persist the fresh ground readings.
 *            8. Seed the state machine from the boot AGL.
 *            9. Spawn sensor (core 0), logic + audio (core 1); enable light-sleep.
 *
 *          The logic task lives here too: it owns the state machine, applies the
 *          ground reference, fires callouts, sets the poll profile, publishes
 *          tone params, and gates light-sleep via audio suspend/resume.
 */

#include "config.h"
#include "shared.h"
#include "sensor_profile.h"
#include "sf30c.h"
#include "boot_buffer.h"
#include "state_machine.h"
#include "audio.h"
#include "callouts.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_pm.h"

static const char *TAG = "app";

/* ---- Definitions of the shared globals declared in shared.h -------------- */
const sensor_profile_t *g_profile        = NULL;
volatile uint32_t       g_poll_period_ms = POLL_MS_GROUND;
QueueHandle_t           q_range_latest   = NULL;
QueueHandle_t           q_callouts       = NULL;
audio_params_t          g_audio_params   = { .tone_agl = TONE_START_FT, .tone_active = false };
SemaphoreHandle_t       g_audio_mutex    = NULL;

/*  The learned ground reference, applied by the logic task to turn measured
 *  range into AGL. Set once during boot.                                       */
static float s_ground_ref_ft = MOUNT_OFFSET_FALLBACK_FT;

/* ---- LWNX update-rate code per poll profile ------------------------------ */
/*  The binary update-rate code (cmd 76). We keep the sensor at a brisk rate in
 *  flight; on the ASCII path this is a no-op. Code 8 ~= 78 readings/sec.        */
#define SF30_RATE_CODE_ACTIVE  8u

/* ---------------------------------------------------------------------------
 *  Boot: capture a ground-fill of BOOT_BUFFER_N readings spread over ~1 s.
 *
 *  We sample up to GROUND_FILL_SAMPLES times, keeping every valid reading, then
 *  pick BOOT_BUFFER_N of them evenly spread across the window. Junk is filtered
 *  later by robust_mean during resolution.
 * ------------------------------------------------------------------------- */
static size_t capture_ground_fill(float *out, size_t want)
{
    float pool[GROUND_FILL_SAMPLES];
    size_t got = 0;

    uint32_t per_sample_ms = GROUND_FILL_MS / GROUND_FILL_SAMPLES;
    if (per_sample_ms == 0) {
        per_sample_ms = 1;
    }

    for (size_t i = 0; i < GROUND_FILL_SAMPLES && got < GROUND_FILL_SAMPLES; ++i) {
        float r; bool v;
        if (sf30c_read_latest_ft(&r, &v) && v) {
            pool[got++] = r;
        }
        vTaskDelay(pdMS_TO_TICKS(per_sample_ms));
    }

    if (got == 0) {
        return 0;
    }

    /* Evenly subsample 'want' readings from the pool. */
    size_t n = (want < got) ? want : got;
    for (size_t i = 0; i < n; ++i) {
        size_t idx = (got - 1) * i / (n > 1 ? (n - 1) : 1);
        out[i] = pool[idx];
    }
    return n;
}

/* ---------------------------------------------------------------------------
 *  Logic task (core 1): the decision loop.
 * ------------------------------------------------------------------------- */
static void logic_task(void *arg)
{
    (void)arg;

    /* arg carries a pointer to the boot-decided initial state (see app_main). */
    sm_state_t initial = (arg != NULL) ? *(sm_state_t *)arg : ST_GROUND;

    sm_ctx_t sm;
    sm_init(&sm, initial);

    int64_t last_us = esp_timer_get_time();
    bool sleep_allowed_prev = false;

    for (;;) {
        /* Peek the freshest range sample (non-destructive). */
        range_sample_t s;
        if (xQueuePeek(q_range_latest, &s, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;   /* no sample yet */
        }

        /* dt since the last decision. */
        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - last_us) / 1e6f;
        last_us = now_us;
        if (dt_s <= 0.0f) {
            dt_s = 0.001f;
        }

        /* Apply the learned ground reference: AGL = range - ground. On a
         * lost-signal sample, hold by feeding the previous AGL (sm clamps). */
        float agl = s.range_ft - s_ground_ref_ft;
        if (agl < 0.0f) {
            agl = 0.0f;
        }

        sm_out_t out;
        sm_step(&sm, agl, dt_s, g_profile, &out);

        /* Fire the callout, if any, mapping the profile height -> clip id. */
        if (out.fired_callout >= 0) {
            float ft = g_profile->callouts[out.fired_callout];
            callout_id_t cid = callout_id_for_ft(ft);
            if (cid != CO_COUNT) {
                audio_request_callout(cid);
            }
            ESP_LOGI(TAG, "callout %.0f ft (state=%d)", ft, out.state);
        }

        /* Publish tone params + poll cadence. */
        audio_set_params(out.tone_agl, out.tone_active);
        g_poll_period_ms = poll_profile_to_ms(out.poll);

        /* Light-sleep policy: allowed only in GROUND/CRUISE, where the tone is
         * silent. Suspend the I2S channel before sleeping, resume on exit.    */
        bool sleep_allowed = (out.state == ST_GROUND || out.state == ST_CRUISE);
        if (sleep_allowed && !sleep_allowed_prev) {
            audio_suspend();
        } else if (!sleep_allowed && sleep_allowed_prev) {
            audio_resume();
        }
        sleep_allowed_prev = sleep_allowed;

        /* Decision cadence: fast in active states, relaxed when sleep-friendly
         * (the PM subsystem drops to light-sleep during the idle between ticks). */
        uint32_t tick_ms = sleep_allowed ? POLL_MS_CRUISE : 20;
        vTaskDelay(pdMS_TO_TICKS(tick_ms));
    }
}

/* ---------------------------------------------------------------------------
 *  Enable automatic light-sleep (needs CONFIG_PM_ENABLE + tickless idle).
 * ------------------------------------------------------------------------- */
static void enable_power_management(void)
{
    esp_pm_config_t pm = {
        .max_freq_mhz       = PM_MAX_FREQ_MHZ,
        .min_freq_mhz       = PM_MIN_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    esp_err_t err = esp_pm_configure(&pm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_pm_configure failed (%s); running without light-sleep",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "power management: %d/%d MHz, light-sleep on",
                 PM_MAX_FREQ_MHZ, PM_MIN_FREQ_MHZ);
    }
}

/* ---------------------------------------------------------------------------
 *  app_main
 * ------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "LidarAGL boot");

    /* WiFi/BT are compiled out (sdkconfig) and never started here — net effect:
     * radios never powered, no draw, no EMI into the audio/avionics path.     */

    /* 1. NVS + reset-button GPIO. */
    boot_buffer_init();

    /* 2. Reset button held at boot -> wipe ground buffer and reboot. */
    if (boot_buffer_reset_pressed()) {
        boot_buffer_wipe_and_reboot();   /* does not return */
    }

    /* 3. Sensor up + autodetect + stream config. */
    sf30c_init();
    g_profile = sf30c_detect_profile();
    sf30c_configure_stream(SF30_RATE_CODE_ACTIVE);
    ESP_LOGI(TAG, "active profile: %s (cruise %.0f ft, %d callouts)",
             g_profile->name, g_profile->cruise_ft, (int)g_profile->n_callouts);

    /* 4. Audio up. */
    audio_init();

    /* 5. Ground-fill + current reading -> reconstruct ground reference. */
    float ground_reads[BOOT_BUFFER_N];
    size_t n_fill = capture_ground_fill(ground_reads, BOOT_BUFFER_N);

    boot_entry_t stored[BOOT_BUFFER_N];
    size_t n_stored = boot_buffer_load(stored, BOOT_BUFFER_N);

    float current_ft = 0.0f;
    bool  cur_valid  = false;
    sf30c_read_latest_ft(&current_ft, &cur_valid);

    /* If the very latest sample was a lost return, fall back to the freshest
     * valid ground-fill reading rather than feeding resolve() a stale/zero
     * value — otherwise an early lost-signal could be mistaken for "on ground
     * at 0 ft" and corrupt the airborne decision.                             */
    if (!cur_valid && n_fill > 0) {
        current_ft = ground_reads[n_fill - 1];
    }

    boot_result_t br;
    boot_buffer_resolve(stored, n_stored, current_ft, &br);
    s_ground_ref_ft = br.ground_ref_ft;

    ESP_LOGI(TAG, "ground_ref=%.2f ft  boot_agl=%.1f ft  airborne=%d calib_err=%d",
             br.ground_ref_ft, br.boot_agl_ft, br.airborne, br.calib_error);

    /* 6. Calibration error -> warn the pilot with the boot chirp. */
    if (br.calib_error) {
        ESP_LOGW(TAG, "no ground reference; using %.1f ft fallback (chirping)",
                 (double)MOUNT_OFFSET_FALLBACK_FT);
        audio_play_chirp();
    }

    /* 7. The ONE NVS write per boot: persist this boot's ground readings.
     *    Only refresh the stored ground when we're actually on the ground —
     *    an in-flight reboot must NOT overwrite the learned ground with an
     *    airborne reading.                                                     */
    if (!br.airborne && n_fill > 0) {
        boot_buffer_commit(ground_reads, n_fill, (uint32_t)(esp_timer_get_time() / 1000));
    }

    /* 8. Seed the state machine from the reconstructed AGL. */
    static sm_state_t initial_state;
    initial_state = sm_initial_state(br.boot_agl_ft, !br.calib_error || br.airborne,
                                     g_profile);
    ESP_LOGI(TAG, "initial state = %d", initial_state);

    /* 9. Create the inter-task primitives. */
    q_range_latest = xQueueCreate(1, sizeof(range_sample_t));
    q_callouts     = xQueueCreate(4, sizeof(callout_id_t));
    g_audio_mutex  = xSemaphoreCreateMutex();
    configASSERT(q_range_latest && q_callouts && g_audio_mutex);

    /* Spawn tasks: sensor on core 0; logic + audio on core 1. */
    xTaskCreatePinnedToCore(sensor_task, "sensor", SENSOR_TASK_STACK, NULL,
                            SENSOR_TASK_PRIO, NULL, SENSOR_TASK_CORE);
    xTaskCreatePinnedToCore(audio_task, "audio", AUDIO_TASK_STACK, NULL,
                            AUDIO_TASK_PRIO, NULL, AUDIO_TASK_CORE);
    xTaskCreatePinnedToCore(logic_task, "logic", LOGIC_TASK_STACK, &initial_state,
                            LOGIC_TASK_PRIO, NULL, LOGIC_TASK_CORE);

    /* Enable automatic light-sleep last, once everything is running. */
    enable_power_management();

    ESP_LOGI(TAG, "running");
}
