/**
 * @file    boot_buffer.c
 * @brief   NVS-backed learned ground reference + in-flight-reboot recovery.
 *
 * @details HARDWARE module (NVS + GPIO). The robust averaging is delegated to
 *          the pure robust_estimate(); this file handles persistence, the
 *          ground-vs-airborne decision, and the reset button.
 */

#include "boot_buffer.h"
#include "config.h"
#include "robust.h"

#include <string.h>

#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bootbuf";

/* ---- NVS bring-up -------------------------------------------------------- */

void boot_buffer_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* The NVS partition is in a state the current code can't use — wipe and
         * retry. This is the standard ESP-IDF idiom.                          */
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Configure the reset button: input with internal pull-up, active low. */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_CONFIG_BTN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

/* ---- Reset button -------------------------------------------------------- */

bool boot_buffer_reset_pressed(void)
{
    /* Simple debounce: require the active level on several spaced reads so a
     * glitch on the line can't trigger a wipe.                                */
    const int SAMPLES = 5;
    for (int i = 0; i < SAMPLES; ++i) {
        if (gpio_get_level(PIN_CONFIG_BTN) != CONFIG_BTN_ACTIVE_LEVEL) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return true;
}

bool boot_buffer_button_down(void)
{
    /* Two quick spaced samples — enough to reject a single-sample glitch while
     * keeping the config-menu poll loop snappy for tap detection.             */
    if (gpio_get_level(PIN_CONFIG_BTN) != CONFIG_BTN_ACTIVE_LEVEL) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(3));
    return gpio_get_level(PIN_CONFIG_BTN) == CONFIG_BTN_ACTIVE_LEVEL;
}

void boot_buffer_wipe_ground(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_GROUNDBUF);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "ground buffer wiped");
}

void boot_buffer_wipe_and_reboot(void)
{
    boot_buffer_wipe_ground();
    ESP_LOGW(TAG, "rebooting after wipe");
    vTaskDelay(pdMS_TO_TICKS(50));   /* let the log flush */
    esp_restart();                   /* does not return */
}

/* ---- Audio config (selected AUDIO_MODE_* index) -------------------------- */

int config_load_audio_mode(void)
{
    /* Stored as a single u8. Absent / unreadable / out-of-range -> the compiled
     * default, so a corrupt value can never leave the box silent or unconfigured. */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return DEFAULT_AUDIO_MODE;
    }
    uint8_t v = DEFAULT_AUDIO_MODE;
    esp_err_t err = nvs_get_u8(h, NVS_KEY_AUDIOCFG, &v);
    nvs_close(h);
    if (err != ESP_OK || v >= AUDIO_MODE_COUNT) {
        return DEFAULT_AUDIO_MODE;
    }
    return (int)v;
}

void config_save_audio_mode(int mode)
{
    if (mode < 0 || mode >= AUDIO_MODE_COUNT) {
        mode = DEFAULT_AUDIO_MODE;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; audio config not persisted");
        return;
    }
    if (nvs_set_u8(h, NVS_KEY_AUDIOCFG, (uint8_t)mode) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "audio config saved: mode %d", mode);
    } else {
        ESP_LOGE(TAG, "nvs_set_u8 failed; audio config not persisted");
    }
    nvs_close(h);
}

void config_wipe_audio_mode(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_AUDIOCFG);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "audio config wiped (will use default)");
}

/* ---- Callout start-altitude cap ------------------------------------------ */

float config_load_start_alt(float default_ft)
{
    /* Stored as a u16 of feet. Absent / unreadable / nonsensical (<=0) -> the
     * caller's profile-dependent default.                                      */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return default_ft;
    }
    uint16_t v = 0;
    esp_err_t err = nvs_get_u16(h, NVS_KEY_STARTALT, &v);
    nvs_close(h);
    if (err != ESP_OK || v == 0) {
        return default_ft;
    }
    return (float)v;
}

void config_save_start_alt(float ft)
{
    if (ft < 0.0f) {
        ft = 0.0f;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; start-altitude not persisted");
        return;
    }
    /* Round to the nearest foot; the menu only offers integer callout heights. */
    uint16_t v = (uint16_t)(ft + 0.5f);
    if (nvs_set_u16(h, NVS_KEY_STARTALT, v) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "start-altitude saved: %u ft", v);
    } else {
        ESP_LOGE(TAG, "nvs_set_u16 failed; start-altitude not persisted");
    }
    nvs_close(h);
}

void config_wipe_start_alt(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_STARTALT);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "start-altitude wiped (will use profile default)");
}

/* ---- Voice volume offset (dB cut, layered on the analog pot) ------------- */

float config_load_voice_volume(void)
{
    /* Stored as a u8 CUT magnitude (|dB|): 0 == no cut, 6 == -6 dB. Absent /
     * unreadable / out-of-range -> the default (0 dB), so a bad value can never
     * silence the callouts.                                                     */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return DEFAULT_VOICE_VOLUME_DB;
    }
    uint8_t cut = 0;
    esp_err_t err = nvs_get_u8(h, NVS_KEY_VOLOFS, &cut);
    nvs_close(h);
    if (err != ESP_OK) {
        return DEFAULT_VOICE_VOLUME_DB;
    }
    /* Convert the magnitude back to a (negative) offset and clamp to the floor. */
    float db = -(float)cut;
    if (db < VOICE_VOLUME_DB_MIN) {
        db = VOICE_VOLUME_DB_MIN;
    }
    if (db > 0.0f) {
        db = 0.0f;
    }
    return db;
}

void config_save_voice_volume(float db)
{
    /* Clamp to [VOICE_VOLUME_DB_MIN, 0], then store the rounded CUT magnitude. */
    if (db > 0.0f) {
        db = 0.0f;
    }
    if (db < VOICE_VOLUME_DB_MIN) {
        db = VOICE_VOLUME_DB_MIN;
    }
    uint8_t cut = (uint8_t)(-db + 0.5f);    /* -4 dB -> 4 */

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; voice volume not persisted");
        return;
    }
    if (nvs_set_u8(h, NVS_KEY_VOLOFS, cut) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "voice volume saved: -%u dB", cut);
    } else {
        ESP_LOGE(TAG, "nvs_set_u8 failed; voice volume not persisted");
    }
    nvs_close(h);
}

void config_wipe_voice_volume(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_VOLOFS);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "voice volume wiped (will use 0 dB)");
}

/* ---- Tone volume offset (dB, SIGNED — can cut OR boost) ------------------ */

float config_load_tone_volume(void)
{
    /* Stored SIGNED as an i8 of dB (e.g. -4 or +4). Absent / unreadable -> the
     * default (0 dB). The value is clamped to the legal range so a corrupt entry
     * can never push the tone past its menu limits.                            */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return DEFAULT_TONE_VOLUME_DB;
    }
    int8_t raw = 0;
    esp_err_t err = nvs_get_i8(h, NVS_KEY_TONEVOL, &raw);
    nvs_close(h);
    if (err != ESP_OK) {
        return DEFAULT_TONE_VOLUME_DB;
    }
    float db = (float)raw;
    if (db < TONE_VOLUME_DB_MIN) {
        db = TONE_VOLUME_DB_MIN;
    }
    if (db > TONE_VOLUME_DB_MAX) {
        db = TONE_VOLUME_DB_MAX;
    }
    return db;
}

void config_save_tone_volume(float db)
{
    /* Clamp to [TONE_VOLUME_DB_MIN, TONE_VOLUME_DB_MAX], then store as signed i8.
     * Round toward nearest, handling the negative side symmetrically.           */
    if (db < TONE_VOLUME_DB_MIN) {
        db = TONE_VOLUME_DB_MIN;
    }
    if (db > TONE_VOLUME_DB_MAX) {
        db = TONE_VOLUME_DB_MAX;
    }
    int8_t val = (int8_t)(db >= 0.0f ? db + 0.5f : db - 0.5f);

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; tone volume not persisted");
        return;
    }
    if (nvs_set_i8(h, NVS_KEY_TONEVOL, val) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "tone volume saved: %+d dB", val);
    } else {
        ESP_LOGE(TAG, "nvs_set_i8 failed; tone volume not persisted");
    }
    nvs_close(h);
}

void config_wipe_tone_volume(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_TONEVOL);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "tone volume wiped (will use 0 dB)");
}

/* ---- "Check gear" descent-callout altitude (0 == OFF) -------------------- */

float config_load_gear_check_alt(void)
{
    /* Stored as a u16 of feet. OFF by default: an absent key OR a stored 0 both
     * mean disabled, so we never need to distinguish "never set" from "set OFF". */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return 0.0f;   /* OFF */
    }
    uint16_t v = 0;
    esp_err_t err = nvs_get_u16(h, NVS_KEY_GEARCHK, &v);
    nvs_close(h);
    if (err != ESP_OK) {
        return 0.0f;   /* absent / unreadable -> OFF */
    }
    return (float)v;   /* 0 == OFF, else the chosen altitude in feet */
}

void config_save_gear_check_alt(float ft)
{
    if (ft < 0.0f) {
        ft = 0.0f;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; gear-check altitude not persisted");
        return;
    }
    /* Round to the nearest foot; the menu only offers integer callout heights
     * (or 0 for OFF).                                                            */
    uint16_t v = (uint16_t)(ft + 0.5f);
    if (nvs_set_u16(h, NVS_KEY_GEARCHK, v) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "gear-check altitude saved: %u ft%s", v, v == 0 ? " (OFF)" : "");
    } else {
        ESP_LOGE(TAG, "nvs_set_u16 failed; gear-check altitude not persisted");
    }
    nvs_close(h);
}

void config_wipe_gear_check_alt(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_GEARCHK);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "gear-check altitude wiped (will use OFF)");
}

/* ---- "Positive rate" climb-callout enable flag (disabled by default) ----- */

bool config_load_positive_rate(void)
{
    /* Stored as a u8 (0/1). Disabled by default: absent / unreadable / zero all
     * read as OFF, so a corrupt value can never enable an unwanted callout.      */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t v = 0;
    esp_err_t err = nvs_get_u8(h, NVS_KEY_POSRATE, &v);
    nvs_close(h);
    if (err != ESP_OK) {
        return false;
    }
    return v != 0;
}

void config_save_positive_rate(bool enabled)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; positive-rate flag not persisted");
        return;
    }
    if (nvs_set_u8(h, NVS_KEY_POSRATE, enabled ? 1u : 0u) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "positive-rate callout saved: %s", enabled ? "ON" : "OFF");
    } else {
        ESP_LOGE(TAG, "nvs_set_u8 failed; positive-rate flag not persisted");
    }
    nvs_close(h);
}

void config_wipe_positive_rate(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_POSRATE);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "positive-rate flag wiped (will use disabled)");
}

/* ---- "Sink rate" vario-blip enable flag (disabled by default) ------------ */

bool config_load_sink_rate(void)
{
    /* Same OFF-by-default u8 contract as the positive-rate flag: absent /
     * unreadable / zero all read as OFF, so a corrupt value never enables blips. */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t v = 0;
    esp_err_t err = nvs_get_u8(h, NVS_KEY_SINKRATE, &v);
    nvs_close(h);
    if (err != ESP_OK) {
        return false;
    }
    return v != 0;
}

void config_save_sink_rate(bool enabled)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; sink-rate flag not persisted");
        return;
    }
    if (nvs_set_u8(h, NVS_KEY_SINKRATE, enabled ? 1u : 0u) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "sink-rate vario saved: %s", enabled ? "ON" : "OFF");
    } else {
        ESP_LOGE(TAG, "nvs_set_u8 failed; sink-rate flag not persisted");
    }
    nvs_close(h);
}

void config_wipe_sink_rate(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_SINKRATE);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "sink-rate flag wiped (will use disabled)");
}

/* ---- "Climb rate" vario-blip enable flag (disabled by default) ----------- */

bool config_load_climb_rate(void)
{
    /* OFF-by-default u8, identical contract to the sink-rate flag above. */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t v = 0;
    esp_err_t err = nvs_get_u8(h, NVS_KEY_CLIMBRATE, &v);
    nvs_close(h);
    if (err != ESP_OK) {
        return false;
    }
    return v != 0;
}

void config_save_climb_rate(bool enabled)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; climb-rate flag not persisted");
        return;
    }
    if (nvs_set_u8(h, NVS_KEY_CLIMBRATE, enabled ? 1u : 0u) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "climb-rate vario saved: %s", enabled ? "ON" : "OFF");
    } else {
        ESP_LOGE(TAG, "nvs_set_u8 failed; climb-rate flag not persisted");
    }
    nvs_close(h);
}

void config_wipe_climb_rate(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_CLIMBRATE);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "climb-rate flag wiped (will use disabled)");
}

/* ---- Load / commit ------------------------------------------------------- */

size_t boot_buffer_load(boot_entry_t *out, size_t cap)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return 0;   /* namespace not created yet -> first boot */
    }

    size_t blob_len = 0;
    esp_err_t err = nvs_get_blob(h, NVS_KEY_GROUNDBUF, NULL, &blob_len);
    if (err != ESP_OK || blob_len == 0) {
        nvs_close(h);
        return 0;
    }

    /* Clamp to what the caller can hold and to BOOT_BUFFER_N. */
    size_t n = blob_len / sizeof(boot_entry_t);
    if (n > cap) {
        n = cap;
    }
    size_t want = n * sizeof(boot_entry_t);
    err = nvs_get_blob(h, NVS_KEY_GROUNDBUF, out, &want);
    nvs_close(h);

    if (err != ESP_OK) {
        return 0;
    }
    return n;
}

void boot_buffer_commit(const float *ground_reads, size_t n, uint32_t marker)
{
    if (n > BOOT_BUFFER_N) {
        n = BOOT_BUFFER_N;
    }

    boot_entry_t entries[BOOT_BUFFER_N];
    for (size_t i = 0; i < n; ++i) {
        entries[i].range_ft = ground_reads[i];
        entries[i].marker   = marker;
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; ground buffer not persisted");
        return;
    }
    /* The ONE write per boot. */
    esp_err_t err = nvs_set_blob(h, NVS_KEY_GROUNDBUF, entries,
                                 n * sizeof(boot_entry_t));
    if (err == ESP_OK) {
        nvs_commit(h);
    } else {
        ESP_LOGE(TAG, "nvs_set_blob failed: %s", esp_err_to_name(err));
    }
    nvs_close(h);
}

/* ---- Ground-reference resolution ----------------------------------------- */

void boot_buffer_resolve(const boot_entry_t *stored, size_t n_stored,
                         float current_ft, boot_result_t *result)
{
    /* Pull the stored ranges into a flat array for the robust estimator. */
    float vals[BOOT_BUFFER_N];
    size_t m = (n_stored > BOOT_BUFFER_N) ? BOOT_BUFFER_N : n_stored;
    for (size_t i = 0; i < m; ++i) {
        vals[i] = stored[i].range_ft;
    }

    /* Robust mean of the stored ground set ALONE — the current reading is NOT
     * blended in, because it may be an in-flight value and we want the learned
     * historical ground here.                                                  */
    bool have_ref = false;
    float ground_ref = robust_mean(vals, m, &have_ref);

    if (!have_ref) {
        /* No usable stored ground. If the current reading itself looks like a
         * plausible ground reading, adopt it as the reference; otherwise fall
         * back to the emergency offset and flag a calibration error.          */
        if (current_ft >= 0.0f && current_ft <= MAX_VALID_FT) {
            result->ground_ref_ft = current_ft;
            result->boot_agl_ft   = 0.0f;
            result->airborne      = false;
            result->calib_error   = false;
        } else {
            result->ground_ref_ft = MOUNT_OFFSET_FALLBACK_FT;
            result->boot_agl_ft   = current_ft - MOUNT_OFFSET_FALLBACK_FT;
            if (result->boot_agl_ft < 0.0f) {
                result->boot_agl_ft = 0.0f;
            }
            result->airborne    = (result->boot_agl_ft > GROUND_DEV_FT);
            result->calib_error = true;   /* warn the pilot via boot chirp     */
        }
        return;
    }

    /* We have a trustworthy stored ground reference. Decide ground vs airborne
     * by how far the current reading sits above it.                           */
    float agl = current_ft - ground_ref;
    if (agl < 0.0f) {
        agl = 0.0f;
    }

    result->ground_ref_ft = ground_ref;
    result->boot_agl_ft   = agl;
    result->airborne      = (current_ft - ground_ref) > GROUND_DEV_FT;
    result->calib_error   = false;
}
