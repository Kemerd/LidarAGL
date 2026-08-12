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
#include <math.h>       /* isfinite() — NaN guards on the resolve path */

/*  HARDWARE-ONLY REGION. boot_buffer_resolve() — the function that decides
 *  where the ground is, whether the box booted airborne, and whether the
 *  calibration can be trusted — is PURE: it reads its inputs and writes its
 *  result, touching no peripheral. That makes it the most safety-critical
 *  host-testable logic in the firmware, and it had no coverage at all while the
 *  NVS/GPIO includes below forced the whole translation unit to be
 *  firmware-only. Fencing them (the same UNIT_TEST split config.h already uses)
 *  lets the resolver be exercised on the desktop, where the obstruction, jack,
 *  in-flight-reboot and NaN cases can each be pinned by a test.                */
#ifndef UNIT_TEST
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
/*  Host build: the persistence/GPIO surface is compiled out entirely (see the
 *  #ifndef UNIT_TEST fence further down), so only these logging shims are
 *  needed to keep the pure code identical to what ships.                       */
#include <stdio.h>
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGE(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#endif

#ifndef UNIT_TEST
static const char *TAG = "bootbuf";   /* only the fenced NVS/GPIO code logs */
#endif

#ifndef UNIT_TEST
/* =========================================================================
 *  PERSISTENCE / GPIO SURFACE (firmware build only).
 *  Everything from here to the matching #endif talks to NVS or the config
 *  button. The host test build skips it and compiles only the pure resolver
 *  at the bottom of the file.
 * ========================================================================= */

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
     * glitch on the line can't trigger a wipe.
     *
     * The spacing MUST be clamped to a whole tick: pdMS_TO_TICKS truncates, so
     * at the 100 Hz FreeRTOS tick 5 ms is ZERO ticks and vTaskDelay(0) merely
     * yields — the five "spaced" reads then run back-to-back in microseconds,
     * compiling the entire glitch filter away. A sub-millisecond transient on
     * the (weak-pull-up, panel-wired) button line at the boot check instant
     * could pass all five reads and drop the unit into the config menu, which
     * wipes the ground reference and every pilot setting unconfirmed. One tick
     * (10 ms at 100 Hz) x 5 samples is a real ~40 ms window again.             */
    const int SAMPLES = 5;
    for (int i = 0; i < SAMPLES; ++i) {
        if (gpio_get_level(PIN_CONFIG_BTN) != CONFIG_BTN_ACTIVE_LEVEL) {
            return false;
        }
        TickType_t gap = pdMS_TO_TICKS(5);
        vTaskDelay(gap > 0 ? gap : 1);
    }
    return true;
}

bool boot_buffer_button_down(void)
{
    /* Two quick spaced samples — enough to reject a single-sample glitch while
     * keeping the config-menu poll loop snappy for tap detection. Same
     * tick-floor clamp as boot_buffer_reset_pressed(): 3 ms truncates to ZERO
     * ticks at the 100 Hz tick, and an unspaced double-read is no debounce.    */
    if (gpio_get_level(PIN_CONFIG_BTN) != CONFIG_BTN_ACTIVE_LEVEL) {
        return false;
    }
    TickType_t gap = pdMS_TO_TICKS(3);
    vTaskDelay(gap > 0 ? gap : 1);
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
    /* Stored as a u16 of feet. Flight builds are OFF by default (absent key ->
     * disabled); DEMO builds default an absent key to the show altitude instead,
     * so a fresh booth unit speaks "check gear" with no menu visit (see the demo
     * defaults block in config.h). A STORED 0 is an explicit pilot OFF and is
     * honoured in both builds — only "never set" falls through to the default.   */
#if DEMO_MODE
    const float dflt = DEMO_GEAR_CHECK_DEFAULT_FT;
#else
    const float dflt = 0.0f;   /* OFF */
#endif
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return dflt;
    }
    uint16_t v = 0;
    esp_err_t err = nvs_get_u16(h, NVS_KEY_GEARCHK, &v);
    nvs_close(h);
    if (err != ESP_OK) {
        return dflt;   /* absent / unreadable -> build default */
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
    /* Stored as a u8 (0/1). Flight builds are disabled by default: absent /
     * unreadable / zero all read as OFF, so a corrupt value can never enable an
     * unwanted callout. DEMO builds flip the ABSENT-key default to ON so a fresh
     * booth unit announces "positive rate" on the hoist (config.h demo defaults);
     * an explicit stored 0 from the menu still reads as OFF in both builds.      */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return DEMO_MODE != 0;
    }
    uint8_t v = 0;
    esp_err_t err = nvs_get_u8(h, NVS_KEY_POSRATE, &v);
    nvs_close(h);
    if (err != ESP_OK) {
        return DEMO_MODE != 0;
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
    /* Same u8 contract as the positive-rate flag: an explicit stored 0 always
     * reads as OFF. Flight builds also default an ABSENT key to OFF (a corrupt
     * value can never enable blips); DEMO builds default it to ON so the booth
     * unit demos the sink-rate vario out of the box (config.h demo defaults).    */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return DEMO_MODE != 0;
    }
    uint8_t v = 0;
    esp_err_t err = nvs_get_u8(h, NVS_KEY_SINKRATE, &v);
    nvs_close(h);
    if (err != ESP_OK) {
        return DEMO_MODE != 0;
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

/* ---- Tone-start altitude (presence-tone fade-in anchor) ------------------ */

float config_load_tone_start(void)
{
    /* Stored as a u16 of feet, same contract as the start-altitude cap above:
     * absent / unreadable / zero -> the compile-time default so a corrupt value
     * can never push the tone-start out of band or silence the tone.            */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return DEFAULT_TONE_START_FT;
    }
    uint16_t v = 0;
    esp_err_t err = nvs_get_u16(h, NVS_KEY_TONESTART, &v);
    nvs_close(h);
    if (err != ESP_OK || v == 0) {
        return DEFAULT_TONE_START_FT;
    }
    return (float)v;
}

void config_save_tone_start(float ft)
{
    if (ft < 0.0f) {
        ft = 0.0f;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; tone-start altitude not persisted");
        return;
    }
    /* Round to the nearest foot; the menu only offers integer altitudes. */
    uint16_t v = (uint16_t)(ft + 0.5f);
    if (nvs_set_u16(h, NVS_KEY_TONESTART, v) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "tone-start altitude saved: %u ft", v);
    } else {
        ESP_LOGE(TAG, "nvs_set_u16 failed; tone-start altitude not persisted");
    }
    nvs_close(h);
}

void config_wipe_tone_start(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_TONESTART);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "tone-start altitude wiped (will use default)");
}

/* ---- Demo AGL gain (DEMO_MODE builds only; see config.h) ------------------ */
#if DEMO_MODE

float config_load_demo_gain(void)
{
    /* Stored as a u16. This key inverts the usual absent-means-off contract:
     * an ABSENT key returns DEMO_GAIN_DEFAULT (a freshly flashed / wiped demo
     * unit must demo out of the box), while an explicitly stored 0 is the user's
     * OFF choice from the menu. Values outside the sane 2..1000 band (mirroring
     * the bench attach-frame clamp) read as corrupt and yield the default.      */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return DEMO_GAIN_DEFAULT;
    }
    uint16_t v = 0;
    esp_err_t err = nvs_get_u16(h, NVS_KEY_DEMOGAIN, &v);
    nvs_close(h);
    if (err != ESP_OK) {
        return DEMO_GAIN_DEFAULT;   /* absent / unreadable -> demo-ready default */
    }
    if (v == 0) {
        return 0.0f;                /* explicit OFF chosen in the config menu    */
    }
    float g = (float)v;
    if (g < 2.0f || g > 1000.0f) {
        return DEMO_GAIN_DEFAULT;   /* corrupt / absurd -> back to the default   */
    }
    return g;
}

void config_save_demo_gain(float gain)
{
    if (gain < 0.0f) {
        gain = 0.0f;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; demo gain not persisted");
        return;
    }
    /* Round to the nearest integer gain; the menu only offers whole multipliers
     * (or 0 for OFF).                                                            */
    uint16_t v = (uint16_t)(gain + 0.5f);
    if (nvs_set_u16(h, NVS_KEY_DEMOGAIN, v) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "demo gain saved: x%u%s", v, v == 0 ? " (OFF)" : "");
    } else {
        ESP_LOGE(TAG, "nvs_set_u16 failed; demo gain not persisted");
    }
    nvs_close(h);
}

void config_wipe_demo_gain(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_DEMOGAIN);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "demo gain wiped (will use x%.0f default)", (double)DEMO_GAIN_DEFAULT);
}

#endif /* DEMO_MODE */

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

    /*  NVS is persistent storage that outlives firmware versions, so its
     *  contents are UNTRUSTED INPUT exactly like the sensor wire. A blob whose
     *  length is not a whole number of entries did not come from this struct
     *  layout — it is a partial write, a legacy record, or corruption — and
     *  reinterpreting those bytes as floats yields arbitrary "ground readings"
     *  that would be adopted as the aircraft's reference. Reject the whole
     *  record rather than decode a prefix of it; a first-boot calibration is a
     *  far better outcome than a confidently wrong ground.                     */
    if ((blob_len % sizeof(boot_entry_t)) != 0u) {
        ESP_LOGW(TAG, "ground buffer blob is %u bytes, not a multiple of %u "
                      "(corrupt or from an older layout) -> ignoring it",
                 (unsigned)blob_len, (unsigned)sizeof(boot_entry_t));
        nvs_close(h);
        return 0;
    }

    /* Clamp to what the caller can hold and to BOOT_BUFFER_N. */
    size_t n = blob_len / sizeof(boot_entry_t);
    if (n > cap) {
        n = cap;
    }
    if (n == 0u) {
        nvs_close(h);
        return 0;
    }

    /*  nvs_get_blob fails with ESP_ERR_NVS_INVALID_LENGTH when the caller's
     *  buffer is smaller than the stored blob, which is precisely the clamped
     *  case above (a blob holding more entries than we can accept). Read the
     *  whole record into a local of known size and copy across the prefix we
     *  want, so a larger stored buffer degrades gracefully instead of being
     *  discarded outright.                                                     */
    boot_entry_t scratch[BOOT_BUFFER_N];
    size_t avail = blob_len / sizeof(boot_entry_t);
    if (avail > BOOT_BUFFER_N) {
        avail = BOOT_BUFFER_N;
    }
    size_t want = avail * sizeof(boot_entry_t);
    err = nvs_get_blob(h, NVS_KEY_GROUNDBUF, scratch, &want);
    nvs_close(h);

    if (err != ESP_OK) {
        return 0;
    }

    /*  Validate every decoded value before it can reach the estimator. A stored
     *  range that is non-finite, negative, or beyond the ground-fill junk cap
     *  cannot describe a parked aircraft, so it is dropped rather than averaged
     *  in. Entries are compacted so the caller always receives a dense, wholly
     *  usable set and never has to know that filtering happened.               */
    size_t kept = 0;
    size_t limit = (n < avail) ? n : avail;
    for (size_t i = 0; i < limit; ++i) {
        float v = scratch[i].range_ft;
        if (!isfinite(v) || v < 0.0f || v > MAX_VALID_FT) {
            continue;
        }
        out[kept++] = scratch[i];
    }
    if (kept < limit) {
        ESP_LOGW(TAG, "ground buffer: dropped %u implausible stored entries "
                      "(kept %u)", (unsigned)(limit - kept), (unsigned)kept);
    }
    return kept;
}

void boot_buffer_commit(const float *ground_reads, size_t n, uint32_t marker)
{
    if (!ground_reads || n == 0u) {
        /*  Writing an empty record would REPLACE a perfectly good stored
         *  reference with nothing — a silent downgrade to first-boot behaviour
         *  on the next power-up. The caller currently gates this, but the
         *  function must not depend on that: refusing here makes the contract
         *  safe for every future caller.                                       */
        ESP_LOGW(TAG, "boot_buffer_commit: nothing to persist; keeping the "
                      "existing stored reference");
        return;
    }
    if (n > BOOT_BUFFER_N) {
        n = BOOT_BUFFER_N;
    }

    /*  Never persist a value we would refuse to load. Filtering on the way IN
     *  as well as on the way out means a corrupt reading cannot become
     *  tomorrow's ground even if some future caller skips its own validation.  */
    boot_entry_t entries[BOOT_BUFFER_N];
    size_t kept = 0;
    for (size_t i = 0; i < n; ++i) {
        float v = ground_reads[i];
        if (!isfinite(v) || v < 0.0f || v > MAX_VALID_FT) {
            continue;
        }
        entries[kept].range_ft = v;
        entries[kept].marker   = marker;
        ++kept;
    }
    if (kept == 0u) {
        ESP_LOGW(TAG, "boot_buffer_commit: every candidate reading was "
                      "implausible; keeping the existing stored reference");
        return;
    }
    n = kept;

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

#endif /* !UNIT_TEST — end of the persistence / GPIO surface */

/* ---- Ground-reference resolution ----------------------------------------- */
/*  PURE from here down: no NVS, no GPIO, no clock. This is the function that
 *  decides where the ground is and whether the box can trust its calibration,
 *  so it is exercised directly by test/test_boot_buffer.c.                    */

void boot_buffer_resolve(const boot_entry_t *stored, size_t n_stored,
                         float current_ft, bool have_current,
                         boot_result_t *result)
{
    /*  A non-finite current reading must never reach the comparisons below.
     *  Every one of them is a floating-point ordering test, and EVERY such test
     *  against NaN is false — so a NaN silently takes whichever branch the code
     *  happens to fall through to, and the `boot_agl < 0` clamp further down is
     *  provably ineffective against it (NaN < 0 is false, so the clamp does not
     *  fire and the NaN propagates into the seeded state). Treat it as "no
     *  current reading at all", which is exactly what it is.                    */
    if (have_current && !isfinite(current_ft)) {
        have_current = false;
        current_ft   = 0.0f;
    }

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
        /* No usable stored ground. Adopt the current reading as the reference
         * ONLY when it (a) is REAL sensor data (have_current) and (b) sits in a
         * plausible MOUNT-OFFSET band. The old test accepted anything up to
         * MAX_VALID_FT — but that constant is a fill-selection junk cap, not a
         * ground bound: a real mount reads ~MOUNT_OFFSET_FALLBACK_FT, so a
         * 20-50 ft "ground" is physically impossible (it is a mid-air reboot
         * over the runway, which must NOT silently zero its own AGL), and a
         * totally silent sensor's fabricated 0.0 initialiser used to pass as a
         * perfect ground here. Both defeated the calibration warning built for
         * exactly these cases; both now fall through to the emergency offset
         * and chirp the pilot.                                                */
        /*  The band was MOUNT_OFFSET_FALLBACK_FT + GROUND_DEV_FT = 13 ft, which
         *  is more than four times a real mount height and squarely inside the
         *  altitudes a bounced landing or a low pass occupies. A freshly wiped
         *  box (the config menu erases the buffer) that power-glitches at 12 ft
         *  AGL therefore adopted 12.4 ft as "ground" with calib_error CLEAR: no
         *  chirp, no warning, the ladder seeded DISARMED for the whole landing,
         *  and the airborne value written to NVS to poison later flights.
         *
         *  MOUNT_GROUND_MAX_FT bounds it to what a mount can physically read
         *  instead. Anything above falls to the emergency-offset branch, which
         *  chirps the pilot and refuses to persist — the honest answer when the
         *  box genuinely cannot tell where the ground is.                       */
        if (have_current && current_ft >= 0.0f &&
            current_ft <= MOUNT_GROUND_MAX_FT) {
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
    float dev = current_ft - ground_ref;      /* signed: +above, -below */
    float agl = dev;
    if (agl < 0.0f) {
        agl = 0.0f;
    }

    result->ground_ref_ft = ground_ref;
    result->boot_agl_ft   = agl;
    result->airborne      = dev > GROUND_DEV_FT;

    /*  DISAGREEMENT IS TWO-SIDED. The old test looked only at the HIGH side and
     *  then cleared calib_error unconditionally, which left the low side — a
     *  reading well BELOW the learned ground — indistinguishable from a perfect
     *  on-ground boot. It is not: nothing legitimately parks the aircraft
     *  measurably closer to the sensor than the ground it learned on.
     *
     *  The realistic causes are an OBSTRUCTION under the sensor (a mechanic's
     *  shoulder, a tow bar, a chock, a puddle giving a specular return) or the
     *  aircraft sitting on JACKS. Both were silently clamped to agl = 0 and
     *  reported healthy — and because the caller's persist gate keys off
     *  !airborne && !calib_error, the obstructed reading was then WRITTEN TO
     *  NVS as the new learned ground. That is the worst failure in this file:
     *  it is not confined to the current flight, it silently offsets every
     *  future one until someone recalibrates on a clean surface.
     *
     *  The threshold is NOT GROUND_DEV_FT. That constant bounds how far ABOVE
     *  the ground the aircraft can be and still count as parked, and 10 ft is
     *  sensible there because the sky is unbounded. Downward it is nonsense:
     *  the ground sits only ~MOUNT_OFFSET_FALLBACK_FT below the sensor, so a
     *  reading cannot be 10 ft below it without being negative. Reusing it
     *  would make the low-side test unreachable — the exact bug this guard
     *  exists to close. GROUND_BELOW_DEV_FT is scaled to the mount instead:
     *  large enough to pass strut compression, surface variation and sensor
     *  noise, small enough that a solid object in the beam is caught.          */
    result->calib_error = (dev < -GROUND_BELOW_DEV_FT);
}
