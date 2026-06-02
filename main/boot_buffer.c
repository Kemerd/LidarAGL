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
