/**
 * @file    boot_buffer.h
 * @brief   Learned ground reference in NVS + in-flight-reboot recovery.
 *
 * @details The box has no static calibration. Instead it LEARNS its ground/mount
 *          offset from the last set of on-ground readings and persists them in
 *          NVS (one write per boot — the whole persistence contract).
 *
 *          The model (as specified by the builder):
 *            - The stored buffer holds the last BOOT_BUFFER_N on-ground readings.
 *              Their robust average is the ground reference. AGL = range - ground.
 *            - On boot we take a fresh current reading. If it is more than
 *              GROUND_DEV_FT above the stored ground average, the box was powered
 *              up (or rebooted) IN FLIGHT: we keep the stored ground reference,
 *              do NOT overwrite it, and report AGL = current - ground.
 *            - If there is no usable stored reference AND the current reading
 *              looks airborne, we fall back to MOUNT_OFFSET_FALLBACK_FT (3 ft)
 *              as an emergency ground and flag a calibration error so the pilot
 *              is warned by a boot chirp.
 *            - A momentary reset button wipes the NVS buffer and reboots, for a
 *              clean install/reinstall.
 *
 *          The outlier math is the pure robust_estimate(); this module is the
 *          thin NVS/GPIO wrapper around it.
 */
#ifndef LIDARAGL_BOOT_BUFFER_H
#define LIDARAGL_BOOT_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** One persisted ground reading. */
typedef struct {
    float    range_ft;   /**< On-ground range in feet.                         */
    uint32_t marker;     /**< Boot counter / uptime marker (no RTC required).  */
} boot_entry_t;

/** Outcome of the boot ground-reference reconstruction. */
typedef struct {
    float ground_ref_ft;   /**< Ground/mount offset to subtract from range.    */
    float boot_agl_ft;     /**< Reconstructed AGL at boot (range - ground).    */
    bool  airborne;        /**< True if the box booted in flight.              */
    bool  calib_error;     /**< True if no real ground reference existed.      */
} boot_result_t;

/**
 * @brief Initialise NVS (erase-retry on version mismatch) for the boot buffer.
 */
void boot_buffer_init(void);

/**
 * @brief Read whether the reset button is currently pressed (debounced).
 * @return True if the button is held in its active state.
 */
bool boot_buffer_reset_pressed(void);

/**
 * @brief Lightly-debounced instantaneous button state (for the config menu).
 *
 * @return True if the button reads its active level on two quick spaced samples.
 *
 * @details Cheaper than boot_buffer_reset_pressed() (which spaces five samples
 *          over ~25 ms): this samples twice ~3 ms apart so the config-menu loop
 *          can do its own edge detection while staying responsive to taps.
 */
bool boot_buffer_button_down(void);

/**
 * @brief Wipe the stored ground buffer and reboot the MCU.
 *
 * @details Erases the NVS key and calls esp_restart(). Does not return.
 */
void boot_buffer_wipe_and_reboot(void);

/**
 * @brief Erase the stored ground reference only (does NOT reboot).
 *
 * @details Used by the boot config menu, which wipes both the ground reference
 *          and the saved audio config in one pass, then continues into the menu
 *          rather than rebooting immediately.
 */
void boot_buffer_wipe_ground(void);

/**
 * @brief Load the saved audio-mode index (AUDIO_MODE_*) from NVS.
 * @return The stored mode, or DEFAULT_AUDIO_MODE if none is saved / on error.
 */
int config_load_audio_mode(void);

/**
 * @brief Persist the chosen audio-mode index (AUDIO_MODE_*) to NVS.
 * @param mode  The AUDIO_MODE_* index to store.
 */
void config_save_audio_mode(int mode);

/**
 * @brief Erase the saved audio config (the next load returns the default).
 */
void config_wipe_audio_mode(void);

/**
 * @brief Load the saved callout start-altitude cap (ft) from NVS.
 *
 * @param default_ft  Value to return when none is saved (the profile top
 *                    callout, since the default depends on the fitted sensor).
 * @return            The stored cap in feet, or @p default_ft if absent/invalid.
 */
float config_load_start_alt(float default_ft);

/**
 * @brief Persist the chosen callout start-altitude cap (ft) to NVS.
 * @param ft  Cap in feet (stored as a u16; rounded).
 */
void config_save_start_alt(float ft);

/**
 * @brief Erase the saved start-altitude cap (the next load returns the default).
 */
void config_wipe_start_alt(void);

/**
 * @brief Load the saved VOICE volume offset (dB) from NVS.
 *
 * @details A cut-only trim on the voice callouts layered on the analog pot: 0 dB
 *          is no cut (the default), down to VOICE_VOLUME_DB_MIN. Stored as a small
 *          non-negative CUT magnitude (a u8 of |dB|, e.g. 4 == -4 dB) so a
 *          corrupt/absent value can never silence the callouts.
 *
 * @return The stored offset in dB (<= 0), or DEFAULT_VOICE_VOLUME_DB if absent.
 */
float config_load_voice_volume(void);

/**
 * @brief Persist the chosen VOICE volume offset (dB) to NVS.
 * @param db  Offset in dB (<= 0); clamped to [VOICE_VOLUME_DB_MIN, 0] and stored
 *            as a u8 cut magnitude.
 */
void config_save_voice_volume(float db);

/**
 * @brief Erase the saved voice volume offset (next load returns the default 0 dB).
 */
void config_wipe_voice_volume(void);

/**
 * @brief Load the saved TONE volume offset (dB) from NVS.
 *
 * @details A trim on the presence tone that can CUT or BOOST, layered on the
 *          analog pot: 0 dB is no change (the default), spanning TONE_VOLUME_DB_MIN
 *          .. TONE_VOLUME_DB_MAX. Stored SIGNED (an i8 of dB) since it goes both
 *          ways; a corrupt/absent value falls back to the default and is clamped
 *          to the legal range.
 *
 * @return The stored offset in dB, or DEFAULT_TONE_VOLUME_DB if absent.
 */
float config_load_tone_volume(void);

/**
 * @brief Persist the chosen TONE volume offset (dB) to NVS.
 * @param db  Offset in dB; clamped to [TONE_VOLUME_DB_MIN, TONE_VOLUME_DB_MAX] and
 *            stored as a signed i8.
 */
void config_save_tone_volume(float db);

/**
 * @brief Erase the saved tone volume offset (next load returns the default 0 dB).
 */
void config_wipe_tone_volume(void);

/**
 * @brief Load the saved "check gear" descent-callout altitude (ft) from NVS.
 *
 * @details Stored as a u16 of feet. The feature is OFF (disabled) by default, so
 *          an ABSENT key — or an explicitly stored 0 — both mean OFF; any other
 *          value is the gear-check altitude in feet.
 *
 * @return The stored altitude in feet, or 0 (OFF) if absent / unreadable.
 */
float config_load_gear_check_alt(void);

/**
 * @brief Persist the chosen "check gear" altitude (ft) to NVS.
 * @param ft  Altitude in feet, or 0 to disable (OFF). Stored as a u16 (rounded).
 */
void config_save_gear_check_alt(float ft);

/**
 * @brief Erase the saved gear-check altitude (next load returns OFF).
 */
void config_wipe_gear_check_alt(void);

/**
 * @brief Load the saved "positive rate" climb-callout enable flag from NVS.
 *
 * @details Stored as a u8 (0/1). The feature is DISABLED by default, so an absent
 *          / unreadable / zero value all mean disabled.
 *
 * @return true if the positive-rate callout is enabled, false otherwise.
 */
bool config_load_positive_rate(void);

/**
 * @brief Persist the "positive rate" climb-callout enable flag to NVS.
 * @param enabled  Whether the callout is enabled.
 */
void config_save_positive_rate(bool enabled);

/**
 * @brief Erase the saved positive-rate flag (next load returns disabled).
 */
void config_wipe_positive_rate(void);

/**
 * @brief Load the saved "sink rate" vario-blip enable flag from NVS.
 *
 * @details Stored as a u8 (0/1). DISABLED by default, so an absent / unreadable /
 *          zero value all mean OFF (the steady tone). When ON the presence tone
 *          blips with the descent rate (see the VARIO_* tunables in config.h).
 *
 * @return true if the sink-rate vario blip is enabled, false otherwise.
 */
bool config_load_sink_rate(void);

/**
 * @brief Persist the "sink rate" vario-blip enable flag to NVS.
 * @param enabled  Whether the feature is enabled.
 */
void config_save_sink_rate(bool enabled);

/**
 * @brief Erase the saved sink-rate flag (next load returns disabled).
 */
void config_wipe_sink_rate(void);

/**
 * @brief Load the saved "climb rate" vario-blip enable flag from NVS.
 *
 * @details Stored as a u8 (0/1). DISABLED by default. When ON, a CLIMB also drives
 *          the tone blips; with it OFF a climb is treated as 0 fpm. Only meaningful
 *          alongside the sink-rate feature (both share the blip gate).
 *
 * @return true if the climb-rate vario blip is enabled, false otherwise.
 */
bool config_load_climb_rate(void);

/**
 * @brief Persist the "climb rate" vario-blip enable flag to NVS.
 * @param enabled  Whether the feature is enabled.
 */
void config_save_climb_rate(bool enabled);

/**
 * @brief Erase the saved climb-rate flag (next load returns disabled).
 */
void config_wipe_climb_rate(void);

/**
 * @brief Load the stored ground readings.
 * @param out  Destination array (capacity @p cap).
 * @param cap  Capacity in entries.
 * @return     Number of entries loaded (0 if none / first boot).
 */
size_t boot_buffer_load(boot_entry_t *out, size_t cap);

/**
 * @brief Reconstruct the ground reference and boot AGL from stored + current.
 *
 * @param stored        Stored ground entries.
 * @param n_stored      Number of stored entries.
 * @param current_ft    The fresh current reading in feet.
 * @param[out] result   Filled with ground ref, boot AGL, airborne & error flags.
 */
void boot_buffer_resolve(const boot_entry_t *stored, size_t n_stored,
                         float current_ft, boot_result_t *result);

/**
 * @brief Persist a fresh set of on-ground readings — the ONE NVS write per boot.
 *
 * @param ground_reads  Newly captured on-ground readings.
 * @param n             Number of readings (clamped to BOOT_BUFFER_N).
 * @param marker        Boot/uptime marker stamped onto the entries.
 */
void boot_buffer_commit(const float *ground_reads, size_t n, uint32_t marker);

#endif /* LIDARAGL_BOOT_BUFFER_H */
