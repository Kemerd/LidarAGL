/**
 * @file    audio.h
 * @brief   I2S audio engine: presence tone + voice callouts into the PCM5102A.
 *
 * @details Runs as a dedicated task on core 1, owning the i2s_std TX channel.
 *          It continuously renders the output: an ascending presence tone whose
 *          pitch and dB-scheduled level track the current AGL, mixed with voice
 *          callout clips that duck the tone while they play. All the perceptual
 *          math is in audio_math.c; this module is the hardware + mixing engine.
 */
#ifndef LIDARAGL_AUDIO_H
#define LIDARAGL_AUDIO_H

#include <stdbool.h>
#include <stdint.h>
#include "callouts.h"

/**
 * @brief Resolved runtime audio behaviour (chosen via the boot config menu).
 *
 * @details The I2S peripheral always runs in stereo at the hardware level; this
 *          struct decides what we actually render into it:
 *            - stereo == false  -> the same signal goes to L and R (mono, safe
 *              even if the panel is wired stereo).
 *            - stereo == true   -> the two streams are gently panned apart
 *              (voice right, tone left) by STEREO_PAN.
 *          The stream-enable flags gate each source independently so the
 *          callouts-only and tone-only modes can suppress the other stream.
 */
typedef struct {
    bool stereo;            /**< true = pan streams apart; false = duplicate L=R. */
    bool callouts_enabled;  /**< false = never play voice clips.                  */
    bool tone_enabled;      /**< false = never sound the presence tone.           */
} audio_config_t;

/**
 * @brief Translate an AUDIO_MODE_* index (config.h) into an audio_config_t.
 * @param mode  One of AUDIO_MODE_* ; out-of-range falls back to the default.
 * @return      The resolved behaviour flags.
 */
audio_config_t audio_config_from_mode(int mode);

/**
 * @brief Initialise the I2S channel and the audio engine state.
 *
 * @param cfg  Resolved runtime behaviour (channel layout + enabled streams).
 *
 * @details Configures i2s_std (16-bit STEREO, SAMPLE_RATE, no MCLK since the
 *          PCM5102A uses its internal PLL with SCK tied to GND) and builds the
 *          sine lookup table. The stereo hardware mode is fixed; cfg only changes
 *          what we render (pan vs duplicate, which streams are live).
 */
void audio_init(const audio_config_t *cfg);

/**
 * @brief Publish the current tone parameters (called by the logic task).
 * @param tone_agl     AGL the tone should track (already smoothed).
 * @param tone_active  Whether the presence tone should sound.
 * @param vert_fps     Smoothed vertical rate (ft/s, +up) driving the vario blip
 *                     cadence (faster sink => faster blips).
 */
void audio_set_params(float tone_agl, bool tone_active, float vert_fps);

/**
 * @brief Select the variometer "blip" active direction (called once at boot).
 *
 * @param sink_on   Sink mode: the below-100 ft tone blips faster the faster you
 *                  DESCEND; level/climbing => a steady constant tone.
 * @param climb_on  Climb mode (the inverse): blips on the way UP, constant tone when
 *                  level/sinking.
 *
 * @details The two are MUTUALLY EXCLUSIVE — the boot menu offers OFF / Sink / Climb,
 *          so at most one is set; with NEITHER set the gate stays open (steady tone).
 *          Below the onset rate the tone is constant in every mode (see the VARIO_*
 *          mapping in config.h). Flags come from NVS at boot (config_load_sink_rate /
 *          config_load_climb_rate); lock-free bool stores read by the render loop.
 */
void audio_set_vario_enable(bool sink_on, bool climb_on);

/**
 * @brief Set the altitude (ft AGL) at which the presence tone begins (called once
 *        at boot).
 *
 * @param ft  Tone-start altitude in feet — the pilot's config-menu choice
 *            (TONE_START_FT by default, or the higher TONE_START_FT_HIGH). Both the
 *            pitch sweep and the dB fade-in anchor on it.
 *
 * @details Forwards to audio_math_set_tone_start() (the pure schedule's anchor) and
 *          re-derives the smoothed-AGL slew so the pitch still crosses the whole
 *          band in ~250 ms. Loaded from NVS by app_main (config_load_tone_start).
 */
void audio_set_tone_start(float ft);

/**
 * @brief Set the pilot's TONE volume offset (dB), applied to the presence tone.
 *
 * @param db  Offset in dB, clamped to [TONE_VOLUME_DB_MIN, TONE_VOLUME_DB_MAX].
 *            A trim layered on the analog pot that can CUT or BOOST the tone only
 *            (the voice is untouched). Converted once to a linear gain and folded
 *            into the tone every sample.
 *
 * @details Set at boot from NVS, and driven live by the volume menu's per-step
 *          preview. Cheap and lock-free (a single float).
 */
void audio_set_tone_db(float db);

/**
 * @brief Set the pilot's VOICE volume offset (dB), applied to the voice callouts.
 *
 * @param db  Offset in dB (<= 0), clamped to [VOICE_VOLUME_DB_MIN, 0]. A cut-only
 *            trim layered on the analog pot that lowers the callouts only (the
 *            tone is untouched). Converted once to a linear gain and folded into
 *            the voice every sample.
 *
 * @details Set at boot from NVS, and driven live by the volume menu's per-step
 *          preview. Cheap and lock-free (a single float).
 */
void audio_set_voice_db(float db);

/**
 * @brief Request a voice callout (called by the logic task).
 * @param id  The callout to play. Missing clips are skipped gracefully.
 */
void audio_request_callout(callout_id_t id);

/**
 * @brief Play the calibration-error chirp once (boot warning).
 */
void audio_play_chirp(void);

/**
 * @brief Synchronously play one clip to completion (centered, full level).
 *
 * @param c  Clip to play; NULL / absent clips are skipped silently.
 *
 * @details Used by the boot config menu to voice prompts before the render
 *          tasks exist. Blocks until the clip has been written to the DMA, so
 *          callers stay simple. Always mono-centered (duplicated to L+R in the
 *          stereo hardware frame); pan settings do not apply to prompts. The
 *          current VOICE volume offset (audio_set_voice_db) is applied, since
 *          these are voice clips, so menu prompts track the chosen voice level.
 */
void audio_play_clip_blocking(const clip_t *c);

/**
 * @brief Synchronously play a fixed-frequency sine burst (config-menu preview).
 *
 * @param freq_hz  Tone frequency in Hz.
 * @param ms       Burst length in milliseconds.
 * @param level_db Burst level in dB relative to full scale (<= 0).
 *
 * @details Used by the boot volume menu to voice the example tone in
 *          "tone .. <number> .. tone" so the pilot hears the chosen level. Like
 *          audio_play_clip_blocking() it runs before the render tasks exist and
 *          is always mono-centered. The CURRENT tone volume offset (set via
 *          audio_set_tone_db) is applied, so the preview matches exactly what the
 *          tone will sound like after the menu commits. Short raised-cosine fades
 *          top and tail the burst so there is no click.
 */
void audio_play_tone_blocking(float freq_hz, int ms, float level_db);

/**
 * @brief Synchronously play the volume-menu "mini-flare" balance preview.
 *
 * @details A short scripted descent the pilot uses to judge the tone-vs-voice
 *          BALANCE exactly as it flies: the REAL presence tone sweeps DOWN the
 *          VOLUME_PREVIEW_SWEEP_FROM_FT -> _TO_FT band (pitch + level on the actual
 *          agl_to_pitch_hz / agl_to_tone_db schedule) in the background, while the
 *          "20" and "10" callouts speak over it and DUCK it through the very same
 *          sidechain (duck_step) the flight render loop uses. The live TONE and
 *          VOICE volume offsets (audio_set_tone_db / audio_set_voice_db) are both
 *          applied, so each menu step auditions the chosen balance. Runs before the
 *          render tasks exist, mono-centered, with click-free fades top and tail.
 */
void audio_play_volume_preview_blocking(void);

/**
 * @brief Pause the I2S channel (before MCU light-sleep in GROUND/CRUISE).
 * @details The tone is silent in those states anyway; pausing lets the channel
 *          clocks gate so light-sleep can take effect without DAC glitches.
 */
void audio_suspend(void);

/**
 * @brief Resume the I2S channel (on the way back into active states).
 * @details Re-enables the channel; the render loop ramps the tone back in via a
 *          raised-cosine envelope so there is no click on wake.
 */
void audio_resume(void);

/**
 * @brief FreeRTOS entry point for the audio task (core 1).
 */
void audio_task(void *arg);

#endif /* LIDARAGL_AUDIO_H */
