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
 */
void audio_set_params(float tone_agl, bool tone_active);

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
 *          stereo hardware frame); pan settings do not apply to prompts.
 */
void audio_play_clip_blocking(const clip_t *c);

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
