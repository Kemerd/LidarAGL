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
#include "callouts.h"

/**
 * @brief Initialise the I2S channel and the audio engine state.
 *
 * @details Configures i2s_std (16-bit mono, SAMPLE_RATE, no MCLK since the
 *          PCM5102A uses its internal PLL with SCK tied to GND) and builds the
 *          sine lookup table.
 */
void audio_init(void);

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
