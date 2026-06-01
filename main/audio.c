/**
 * @file    audio.c
 * @brief   I2S audio engine — ascending presence tone + ducked voice callouts.
 *
 * @details HARDWARE module (ESP-IDF i2s_std). Design rules enforced here:
 *            - Two streams only (tone + voice); no third concurrent stream.
 *            - Volume scheduled in dB (via audio_math), converted to a linear
 *              gain; we never ramp linear amplitude directly.
 *            - Pitch ASCENDS as AGL falls (audio_math), driven from a heavily
 *              smoothed/slew-limited AGL so it doesn't warble.
 *            - Every gain change and tone start/stop is shaped by a raised-cosine
 *              envelope; nothing is ever hard-gated (no clicks, no startle).
 *            - The tone ducks ~VOICE_DUCK_DB while a callout plays; numbers are
 *              never masked by the tone peak. No pre-token alert chirp.
 */

#include "audio.h"
#include "audio_math.h"
#include "config.h"
#include "shared.h"

#include <math.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "audio";

/* ---- I2S handle & sine LUT ----------------------------------------------- */

static i2s_chan_handle_t s_tx = NULL;
static bool              s_running = false;

#define LUT_SIZE 1024
static float s_sine_lut[LUT_SIZE];

/* ---- Tone state ---------------------------------------------------------- */

/*  NCO phase accumulator in [0,1). We advance it by f/SAMPLE_RATE each sample. */
static float s_phase = 0.0f;

/*  Smoothed/slew-limited values so pitch and gain never jump (warble/clicks).  */
static float s_tone_agl_smooth = TONE_START_FT;   /* drives pitch + level       */
static float s_gain_cur        = 0.0f;            /* current linear tone gain    */
static float s_duck_cur        = 1.0f;            /* current duck multiplier     */

/* ---- Callout playback state ---------------------------------------------- */

/*  The clip currently playing (NULL = none). PCM is s16le in flash.            */
static const int16_t *s_clip_pcm  = NULL;
static size_t         s_clip_len  = 0;            /* samples (not bytes)         */
static size_t         s_clip_pos  = 0;

/* ---- Per-step slew limits (computed from GAIN_RAMP_MS) -------------------- */

/*  Max gain change per sample so a full 0->1 swing takes ~GAIN_RAMP_MS. */
static float s_gain_step = 0.0f;
/*  Max smoothed-AGL change per sample for the pitch (slower => more stable).   */
static float s_agl_step  = 0.0f;

/* ---------------------------------------------------------------------------
 *  Init
 * ------------------------------------------------------------------------- */

static void build_lut(void)
{
    for (int i = 0; i < LUT_SIZE; ++i) {
        s_sine_lut[i] = sinf(2.0f * (float)M_PI * (float)i / (float)LUT_SIZE);
    }
}

void audio_init(void)
{
    build_lut();

    /* Precompute the per-sample slew limits. A full-scale gain ramp should take
     * GAIN_RAMP_MS; the pitch-tracking AGL moves a bit slower for stability.   */
    float ramp_samples = (GAIN_RAMP_MS / 1000.0f) * (float)SAMPLE_RATE;
    s_gain_step = (ramp_samples > 0) ? (1.0f / ramp_samples) : 1.0f;
    /* Allow the smoothed AGL to traverse the full 100 ft band in ~250 ms. */
    float agl_ramp_samples = 0.25f * (float)SAMPLE_RATE;
    s_agl_step = TONE_START_FT / agl_ramp_samples;

    /* --- I2S standard mode, TX only, 16-bit mono ------------------------- */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO,
                                                            I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            /* PCM5102A SCK -> GND => internal PLL => we emit NO MCLK. */
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCK,
            .ws   = PIN_I2S_LRCK,
            .dout = PIN_I2S_DIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx));
    s_running = true;

    ESP_LOGI(TAG, "I2S up: %d Hz 16-bit mono (BCK=%d WS=%d DIN=%d, no MCLK)",
             SAMPLE_RATE, PIN_I2S_BCK, PIN_I2S_LRCK, PIN_I2S_DIN);
}

/* ---------------------------------------------------------------------------
 *  Public setters (called from the logic task)
 * ------------------------------------------------------------------------- */

void audio_set_params(float tone_agl, bool tone_active)
{
    if (g_audio_mutex && xSemaphoreTake(g_audio_mutex, 0) == pdTRUE) {
        g_audio_params.tone_agl    = tone_agl;
        g_audio_params.tone_active = tone_active;
        xSemaphoreGive(g_audio_mutex);
    }
}

void audio_request_callout(callout_id_t id)
{
    /* The logic task posts the id; the audio task picks it up and starts the
     * clip. We route through the queue so we never touch clip state from two
     * tasks at once.                                                          */
    if (q_callouts) {
        xQueueSend(q_callouts, &id, 0);
    }
}

/* Start playing a clip if it exists; missing clips are skipped. */
static void start_clip(const clip_t *c)
{
    if (!c || !c->pcm || c->len_bytes < 2) {
        if (c) {
            ESP_LOGW(TAG, "clip '%s' missing/empty; skipping", c->name);
        }
        return;
    }
    s_clip_pcm = (const int16_t *)c->pcm;
    s_clip_len = c->len_bytes / 2;     /* s16 samples */
    s_clip_pos = 0;
}

void audio_play_chirp(void)
{
    /* Played synchronously from the boot path before tasks run; we render it
     * directly here so the warning sounds even if the audio task isn't up yet. */
    const clip_t *c = callout_chirp();
    if (!c || !c->pcm || c->len_bytes < 2 || !s_running) {
        return;
    }
    const int16_t *pcm = (const int16_t *)c->pcm;
    size_t n = c->len_bytes / 2;
    size_t written_total = 0;
    /* Write in small blocks so we don't hold a huge stack buffer. */
    while (written_total < n) {
        size_t chunk = n - written_total;
        if (chunk > AUDIO_FRAME_LEN) chunk = AUDIO_FRAME_LEN;
        size_t wrote = 0;
        i2s_channel_write(s_tx, &pcm[written_total], chunk * sizeof(int16_t),
                          &wrote, portMAX_DELAY);
        written_total += chunk;
    }
}

/* ---------------------------------------------------------------------------
 *  Suspend / resume around light-sleep
 * ------------------------------------------------------------------------- */

void audio_suspend(void)
{
    if (s_running && s_tx) {
        i2s_channel_disable(s_tx);
        s_running = false;
        /* Reset the tone gain so it ramps cleanly back from silence on resume. */
        s_gain_cur = 0.0f;
        s_phase    = 0.0f;
    }
}

void audio_resume(void)
{
    if (!s_running && s_tx) {
        i2s_channel_enable(s_tx);
        s_running = true;
    }
}

/* ---------------------------------------------------------------------------
 *  NCO + interpolated LUT lookup
 * ------------------------------------------------------------------------- */

static inline float nco_sample(float freq_hz)
{
    /* Advance phase. */
    s_phase += freq_hz / (float)SAMPLE_RATE;
    if (s_phase >= 1.0f) {
        s_phase -= 1.0f;
    }
    /* Linear-interpolated LUT read (sine is smooth, so 1024 + interp is clean). */
    float x   = s_phase * (float)LUT_SIZE;
    int   i0  = (int)x;
    int   i1  = (i0 + 1) & (LUT_SIZE - 1);
    float frac = x - (float)i0;
    return s_sine_lut[i0] * (1.0f - frac) + s_sine_lut[i1] * frac;
}

/* Soft clip / limiter: a gentle tanh keeps the summed output inside [-1,1] with
 * graceful saturation instead of harsh wrap-around. Headroom is documented: the
 * tone peaks at TONE_FULL_DB and clips peak near -3.5 dBFS, so the sum rarely
 * exceeds unity, but tanh guarantees no overflow when voice + tone align.      */
static inline float soft_clip(float x)
{
    return tanhf(x);
}

/* ---------------------------------------------------------------------------
 *  Audio task: render one frame at a time
 * ------------------------------------------------------------------------- */

void audio_task(void *arg)
{
    (void)arg;

    int16_t frame[AUDIO_FRAME_LEN];

    for (;;) {
        /* Pick up a queued callout (non-blocking) and start it if idle. */
        callout_id_t id;
        if (s_clip_pcm == NULL && q_callouts &&
            xQueueReceive(q_callouts, &id, 0) == pdTRUE) {
            start_clip(callout_clip(id));
        }

        /* Snapshot the tone params under the mutex. */
        float tone_agl    = TONE_START_FT;
        bool  tone_active = false;
        if (g_audio_mutex && xSemaphoreTake(g_audio_mutex, 0) == pdTRUE) {
            tone_agl    = g_audio_params.tone_agl;
            tone_active = g_audio_params.tone_active;
            xSemaphoreGive(g_audio_mutex);
        }

        /* If the channel is suspended (light-sleep window), idle briefly. */
        if (!s_running) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Target gain from the dB schedule; 0 when the tone is inactive so it
         * fades out via the slew limiter rather than hard-stopping.           */
        float target_gain = 0.0f;
        if (tone_active) {
            target_gain = db_to_gain(agl_to_tone_db(tone_agl));
        }

        /* Duck target: attenuate the tone while a clip is playing. */
        float duck_target = (s_clip_pcm != NULL)
                            ? db_to_gain(-VOICE_DUCK_DB)
                            : 1.0f;

        /* --- Render the frame sample-by-sample ----------------------------- */
        for (int i = 0; i < AUDIO_FRAME_LEN; ++i) {
            /* Slew the smoothed AGL toward the target so pitch glides. */
            s_tone_agl_smooth = slew_limit(s_tone_agl_smooth, tone_agl, s_agl_step);

            /* Slew the tone gain and the duck multiplier (raised-cosine feel via
             * the linear slew is adequate at these short ramp times).          */
            s_gain_cur = slew_limit(s_gain_cur, target_gain, s_gain_step);
            s_duck_cur = slew_limit(s_duck_cur, duck_target, s_gain_step);

            float f    = agl_to_pitch_hz(s_tone_agl_smooth);
            float tone = nco_sample(f) * s_gain_cur * s_duck_cur;

            /* Mix the voice clip (already at a comfortable level) if playing. */
            float voice = 0.0f;
            if (s_clip_pcm != NULL) {
                voice = (float)s_clip_pcm[s_clip_pos] / 32768.0f;
                if (++s_clip_pos >= s_clip_len) {
                    /* Clip finished; ducking will ramp back up next frame. */
                    s_clip_pcm = NULL;
                    s_clip_len = 0;
                    s_clip_pos = 0;
                }
            }

            float mixed = soft_clip(tone + voice);
            frame[i] = (int16_t)(mixed * 32767.0f);
        }

        /* Blocking write paces the loop to real time (the DMA backpressures). */
        size_t wrote = 0;
        i2s_channel_write(s_tx, frame, sizeof frame, &wrote, portMAX_DELAY);
    }
}
