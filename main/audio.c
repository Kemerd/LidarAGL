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
 *            - Channel layout is a RUNTIME choice (set by the boot config menu,
 *              stored in NVS). The I2S hardware always runs stereo so the unit
 *              works however the panel is wired; s_cfg.stereo decides whether we
 *              pan the streams apart (voice right, tone left, equal-power
 *              STEREO_PAN) or send identical audio to both channels (mono). The
 *              callouts/tone enable flags gate each stream for the
 *              callouts-only / tone-only modes.
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

/*  Resolved runtime behaviour (channel layout + which streams are live). Set by
 *  audio_init() from the boot config; read in the render loop. Defaults are
 *  harmless until init runs.                                                    */
static audio_config_t s_cfg = { .stereo = true, .callouts_enabled = true,
                                .tone_enabled = true };

/*  The DMA frame is ALWAYS interleaved stereo (the hardware runs stereo); mono
 *  modes simply write the same sample to both channels.                         */
#define AUDIO_CH 2

#define LUT_SIZE 1024
static float s_sine_lut[LUT_SIZE];

/* ---- Tone state ---------------------------------------------------------- */

/*  NCO phase accumulator in [0,1). We advance it by f/SAMPLE_RATE each sample. */
static float s_phase = 0.0f;

/*  Smoothed/slew-limited values so pitch and gain never jump (warble/clicks).  */
static float s_tone_agl_smooth = TONE_START_FT;   /* drives pitch + level       */
static float s_gain_cur        = 0.0f;            /* current linear tone gain    */
static float s_duck_cur        = 1.0f;            /* current duck multiplier     */

/*  Flare fade multiplier: 1.0 = tone fully present, 0.0 = faded out under the
 *  flare. It slews toward 0 (slowly) below FLARE_FADE_FT and back toward 1
 *  (quickly) above it. Multiplied into the tone gain so a re-cross part-way
 *  through the fade reverses smoothly from wherever the envelope sits.          */
static float s_flare_fade      = 1.0f;            /* current flare-fade level    */

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

/*  Flare-fade slew steps (per sample): a full 1->0 swing takes FLARE_FADE_OUT_MS
 *  on the way DOWN, and FLARE_FADE_IN_MS on the way UP. The asymmetry is what
 *  makes "leave the flare -> tone snaps back" feel instant while "enter the
 *  flare -> tone eases out" stays gentle.                                       */
static float s_fade_out_step = 0.0f;   /* toward silence (slow) */
static float s_fade_in_step  = 0.0f;   /* toward full   (fast) */

/* ---- Stereo pan weights (compile-time constants) ------------------------- */
/*  Equal-power pan used when s_cfg.stereo is set: a stream's energy is split so
 *  STEREO_PAN of it leans to its far channel and the rest stays near, with
 *  NEAR^2 + FAR^2 == 1 so panning never changes a stream's perceived loudness.
 *  The VOICE leans RIGHT and the TONE leans LEFT (opposite sides): the right ear
 *  has a well-documented advantage for processing speech (the right-ear /
 *  dichotic-listening effect — right ear -> left auditory cortex, the
 *  language-dominant hemisphere in most people), so the spoken numbers get the
 *  speech-favoured ear while the tone takes the other. The result is a steady
 *  center image with gentle separation. At STEREO_PAN = 0.15 it is ~0.92/~0.27. */
#define PAN_NEAR  (sqrtf(1.0f - 0.5f * (STEREO_PAN)))   /* dominant side         */
#define PAN_FAR   (sqrtf(0.5f * (STEREO_PAN)))          /* bled-across side      */

/* ---------------------------------------------------------------------------
 *  Init
 * ------------------------------------------------------------------------- */

static void build_lut(void)
{
    for (int i = 0; i < LUT_SIZE; ++i) {
        s_sine_lut[i] = sinf(2.0f * (float)M_PI * (float)i / (float)LUT_SIZE);
    }
}

audio_config_t audio_config_from_mode(int mode)
{
    /* Map the menu index to concrete behaviour. Anything out of range falls back
     * to the compiled default so a corrupt NVS value can never silence the box. */
    switch (mode) {
        case AUDIO_MODE_MONO_BOTH:
            return (audio_config_t){ .stereo = false, .callouts_enabled = true,  .tone_enabled = true  };
        case AUDIO_MODE_STEREO_BOTH:
            return (audio_config_t){ .stereo = true,  .callouts_enabled = true,  .tone_enabled = true  };
        case AUDIO_MODE_MONO_CALLOUTS:
            return (audio_config_t){ .stereo = false, .callouts_enabled = true,  .tone_enabled = false };
        case AUDIO_MODE_MONO_TONE:
            return (audio_config_t){ .stereo = false, .callouts_enabled = false, .tone_enabled = true  };
        default:
            return audio_config_from_mode(DEFAULT_AUDIO_MODE);
    }
}

void audio_init(const audio_config_t *cfg)
{
    if (cfg) {
        s_cfg = *cfg;
    }

    build_lut();

    /* Precompute the per-sample slew limits. A full-scale gain ramp should take
     * GAIN_RAMP_MS; the pitch-tracking AGL moves a bit slower for stability.   */
    float ramp_samples = (GAIN_RAMP_MS / 1000.0f) * (float)SAMPLE_RATE;
    s_gain_step = (ramp_samples > 0) ? (1.0f / ramp_samples) : 1.0f;
    /* Allow the smoothed AGL to traverse the full 100 ft band in ~250 ms. */
    float agl_ramp_samples = 0.25f * (float)SAMPLE_RATE;
    s_agl_step = TONE_START_FT / agl_ramp_samples;

    /* Flare-fade steps: a full 0..1 swing covers the configured fade time. The
     * out-step is the SLOW 3 s fade under the flare; the in-step is the QUICK
     * restore when the aircraft climbs back through FLARE_FADE_FT.              */
    float fade_out_samples = (FLARE_FADE_OUT_MS / 1000.0f) * (float)SAMPLE_RATE;
    float fade_in_samples  = (FLARE_FADE_IN_MS  / 1000.0f) * (float)SAMPLE_RATE;
    s_fade_out_step = (fade_out_samples > 0) ? (1.0f / fade_out_samples) : 1.0f;
    s_fade_in_step  = (fade_in_samples  > 0) ? (1.0f / fade_in_samples)  : 1.0f;

    /* --- I2S standard mode, TX only, 16-bit. The hardware ALWAYS runs stereo
     * (interleaved L/R) so the unit works however the panel is wired; whether we
     * pan the streams apart or duplicate them to both channels is decided per
     * sample from s_cfg.stereo in the render loop.                            */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO,
                                                            I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
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

    ESP_LOGI(TAG, "I2S up: %d Hz 16-bit stereo-HW [%s, callouts=%d tone=%d] "
             "(BCK=%d WS=%d DIN=%d, no MCLK)",
             SAMPLE_RATE, s_cfg.stereo ? "panned" : "mono-dup",
             s_cfg.callouts_enabled, s_cfg.tone_enabled,
             PIN_I2S_BCK, PIN_I2S_LRCK, PIN_I2S_DIN);
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

void audio_play_clip_blocking(const clip_t *c)
{
    /* Played synchronously from the boot path before the render tasks run (config
     * menu prompts + the calibration chirp), so the prompt sounds even with no
     * audio task up. Always centered: each mono sample is duplicated to both L
     * and R of the stereo hardware frame; pan settings do not apply to prompts. */
    if (!c || !c->pcm || c->len_bytes < 2 || !s_running) {
        return;
    }
    const int16_t *pcm = (const int16_t *)c->pcm;
    size_t n = c->len_bytes / 2;       /* mono s16 samples */
    size_t written_total = 0;

    /* Write in small interleaved blocks so we don't hold a huge stack buffer. */
    int16_t buf[AUDIO_FRAME_LEN * AUDIO_CH];
    while (written_total < n) {
        size_t chunk = n - written_total;
        if (chunk > AUDIO_FRAME_LEN) chunk = AUDIO_FRAME_LEN;
        for (size_t i = 0; i < chunk; ++i) {
            buf[2 * i]     = pcm[written_total + i];   /* L */
            buf[2 * i + 1] = pcm[written_total + i];   /* R */
        }
        size_t wrote = 0;
        i2s_channel_write(s_tx, buf, chunk * AUDIO_CH * sizeof(int16_t),
                          &wrote, portMAX_DELAY);
        written_total += chunk;
    }
}

void audio_play_chirp(void)
{
    /* Convenience wrapper: the calibration-error chirp is just a blocking clip. */
    audio_play_clip_blocking(callout_chirp());
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
        /* Re-arm the flare fade: suspend only happens in GROUND/CRUISE (well
         * above FLARE_FADE_FT), so the next active descent must start with the
         * tone fully present, not stuck faded-out from a previous landing.      */
        s_flare_fade = 1.0f;
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

    /*  Always interleaved stereo (L,R,L,R,...); mono modes write L == R. */
    int16_t frame[AUDIO_FRAME_LEN * AUDIO_CH];

    for (;;) {
        /* Pick up a queued callout (non-blocking) and start it if idle. When the
         * active mode has callouts disabled (tone-only) we still DRAIN the queue
         * so requests can't pile up, but we never start the clip.              */
        callout_id_t id;
        if (s_clip_pcm == NULL && q_callouts &&
            xQueueReceive(q_callouts, &id, 0) == pdTRUE) {
            if (s_cfg.callouts_enabled) {
                start_clip(callout_clip(id));
            }
        }

        /* Snapshot the tone params under the mutex. */
        float tone_agl    = TONE_START_FT;
        bool  tone_active = false;
        if (g_audio_mutex && xSemaphoreTake(g_audio_mutex, 0) == pdTRUE) {
            tone_agl    = g_audio_params.tone_agl;
            tone_active = g_audio_params.tone_active;
            xSemaphoreGive(g_audio_mutex);
        }

        /* The tone-disabled modes (callouts-only) silence the tone outright. */
        if (!s_cfg.tone_enabled) {
            tone_active = false;
        }

        /* If the channel is suspended (light-sleep window), idle briefly. */
        if (!s_running) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Target gain from the dB SCHEDULE only; 0 when the tone is inactive so
         * it fades out via the slew limiter rather than hard-stopping. The
         * frequency-dependent equal-loudness term is applied per-sample below,
         * since the pitch glides within a frame.                              */
        float target_gain = 0.0f;
        if (tone_active) {
            target_gain = db_to_gain(agl_to_tone_db(tone_agl));
        }

        /* Duck target: attenuate the tone while a clip is playing. */
        float duck_target = (s_clip_pcm != NULL)
                            ? db_to_gain(-VOICE_DUCK_DB)
                            : 1.0f;

        /* Flare-fade target + the step to use this frame. Below FLARE_FADE_FT we
         * fade the tone OUT (target 0) using the SLOW out-step; at/above it we
         * restore (target 1) using the FAST in-step. Keyed off the live tone_agl
         * the logic task publishes every tick, so a bounce back above 10 ft is
         * picked up within one frame and reverses the envelope immediately.     */
        float fade_target = (tone_agl < FLARE_FADE_FT) ? 0.0f : 1.0f;
        float fade_step   = (fade_target < s_flare_fade) ? s_fade_out_step
                                                         : s_fade_in_step;

        /* --- Render the frame sample-by-sample ----------------------------- */
        for (int i = 0; i < AUDIO_FRAME_LEN; ++i) {
            /* Slew the smoothed AGL toward the target so pitch glides. */
            s_tone_agl_smooth = slew_limit(s_tone_agl_smooth, tone_agl, s_agl_step);

            /* Slew the tone gain and the duck multiplier (raised-cosine feel via
             * the linear slew is adequate at these short ramp times).          */
            s_gain_cur = slew_limit(s_gain_cur, target_gain, s_gain_step);
            s_duck_cur = slew_limit(s_duck_cur, duck_target, s_gain_step);

            /* Slew the flare fade toward its target. Asymmetric step: slow out
             * under the flare, fast in on a climb-back-up. The fade-out step is
             * far smaller than the gain/duck steps so the tone eases away over
             * the full 3 s rather than snapping with the level schedule.        */
            s_flare_fade = slew_limit(s_flare_fade, fade_target, fade_step);

            float f = agl_to_pitch_hz(s_tone_agl_smooth);

            /* Equal-loudness correction: fold the ISO-226 flattening for THIS
             * frequency into the gain so the perceived loudness follows the
             * scheduled dB instead of rising as the pitch ascends. The urgency
             * cue stays in the PITCH; loudness holds steady through the flare.  */
            float eql_gain = db_to_gain(equal_loudness_db(f));

            /* s_flare_fade silences the tone under the flare (see config.h). */
            float tone = nco_sample(f) * s_gain_cur * s_duck_cur * eql_gain
                         * s_flare_fade;

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

            float left, right;
            if (s_cfg.stereo) {
                /* Gently pan the two streams to OPPOSITE sides (PAN_NEAR/FAR):
                 * voice leans RIGHT — the right ear processes speech better, so
                 * the spoken numbers get the speech-favoured ear (lol) — and the
                 * tone leans LEFT. Each channel soft-clips its own sum.         */
                left  = soft_clip(tone * PAN_NEAR + voice * PAN_FAR);
                right = soft_clip(tone * PAN_FAR  + voice * PAN_NEAR);
            } else {
                /* Mono: identical signal to both channels, so the box works even
                 * if only one channel is wired or L+R are tied together.        */
                left = right = soft_clip(tone + voice);
            }
            frame[2 * i]     = (int16_t)(left  * 32767.0f);   /* L */
            frame[2 * i + 1] = (int16_t)(right * 32767.0f);   /* R */
        }

        /* Blocking write paces the loop to real time (the DMA backpressures). */
        size_t wrote = 0;
        i2s_channel_write(s_tx, frame, sizeof frame, &wrote, portMAX_DELAY);
    }
}
