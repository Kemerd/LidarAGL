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
#include "sf30c.h"     /* sf30c_sim_active() — gates the TEMP DEBUG render meter */

#include <math.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_pm.h"        /* NO_LIGHT_SLEEP lock — keep the I2S clock alive while playing */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "audio";

/* ---- I2S handle & sine LUT ----------------------------------------------- */

static i2s_chan_handle_t s_tx = NULL;
static bool              s_running = false;

/*  NO_LIGHT_SLEEP power-management lock. The SoC's automatic light-sleep gates the
 *  I2S bit clock, which freezes the DMA mid-stream and produces a constant train of
 *  clicks during playback (the config menu is clean only because PM is enabled AFTER
 *  it). We hold this lock for exactly as long as the I2S channel is enabled, and
 *  release it whenever the channel is suspended (GROUND/CRUISE) so the box can still
 *  light-sleep to save power while it is silent. NULL if PM is not built in.        */
static esp_pm_lock_handle_t s_pm_lock = NULL;

/*  Suspend REQUEST from the logic task. Only the audio task acts on it (it is the
 *  single owner of the I2S channel), so a render write can never race a disable
 *  done on another core — which is what produced the harmless but noisy
 *  "channel not enabled" driver errors. See audio_suspend() / audio_task().      */
static volatile bool     s_suspend_req = false;

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

/*  Separate accumulator for the 2nd-harmonic oscillator (see TONE_HARMONIC2_LVL
 *  in config.h). It runs at 2*f and MUST keep its own phase — sharing s_phase
 *  would make the two partials' relative phase jump every sample and warble.    */
static float s_phase2 = 0.0f;

/*  Smoothed/slew-limited values so pitch and gain never jump (warble/clicks).  */
static float s_tone_agl_smooth = TONE_START_FT;   /* drives pitch + level       */
static float s_gain_cur        = 0.0f;            /* current linear tone gain    */
static float s_duck_cur        = 1.0f;            /* current duck multiplier     */

/*  Sidechain duck follower: a one-pole peak detector tracking the rectified voice
 *  |sample|. This IS the "how loud is the voice right now" signal the compressor
 *  ducks against — rising with the attack coefficient, falling with the release.  */
static float s_voice_env       = 0.0f;            /* tracked voice amplitude     */

/*  Pilot's INDEPENDENT tone + voice volume offsets as LINEAR gains (1.0 == 0 dB
 *  == no change). Set from NVS at boot (and live by the config-menu preview) via
 *  audio_set_tone_db / audio_set_voice_db. The tone gain folds into the presence
 *  tone before the mix; the voice gain folds into the voice clip before the mix —
 *  so the pilot dials the tone-vs-voice balance, not one master. Atomic single-
 *  float writes, so the render loop reads them lock-free. The tone gain may exceed
 *  1.0 (a boost); the voice gain is cut-only (<= 1.0).                            */
static volatile float s_tone_gain  = 1.0f;
static volatile float s_voice_gain = 1.0f;

/*  Fixed output headroom as a LINEAR gain (db_to_gain(OUTPUT_HEADROOM_DB)). Resolved
 *  once at init and multiplied into EVERY output sample — flight render and the
 *  blocking config-menu playback alike — so the equal-loudness boost can never push
 *  the mix to full scale (see OUTPUT_HEADROOM_DB in config.h). 1.0 until init runs.  */
static float s_headroom = 1.0f;

/*  Output makeup gain as a LINEAR multiplier (db_to_gain(OUTPUT_MAKEUP_DB)). A
 *  deliberate loudness lift applied alongside the headroom to every output sample
 *  (flight render + blocking config playback). 1.0 until init runs. See config.h.   */
static float s_makeup = 1.0f;

/*  Steady baseline trim applied to the presence tone whenever the active mode
 *  plays callouts (TONE_TRIM_WITH_VOICE_DB). Resolved once at init from
 *  s_cfg.callouts_enabled into a linear gain so the per-sample path is a bare
 *  multiply; 1.0 (no trim) in tone-only modes.                                   */
static float s_tone_trim = 1.0f;

/*  Flare fade multiplier: 1.0 = tone fully present, 0.0 = faded out under the
 *  flare. It slews toward 0 (slowly) below FLARE_FADE_FT and back toward 1
 *  (quickly) above it. Multiplied into the tone gain so a re-cross part-way
 *  through the fade reverses smoothly from wherever the envelope sits.          */
static float s_flare_fade      = 1.0f;            /* current flare-fade level    */

/* ---- Variometer "blip" gate (sink/climb-rate feature) -------------------- */

/*  Feature enables, set once at boot from NVS via audio_set_vario_enable(). When
 *  s_vario_sink_on is false the gate is held fully OPEN (1.0) and the tone is the
 *  steady sound it has always been — the whole blip path is bypassed. s_vario_
 *  climb_on additionally lets a CLIMB drive the blips (otherwise a climb reads as
 *  0 fpm). Atomic single-bool stores, read lock-free by the render loop.          */
static volatile bool  s_vario_sink_on  = false;
static volatile bool  s_vario_climb_on = false;

/*  The live vertical rate + acceleration the logic task publishes (ft/s, +up),
 *  snapshotted from g_audio_params each frame. They drive the beep/silence
 *  durations recomputed at every phase flip.                                      */
static volatile float s_vario_vfps = 0.0f;
static volatile float s_vario_vacc = 0.0f;

/*  Gate phase state. The gate is a slow square wave with asymmetric, rate-driven
 *  half-periods: s_blip_in_beep says which half we're in, s_blip_timer counts the
 *  remaining samples in it, and s_blip_gate is the actual multiplier — slewed
 *  toward 1.0 (beep) or 0.0 (silence) over VARIO_EDGE_MS so the edges don't click.
 *  Durations are recomputed only when the timer expires (a phase flip), so the
 *  per-sample cost is a compare + a slew.                                          */
static bool  s_blip_in_beep = false;              /* current half (beep vs silence)*/
static int   s_blip_timer   = 0;                  /* samples left in this half     */
static float s_blip_gate    = 1.0f;               /* current gate multiplier [0,1] */
static float s_blip_edge_step = 1.0f;             /* per-sample slew (VARIO_EDGE_MS)*/

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

/*  Sidechain-follower coefficients (one-pole). The follower rises toward a louder
 *  voice with s_duck_atk_a and falls toward a quieter one with s_duck_rel_a, each
 *  derived from DUCK_ATTACK_MS / DUCK_RELEASE_MS at init: a = 1 - exp(-1/(t*fs)).
 *  ATTACK is short (the duck opens promptly WITH the syllable, not ahead of it);
 *  RELEASE is long (the tone breathes back gently after the word).               */
static float s_duck_atk_a = 1.0f;   /* follower attack coeff (voice getting louder)*/
static float s_duck_rel_a = 1.0f;   /* follower release coeff (voice getting quieter)*/

/*  Precomputed full-duck linear gain (db_to_gain(-VOICE_DUCK_DB)) so the per-sample
 *  path scales between 1.0 (no duck) and this without a pow() each sample.        */
static float s_duck_floor = 1.0f;

/*  Flare-fade slew steps (per sample): a full 1->0 swing takes FLARE_FADE_OUT_MS
 *  on the way DOWN, and FLARE_FADE_IN_MS on the way UP. The asymmetry is what
 *  makes "leave the flare -> tone snaps back" feel instant while "enter the
 *  flare -> tone eases out" stays gentle.                                       */
static float s_fade_out_step = 0.0f;   /* toward silence (slow) */
static float s_fade_in_step  = 0.0f;   /* toward full   (fast) */

/* ---- Mix-bus low-pass (anti-harshness) ----------------------------------- */
/*  One-pole LPF state, one running value per channel (the previous output). The
 *  filter is y += a*(x - y); 'a' is the smoothing coefficient derived from
 *  MIX_LPF_FC_HZ at init. Softens the tanh() odd harmonics + the 2nd-harmonic
 *  edge so the high end of the sweep is silk, not glass (see config.h).         */
static float s_lpf_a    = 1.0f;        /* coefficient (1.0 == filter disabled)  */
static float s_lpf_l    = 0.0f;        /* last LPF output, LEFT  channel        */
static float s_lpf_r    = 0.0f;        /* last LPF output, RIGHT channel        */

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

/*  Forward declaration: the single-NCO sine stepper (defined with the render
 *  loop's oscillator helpers below) is also used by the blocking volume-preview
 *  tone, which appears earlier in the file.                                      */
static inline float nco_advance(float *phase, float freq_hz);

/*  Forward declaration: the float->s16 output converter (headroom + limiter
 *  backstop, defined with the render helpers below) is also used by the blocking
 *  volume-preview tone above it.                                                  */
static inline int16_t f32_to_s16(float x);

/*  Forward declarations: the conditional soft-clip and the shared voice-sidechain
 *  duck step (both defined with the render helpers below) are also used by the
 *  blocking volume-preview "mini-flare" above them.                               */
static inline float mix_limit(float x, bool voice_active);
static inline float duck_step(float *env, float voice);

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

    /* Sidechain-follower coefficients (one-pole, a = 1 - exp(-1/(tau*fs))). The
     * follower tracks the voice's rectified amplitude, rising with the short
     * ATTACK and falling with the long RELEASE so the tone ducks WITH the voice's
     * own loudness rather than snapping ahead of it (see config.h). Clamped to
     * (0,1] so a degenerate time constant just becomes instantaneous-but-stable.  */
    float atk_tau_samp = (DUCK_ATTACK_MS  / 1000.0f) * (float)SAMPLE_RATE;
    float rel_tau_samp = (DUCK_RELEASE_MS / 1000.0f) * (float)SAMPLE_RATE;
    s_duck_atk_a = (atk_tau_samp > 0.0f) ? (1.0f - expf(-1.0f / atk_tau_samp)) : 1.0f;
    s_duck_rel_a = (rel_tau_samp > 0.0f) ? (1.0f - expf(-1.0f / rel_tau_samp)) : 1.0f;
    if (s_duck_atk_a <= 0.0f || s_duck_atk_a > 1.0f) s_duck_atk_a = 1.0f;
    if (s_duck_rel_a <= 0.0f || s_duck_rel_a > 1.0f) s_duck_rel_a = 1.0f;
    s_voice_env = 0.0f;

    /* Full-duck gain floor: the tone is attenuated to this when the voice is at
     * or above DUCK_KNEE_LEVEL; it scales smoothly up to 1.0 (no duck) as the
     * voice envelope falls to DUCK_THRESHOLD. Precomputed so the loop has no pow().*/
    s_duck_floor = db_to_gain(-VOICE_DUCK_DB);

    /* Resolve the steady tone trim once: a constant cut while callouts are on so
     * the voice always reads a touch clearer over the presence tone; no trim at
     * all in a tone-only mode (nothing to make room for).                       */
    s_tone_trim = s_cfg.callouts_enabled ? db_to_gain(TONE_TRIM_WITH_VOICE_DB)
                                         : 1.0f;

    /* Resolve the fixed output headroom once (linear). Multiplied into every output
     * sample so the equal-loudness boost can never reach full scale — see
     * OUTPUT_HEADROOM_DB. The hard limiter in f32_to_s16() stays as a backstop. */
    s_headroom = db_to_gain(OUTPUT_HEADROOM_DB);
    s_makeup   = db_to_gain(OUTPUT_MAKEUP_DB);   /* deliberate loudness lift */

    /* Flare-fade steps: a full 0..1 swing covers the configured fade time. The
     * out-step is the SLOW 3 s fade under the flare; the in-step is the QUICK
     * restore when the aircraft climbs back through FLARE_FADE_FT.              */
    float fade_out_samples = (FLARE_FADE_OUT_MS / 1000.0f) * (float)SAMPLE_RATE;
    float fade_in_samples  = (FLARE_FADE_IN_MS  / 1000.0f) * (float)SAMPLE_RATE;
    s_fade_out_step = (fade_out_samples > 0) ? (1.0f / fade_out_samples) : 1.0f;
    s_fade_in_step  = (fade_in_samples  > 0) ? (1.0f / fade_in_samples)  : 1.0f;

    /* One-pole mix-bus LPF coefficient: a = 1 - exp(-2*pi*fc/fs). At fc == the
     * Nyquist-ish ceiling 'a' approaches 1 (pass-through); lower fc => smaller a
     * => more smoothing. We clamp to (0,1] so the filter is always stable.       */
    float lpf_rc = expf(-2.0f * (float)M_PI * MIX_LPF_FC_HZ / (float)SAMPLE_RATE);
    s_lpf_a = 1.0f - lpf_rc;
    if (s_lpf_a <= 0.0f || s_lpf_a > 1.0f) {
        s_lpf_a = 1.0f;                 /* degenerate fc => disable (pass-through) */
    }
    /* Start the filter memories at silence so the very first frame has no step. */
    s_lpf_l = 0.0f;
    s_lpf_r = 0.0f;

    /* Vario blip-gate edge slew: a full 0..1 swing covers VARIO_EDGE_MS, so each
     * beep/silence transition is a short raised-cosine-ish ramp rather than a hard
     * gate (which would click). The mix-bus LPF further rounds whatever remains.   */
    float blip_edge_samples = (VARIO_EDGE_MS / 1000.0f) * (float)SAMPLE_RATE;
    s_blip_edge_step = (blip_edge_samples > 0.0f) ? (1.0f / blip_edge_samples) : 1.0f;
    s_blip_in_beep   = false;    /* first phase flip turns this true -> opens on a beep*/
    s_blip_timer     = 0;        /* expires immediately -> durations set on tick 1    */
    s_blip_gate      = 1.0f;

    /* --- I2S standard mode, TX only, 16-bit. The hardware ALWAYS runs stereo
     * (interleaved L/R) so the unit works however the panel is wired; whether we
     * pan the streams apart or duplicate them to both channels is decided per
     * sample from s_cfg.stereo in the render loop.                            */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO,
                                                            I2S_ROLE_MASTER);
    /* Auto-clear the DMA TX buffer on underrun. Without this the peripheral keeps
     * re-clocking whatever stale samples are still in the last DMA descriptor when
     * we stop feeding it — which is exactly the held "eeee" tone heard after a
     * blocking config-menu clip ends (the render task isn't up yet to push silence).
     * With auto_clear the driver substitutes zeros the moment our data runs out, so
     * a finished clip falls to true silence instead of looping its tail.            */
    chan_cfg.auto_clear = true;
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

    /* Force a 64·fS bit clock. The PCM5102A's internal PLL (SCK->GND mode) does NOT
     * support a 32·fS BCK at 16 kHz: datasheet Table 11 lists 16 kHz @ 32·fS as
     * unsupported and requires 64·fS (1.024 MHz) for the PLL to lock. The default
     * 16-bit slot gives only 32·fS (16 bits × 2 ch × 16 kHz = 512 kHz), below spec.
     * Widening the SLOT to 32 bits (data stays 16-bit, padded in the slot LSBs) makes
     * BCK = 32 × 2 × 16 kHz = 1.024 MHz = 64·fS, squarely inside the PLL's range, so
     * the DAC's clock recovery is solid instead of marginal.                        */
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx, &std_cfg));

    /* Create + take the NO_LIGHT_SLEEP lock BEFORE enabling the channel so the very
     * first samples already clock out with light-sleep inhibited. If PM isn't built
     * in, lock_create returns an error and s_pm_lock stays NULL (all uses guard on
     * it), so the audio path is unchanged on a no-PM build.                         */
    esp_err_t pmerr = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "audio_i2s",
                                         &s_pm_lock);
    if (pmerr != ESP_OK) {
        s_pm_lock = NULL;
    }
    if (s_pm_lock) {
        esp_pm_lock_acquire(s_pm_lock);
    }
    /* TEMP DEBUG: confirm the no-light-sleep guard is actually live. If this says
     * UNAVAILABLE, PM locks aren't compiled in; if HELD but stalls persist, then
     * light-sleep is not what's gating the I2S clock and we look elsewhere.        */
    ESP_LOGW(TAG, "PM no-light-sleep lock: %s (create=%s)",
             s_pm_lock ? "HELD" : "UNAVAILABLE", esp_err_to_name(pmerr));

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

void audio_set_params(float tone_agl, bool tone_active, float vert_fps)
{
    if (g_audio_mutex && xSemaphoreTake(g_audio_mutex, 0) == pdTRUE) {
        g_audio_params.tone_agl    = tone_agl;
        g_audio_params.tone_active = tone_active;
        g_audio_params.vert_fps    = vert_fps;
        xSemaphoreGive(g_audio_mutex);
    }
}

void audio_set_vario_enable(bool sink_on, bool climb_on)
{
    /* Plain single-bool stores — the render loop reads them lock-free, exactly like
     * the tone/voice volume gains. Climb only matters while sink is on (they share
     * the one gate), but we keep both flags verbatim so the caller's intent is
     * exactly what flies.                                                          */
    s_vario_sink_on  = sink_on;
    s_vario_climb_on = climb_on;
}

void audio_set_tone_db(float db)
{
    /* Clamp to the tone menu's range (it may CUT or BOOST), convert once to a
     * linear gain. A single volatile float write — no mutex needed; the render
     * loop reads whatever is current, and a half-updated float can't occur for a
     * 32-bit aligned store on the S3.                                            */
    if (db < TONE_VOLUME_DB_MIN) {
        db = TONE_VOLUME_DB_MIN;
    }
    if (db > TONE_VOLUME_DB_MAX) {
        db = TONE_VOLUME_DB_MAX;
    }
    s_tone_gain = db_to_gain(db);
}

void audio_set_voice_db(float db)
{
    /* Clamp to attenuation only (the clips already sit near full scale), convert
     * once to a linear gain. Same lock-free single-float store as the tone gain. */
    if (db > 0.0f) {
        db = 0.0f;
    }
    if (db < VOICE_VOLUME_DB_MIN) {
        db = VOICE_VOLUME_DB_MIN;
    }
    s_voice_gain = db_to_gain(db);
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

    /* These are VOICE clips (config-menu prompts + the volume preview's spoken
     * number), so apply the pilot's VOICE volume offset — the prompts then play at
     * exactly the callout level the running box will use. 1.0 == 0 dB == unchanged. */
    float mg = s_voice_gain;

    /* Write in small interleaved blocks so we don't hold a huge stack buffer. */
    int16_t buf[AUDIO_FRAME_LEN * AUDIO_CH];
    while (written_total < n) {
        size_t chunk = n - written_total;
        if (chunk > AUDIO_FRAME_LEN) chunk = AUDIO_FRAME_LEN;
        for (size_t i = 0; i < chunk; ++i) {
            /* Same fixed output headroom as the flight render path so the spoken
             * config-menu prompts (and the volume preview) play at the exact net
             * level the running box will use — not 3.2 dB hotter. */
            int16_t s = (int16_t)((float)pcm[written_total + i] * mg
                                  * s_headroom * s_makeup);
            buf[2 * i]     = s;   /* L */
            buf[2 * i + 1] = s;   /* R */
        }
        size_t wrote = 0;
        i2s_channel_write(s_tx, buf, chunk * AUDIO_CH * sizeof(int16_t),
                          &wrote, portMAX_DELAY);
        written_total += chunk;
    }
}

void audio_play_tone_blocking(float freq_hz, int ms, float level_db)
{
    /* A standalone NCO so the menu preview never disturbs the render loop's tone
     * phase. Mirrors the firmware tone chain at a FIXED pitch: dB level -> linear
     * gain, equal-loudness flattening at this frequency (so 1 kHz here sounds the
     * same loudness it would in flight), then the pilot's TONE offset (this burst
     * IS the tone preview). Short raised-cosine fades top and tail it, no click.  */
    if (!s_running || ms <= 0 || freq_hz <= 0.0f) {
        return;
    }
    if (level_db > 0.0f) {
        level_db = 0.0f;
    }

    float gain = db_to_gain(level_db + equal_loudness_db(freq_hz)) * s_tone_gain;

    size_t total = (size_t)((float)ms / 1000.0f * (float)SAMPLE_RATE);
    /* Fade window: a few ms each end, never more than a third of the burst. */
    size_t fade = (size_t)(0.004f * (float)SAMPLE_RATE);
    if (fade > total / 3) {
        fade = total / 3;
    }

    float phase = 0.0f;
    size_t done = 0;
    int16_t buf[AUDIO_FRAME_LEN * AUDIO_CH];
    while (done < total) {
        size_t chunk = total - done;
        if (chunk > AUDIO_FRAME_LEN) chunk = AUDIO_FRAME_LEN;
        for (size_t i = 0; i < chunk; ++i) {
            size_t idx = done + i;

            /* Raised-cosine edges (reuse the shared shaper for click-free ends). */
            float env = 1.0f;
            if (fade > 0) {
                if (idx < fade) {
                    env = raised_cosine((float)idx / (float)fade);
                } else if (idx >= total - fade) {
                    env = raised_cosine((float)(total - 1 - idx) / (float)fade);
                }
            }

            float s = nco_advance(&phase, freq_hz) * gain * env;
            /* Through the shared converter so the preview tone carries the same
             * output headroom (and limiter backstop) as the flight render path. */
            int16_t v = f32_to_s16(s);
            buf[2 * i]     = v;   /* L */
            buf[2 * i + 1] = v;   /* R */
        }
        size_t wrote = 0;
        i2s_channel_write(s_tx, buf, chunk * AUDIO_CH * sizeof(int16_t),
                          &wrote, portMAX_DELAY);
        done += chunk;
    }
}

void audio_play_volume_preview_blocking(void)
{
    /* A short "mini-flare" balance preview: the presence tone sweeps down the
     * 20->10 ft band (pitch rising on the real schedule) in the BACKGROUND while the
     * "20" then "10" callouts speak over it and DUCK it — the exact sidechain that
     * flies (shared duck_step()). Both tone and voice carry the pilot's live tone/
     * voice offsets, so each menu step auditions the chosen balance. Blocking and
     * mono-centered; runs before the render task exists.                           */
    if (!s_running) {
        return;
    }

    const float agl_from = VOLUME_PREVIEW_SWEEP_FROM_FT;
    const float agl_to   = VOLUME_PREVIEW_SWEEP_TO_FT;
    size_t total = (size_t)((float)VOLUME_PREVIEW_SWEEP_MS / 1000.0f * (float)SAMPLE_RATE);
    if (total < 2) {
        return;
    }

    /* The two spoken callouts, scheduled at their matching heights in the sweep:
     * "20" near the top (~12 % in), "10" past the middle (~58 % in). A missing clip
     * simply contributes silence (NULL pcm), so the preview still runs.            */
    const clip_t  *c20   = callout_clip(CO_TWENTY);
    const clip_t  *c10   = callout_clip(CO_TEN);
    const int16_t *pcm20 = (c20 && c20->pcm) ? (const int16_t *)c20->pcm : NULL;
    const int16_t *pcm10 = (c10 && c10->pcm) ? (const int16_t *)c10->pcm : NULL;
    size_t len20   = pcm20 ? c20->len_bytes / 2 : 0;
    size_t len10   = pcm10 ? c10->len_bytes / 2 : 0;
    size_t start20 = (size_t)(0.12f * (float)total);
    size_t start10 = (size_t)(0.58f * (float)total);

    /* Click-free edges, plus a PRIVATE duck follower so we never touch the render
     * loop's s_voice_env, and a standalone NCO phase for the swept fundamental.    */
    size_t fade = (size_t)(0.010f * (float)SAMPLE_RATE);   /* 10 ms */
    if (fade > total / 4) {
        fade = total / 4;
    }
    float env   = 0.0f;
    float phase = 0.0f;

    size_t done = 0;
    int16_t buf[AUDIO_FRAME_LEN * AUDIO_CH];
    while (done < total) {
        size_t chunk = total - done;
        if (chunk > AUDIO_FRAME_LEN) chunk = AUDIO_FRAME_LEN;
        for (size_t i = 0; i < chunk; ++i) {
            size_t idx = done + i;

            /* Scripted AGL -> pitch + scheduled level, exactly like flight. */
            float prog = (float)idx / (float)(total - 1);
            float agl  = agl_from + (agl_to - agl_from) * prog;
            float f    = agl_to_pitch_hz(agl);

            /* Pull the active callout sample (if any), then the voice volume offset. */
            float voice        = 0.0f;
            bool  voice_active = false;
            if (pcm20 && idx >= start20 && (idx - start20) < len20) {
                voice = (float)pcm20[idx - start20] / 32768.0f;
                voice_active = true;
            } else if (pcm10 && idx >= start10 && (idx - start10) < len10) {
                voice = (float)pcm10[idx - start10] / 32768.0f;
                voice_active = true;
            }
            voice *= s_voice_gain;

            /* SHARED sidechain: the voice pulls the tone down with the same duck the
             * flight render loop runs, so the preview balance is honest.            */
            float duck = duck_step(&env, voice);

            /* Tone on the real schedule, ducked, at the pilot's live tone gain. */
            float tone = nco_advance(&phase, f)
                         * db_to_gain(agl_to_tone_db(agl) + equal_loudness_db(f))
                         * duck * s_tone_gain;

            /* Whole-snippet raised-cosine fade in/out so there is no click. */
            float edge = 1.0f;
            if (fade > 0) {
                if (idx < fade) {
                    edge = raised_cosine((float)idx / (float)fade);
                } else if (idx >= total - fade) {
                    edge = raised_cosine((float)(total - 1 - idx) / (float)fade);
                }
            }

            /* Mono mix + the same conditional soft-clip the render loop applies. */
            float s = mix_limit(tone + voice, voice_active) * edge;
            int16_t v = f32_to_s16(s);
            buf[2 * i]     = v;   /* L */
            buf[2 * i + 1] = v;   /* R */
        }
        size_t wrote = 0;
        i2s_channel_write(s_tx, buf, chunk * AUDIO_CH * sizeof(int16_t),
                          &wrote, portMAX_DELAY);
        done += chunk;
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
    /* REQUEST only. The audio task (the single owner of the I2S channel) does the
     * actual i2s_channel_disable at the top of its loop, so a render write can
     * never race a disable performed here from another core (that race produced
     * the harmless but noisy "channel not enabled" driver errors).             */
    s_suspend_req = true;
}

void audio_resume(void)
{
    s_suspend_req = false;
}

/* ---------------------------------------------------------------------------
 *  NCO + interpolated LUT lookup
 * ------------------------------------------------------------------------- */

/*  Advance one NCO (its *phase accumulator passed by ref) by freq_hz and return
 *  the interpolated sine. Pulling the accumulator out as a parameter lets the
 *  fundamental and the 2nd harmonic each keep an independent phase.             */
static inline float nco_advance(float *phase, float freq_hz)
{
    /* Advance phase, wrapping into [0,1). */
    *phase += freq_hz / (float)SAMPLE_RATE;
    if (*phase >= 1.0f) {
        *phase -= 1.0f;
    }
    /* Linear-interpolated LUT read (sine is smooth, so 1024 + interp is clean). */
    float x   = *phase * (float)LUT_SIZE;
    int   i0  = (int)x;
    int   i1  = (i0 + 1) & (LUT_SIZE - 1);
    float frac = x - (float)i0;
    return s_sine_lut[i0] * (1.0f - frac) + s_sine_lut[i1] * frac;
}

/*  The presence tone's full waveform at frequency f: the fundamental plus a
 *  small EVEN (2nd) harmonic for warmth. The pair is normalised by (1 + level)
 *  so adding body never raises the peak amplitude — the dB schedule and the
 *  equal-loudness correction keep meaning exactly what they did for a pure sine.
 *  With TONE_HARMONIC2_LVL == 0 this degenerates to the original clean sine.    */
static inline float tone_sample(float freq_hz)
{
    float fundamental = nco_advance(&s_phase,  freq_hz);
    float harmonic2   = nco_advance(&s_phase2, freq_hz * 2.0f);
    return (fundamental + TONE_HARMONIC2_LVL * harmonic2)
           / (1.0f + TONE_HARMONIC2_LVL);
}

/* Soft clip / limiter: a gentle tanh keeps the summed output inside [-1,1] with
 * graceful saturation instead of harsh wrap-around. Headroom is documented: the
 * tone peaks at TONE_FULL_DB and clips peak near -3.5 dBFS, so the sum rarely
 * exceeds unity, but tanh guarantees no overflow when voice + tone align.      */
static inline float soft_clip(float x)
{
    return tanhf(x);
}

/*  Conditional limiter for the channel sum. tanh() is a nonlinearity, so it adds
 *  ODD harmonics to whatever passes through it — harmless on a near-clipping
 *  voice+tone sum, but pure harshness on a solo tone that never approaches full
 *  scale. When TONE_SOFTCLIP_ONLY_WITH_VOICE is set we therefore only engage the
 *  soft-clip while a voice clip is mixed in (the one case the sum can exceed 1);
 *  a solo tone passes through clean. With the flag at 0 we always soft-clip.     */
static inline float mix_limit(float x, bool voice_active)
{
#if TONE_SOFTCLIP_ONLY_WITH_VOICE
    return voice_active ? soft_clip(x) : x;
#else
    (void)voice_active;
    return soft_clip(x);
#endif
}

/*  One sample of the voice-sidechain duck, SHARED by the flight render loop and the
 *  config-menu volume preview so both pull the tone down off the voice identically.
 *  Advances the caller's one-pole envelope follower with |voice| (fast ATTACK coeff
 *  when louder, slow RELEASE when quieter), then maps that envelope through the soft
 *  knee to a tone-gain multiplier: 1.0 (un-ducked) at/below DUCK_THRESHOLD, easing
 *  to s_duck_floor (full VOICE_DUCK_DB) at/above DUCK_KNEE_LEVEL. The follower state
 *  is caller-owned (a float*), so the render loop and the preview keep independent
 *  envelopes and never cross-contaminate.                                          */
static inline float duck_step(float *env, float voice)
{
    float rect = fabsf(voice);
    float a = (rect > *env) ? s_duck_atk_a : s_duck_rel_a;
    *env += a * (rect - *env);

    float duck = 1.0f;
    if (*env > DUCK_THRESHOLD) {
        float t = (*env - DUCK_THRESHOLD) / (DUCK_KNEE_LEVEL - DUCK_THRESHOLD);
        if (t > 1.0f) t = 1.0f;
        duck = 1.0f + t * (s_duck_floor - 1.0f);
    }
    return duck;
}

/*  Vario blip — turn the live vertical rate into the beep / silence half-period
 *  lengths (in SAMPLES). The SILENCE is the sink-rate knob: a long gap when level
 *  (VARIO_SIL_BASE_MS), shrinking toward VARIO_SIL_MIN_MS at the reference rate, so
 *  faster sink => faster, tighter blips. The BEEP is simply a fraction of the
 *  silence (VARIO_BEEP_FACTOR), so the whole cadence scales together — no derivative
 *  term. See the VARIO_* block in config.h for the full mapping + tuning rationale.  */
static void blip_durations(float vfps, int *beep_samples, int *silence_samples)
{
    /* Work in fpm (how the pilot thinks). vfps is +up, so a descent is negative.    */
    float v_fpm     = vfps * 60.0f;
    float sink_fpm  = (v_fpm < 0.0f) ? -v_fpm : 0.0f;
    float climb_fpm = (v_fpm > 0.0f) ?  v_fpm : 0.0f;

    /* Sink always drives the cadence; a climb only counts when the pilot enabled the
     * climb-rate toggle (otherwise a climb reads as 0 fpm => the baseline beep).     */
    float rate_fpm = s_vario_climb_on ? fmaxf(sink_fpm, climb_fpm) : sink_fpm;

    /* Normalise 0..reference -> 0..1 and ramp the SILENCE from base down to min.     */
    float t = rate_fpm / VARIO_REF_FPM;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float silence_ms = VARIO_SIL_BASE_MS + t * (VARIO_SIL_MIN_MS - VARIO_SIL_BASE_MS);

    /* Beep is a fixed fraction of the silence (clamped), so faster sink shrinks both. */
    float beep_ms = VARIO_BEEP_FACTOR * silence_ms;
    if (beep_ms < VARIO_BEEP_MIN_MS) beep_ms = VARIO_BEEP_MIN_MS;
    if (beep_ms > VARIO_BEEP_MAX_MS) beep_ms = VARIO_BEEP_MAX_MS;

    *beep_samples    = (int)(beep_ms    * (float)SAMPLE_RATE / 1000.0f);
    *silence_samples = (int)(silence_ms * (float)SAMPLE_RATE / 1000.0f);
    if (*beep_samples    < 1) *beep_samples    = 1;
    if (*silence_samples < 1) *silence_samples = 1;
}

/*  Advance the blip gate one sample and return its multiplier [0,1]. When the
 *  sink-rate feature is OFF the gate is held fully OPEN, so the tone is byte-for-byte
 *  the steady sound it has always been. Durations are recomputed only at a phase
 *  flip (timer expiry) from the latest published rate, so a changing sink retimes
 *  from the very next blip; the per-sample cost is a compare + the edge slew.        */
static inline float blip_advance_gate(float vfps)
{
    if (!s_vario_sink_on) {
        /* Feature off: hold open and prime the phase so the first blip after an
         * enable would open on a BEEP (next flip turns s_blip_in_beep true).        */
        s_blip_gate    = 1.0f;
        s_blip_in_beep = false;
        s_blip_timer   = 0;
        return 1.0f;
    }

    if (s_blip_timer <= 0) {
        int beep_n, sil_n;
        blip_durations(vfps, &beep_n, &sil_n);
        s_blip_in_beep = !s_blip_in_beep;                 /* flip into the next half */
        s_blip_timer   = s_blip_in_beep ? beep_n : sil_n;
    }
    s_blip_timer--;

    /* Slew toward the half's target so the gate opens/closes without a click. */
    float target = s_blip_in_beep ? 1.0f : 0.0f;
    s_blip_gate  = slew_limit(s_blip_gate, target, s_blip_edge_step);
    return s_blip_gate;
}

/*  Final float -> s16 conversion. Two-stage by design:
 *    1) Apply the fixed OUTPUT HEADROOM (s_headroom). This is the PRIMARY defence:
 *       it gain-stages the whole mix down so the equal-loudness boost can never
 *       reach full scale, so in practice the clamp below never fires. (Why mono
 *       needed this and stereo didn't: stereo pans tone/voice apart at ~0.92/0.27
 *       so neither channel gets hot; mono sums them into one channel at full
 *       weight, so the eql boost there could tip the mix over 0 dBFS.)
 *    2) HARD limiter as a backstop only. A bare (int16_t) cast of a value past
 *       +/-32767 WRAPS to the opposite sign — a full-scale discontinuity heard as
 *       a click. With the headroom in place this should never engage, but it stays
 *       so a future gain mistake degrades to a tick instead of a wrap.             */
static inline int16_t f32_to_s16(float x)
{
    x *= s_headroom * s_makeup;           /* reserve headroom, then the loudness lift */
    if (x >  1.0f) x =  1.0f;             /* backstop limiter — guards the rare peak  */
    if (x < -1.0f) x = -1.0f;
    return (int16_t)(x * 32767.0f);
}

/* ---------------------------------------------------------------------------
 *  Audio task: render one frame at a time
 * ------------------------------------------------------------------------- */

void audio_task(void *arg)
{
    (void)arg;

    /*  Always interleaved stereo (L,R,L,R,...); mono modes write L == R. */
    int16_t frame[AUDIO_FRAME_LEN * AUDIO_CH];

    /* TEMP DEBUG — ~1 Hz render-health meter to root-cause the flight-audio jitter.
     * Remove this block (and the two tagged spots below) once it's diagnosed:
     *   peak  = loudest output sample after headroom (>1.0 => backstop is clipping)
     *   clip  = samples the backstop limiter actually clamped this second
     *   stall = bounded i2s_channel_write stalls (DMA not draining / timeout)
     *   part  = SHORT writes — driver accepted < a full frame, so we DROPPED the
     *           tail of that frame (a genuine discontinuity / click source).        */
    uint32_t dbg_frames = 0, dbg_clip = 0, dbg_stall = 0, dbg_part = 0;
    float    dbg_peak = 0.0f;

    for (;;) {
        /* Single-owner channel control: apply the logic task's suspend/resume
         * REQUEST here, so THIS task is the only code that ever enables or
         * disables the I2S channel. That removes the cross-core race that briefly
         * let a render write hit a just-disabled channel (the harmless but noisy
         * "channel not enabled" errors and the resume-time write timeouts). On
         * suspend we also reset the tone envelopes so a later descent ramps
         * cleanly from silence.                                                  */
        if (s_suspend_req && s_running) {
            i2s_channel_disable(s_tx);
            /* Channel is down: drop the NO_LIGHT_SLEEP lock so the box can actually
             * light-sleep and save power while it is silent (GROUND/CRUISE).        */
            if (s_pm_lock) {
                esp_pm_lock_release(s_pm_lock);
                ESP_LOGW(TAG, "PM lock RELEASED (audio suspended)");  /* TEMP DEBUG */
            }
            s_running    = false;
            s_gain_cur   = 0.0f;   /* ramp the tone back up from silence on resume */
            s_duck_cur   = 1.0f;   /* un-duck so the next descent starts full tone  */
            s_voice_env  = 0.0f;   /* drain the sidechain follower (no stale duck)  */
            s_phase      = 0.0f;
            s_phase2     = 0.0f;   /* keep the 2nd harmonic in step                 */
            s_lpf_l      = 0.0f;   /* drain the mix LPF so resume starts at silence */
            s_lpf_r      = 0.0f;
            s_flare_fade = 1.0f;   /* re-arm the flare fade for the next descent    */
        } else if (!s_suspend_req && !s_running) {
            /* Re-arm the clock guard BEFORE enabling so resume clocks out cleanly. */
            if (s_pm_lock) {
                esp_pm_lock_acquire(s_pm_lock);
                ESP_LOGW(TAG, "PM lock ACQUIRED (audio resumed)");  /* TEMP DEBUG */
            }
            i2s_channel_enable(s_tx);
            s_running = true;
        }

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

        /* Snapshot the tone params under the mutex. CRITICAL: the take uses a ZERO
         * timeout, so it can fail simply because the logic task held the lock for the
         * microseconds it takes to publish new params. If that happens we must KEEP
         * THE LAST SNAPSHOT — NOT fall back to "tone off / 100 ft". The old code reset
         * to those defaults on every missed take, which for one frame cut the tone
         * gain toward 0 and slammed the smoothed AGL toward its 100 ft floor; a run of
         * missed takes (more likely when the logic task is busy, e.g. a fast sim
         * stream) therefore injected gain/pitch flutter that reads as audible jitter.
         * The snapshot is static so a lost lock is just a held value, never a glitch. */
        static float s_snap_agl    = TONE_START_FT;
        static bool  s_snap_active = false;
        static float s_snap_vfps   = 0.0f;   /* vertical rate (held on a lost lock) */
        if (g_audio_mutex && xSemaphoreTake(g_audio_mutex, 0) == pdTRUE) {
            s_snap_agl    = g_audio_params.tone_agl;
            s_snap_active = g_audio_params.tone_active;
            s_snap_vfps   = g_audio_params.vert_fps;
            xSemaphoreGive(g_audio_mutex);
        }
        float tone_agl    = s_snap_agl;
        bool  tone_active = s_snap_active;
        float vert_fps    = s_snap_vfps;

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

        /* The voice-duck is no longer a per-frame target: it is derived PER SAMPLE
         * from the live voice envelope (sidechain compressor) further down, so the
         * tone ducks with the voice's actual loudness instead of a fixed level.   */

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

            /* Slew the tone gain (raised-cosine feel via the linear slew is
             * adequate at these short ramp times).                             */
            s_gain_cur = slew_limit(s_gain_cur, target_gain, s_gain_step);

            /* Slew the flare fade toward its target. Asymmetric step: slow out
             * under the flare, fast in on a climb-back-up. The fade-out step is
             * far smaller than the gain step so the tone eases away over the full
             * 3 s rather than snapping with the level schedule.                  */
            s_flare_fade = slew_limit(s_flare_fade, fade_target, fade_step);

            /* --- Read the voice sample FIRST so the duck can key off it -------
             * voice_active tracks PRESENCE (not amplitude): a clip mid-stream can
             * pass through a zero-crossing, so we must not infer "no voice" from
             * voice == 0 — the soft-clip decision below relies on this.          */
            float voice = 0.0f;
            bool  voice_active = (s_clip_pcm != NULL);
            if (s_clip_pcm != NULL) {
                voice = (float)s_clip_pcm[s_clip_pos] / 32768.0f;
                if (++s_clip_pos >= s_clip_len) {
                    /* Clip finished; the follower decays and the tone breathes
                     * back up over DUCK_RELEASE_MS — no hard un-duck step.       */
                    s_clip_pcm = NULL;
                    s_clip_len = 0;
                    s_clip_pos = 0;
                }
            }

            /* Fold in the pilot's VOICE volume offset (cut-only) right here, before
             * the sidechain follower, so the duck keys off the post-trim voice level
             * and a quieter voice ducks the tone proportionally less.                */
            voice *= s_voice_gain;

            /* --- Sidechain duck (a compressor keyed off the voice envelope) ---
             * Advance the voice follower + map it to the tone-duck multiplier via the
             * SHARED duck_step() — the very same sidechain the config-menu volume
             * preview runs, so what the pilot auditions in the menu is what flies.
             * s_voice_env is the render loop's own follower (the preview keeps its
             * own), and duck_step bakes in the fast-attack/slow-release + soft knee.  */
            float duck = duck_step(&s_voice_env, voice);
            s_duck_cur = duck;   /* exposed for clarity; the follower IS the smoothing */

            /* Tone pitch for this sample + equal-loudness flattening at that
             * frequency, so perceived loudness follows the scheduled dB instead of
             * rising as the pitch ascends (urgency stays in PITCH, not loudness).  */
            float f = agl_to_pitch_hz(s_tone_agl_smooth);
            float eql_gain = db_to_gain(equal_loudness_db(f));

            /* Vario blip gate: chops the tone into rate-driven blips when the sink-
             * rate feature is on, else holds fully open (1.0 => today's steady tone).
             * It sits OUTSIDE s_flare_fade below, so the flare fade-out under 10 ft
             * still wins; advanced once per sample so the cadence stays sample-exact. */
            float blip_gate = blip_advance_gate(vert_fps);

            /* Build the tone: schedule gain · sidechain duck · equal-loudness ·
             * flare fade · steady callout trim · pilot tone volume · vario blip gate.
             * tone_sample() = fundamental + a small 2nd harmonic (warmth); s_flare_fade
             * silences it under the flare; s_tone_trim holds it a constant
             * TONE_TRIM_WITH_VOICE_DB down while callouts are enabled (1.0 in tone-only
             * modes); s_tone_gain is the pilot's tone volume offset (may cut OR boost). */
            float tone = tone_sample(f) * s_gain_cur * duck * eql_gain
                         * s_flare_fade * s_tone_trim * s_tone_gain * blip_gate;

            float left, right;
            if (s_cfg.stereo) {
                /* Gently pan the two streams to OPPOSITE sides (PAN_NEAR/FAR):
                 * voice leans RIGHT — the right ear processes speech better, so
                 * the spoken numbers get the speech-favoured ear (lol) — and the
                 * tone leans LEFT. Each channel limits its own sum (only when a
                 * voice clip is present — see mix_limit()).                      */
                left  = mix_limit(tone * PAN_NEAR + voice * PAN_FAR, voice_active);
                right = mix_limit(tone * PAN_FAR  + voice * PAN_NEAR, voice_active);
            } else {
                /* Mono: identical signal to both channels, so the box works even
                 * if only one channel is wired or L+R are tied together.        */
                left = right = mix_limit(tone + voice, voice_active);
            }

            /* One-pole low-pass on the final mix: smooths the small odd harmonics
             * the limiter can add and rounds the 2nd-harmonic's edge, so the high
             * end of the sweep is silk, not glass. Per-channel running state.     */
            s_lpf_l += s_lpf_a * (left  - s_lpf_l);
            s_lpf_r += s_lpf_a * (right - s_lpf_r);
            left  = s_lpf_l;
            right = s_lpf_r;

            /* The pilot's tone + voice volume offsets were already folded into the
             * tone and voice individually (before the mix), so there is no master
             * trim to apply here — just the fixed headroom + makeup lift.          */

            /* TEMP DEBUG: measure the post-headroom magnitude so we can see whether
             * the backstop limiter is firing (peak > 1.0) and how often. */
            float dl = fabsf(left)  * s_headroom * s_makeup;
            float dr = fabsf(right) * s_headroom * s_makeup;
            if (dl > dbg_peak) dbg_peak = dl;
            if (dr > dbg_peak) dbg_peak = dr;
            if (dl > 1.0f || dr > 1.0f) dbg_clip++;

            frame[2 * i]     = f32_to_s16(left);    /* L (headroom + backstop) */
            frame[2 * i + 1] = f32_to_s16(right);   /* R (headroom + backstop) */
        }

        /* Push the WHOLE frame into the DMA, ACCUMULATING across partial writes and
         * RE-trying a transient stall — we never discard generated audio. The old
         * code wrote once with a bounded timeout and, on any stall/short write, just
         * moved on: but the per-sample loop above had already ADVANCED the oscillator
         * phase by a full frame, so a dropped frame is a phase discontinuity = click.
         * That is the click. The clean blocking config path never discards either —
         * it waits — so we mirror it here. The i2s write itself blocks (bounded), so
         * looping on a stall can't tight-spin or starve the idle task; we just keep
         * offering the remainder until the frame is fully queued or the channel is
         * being suspended out from under us (then we abandon the tail cleanly).      */
        size_t off = 0;
        while (off < sizeof frame) {
            size_t wrote = 0;
            esp_err_t werr = i2s_channel_write(s_tx, (const uint8_t *)frame + off,
                                               sizeof frame - off, &wrote,
                                               pdMS_TO_TICKS(100));
            off += wrote;                      /* advance past whatever was accepted  */
            if (off >= sizeof frame) {
                break;                         /* whole frame queued — done           */
            }
            /* Channel going down (single-owner suspend handled at loop top): stop
             * pushing this frame rather than block against a disabled channel.       */
            if (werr == ESP_ERR_INVALID_STATE || s_suspend_req) {
                break;
            }
            if (wrote == 0) {
                dbg_stall++;   /* TEMP DEBUG: transient stall — we RETRY, never drop  */
                static bool s_warned_stall = false;
                if (!s_warned_stall) {
                    ESP_LOGW(TAG, "i2s write slow (%s) — retrying, not dropping",
                             esp_err_to_name(werr));
                    s_warned_stall = true;
                }
            }
            /* loop: the blocking write paces us; the remainder goes next attempt */
        }

        /* TEMP DEBUG: dump the render-health meter ~once per second
         * (SAMPLE_RATE / AUDIO_FRAME_LEN frames ~= 1 s of audio). Gated on bench-sim
         * mode ONLY — never logs on a real-sensor/flight boot, so the production
         * console stays clean and we add no log traffic to the live USB link. */
        if (sf30c_sim_active() && ++dbg_frames >= (SAMPLE_RATE / AUDIO_FRAME_LEN)) {
            ESP_LOGI(TAG, "rmeter peak=%.3f clip=%u stall=%u part=%u | "
                     "agl=%.1f act=%d voice=%d",
                     (double)dbg_peak, (unsigned)dbg_clip, (unsigned)dbg_stall,
                     (unsigned)dbg_part, (double)tone_agl, (int)tone_active,
                     (int)(s_clip_pcm != NULL));
            dbg_frames = 0; dbg_peak = 0.0f; dbg_clip = 0;
            dbg_stall  = 0; dbg_part = 0;
        }
    }
}
