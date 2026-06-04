/**
 * @file    app_main.c
 * @brief   LidarAGL entry point: boot sequence, task spawning, and logic loop.
 *
 * @details Boot order (spec §5, §6, §8 + the builder's ground-reference model):
 *            1. NVS up; radios stay off (compiled out + never started). Load the
 *               saved audio config (mono/stereo + which streams play).
 *            2. UART up; autodetect the sensor -> g_profile; start streaming.
 *               (Done before the menu so it can cycle the right callout ladder.)
 *            3. I2S up with the resolved audio config.
 *            4. Reset button held? -> enter the config menu (wipes ground +
 *               config, two-level tap/double-tap menu, reboots on commit). Then
 *               resolve the callout start-altitude cap from the saved config.
 *            5. Capture a ground-fill (10 readings spread over ~1 s) + one
 *               current reading; reconstruct the ground reference and boot AGL.
 *            6. If no real reference existed -> calibration-error chirp.
 *            7. One NVS write: persist the fresh ground readings.
 *            8. Seed the state machine from the boot AGL.
 *            9. Spawn sensor (core 0), logic + audio (core 1); enable light-sleep.
 *
 *          The logic task lives here too: it owns the state machine, applies the
 *          ground reference, fires callouts, sets the poll profile, publishes
 *          tone params, and gates light-sleep via audio suspend/resume.
 */

#include "config.h"
#include "shared.h"
#include "sensor_profile.h"
#include "sf30c.h"
#include "boot_buffer.h"
#include "state_machine.h"
#include "audio.h"
#include "callouts.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "esp_system.h"   /* esp_restart() — config menu reboot */
#include "esp_attr.h"     /* RTC_NOINIT_ATTR — survives a software reboot         */
#include "driver/usb_serial_jtag.h"  /* bench HIL: read the sim stream from USB  */
#include "lwnx.h"                     /* bench HIL: parse bench-control frames    */

static const char *TAG = "app";

/* ---- Definitions of the shared globals declared in shared.h -------------- */
const sensor_profile_t *g_profile        = NULL;
volatile uint32_t       g_poll_period_ms = POLL_MS_GROUND;
QueueHandle_t           q_range_latest   = NULL;
QueueHandle_t           q_callouts       = NULL;
audio_params_t          g_audio_params   = { .tone_agl = TONE_START_FT, .tone_active = false };
SemaphoreHandle_t       g_audio_mutex    = NULL;

/*  The learned ground reference, applied by the logic task to turn measured
 *  range into AGL. Set once during boot.                                       */
static float s_ground_ref_ft = MOUNT_OFFSET_FALLBACK_FT;

/*  Callout start-altitude cap (ft): the highest callout that is allowed to
 *  speak. Numbers ABOVE this are suppressed (the tone is unaffected). Set once
 *  during boot from the saved config; defaults to the profile's top callout, so
 *  the behaviour is unchanged unless the pilot lowers it in the config menu.    */
static float s_start_alt_ft = 0.0f;   /* 0 == uncapped until boot sets it       */

/*  "Check gear" descent-callout altitude (ft): when a fired descent callout lands
 *  on this height the box appends a "check gear" clip after the number and lets it
 *  through even if the start-altitude cap would otherwise suppress it. 0 == OFF
 *  (the default). Set once during boot from the saved config.                     */
static float s_gear_check_ft = 0.0f;

/*  Tone-start altitude (ft): the height at/below which the presence tone is allowed
 *  to sound. Set once during boot from the saved config (default TONE_START_FT, or
 *  the pilot's higher TONE_START_FT_HIGH). Shared by the logic task's tone gate
 *  (via the sm context) and the audio engine's fade-in schedule so both agree.     */
static float s_tone_start_ft = TONE_START_FT;

/*  "Positive rate" climb callout enable flag. Disabled by default; the state
 *  machine always evaluates the detector, but we only speak it when enabled.      */
static bool  s_posrate_enabled = false;

/*  Vario "blip" enables. Both disabled by default. The state machine always
 *  publishes the vertical rate/accel; the audio engine only chops the tone into
 *  blips when sink-rate is on (climb-rate additionally lets a climb drive them).  */
static bool  s_sinkrate_enabled  = false;
static bool  s_climbrate_enabled = false;

/*  Bench HIL: set during the boot attach probe when the connected bench tool
 *  asked to open the config menu this boot. Runtime-only (never persisted).      */
static volatile bool s_sim_want_config = false;

/*  Bench HIL (real-sensor variant): set during the attach probe when the bench
 *  asked for OP_BENCH_REAL. Unlike sim mode this keeps the real LiDAR on UART1;
 *  it only arms the AGL scaling + raw-reading debug log below.                    */
static volatile bool s_sim_want_bench_real = false;

/*  True once bench real-sensor scaled debug mode is armed for this boot. Read by
 *  the logic task to multiply the AGL by BENCH_SCALE_GAIN and emit the throttled
 *  raw-reading line. Never set on a normal/flight boot.                           */
static bool s_bench_scale = false;

/*  Bench HIL: a "reboot into the config menu" request that survives the SOFTWARE
 *  reboot esp_restart() performs (RTC memory keeps its value across a restart,
 *  but NOT across a power cycle — so it can never strand a real unit in config).
 *  We need this because while the USB-Serial-JTAG driver is installed in sim mode
 *  the chip's RTS hardware-reset is suppressed, so the bench can't reboot us with
 *  RTS; instead OP_ENTER_CONFIG sets this flag and we reboot in software. Checked
 *  once at boot, then cleared.                                                    */
#define SIM_CFG_BOOT_MAGIC  0x43464721u   /* 'CFG!' */
RTC_NOINIT_ATTR static uint32_t s_sim_cfg_boot_flag;

/* ---- LWNX update-rate code per poll profile ------------------------------ */
/*  The binary update-rate code (cmd 76). We keep the sensor at a brisk rate in
 *  flight; on the ASCII path this is a no-op. Code 8 ~= 78 readings/sec.        */
#define SF30_RATE_CODE_ACTIVE  8u

/* ---------------------------------------------------------------------------
 *  Boot: capture a ground-fill of BOOT_BUFFER_N readings spread over ~1 s.
 *
 *  We sample up to GROUND_FILL_SAMPLES times, keeping every valid reading, then
 *  pick BOOT_BUFFER_N of them evenly spread across the window. Junk is filtered
 *  later by robust_mean during resolution.
 * ------------------------------------------------------------------------- */
static size_t capture_ground_fill(float *out, size_t want)
{
    float pool[GROUND_FILL_SAMPLES];
    size_t got = 0;

    uint32_t per_sample_ms = GROUND_FILL_MS / GROUND_FILL_SAMPLES;
    if (per_sample_ms == 0) {
        per_sample_ms = 1;
    }

    for (size_t i = 0; i < GROUND_FILL_SAMPLES && got < GROUND_FILL_SAMPLES; ++i) {
        float r; bool v;
        if (sf30c_read_latest_ft(&r, &v) && v) {
            pool[got++] = r;
        }
        vTaskDelay(pdMS_TO_TICKS(per_sample_ms));
    }

    if (got == 0) {
        return 0;
    }

    /* Evenly subsample 'want' readings from the pool. */
    size_t n = (want < got) ? want : got;
    for (size_t i = 0; i < n; ++i) {
        size_t idx = (got - 1) * i / (n > 1 ? (n - 1) : 1);
        out[i] = pool[idx];
    }
    return n;
}

/* ---------------------------------------------------------------------------
 *  Boot config menu (entered by HOLDING the button at power-on).
 *
 *  Flow (audio + the sensor profile must already be up):
 *    1. Chirp, speak "config mode, memory cleared", and wipe BOTH the learned
 *       ground reference and the saved audio/start-altitude config.
 *    2. LEVEL 1 — audio mode. A single TAP cycles the four modes (each announced
 *       as two composed pieces, e.g. "Stereo" + "Callouts and Tone"); a
 *       DOUBLE-TAP (or CONFIG_COMMIT_MS of silence) confirms the current mode.
 *    3. LEVEL 2 — callout start altitude. Skipped entirely for the tone-only
 *       mode (no callouts to gate). Otherwise: speak "Callout Start Altitude",
 *       then a TAP cycles the profile's callout ladder (200..10 / 500..10),
 *       each spoken with the existing number clips; DOUBLE-TAP / timeout confirms.
 *    4. LEVEL 3 — TONE volume. Speak "Volume Adjustment" + "Tone Only", then a TAP
 *       cycles the tone offset in TONE_VOLUME_DB_STEP steps across the full
 *       cut-or-boost range (TONE_VOLUME_DB_MIN..TONE_VOLUME_DB_MAX). Each step
 *       previews by ear as a 1 kHz tone burst AT the selected offset. Always runs.
 *    5. LEVEL 4 — VOICE volume. Speak "Volume Adjustment" + "Callouts Only", then a
 *       TAP cycles the voice offset down in VOICE_VOLUME_DB_STEP steps (cut-only,
 *       0..VOICE_VOLUME_DB_MIN). Each step previews by ear as one spoken number AT
 *       the selected offset. Skipped for the tone-only mode (no callouts to trim).
 *       Both offsets apply live so prompts afterward reflect them; DOUBLE-TAP /
 *       timeout confirms each.
 *    6. LEVEL 5 — "Check Gear" altitude. Skipped in tone-only mode. Cycle OFF ->
 *       the profile's gear-check altitudes -> OFF, starting on OFF (the default).
 *    7. LEVEL 6 — "Positive Rate" enable toggle. Skipped in tone-only mode. A
 *       simple ON/OFF, starting OFF (the default).
 *    8. LEVEL 7 — Vario "blip" direction. A 3-way OFF -> Sink -> Climb -> OFF,
 *       starting OFF. Sink/Climb are mutually exclusive (the active direction blips;
 *       the other way is a steady tone). Acts on the TONE, so it runs in EVERY mode.
 *    9. LEVEL 8 — tone-start altitude. Voice "Tone Only" + the altitude number; a
 *       TAP toggles TONE_START_FT (100 ft) <-> TONE_START_FT_HIGH (200 ft), starting
 *       on the default. A tone feature, so it runs in every mode.
 *   10. A chirp marks each confirm. Persist all values and reboot.
 *
 *  This function does not return — it always ends in esp_restart().
 * ------------------------------------------------------------------------- */
#define CONFIG_COMMIT_MS    10000  /* idle time with no tap that auto-confirms   */
#define CONFIG_POLL_MS      15     /* button poll cadence inside the menu        */
#define CONFIG_DTAP_MS      400    /* two taps within this window == double-tap  */

/*  Classified result of one menu interaction. */
typedef enum {
    TAP_NONE = 0,   /* nothing happened this poll                                */
    TAP_SINGLE,     /* a single tap (cycle)                                      */
    TAP_DOUBLE,     /* a double tap (confirm)                                    */
    TAP_TIMEOUT,    /* CONFIG_COMMIT_MS elapsed with no tap (confirm)            */
} tap_event_t;

/* ---------------------------------------------------------------------------
 *  Bench HIL: control-frame handling + serial-driven config-menu navigation.
 *
 *  Control frames (LWNX_CMD_BENCH_CTRL) decoded on the sim stream are dispatched
 *  here. OP_REBOOT restarts immediately; the menu opcodes are latched into
 *  s_sim_tap for sim_wait_tap_event() to consume, so the boot config menu can be
 *  driven entirely over serial with the same tap / double-tap / timeout
 *  semantics as the physical button.
 * ------------------------------------------------------------------------- */
static volatile tap_event_t s_sim_tap = TAP_NONE;

static void bench_ctrl_handler(uint8_t op, const uint8_t *arg, size_t n)
{
    (void)arg;
    (void)n;
    switch (op) {
        case OP_REBOOT:
            ESP_LOGW(TAG, "sim: reboot requested by bench");
            vTaskDelay(pdMS_TO_TICKS(50));   /* let the log + DMA flush */
            esp_restart();                   /* does not return */
            break;
        case OP_ENTER_CONFIG:
            /* Software reboot into the config menu (RTS reset is suppressed while
             * the USB driver is installed, so we can't rely on a hardware reset).
             * The RTC flag carries the intent across the restart.                */
            ESP_LOGW(TAG, "sim: reboot-to-config requested by bench");
            s_sim_cfg_boot_flag = SIM_CFG_BOOT_MAGIC;
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_restart();                   /* does not return */
            break;
        case OP_MENU_NEXT:    s_sim_tap = TAP_SINGLE; break;
        case OP_MENU_CONFIRM: s_sim_tap = TAP_DOUBLE; break;
        default: break;
    }
}

/*  Bench HIL replacement for wait_tap_event(): pump the sim stream and translate
 *  the menu opcodes into tap events. CONFIG_COMMIT_MS of silence confirms, just
 *  like the physical-button path.                                               */
static tap_event_t sim_wait_tap_event(int *idle_ms)
{
    for (;;) {
        sf30c_sim_read_drain();              /* may latch s_sim_tap via the cb */
        if (s_sim_tap != TAP_NONE) {
            tap_event_t ev = s_sim_tap;
            s_sim_tap = TAP_NONE;
            *idle_ms  = 0;
            return ev;
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_POLL_MS));
        *idle_ms += CONFIG_POLL_MS;
        if (*idle_ms >= CONFIG_COMMIT_MS) {
            return TAP_TIMEOUT;
        }
    }
}

/*  Wait for the next tap event. Tracks a rolling idle timer for the timeout and,
 *  on a press, opens a short window to see whether a second press makes it a
 *  double-tap. *idle_ms accumulates across calls so the timeout spans the whole
 *  level, not just one call.                                                    */
static tap_event_t wait_tap_event(int *idle_ms)
{
    /* Bench HIL: when running in sim mode the config menu is driven over serial,
     * not the physical button. */
    if (sf30c_sim_active()) {
        return sim_wait_tap_event(idle_ms);
    }

    /* Wait for a rising edge (press), accumulating idle time toward timeout. */
    while (!boot_buffer_button_down()) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_POLL_MS));
        *idle_ms += CONFIG_POLL_MS;
        if (*idle_ms >= CONFIG_COMMIT_MS) {
            return TAP_TIMEOUT;
        }
    }
    *idle_ms = 0;   /* a press resets the idle timer */

    /* Wait for this press to release (so the next press is a distinct tap). */
    while (boot_buffer_button_down()) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_POLL_MS));
    }

    /* Double-tap window: a second press within CONFIG_DTAP_MS makes it double. */
    for (int t = 0; t < CONFIG_DTAP_MS; t += CONFIG_POLL_MS) {
        if (boot_buffer_button_down()) {
            /* Consume the second tap's release, then report a double-tap. */
            while (boot_buffer_button_down()) {
                vTaskDelay(pdMS_TO_TICKS(CONFIG_POLL_MS));
            }
            return TAP_DOUBLE;
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_POLL_MS));
    }
    return TAP_SINGLE;
}

/*  Speak an audio-mode option as two composed pieces (channel + stream) with a
 *  short gap so they don't run together. */
static void announce_mode(int mode)
{
    config_piece_t channel = (mode == AUDIO_MODE_STEREO_BOTH)
                             ? CFG_PIECE_STEREO : CFG_PIECE_MONO;
    config_piece_t stream;
    switch (mode) {
        case AUDIO_MODE_MONO_CALLOUTS: stream = CFG_PIECE_CALLOUTS_ONLY;     break;
        case AUDIO_MODE_MONO_TONE:     stream = CFG_PIECE_TONE_ONLY;         break;
        default:                       stream = CFG_PIECE_CALLOUTS_AND_TONE; break;
    }
    audio_play_clip_blocking(config_clip_piece(channel));
    vTaskDelay(pdMS_TO_TICKS(120));   /* small gap between the two pieces */
    audio_play_clip_blocking(config_clip_piece(stream));
}

/*  Preview the current tone/voice BALANCE by ear: the "mini-flare" — the presence
 *  tone sweeping down the 20->10 ft band with the "20" and "10" callouts ducking it,
 *  exactly as it flies (audio_play_volume_preview_blocking, sharing the render
 *  loop's duck). Both streams play at the live tone/voice offsets, so the SAME
 *  preview serves both volume levels: in LEVEL 3 the tone offset moves under it, in
 *  LEVEL 4 the voice offset does, and the pilot hears the balance shift either way.  */
static void preview_balance(void)
{
    audio_play_volume_preview_blocking();
}

static void run_config_menu(void)
{
    ESP_LOGW(TAG, "config mode: chirp + wipe ground/audio/start-altitude config");

    /* Entry chirp, then the spoken "config mode, memory cleared", then the wipe.*/
    audio_play_clip_blocking(config_clip_chirp());
    audio_play_clip_blocking(config_clip_enter());
    boot_buffer_wipe_ground();
    config_wipe_audio_mode();
    config_wipe_start_alt();
    config_wipe_tone_volume();
    config_wipe_voice_volume();
    config_wipe_gear_check_alt();
    config_wipe_positive_rate();
    config_wipe_sink_rate();
    config_wipe_climb_rate();
    config_wipe_tone_start();

    /* Start the menu at 0 dB on BOTH offsets (no change) so the prompts/previews
     * play un-trimmed until the pilot chooses levels in LEVELs 3 & 4 below.       */
    audio_set_tone_db(0.0f);
    audio_set_voice_db(0.0f);

    /* Wait out the original hold so it isn't mistaken for a menu tap (bounded). */
    for (int i = 0; i < 300 && boot_buffer_button_down(); ++i) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_POLL_MS));
    }

    /* ---- LEVEL 1: audio mode ---------------------------------------------- */
    int selected_mode = DEFAULT_AUDIO_MODE;
    announce_mode(selected_mode);          /* announce the starting selection */

    int idle_ms = 0;
    for (;;) {
        tap_event_t ev = wait_tap_event(&idle_ms);
        if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
            break;                          /* confirm this mode */
        }
        if (ev == TAP_SINGLE) {
            selected_mode = (selected_mode + 1) % AUDIO_MODE_COUNT;
            ESP_LOGI(TAG, "config: mode -> %d", selected_mode);
            announce_mode(selected_mode);
        }
    }
    audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
    config_save_audio_mode(selected_mode);

    /* ---- LEVEL 2: callout start altitude ---------------------------------- */
    /* Resolve the active behaviour so we can skip this level when there are no
     * callouts to gate (tone-only mode).                                       */
    audio_config_t chosen = audio_config_from_mode(selected_mode);
    if (chosen.callouts_enabled) {
        /* The profile ladder is descending (callouts[0] is the top). The cap
         * starts at the top (= the default) and tapping steps DOWN the ladder. */
        size_t n_cal   = g_profile->n_callouts;
        size_t cap_idx = 0;                 /* index into g_profile->callouts[] */

        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_START_ALT));
        vTaskDelay(pdMS_TO_TICKS(120));
        /* Announce the starting (top) altitude via the number clips. */
        callout_id_t cid = callout_id_for_ft(g_profile->callouts[cap_idx]);
        audio_play_clip_blocking(callout_clip(cid));

        idle_ms = 0;
        for (;;) {
            tap_event_t ev = wait_tap_event(&idle_ms);
            if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
                break;                      /* confirm this altitude */
            }
            if (ev == TAP_SINGLE) {
                cap_idx = (cap_idx + 1) % n_cal;   /* step down, wrap to top */
                float ft = g_profile->callouts[cap_idx];
                ESP_LOGI(TAG, "config: start-alt -> %.0f ft", ft);
                audio_play_clip_blocking(callout_clip(callout_id_for_ft(ft)));
            }
        }
        audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
        config_save_start_alt(g_profile->callouts[cap_idx]);
    } else {
        ESP_LOGI(TAG, "config: tone-only mode, skipping start-altitude menu");
    }

    /* ---- LEVEL 3: TONE volume ("Volume Adjustment, Tone Only") ------------- */
    /* A pilot-set trim on the presence TONE that can CUT or BOOST. Cycling steps
     * DOWN by TONE_VOLUME_DB_STEP and wraps from below the floor back to the top,
     * so a single button still reaches every value (0,-2,-4,-6,+6,+4,+2,0,...).
     * Always runs (it trims the tone, which sounds in every mode). Each tap applies
     * the offset LIVE then previews it as two 1 kHz tone bursts.                  */
    {
        float tone_db = 0.0f;               /* start at no change (the default)  */
        audio_set_tone_db(tone_db);

        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_VOLUME_ADJ));
        vTaskDelay(pdMS_TO_TICKS(120));
        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_TONE_ONLY));
        vTaskDelay(pdMS_TO_TICKS(120));
        preview_balance();                  /* preview the starting (0 dB) level */

        idle_ms = 0;
        for (;;) {
            tap_event_t ev = wait_tap_event(&idle_ms);
            if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
                break;                      /* confirm this tone volume          */
            }
            if (ev == TAP_SINGLE) {
                /* Step DOWN one increment; wrap from below the floor to the top. */
                tone_db -= TONE_VOLUME_DB_STEP;
                if (tone_db < TONE_VOLUME_DB_MIN - 0.001f) {
                    tone_db = TONE_VOLUME_DB_MAX;   /* wrap to the loudest boost */
                }
                ESP_LOGI(TAG, "config: tone volume -> %+.0f dB", tone_db);
                audio_set_tone_db(tone_db);     /* apply live for the preview */
                preview_balance();
            }
        }
        audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
        config_save_tone_volume(tone_db);
    }

    /* ---- LEVEL 4: VOICE volume ("Volume Adjustment, Callouts Only") -------- */
    /* A cut-only trim on the voice CALLOUTS. Cycling steps DOWN by
     * VOICE_VOLUME_DB_STEP and wraps from below the floor back to 0 dB. Skipped in
     * tone-only mode (no callouts to trim). Each tap applies the offset LIVE then
     * previews it as one spoken number.                                          */
    if (chosen.callouts_enabled) {
        float voice_db = 0.0f;              /* start at no cut (the default)     */
        audio_set_voice_db(voice_db);

        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_VOLUME_ADJ));
        vTaskDelay(pdMS_TO_TICKS(120));
        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_CALLOUTS_ONLY));
        vTaskDelay(pdMS_TO_TICKS(120));
        preview_balance();                  /* preview the starting (0 dB) level */

        idle_ms = 0;
        for (;;) {
            tap_event_t ev = wait_tap_event(&idle_ms);
            if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
                break;                      /* confirm this voice volume         */
            }
            if (ev == TAP_SINGLE) {
                /* Step DOWN one increment; wrap back to 0 dB past the floor. */
                voice_db -= VOICE_VOLUME_DB_STEP;
                if (voice_db < VOICE_VOLUME_DB_MIN - 0.001f) {
                    voice_db = 0.0f;        /* wrap to no-cut */
                }
                ESP_LOGI(TAG, "config: voice volume -> %.0f dB", voice_db);
                audio_set_voice_db(voice_db);   /* apply live for the preview */
                preview_balance();
            }
        }
        audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
        config_save_voice_volume(voice_db);
    } else {
        ESP_LOGI(TAG, "config: tone-only mode, skipping voice-volume menu");
    }

    /* ---- LEVEL 5: "Check Gear" altitude ----------------------------------- */
    /* The descent "check gear" reminder. Cycle is OFF -> highest -> ... -> lowest
     * -> OFF, starting on OFF (the default). The page title reuses the "check gear"
     * clip itself; altitudes are spoken with the existing number clips and OFF with
     * the "off" piece. Skipped in tone-only mode (it is a voice callout).         */
    if (chosen.callouts_enabled) {
        const float *opts   = g_profile->gear_check_opts;
        size_t       n_opts = g_profile->n_gear_check_opts;
        /* Selection index across [OFF, opts[0], opts[1], ...]: 0 == OFF, and
         * 1..n_opts select opts[sel-1]. Start on OFF.                            */
        size_t sel = 0;

        audio_play_clip_blocking(callout_clip(CO_CHECK_GEAR));      /* "check gear" */
        vTaskDelay(pdMS_TO_TICKS(120));
        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_OFF)); /* starting OFF */

        idle_ms = 0;
        for (;;) {
            tap_event_t ev = wait_tap_event(&idle_ms);
            if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
                break;                          /* confirm this altitude (or OFF) */
            }
            if (ev == TAP_SINGLE) {
                sel = (sel + 1) % (n_opts + 1); /* cycle OFF -> opts... -> OFF */
                if (sel == 0) {
                    ESP_LOGI(TAG, "config: gear-check -> OFF");
                    audio_play_clip_blocking(config_clip_piece(CFG_PIECE_OFF));
                } else {
                    float ft = opts[sel - 1];
                    ESP_LOGI(TAG, "config: gear-check -> %.0f ft", ft);
                    audio_play_clip_blocking(callout_clip(callout_id_for_ft(ft)));
                }
            }
        }
        audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
        config_save_gear_check_alt(sel == 0 ? 0.0f : opts[sel - 1]);
    } else {
        ESP_LOGI(TAG, "config: tone-only mode, skipping gear-check menu");
    }

    /* ---- LEVEL 6: "Positive Rate" enable toggle --------------------------- */
    /* A simple ON/OFF toggle for the takeoff climb callout, starting OFF (the
     * default). The page title reuses the "positive rate" clip; ON speaks the "on"
     * piece and OFF speaks the "off" piece. Skipped in tone-only mode.            */
    if (chosen.callouts_enabled) {
        bool pr_on = false;                     /* start OFF (the default) */

        audio_play_clip_blocking(callout_clip(CO_POSITIVE_RATE));   /* title */
        vTaskDelay(pdMS_TO_TICKS(120));
        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_OFF)); /* starting OFF */

        idle_ms = 0;
        for (;;) {
            tap_event_t ev = wait_tap_event(&idle_ms);
            if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
                break;                          /* confirm this setting */
            }
            if (ev == TAP_SINGLE) {
                pr_on = !pr_on;
                ESP_LOGI(TAG, "config: positive-rate -> %s", pr_on ? "ON" : "OFF");
                audio_play_clip_blocking(config_clip_piece(pr_on ? CFG_PIECE_ON
                                                                 : CFG_PIECE_OFF));
            }
        }
        audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
        config_save_positive_rate(pr_on);
    } else {
        ESP_LOGI(TAG, "config: tone-only mode, skipping positive-rate menu");
    }

    /* ---- LEVEL 7: Vario "blip" direction (OFF / Sink / Climb) ------------- */
    /* A single 3-way selector for the varioshore tone-blip. Cycle OFF -> SINK ->
     * CLIMB -> OFF, starting OFF (the default). OFF = steady tone; SINK = the tone
     * blips faster the faster you DESCEND (constant when level/climbing); CLIMB =
     * the inverse (blips on the way up). The two directions are MUTUALLY EXCLUSIVE.
     * NOT gated on callouts_enabled: it acts on the TONE, so it must be reachable in
     * tone-only mode too (the spoken prompts still play in any mode). Each tap speaks
     * the new selection: "off" / "sink rate" / "climb rate".                        */
    {
        int vsel = 0;   /* 0 = OFF, 1 = SINK, 2 = CLIMB */

        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_OFF)); /* starting OFF */

        idle_ms = 0;
        for (;;) {
            tap_event_t ev = wait_tap_event(&idle_ms);
            if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
                break;                          /* confirm this setting */
            }
            if (ev == TAP_SINGLE) {
                vsel = (vsel + 1) % 3;          /* OFF -> SINK -> CLIMB -> OFF */
                if (vsel == 1) {
                    ESP_LOGI(TAG, "config: vario -> SINK");
                    audio_play_clip_blocking(callout_clip(CO_SINK_RATE));
                } else if (vsel == 2) {
                    ESP_LOGI(TAG, "config: vario -> CLIMB");
                    audio_play_clip_blocking(callout_clip(CO_CLIMB_RATE));
                } else {
                    ESP_LOGI(TAG, "config: vario -> OFF");
                    audio_play_clip_blocking(config_clip_piece(CFG_PIECE_OFF));
                }
            }
        }
        audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
        /* Persist as the two mutually-exclusive flags (at most one set). */
        config_save_sink_rate(vsel == 1);
        config_save_climb_rate(vsel == 2);
    }

    /* ---- LEVEL 8: tone-start altitude ------------------------------------- */
    /* The altitude at which the presence TONE begins its fade-in. Only two choices
     * are offered: the default TONE_START_FT (100 ft) or the higher
     * TONE_START_FT_HIGH (200 ft) for an earlier, gentler swell on a long final. A
     * single TAP toggles between them; double-tap / timeout confirms. Like the vario
     * levels above this acts on the TONE, so it is NOT gated on callouts_enabled —
     * it must be reachable in tone-only mode too. The page has no dedicated voice
     * clip, so we voice it with the existing "Tone Only" piece followed by the
     * altitude NUMBER ("Tone Only ... one hundred") — unambiguous since this is the
     * only level that pairs "Tone Only" with a spoken altitude.                     */
    {
        /* Two-entry cycle: index 0 == default, index 1 == the high option. */
        const float ts_opts[2] = { TONE_START_FT, TONE_START_FT_HIGH };
        size_t      ts_sel      = 0;            /* start on the default */

        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_TONE_ONLY)); /* "tone only" */
        vTaskDelay(pdMS_TO_TICKS(120));
        /* Announce the starting (default) altitude via the number clips. */
        audio_play_clip_blocking(callout_clip(callout_id_for_ft(ts_opts[ts_sel])));

        idle_ms = 0;
        for (;;) {
            tap_event_t ev = wait_tap_event(&idle_ms);
            if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
                break;                          /* confirm this tone-start altitude */
            }
            if (ev == TAP_SINGLE) {
                ts_sel = (ts_sel + 1) % 2;      /* toggle 100 <-> 200 */
                float ft = ts_opts[ts_sel];
                ESP_LOGI(TAG, "config: tone-start -> %.0f ft", ft);
                audio_play_clip_blocking(callout_clip(callout_id_for_ft(ft)));
            }
        }
        audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
        config_save_tone_start(ts_opts[ts_sel]);
    }

    ESP_LOGW(TAG, "config committed (mode %d); rebooting", selected_mode);
    vTaskDelay(pdMS_TO_TICKS(50));   /* let the log + DMA flush */
    esp_restart();                   /* does not return */
}

/* ---------------------------------------------------------------------------
 *  Logic task (core 1): the decision loop.
 * ------------------------------------------------------------------------- */
static void logic_task(void *arg)
{
    (void)arg;

    /* arg carries a pointer to the boot-decided initial state (see app_main). */
    sm_state_t initial = (arg != NULL) ? *(sm_state_t *)arg : ST_GROUND;

    sm_ctx_t sm;
    sm_init(&sm, initial);

    /* Override the tone gate's start altitude with the pilot's saved choice (sm_init
     * seeds the default). Boot resolves s_tone_start_ft before this task is created,
     * so it is final by the time we read it here.                                  */
    sm.tone_start_ft = s_tone_start_ft;

    int64_t last_us = esp_timer_get_time();
    bool sleep_allowed_prev = false;

    for (;;) {
        /* Peek the freshest range sample (non-destructive). */
        range_sample_t s;
        if (xQueuePeek(q_range_latest, &s, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;   /* no sample yet */
        }

        /* dt since the last decision. */
        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - last_us) / 1e6f;
        last_us = now_us;
        if (dt_s <= 0.0f) {
            dt_s = 0.001f;
        }

        /* Apply the learned ground reference: AGL = range - ground. On a
         * lost-signal sample, hold by feeding the previous AGL (sm clamps). */
        float agl = s.range_ft - s_ground_ref_ft;
        if (agl < 0.0f) {
            agl = 0.0f;
        }

        /* Bench real-sensor debug: a close bench target only swings a few feet,
         * so multiply the (real-feet) AGL up to flight altitudes — one foot above
         * the learned ground becomes BENCH_SCALE_GAIN feet, so ~5 ft of hand
         * travel sweeps the whole 0..400 ft callout band. The raw sensor reading
         * is logged (throttled) so the dev can confirm the LiDAR is streaming.
         * Armed only for an OP_BENCH_REAL boot; a flight build never enters here. */
        if (s_bench_scale) {
            static int64_t s_bench_log_us = 0;   /* logic task is single-threaded */
            if (now_us - s_bench_log_us >= (int64_t)BENCH_DEBUG_LOG_MS * 1000) {
                s_bench_log_us = now_us;
                ESP_LOGI(TAG, "bench: raw=%.2f ft  ground=%.2f ft  agl=%.2f ft -> %.0f ft%s  seq=%u",
                         (double)s.range_ft, (double)s_ground_ref_ft, (double)agl,
                         (double)(agl * BENCH_SCALE_GAIN), s.valid ? "" : " (HOLD)",
                         (unsigned)s.seq);
            }
            agl *= BENCH_SCALE_GAIN;
        }

        sm_out_t out;
        sm_step(&sm, agl, dt_s, g_profile, &out);

        /* Fire the callout, if any, mapping the profile height -> clip id.
         * Suppress any callout ABOVE the configured start-altitude cap so the
         * pilot only hears numbers from their chosen ceiling down (the tone is
         * unaffected). With the cap at the profile top this never suppresses.
         *
         * The "check gear" reminder rides on this fire: when the crossed height
         * is the pilot's gear-check altitude we (a) let the number through even
         * if the start-altitude cap would have suppressed it — the reminder is a
         * deliberate, independent safety call — and (b) queue the "check gear"
         * clip right after the number so it speaks "<height> ... check gear".    */
        if (out.fired_callout >= 0) {
            float ft = g_profile->callouts[out.fired_callout];
            bool  is_gear_check = (s_gear_check_ft > 0.0f) &&
                                  (fabsf(ft - s_gear_check_ft) < 0.5f);
            if (ft <= s_start_alt_ft || is_gear_check) {
                callout_id_t cid = callout_id_for_ft(ft);
                if (cid != CO_COUNT) {
                    audio_request_callout(cid);           /* the altitude number */
                }
                if (is_gear_check) {
                    audio_request_callout(CO_CHECK_GEAR); /* "... check gear"     */
                }
                ESP_LOGI(TAG, "callout %.0f ft%s (state=%d)",
                         ft, is_gear_check ? " + check gear" : "", out.state);
            } else {
                ESP_LOGI(TAG, "callout %.0f ft suppressed (cap %.0f ft)",
                         ft, s_start_alt_ft);
            }
        }

        /* "Positive rate" climb callout: the state machine confirms a sustained
         * post-liftoff climb (see sm_step); we only voice it when the pilot has
         * enabled the feature.                                                    */
        if (out.fired_positive_rate && s_posrate_enabled) {
            audio_request_callout(CO_POSITIVE_RATE);
            ESP_LOGI(TAG, "positive rate (climb confirmed, state=%d)", out.state);
        }

        /* Publish tone params + poll cadence. The vertical rate rides along so the
         * audio engine's vario blip can chop the tone by descent rate.              */
        audio_set_params(out.tone_agl, out.tone_active, out.vert_fps);
        g_poll_period_ms = poll_profile_to_ms(out.poll);

        /* Light-sleep policy: allowed only in GROUND/CRUISE, where the tone is
         * silent. Suspend the I2S channel before sleeping, resume on exit.    */
        bool sleep_allowed = (out.state == ST_GROUND || out.state == ST_CRUISE);
        if (sleep_allowed && !sleep_allowed_prev) {
            audio_suspend();
        } else if (!sleep_allowed && sleep_allowed_prev) {
            audio_resume();
        }
        sleep_allowed_prev = sleep_allowed;

        /* Decision cadence: fast in active states, relaxed when sleep-friendly
         * (the PM subsystem drops to light-sleep during the idle between ticks). */
        uint32_t tick_ms = sleep_allowed ? POLL_MS_CRUISE : 20;
        vTaskDelay(pdMS_TO_TICKS(tick_ms));
    }
}

/* ---------------------------------------------------------------------------
 *  Enable automatic light-sleep (needs CONFIG_PM_ENABLE + tickless idle).
 * ------------------------------------------------------------------------- */
static void enable_power_management(void)
{
    esp_pm_config_t pm = {
        .max_freq_mhz       = PM_MAX_FREQ_MHZ,
        .min_freq_mhz       = PM_MIN_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    esp_err_t err = esp_pm_configure(&pm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_pm_configure failed (%s); running without light-sleep",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "power management: %d/%d MHz, light-sleep on",
                 PM_MAX_FREQ_MHZ, PM_MIN_FREQ_MHZ);
    }
}

/* ---------------------------------------------------------------------------
 *  Bench HIL attach probe (runs once, early in boot).
 *
 *  Installs the USB-Serial-JTAG driver and listens for a bench "hello" frame.
 *  If a USB host is on the bus we wait up to SIM_ATTACH_WINDOW_MS for it; if
 *  nothing is plugged in (no USB SOF) we bail after SIM_ATTACH_GRACE_MS so a
 *  normal/flight boot is barely delayed. On no-attach we uninstall the driver so
 *  the console is left exactly as it was. Returns true if sim mode should be
 *  entered; sets s_sim_want_config when the host asked to open the config menu.
 * ------------------------------------------------------------------------- */
static bool bench_attach_detected(void)
{
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t cfg = {
            .tx_buffer_size = SIM_USJ_TX_BUF,
            .rx_buffer_size = SIM_USJ_RX_BUF,
        };
        if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) {
            return false;
        }
    }

    lwnx_parser_t p;
    lwnx_parser_reset(&p);

    TickType_t start         = xTaskGetTickCount();
    TickType_t hard_deadline = start + pdMS_TO_TICKS(SIM_ATTACH_WINDOW_MS);
    bool found = false;

    while (!found && xTaskGetTickCount() < hard_deadline) {
        uint8_t rx[64];
        int n = usb_serial_jtag_read_bytes(rx, sizeof rx, pdMS_TO_TICKS(20));
        for (int i = 0; i < n; ++i) {
            lwnx_frame_t f;
            if (lwnx_feed(&p, rx[i], &f) &&
                f.cmd == LWNX_CMD_BENCH_CTRL && f.plen >= 1) {
                uint8_t op = f.payload[0];
                if (op == OP_ENTER_CONFIG) {
                    s_sim_want_config = true;
                    found = true;
                    break;
                }
                if (op == OP_HELLO) {
                    found = true;
                    break;
                }
                if (op == OP_BENCH_REAL) {
                    /* Real-sensor scaled debug boot: caught here so the driver is
                     * left installed for app_main to tidy up, just like HELLO.    */
                    s_sim_want_bench_real = true;
                    found = true;
                    break;
                }
            }
        }
        /* Fast-bail a normal boot: if no USB host is on the bus by the grace
         * point, no bench is attached — don't keep a flight boot waiting.       */
        if (!found && !usb_serial_jtag_is_connected() &&
            (xTaskGetTickCount() - start) > pdMS_TO_TICKS(SIM_ATTACH_GRACE_MS)) {
            break;
        }
    }

    if (!found) {
        usb_serial_jtag_driver_uninstall();   /* restore the pristine console */
    }
    return found;
}

/* ---------------------------------------------------------------------------
 *  app_main
 * ------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "LidarAGL boot");

    /* WiFi/BT are compiled out (sdkconfig) and never started here — net effect:
     * radios never powered, no draw, no EMI into the audio/avionics path.     */

    /* 1. NVS + reset-button GPIO. */
    boot_buffer_init();

    /* 1b. Bench HIL: if the tools/bench_sim app is attached over USB, enter the
     * runtime-only simulation mode — the sensor read path then draws simulated
     * LWNX frames from USB instead of the (absent) LiDAR. Never persisted, so a
     * plain power cycle returns the box to the real sensor.                      */
    if (bench_attach_detected()) {
        if (s_sim_want_bench_real) {
            /* Real-sensor scaled debug: the LiDAR stays on UART1 (sim mode is
             * left OFF), so we only arm the AGL scaling + raw-reading log the
             * logic task applies. We never read the USB stream in this mode, so
             * uninstall the driver the attach probe left installed to restore the
             * pristine console for log monitoring.                               */
            s_bench_scale = true;
            usb_serial_jtag_driver_uninstall();
            ESP_LOGW(TAG, "BENCH REAL-SENSOR MODE: real LiDAR on UART, AGL scaled "
                          "x%.0f for bench callout testing", (double)BENCH_SCALE_GAIN);
        } else {
            sf30c_set_bench_ctrl_cb(bench_ctrl_handler);
            sf30c_sim_enable();
        }
    }

    /* A bench "reboot into config" (OP_ENTER_CONFIG) lands here: the RTC flag
     * survived the software restart, so open the config menu this boot. Cleared
     * immediately so a later plain reboot doesn't re-enter the menu.            */
    if (sf30c_sim_active() && s_sim_cfg_boot_flag == SIM_CFG_BOOT_MAGIC) {
        s_sim_cfg_boot_flag = 0;
        s_sim_want_config   = true;
    }

    /* Resolve the stored audio configuration (mono/stereo + which streams play).
     * NVS wins at runtime; a missing/corrupt value yields DEFAULT_AUDIO_MODE.   */
    audio_config_t audio_cfg = audio_config_from_mode(config_load_audio_mode());

    /* 2. Sensor up + autodetect + stream config. We detect the profile BEFORE the
     * config menu so the start-altitude sub-menu can cycle the correct callout
     * ladder (200..10 for SF30/C, 500..10 for SF30/D).                          */
    sf30c_init();
    g_profile = sf30c_detect_profile();
    sf30c_configure_stream(SF30_RATE_CODE_ACTIVE);
    ESP_LOGI(TAG, "active profile: %s (cruise %.0f ft, %d callouts)",
             g_profile->name, g_profile->cruise_ft, (int)g_profile->n_callouts);

    /* 3. Audio up with the resolved runtime configuration. */
    audio_init(&audio_cfg);

    /* 4. Config button HELD at boot -> enter the config menu. Audio + profile are
     * both up, so the menu can speak prompts and cycle the right callout ladder.
     * run_config_menu() wipes ground + config and never returns (it reboots once
     * the selection commits).                                                    */
    if (boot_buffer_reset_pressed() ||
        (sf30c_sim_active() && s_sim_want_config)) {
        run_config_menu();               /* does not return                      */
    }

    /* Resolve the callout start-altitude cap now the profile is known (its top
     * callout is the default when nothing has been configured).                 */
    s_start_alt_ft = config_load_start_alt(g_profile->callouts[0]);
    ESP_LOGI(TAG, "callout start-altitude cap: %.0f ft", s_start_alt_ft);

    /* Resolve the optional callouts + vario blips. Gear-check is OFF (0) by default,
     * positive-rate is disabled, and both vario blips are disabled, so an un-config-
     * ured box behaves exactly as before until the pilot turns them on in the menu. */
    s_gear_check_ft     = config_load_gear_check_alt();
    s_posrate_enabled   = config_load_positive_rate();
    s_sinkrate_enabled  = config_load_sink_rate();
    s_climbrate_enabled = config_load_climb_rate();
    ESP_LOGI(TAG, "gear-check altitude: %.0f ft%s | positive-rate: %s",
             s_gear_check_ft, s_gear_check_ft <= 0.0f ? " (OFF)" : "",
             s_posrate_enabled ? "ON" : "OFF");
    ESP_LOGI(TAG, "vario blip: sink-rate %s, climb-rate %s",
             s_sinkrate_enabled ? "ON" : "OFF",
             s_climbrate_enabled ? "ON" : "OFF");

    /* Hand the vario enables to the audio engine once (lock-free bools thereafter). */
    audio_set_vario_enable(s_sinkrate_enabled, s_climbrate_enabled);

    /* Resolve the pilot's tone-start altitude (default 100 ft, or 200 ft) and hand
     * it to the audio engine so the presence-tone pitch + fade-in anchor on it. The
     * logic task also feeds it into the sm tone gate (see logic_task) so the gate and
     * the audio schedule share one start altitude. Done after the menu branch so a
     * freshly-committed value takes effect this boot.                              */
    s_tone_start_ft = config_load_tone_start();
    audio_set_tone_start(s_tone_start_ft);
    ESP_LOGI(TAG, "tone-start altitude: %.0f ft", s_tone_start_ft);

    /* Apply the saved TONE + VOICE volume offsets (0 dB == no change). Done after
     * the menu branch so freshly-committed values are the ones in effect this boot. */
    float tone_vol_db  = config_load_tone_volume();
    float voice_vol_db = config_load_voice_volume();
    audio_set_tone_db(tone_vol_db);
    audio_set_voice_db(voice_vol_db);
    ESP_LOGI(TAG, "volume offsets: tone %+.0f dB, voice %.0f dB",
             tone_vol_db, voice_vol_db);

    /* 5. Ground-fill + current reading -> reconstruct ground reference. */
    float ground_reads[BOOT_BUFFER_N];
    size_t n_fill = capture_ground_fill(ground_reads, BOOT_BUFFER_N);

    boot_entry_t stored[BOOT_BUFFER_N];
    size_t n_stored = boot_buffer_load(stored, BOOT_BUFFER_N);

    float current_ft = 0.0f;
    bool  cur_valid  = false;
    sf30c_read_latest_ft(&current_ft, &cur_valid);

    /* If the very latest sample was a lost return, fall back to the freshest
     * valid ground-fill reading rather than feeding resolve() a stale/zero
     * value — otherwise an early lost-signal could be mistaken for "on ground
     * at 0 ft" and corrupt the airborne decision.                             */
    if (!cur_valid && n_fill > 0) {
        current_ft = ground_reads[n_fill - 1];
    }

    boot_result_t br;
    boot_buffer_resolve(stored, n_stored, current_ft, &br);
    s_ground_ref_ft = br.ground_ref_ft;

    ESP_LOGI(TAG, "ground_ref=%.2f ft  boot_agl=%.1f ft  airborne=%d calib_err=%d",
             br.ground_ref_ft, br.boot_agl_ft, br.airborne, br.calib_error);

    /* 6. Calibration error -> warn the pilot: the chirp grabs attention, then the
     *    spoken instruction tells them what to do ("please reset unit on the
     *    ground"). Both are blocking so they play in order before tasks start.   */
    if (br.calib_error) {
        ESP_LOGW(TAG, "no ground reference; using %.1f ft fallback (chirp + voice)",
                 (double)MOUNT_OFFSET_FALLBACK_FT);
        audio_play_chirp();
        audio_play_clip_blocking(callout_calib_voice());
    }

    /* 7. The ONE NVS write per boot: persist this boot's ground readings.
     *    Only refresh the stored ground when we're actually on the ground —
     *    an in-flight reboot must NOT overwrite the learned ground with an
     *    airborne reading.                                                     */
    if (!br.airborne && n_fill > 0) {
        boot_buffer_commit(ground_reads, n_fill, (uint32_t)(esp_timer_get_time() / 1000));
    }

    /* 8. Seed the state machine from the reconstructed AGL. */
    static sm_state_t initial_state;
    initial_state = sm_initial_state(br.boot_agl_ft, !br.calib_error || br.airborne,
                                     g_profile);
    ESP_LOGI(TAG, "initial state = %d", initial_state);

    /* 9. Create the inter-task primitives. */
    q_range_latest = xQueueCreate(1, sizeof(range_sample_t));
    q_callouts     = xQueueCreate(4, sizeof(callout_id_t));
    g_audio_mutex  = xSemaphoreCreateMutex();
    configASSERT(q_range_latest && q_callouts && g_audio_mutex);

    /* Spawn tasks: sensor on core 0; logic + audio on core 1. */
    xTaskCreatePinnedToCore(sensor_task, "sensor", SENSOR_TASK_STACK, NULL,
                            SENSOR_TASK_PRIO, NULL, SENSOR_TASK_CORE);
    xTaskCreatePinnedToCore(audio_task, "audio", AUDIO_TASK_STACK, NULL,
                            AUDIO_TASK_PRIO, NULL, AUDIO_TASK_CORE);
    xTaskCreatePinnedToCore(logic_task, "logic", LOGIC_TASK_STACK, &initial_state,
                            LOGIC_TASK_PRIO, NULL, LOGIC_TASK_CORE);

    /* Enable automatic light-sleep last, once everything is running. */
    enable_power_management();

    ESP_LOGI(TAG, "running");
}
