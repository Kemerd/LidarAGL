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
#include "robust.h"          /* boot-reading sanity: robust_estimate/robust_mean */
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
#include "esp_task_wdt.h" /* TWDT: the flight-critical loops subscribe + feed    */
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
 *  the logic task to multiply the AGL by s_bench_scale_gain and emit the throttled
 *  raw-reading line. Never set on a normal/flight boot. DEMO_MODE builds also arm
 *  it (standalone, from the NVS demo gain) — see the demo block in app_main().    */
static bool s_bench_scale = false;

/*  The actual AGL scale used in bench real-sensor mode. Seeded with the compile-
 *  time BENCH_SCALE_GAIN default, but the bench tool can override it per boot by
 *  appending a little-endian float to the OP_BENCH_REAL attach frame (parsed in
 *  bench_attach_detected). Lets the dev dial the sweep without a reflash.         */
static float s_bench_scale_gain = BENCH_SCALE_GAIN;

#if DEMO_MODE
/*  True when THIS boot's AGL scaling was armed by the saved DEMO gain rather than
 *  a USB bench attach. Gates the demo-only ground handling on top of the shared
 *  s_bench_scale machinery: the real-feet parked watchdog in the logic task and
 *  the self-calibrating, never-persisted ground reference (see app_main).        */
static bool s_demo_scale = false;
#endif

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
#if DEMO_MODE
    /* The wiped demo gain reloads as DEMO_GAIN_DEFAULT (not OFF) so a wiped demo
     * unit stays demo-ready even if the user bails out of the menu before LEVEL 9. */
    config_wipe_demo_gain();
#endif

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

#if DEMO_MODE
    /* ---- LEVEL 9 (DEMO builds only): demo AGL multiplier ------------------- */
    /* The show-floor pulley-rig gain: each real foot above the learned ground is
     * flown as this many feet of AGL, so a couple of feet of target travel sweeps
     * the whole callout ladder (see the DEMO_MODE section of config.h). The cycle
     * is x50 -> x100 -> x200 -> x400 -> OFF -> x50..., starting on the
     * DEMO_GAIN_DEFAULT (x50 == "2 ft real = 100 ft demo"); OFF (stored 0) makes
     * the demo image behave exactly like flight firmware. There is no dedicated
     * "demo" voice clip, so the level announces itself with a DOUBLE chirp — no
     * other level opens that way — followed by the current selection: a gain is
     * spoken with the existing number clips ("two hundred"), OFF with "off".      */
    {
        const float  dg_opts[] = { 50.0f, 100.0f, 200.0f, 400.0f, 0.0f };
        const size_t n_dg      = sizeof(dg_opts) / sizeof(dg_opts[0]);

        /* Start on whichever entry matches the compile-time default (falls back
         * to the first entry if the default is ever tuned off-list).             */
        size_t dg_sel = 0;
        for (size_t i = 0; i < n_dg; ++i) {
            if (fabsf(dg_opts[i] - DEMO_GAIN_DEFAULT) < 0.5f) {
                dg_sel = i;
                break;
            }
        }

        /* Level signature: double chirp, then speak the starting selection. */
        audio_play_clip_blocking(config_clip_chirp());
        vTaskDelay(pdMS_TO_TICKS(120));
        audio_play_clip_blocking(config_clip_chirp());
        vTaskDelay(pdMS_TO_TICKS(120));
        audio_play_clip_blocking(callout_clip(callout_id_for_ft(dg_opts[dg_sel])));

        idle_ms = 0;
        for (;;) {
            tap_event_t ev = wait_tap_event(&idle_ms);
            if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
                break;                          /* confirm this gain (or OFF)     */
            }
            if (ev == TAP_SINGLE) {
                dg_sel = (dg_sel + 1) % n_dg;   /* cycle gains, then OFF, wrap    */
                if (dg_opts[dg_sel] <= 0.0f) {
                    ESP_LOGI(TAG, "config: demo gain -> OFF");
                    audio_play_clip_blocking(config_clip_piece(CFG_PIECE_OFF));
                } else {
                    ESP_LOGI(TAG, "config: demo gain -> x%.0f", (double)dg_opts[dg_sel]);
                    audio_play_clip_blocking(callout_clip(callout_id_for_ft(dg_opts[dg_sel])));
                }
            }
        }
        audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
        config_save_demo_gain(dg_opts[dg_sel]);
    }
#endif /* DEMO_MODE */

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

    /*  Per-sample decision pacing. The state machine now steps ONCE PER FRESH
     *  SENSOR SAMPLE with the dt that actually elapsed between samples, instead
     *  of every 20 ms with a sample-and-hold input. The old scheme attributed a
     *  whole poll's range change to a single 20 ms tick, inflating the
     *  instantaneous vertical rate up to 37x — which made the trend dead-band
     *  meaningless and (before its own fix) let centimetre jitter reset the
     *  taxi-back disarm timer forever. SAMPLE_STALE_MS forces a decision on the
     *  held value if the sensor goes silent, so dwell timers keep accruing.      */
    uint32_t last_seq     = 0;
    bool     have_seq     = false;
    float    pending_dt_s = 0.0f;
    uint32_t last_tick_ms = 20;

    /*  Fail-safe no-data tracking (see LOST_SIGNAL_MUTE_MS in config.h).
     *  last_good_us is the wall clock of the last FRESH sample the range filter
     *  marked VALID. The filter deliberately republishes its held EMA through a
     *  lost-signal stretch (seq keeps advancing, s.valid == false), which keeps
     *  momentary dropouts inaudible — but the presence tone encodes ALTITUDE as
     *  PITCH, so a hold sustained past the bound becomes an actively misleading
     *  "altitude steady" cue over water / a dark wet runway while the aircraft
     *  may still be descending. Once the data's age exceeds the bound the tone
     *  is force-muted (silence == "no data", an honest cue); the first live
     *  sample restores it instantly. tone_muted edge-detects for one-shot logs. */
    int64_t last_good_us = esp_timer_get_time();
    bool    tone_muted   = false;

    /*  Subscribe to the task watchdog. This loop owns every audible decision the
     *  box makes; if it wedges, the box is silent with no reset and no cue. It
     *  blocks only on bounded waits (a 100 ms queue peek and its own tick delay),
     *  so a healthy pass always reaches the feed below well inside the timeout. */
    esp_task_wdt_add(NULL);

    for (;;) {
        /* Feed FIRST: every path below can continue the loop, and a missed feed
         * on any of them would be a false trip rather than a real hang.        */
        esp_task_wdt_reset();

        /* Peek the freshest range sample (non-destructive). */
        range_sample_t s;
        if (xQueuePeek(q_range_latest, &s, pdMS_TO_TICKS(100)) != pdTRUE) {
            /* No sample means the sensor task has published NOTHING since boot:
             * the LiDAR is silent (dead, miswired, or configured for the wrong
             * serial mode). Surface that unconditionally (throttled to ~1 s) —
             * this used to be bench-gated, which left a flight unit that failed
             * silent audibly indistinguishable from a healthy parked one. The
             * boot path has already chirped the pilot for this case (see the
             * ground-fill check in app_main); here we keep the console honest.  */
            static int64_t s_silent_log_us = 0;
            int64_t t = esp_timer_get_time();
            if (t - s_silent_log_us >= 1000 * 1000) {   /* ~1 s */
                s_silent_log_us = t;
                ESP_LOGW(TAG, "no sensor data yet (LiDAR silent / miswired?)");
            }
            continue;   /* no sample yet */
        }

        /* Wall-clock since the last loop pass, accumulated toward the next
         * decision (multiple passes can elapse between fresh samples).        */
        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - last_us) / 1e6f;
        last_us = now_us;
        if (dt_s <= 0.0f) {
            dt_s = 0.001f;
        }
        pending_dt_s += dt_s;

        /* Decide only on a FRESH sample (new seq), or when the stale watchdog
         * says the sensor has gone quiet and the dwell timers must advance on
         * the held value anyway.                                              */
        bool fresh      = !have_seq || (s.seq != last_seq);
        bool stale_kick = pending_dt_s * 1000.0f >= (float)SAMPLE_STALE_MS;
        if (!fresh && !stale_kick) {
            vTaskDelay(pdMS_TO_TICKS(last_tick_ms));
            continue;
        }
        float dt_dec = pending_dt_s;   /* true elapsed time for this decision  */
        pending_dt_s = 0.0f;
        last_seq     = s.seq;
        have_seq     = true;

        /* Fail-safe data-age clock: refreshed ONLY by a fresh sample the filter
         * accepted as valid. A held/lost-signal republish (s.valid == false) and
         * the stale-watchdog tick both let it age toward the tone mute below.   */
        if (fresh && s.valid) {
            last_good_us = now_us;
        }
        bool data_stale = (now_us - last_good_us) >
                          (int64_t)LOST_SIGNAL_MUTE_MS * 1000;

        /* Apply the learned ground reference: AGL = range - ground. On a
         * lost-signal sample, hold by feeding the previous AGL (sm clamps). */
        float agl = s.range_ft - s_ground_ref_ft;
        if (agl < 0.0f) {
            agl = 0.0f;
        }

#if DEMO_MODE
        /* Demo parked watchdog — evaluated on REAL feet, BEFORE the scaling.
         * The sm's own ground-dwell disarm tests "parked" on the scaled AGL,
         * where its 2 ft stillness band is millimetres of real travel — below
         * the sensor's 1 cm quantization, so it can never hold and the box
         * would stay armed (tone blipping on noise) forever between demo runs.
         * Instead: once the target has rested within DEMO_PARKED_BAND_FT of the
         * learned ground for the same GROUND_RESET_MS as flight, re-seed the
         * state machine into a fresh, disarmed GROUND — silence until the next
         * hoist climbs back through ARM_FT and re-arms it naturally.             */
        if (s_demo_scale) {
            /* Dwell accounting + a running mean of the parked readings. All five
             * statics are private to the logic task (its sole writer).           */
            static float    s_demo_parked_ms = 0.0f;
            static float    s_park_sum_ft    = 0.0f;
            static uint32_t s_park_n         = 0u;
            static float    s_park_far_ft    = 0.0f;   /* farthest range this rest */
            static bool     s_park_latched   = false;  /* one-shot per rest        */
            if (agl <= DEMO_PARKED_BAND_FT) {
                s_demo_parked_ms += dt_dec * 1000.0f;
                if (!s_park_latched) {
                    /* Farthest-cluster the mean, mirroring the boot fill guard: a
                     * beam intrusion (a hand under the rig) can only read NEARER
                     * than the rest surface, yet its clamped agl of 0 still tests
                     * as parked — so unguarded it would be averaged in and drag
                     * the re-anchored ground down, possibly past the parked band
                     * (stranding the box armed until a power cycle). Track the
                     * farthest reading of this rest and average ONLY the samples
                     * clustered at it; a genuinely farther cluster (the hand
                     * leaves, or the rig itself settled lower) restarts the mean. */
                    if (s.range_ft > s_park_far_ft + DEMO_GROUND_CLUSTER_FT) {
                        s_park_sum_ft = 0.0f;       /* farther cluster found:      */
                        s_park_n      = 0u;         /* restart the mean on it      */
                    }
                    if (s.range_ft > s_park_far_ft) {
                        s_park_far_ft = s.range_ft;
                    }
                    if (s.range_ft >= s_park_far_ft - DEMO_GROUND_CLUSTER_FT) {
                        s_park_sum_ft += s.range_ft;
                        s_park_n      += 1u;
                    }
                    if (s_demo_parked_ms >= (float)GROUND_RESET_MS) {
                        s_park_latched = true;
                        /* RE-ANCHOR the ground to this rest's cluster mean, so
                         * gradual DOWNWARD/in-band rig drift (rope stretch, a
                         * settling frame) is absorbed at every rest instead of
                         * accumulating past the parked band — which would strand
                         * the box armed all day. (An upward rest shift beyond the
                         * band can't be seen from inside it; a power cycle's boot
                         * self-calibration covers that case.) The minimum-cluster
                         * gate keeps a transitional sliver of samples — e.g. an
                         * intrusion ending right at the 30 s boundary — from
                         * moving the ground; the previous reference is kept.      */
                        if (s_park_n >= DEMO_REANCHOR_MIN_N) {
                            s_ground_ref_ft = s_park_sum_ft / (float)s_park_n;
                        }
                        if (sm.armed) {
                            sm_init(&sm, ST_GROUND);
                            sm.tone_start_ft = s_tone_start_ft; /* pilot override */
                            ESP_LOGI(TAG, "demo: parked %us -> disarmed + ground "
                                          "re-anchored at %.2f ft (hoist to re-arm)",
                                     (unsigned)(GROUND_RESET_MS / 1000u),
                                     (double)s_ground_ref_ft);
                        }
                    }
                }
            } else {
                s_demo_parked_ms = 0.0f;            /* any lift resets the dwell   */
                s_park_sum_ft    = 0.0f;
                s_park_n         = 0u;
                s_park_far_ft    = 0.0f;
                s_park_latched   = false;
            }
        }
#endif

        /* Bench real-sensor debug: a close bench target only swings a few feet,
         * so multiply the (real-feet) AGL up to flight altitudes — one foot above
         * the learned ground becomes s_bench_scale_gain feet, so a few feet of hand
         * travel sweeps the whole 0..400 ft callout band. The raw sensor reading
         * is logged (throttled) so the dev can confirm the LiDAR is streaming.
         * Armed by an OP_BENCH_REAL boot or, in DEMO_MODE builds, by the saved
         * NVS demo gain (see app_main); a flight build never enters here.        */
        if (s_bench_scale) {
            float scaled = agl * s_bench_scale_gain;
            /* Fast, compact value line — the host altitude tape parses THIS, so it
             * tracks smoothly without drowning the human log in the full breakdown.*/
            static int64_t s_tape_log_us = 0;    /* logic task is single-threaded  */
            if (now_us - s_tape_log_us >= (int64_t)BENCH_TAPE_LOG_MS * 1000) {
                s_tape_log_us = now_us;
                ESP_LOGI(TAG, "bench: -> %.0f ft", (double)scaled);
            }
            /* Slower, verbose breakdown for eyeballing raw/ground/agl + liveness. */
            static int64_t s_dbg_log_us = 0;
            if (now_us - s_dbg_log_us >= (int64_t)BENCH_DEBUG_LOG_MS * 1000) {
                s_dbg_log_us = now_us;
                ESP_LOGI(TAG, "bench: raw=%.2f ft  ground=%.2f ft  agl=%.2f ft -> %.0f ft%s  seq=%u",
                         (double)s.range_ft, (double)s_ground_ref_ft, (double)agl,
                         (double)scaled, s.valid ? "" : " (HOLD)", (unsigned)s.seq);
            }
            agl = scaled;
        }

        /*  Broken track: the filter re-acquired onto a level that is
         *  DISCONTINUOUS with the one it was following (a genuine terrain step,
         *  an in-flight power-up, or recovery from a stretch of untrustworthy
         *  data). The new altitude is trustworthy, but the aircraft did not fly
         *  through the heights in between — so re-anchor the state machine on it
         *  rather than letting the gap read as a descent and speak every rung it
         *  spans. That phantom is the worst thing this box can say: an out-of-
         *  range stretch that settled on a low erroneous cluster once snapped
         *  335 ft -> 12 ft in one poll and called "twenty" at altitude.          */
        if (fresh && s.track_break) {
            sm_reanchor(&sm, agl);
            ESP_LOGW(TAG, "range track BROKEN -> re-anchored at %.1f ft AGL "
                          "(gap is not flown motion; no rungs spoken across it)",
                     (double)agl);
        }

        sm_out_t out;
        sm_step(&sm, agl, dt_dec, g_profile, &out);

        /* --- Light-sleep policy: decided FIRST, before anything is queued ------
         * Allowed only in GROUND/CRUISE, where the tone is silent. The RESUME
         * edge must be published BEFORE any callout is queued below: while the
         * channel is suspended the audio task drain-and-DISCARDS the callout
         * queue (nothing may arm while the channel is down), so a callout queued
         * ahead of the resume can be destroyed by an audio-task poll landing in
         * the gap. One sm_step tick can BOTH leave CRUISE and cross a rung — a
         * steep descent or a re-acquire snap does exactly that — and because a
         * fired rung is one-shot disarmed it could never speak again for the
         * rest of the approach. Clearing the suspend request first closes that
         * window: by the time anything is queued the audio task is either
         * already running or will observe the cleared request before it drains.
         *
         * The SUSPEND edge stays after the queueing (see below), so a callout
         * fired on the tick we ENTER a sleep state still gets queued to a live
         * channel rather than into a channel we just tore down.                */
        bool sleep_allowed = (out.state == ST_GROUND || out.state == ST_CRUISE);

        /* Compile-time master switch (config.h). Defaulted OFF: on a panel-powered
         * installation the power saving buys little, while the suspend/poll-relax
         * it implies has sat in the causal chain of every flight-critical silence
         * this box has shipped. Build -DSLEEP_MODE_ENABLE=1 to restore it.       */
#if !SLEEP_MODE_ENABLE
        sleep_allowed = false;
#endif

        /* Never light-sleep on the bench: sleeping gates the UART/poll cadence and
         * makes the live altitude stutter. Stay awake + fast for responsive testing.*/
        if (s_bench_scale) {
            sleep_allowed = false;
        }

        /* Never light-sleep on data we KNOW is stale. Sleeping relaxes the poll to
         * 500 ms and tears the audio channel down, so entering it on a held value
         * costs exactly the latency we most need on a descent — and a held value
         * is precisely when we are least entitled to assume nothing is happening. */
        if (data_stale) {
            sleep_allowed = false;
        }
        if (!sleep_allowed && sleep_allowed_prev) {
            audio_resume();
        }

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
            /* The gear reminder pairs with ANY rung crossed this tick, not just
             * the one (lowest) number spoken: a terrain drop or a re-acquire
             * snap can cross several thresholds in one step, and the state
             * machine deliberately speaks only the lowest — but a SKIPPED
             * gear-check rung must still deliver its safety call, because its
             * armed bit is already cleared and it cannot recover during the
             * same approach. crossed_mask reports the full crossed set.         */
            bool is_gear_check = false;
            if (s_gear_check_ft > 0.0f) {
                for (size_t i = 0; i < g_profile->n_callouts; ++i) {
                    if ((out.crossed_mask & (1u << i)) != 0u &&
                        fabsf(g_profile->callouts[i] - s_gear_check_ft) < 0.5f) {
                        is_gear_check = true;
                        break;
                    }
                }
            }
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
         * audio engine's vario blip can chop the tone by descent rate.
         *
         * Fail-safe overlay: when the sensor data has gone STALE (a sustained
         * lost-signal hold or a silent sensor, older than LOST_SIGNAL_MUTE_MS)
         * the tone is force-muted regardless of what the state machine wants —
         * a pitch frozen on held data reads as "altitude steady" and is an
         * actively WRONG cue; silence is an honest one. Callouts are unaffected:
         * they only edge-fire on genuinely accepted crossings, and the filter's
         * re-acquire already speaks the nearest number when live data returns.  */
        bool tone_on = out.tone_active && !data_stale;
        if (data_stale != tone_muted) {
            tone_muted = data_stale;
            if (data_stale) {
                ESP_LOGW(TAG, "sensor data stale >%u ms (lost signal / silent "
                              "sensor) -> tone muted", (unsigned)LOST_SIGNAL_MUTE_MS);
                /* Annunciate the failure. Muting the tone is the honest cue only
                 * when the tone was actually sounding — i.e. below the tone band.
                 * ABOVE it (cruise, pattern work, anywhere the ladder has not
                 * started) the tone was already silent, so the mute is audibly a
                 * NO-OP and a dead LiDAR is indistinguishable from a healthy box
                 * with nothing to say. That is the failed-silent trap the boot
                 * chirp exists to close on the ground, and it applies just as
                 * much in the air: a pilot trained on "500 ... 100 ... 50" would
                 * otherwise discover the failure on short final, or read the
                 * silence as "not yet at the first callout". Chirp instead.     */
                if (!out.tone_active) {
                    audio_request_sensor_alert();
                }
            } else {
                ESP_LOGI(TAG, "sensor data live again -> tone restored");
            }
        }
        audio_set_params(out.tone_agl, tone_on, out.vert_fps);
        g_poll_period_ms = poll_profile_to_ms(out.poll);

        /* Bench real-sensor debug: keep the sensor draining briskly. GROUND/CRUISE
         * normally relax the poll to ~750 ms, which makes the live readout look
         * frozen between updates — cap it so the tape tracks the LiDAR in real time.*/
        if (s_bench_scale && g_poll_period_ms > SIM_POLL_MS) {
            g_poll_period_ms = SIM_POLL_MS;
        }

        /* Light-sleep policy, SUSPEND edge (the resume edge ran before the
         * callout block above). Tearing the channel down last means a callout
         * fired on this very tick was already queued to a live channel and gets
         * spoken; the suspend then takes effect on a later tick once the state
         * has genuinely settled into GROUND/CRUISE.                            */
        if (sleep_allowed && !sleep_allowed_prev) {
            audio_suspend();
        }
        sleep_allowed_prev = sleep_allowed;

        /* Decision cadence: fast in active states, relaxed when sleep-friendly
         * (the PM subsystem drops to light-sleep during the idle between ticks). */
        uint32_t tick_ms = sleep_allowed ? POLL_MS_CRUISE : 20;
        last_tick_ms = tick_ms;   /* the stale-sample skip path paces off this  */
        vTaskDelay(pdMS_TO_TICKS(tick_ms));
    }
}

/* ---------------------------------------------------------------------------
 *  Enable automatic light-sleep (needs CONFIG_PM_ENABLE + tickless idle).
 * ------------------------------------------------------------------------- */
static void enable_power_management(void)
{
    /*  SLEEP_MODE_ENABLE (config.h) is the single master switch. With it clear we
     *  still configure PM — dynamic frequency scaling is free and the audio task's
     *  CPU_FREQ_MAX lock depends on the PM subsystem being live — but we never let
     *  the SoC actually light-sleep.                                              */
    esp_pm_config_t pm = {
        .max_freq_mhz       = PM_MAX_FREQ_MHZ,
        .min_freq_mhz       = PM_MIN_FREQ_MHZ,
        .light_sleep_enable = (SLEEP_MODE_ENABLE != 0),
    };
    esp_err_t err = esp_pm_configure(&pm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_pm_configure failed (%s); running without light-sleep",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "power management: %d/%d MHz, light-sleep %s",
                 PM_MAX_FREQ_MHZ, PM_MIN_FREQ_MHZ,
                 SLEEP_MODE_ENABLE ? "on" : "OFF (SLEEP_MODE_ENABLE=0)");
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
                    /* The host may append a little-endian float (the desired AGL
                     * scale) right after the opcode. The S3 is little-endian like
                     * the host, so a straight copy decodes it. Sanity-clamp before
                     * accepting so a garbled frame can't produce an absurd sweep.  */
                    if (f.plen >= 1 + sizeof(float)) {
                        float g;
                        memcpy(&g, &f.payload[1], sizeof g);
                        if (g >= 1.0f && g <= 1000.0f) {
                            s_bench_scale_gain = g;
                        }
                    }
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
    ESP_LOGI(TAG, "LidarAGL boot — firmware %s", FIRMWARE_VERSION);

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
             * logic task applies. We deliberately LEAVE the USB-Serial-JTAG driver
             * installed (exactly as sim mode does): uninstalling it forces a USB
             * re-enumeration, which the host sees as a disconnect and reopens —
             * re-asserting RTS and resetting the chip straight back OUT of this
             * mode. ESP_LOG keeps using its direct-FIFO path either way, so the
             * console + this banner are unaffected by leaving the driver up.       */
            s_bench_scale = true;
            sf30c_enable_raw_debug();   /* dump the raw sensor bytes for diagnosis */
            ESP_LOGW(TAG, "BENCH REAL-SENSOR MODE: real LiDAR on UART, AGL scaled "
                          "x%.0f for bench callout testing", (double)s_bench_scale_gain);
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

    /*  Verify every rung of the active ladder actually has a clip behind it.
     *  The embedded clip set is GLOBBED at build time (see main/CMakeLists.txt),
     *  so a renamed, moved, or failed-to-convert .pcm does not break the build —
     *  it silently produces a {NULL,0} manifest entry. In the air that rung then
     *  passes through the whole pipeline normally and simply makes no sound: a
     *  missing "fifty" on short final looks identical to a working box that has
     *  not reached 50 ft yet. Check it once at boot, where it is cheap and where
     *  the console can name the exact clip, and chirp so the failure is audible
     *  rather than buried in a log nobody reads at the hangar.                  */
    size_t missing_clips = 0;
    for (size_t i = 0; i < g_profile->n_callouts; ++i) {
        float        ft  = g_profile->callouts[i];
        callout_id_t cid = callout_id_for_ft(ft);
        const clip_t *cl = (cid != CO_COUNT) ? callout_clip(cid) : NULL;
        if (!cl || !cl->pcm || cl->len_bytes < 2) {
            missing_clips++;
            ESP_LOGE(TAG, "MISSING CLIP for %.0f ft callout (%s) — that rung "
                          "will be SILENT in flight", (double)ft,
                     (cid != CO_COUNT) ? callout_clip(cid)->name : "unmapped");
        }
    }
    /* The gear reminder is a safety call in its own right; check it too. */
    const clip_t *gear_clip = callout_clip(CO_CHECK_GEAR);
    if (!gear_clip->pcm || gear_clip->len_bytes < 2) {
        missing_clips++;
        ESP_LOGE(TAG, "MISSING CLIP: 'check gear' reminder will be SILENT");
    }
    if (missing_clips > 0) {
        ESP_LOGE(TAG, "%u callout clip(s) missing from this build",
                 (unsigned)missing_clips);
        audio_play_chirp();
    }

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

#if DEMO_MODE
    /* 4b. DEMO build: arm the scaled-AGL show mode from the saved demo gain.
     * This is the standalone show-floor cousin of the USB bench real-sensor mode:
     * it reuses the exact same s_bench_scale machinery the logic task already
     * applies (AGL x gain, capped poll cadence, no light-sleep, liveness logs) —
     * just armed from NVS on every boot instead of a bench attach frame. The
     * scaling multiplies AGL AFTER the ground reference is subtracted, so the
     * ground-fill and the airborne decision keep working in honest feet.
     *
     * Two bench workflows must keep working on a demo image, so the demo gain
     * yields to BOTH: an OP_BENCH_REAL attach already armed the scaling with the
     * gain the tool asked for (it wins for that session), and a plain sim attach
     * (OP_HELLO) streams fabricated altitudes that are ALREADY flight-scale feet
     * — stacking the demo gain on top would multiply a simulated 300 ft approach
     * to 60,000 ft, so sim boots must never arm it. Gain 0 means OFF was chosen
     * in LEVEL 9: the demo image flies like a stock build.
     *
     * When the gain DOES arm, the box announces it: the LEVEL 9 double-chirp
     * signature followed by the spoken gain ("two hundred"). That makes a demo
     * image unmistakable at power-on — a flight box boots silent, so a unit
     * accidentally flashed with demo firmware reveals itself before it ever
     * reaches an aircraft, and the show crew hears "demo ready" at the booth.    */
    if (!s_bench_scale && !sf30c_sim_active()) {
        float demo_gain = config_load_demo_gain();
        if (demo_gain >= 2.0f) {
            s_bench_scale      = true;
            s_bench_scale_gain = demo_gain;
            s_demo_scale       = true;
            ESP_LOGW(TAG, "DEMO MODE: real LiDAR, AGL scaled x%.0f "
                          "(2 ft real = %.0f ft demo); retune in config-menu LEVEL 9",
                     (double)demo_gain, (double)(2.0f * demo_gain));
            /* Audible demo signature (blocking, so it finishes before tasks run). */
            audio_play_clip_blocking(config_clip_chirp());
            vTaskDelay(pdMS_TO_TICKS(120));
            audio_play_clip_blocking(config_clip_chirp());
            vTaskDelay(pdMS_TO_TICKS(120));
            audio_play_clip_blocking(callout_clip(callout_id_for_ft(demo_gain)));
        } else {
            ESP_LOGI(TAG, "DEMO build: demo gain OFF — behaving like flight firmware");
        }
    }
#endif

    /* 5. Ground-fill + current reading -> reconstruct ground reference. */
    float ground_reads[BOOT_BUFFER_N];
    size_t n_fill = capture_ground_fill(ground_reads, BOOT_BUFFER_N);

    boot_entry_t stored[BOOT_BUFFER_N];
    size_t n_stored = boot_buffer_load(stored, BOOT_BUFFER_N);

#if DEMO_MODE
    /* Demo boots SELF-CALIBRATE: wherever the pulley target rests at power-on IS
     * the show's ground. Feed THIS boot's fresh fill through the same robust
     * resolve in place of the persisted buffer, so no earlier session (a home
     * bench, a power-off with the target hoisted) can hand the show a stale
     * reference — and a boot that DOES catch the target hoisted is fully cured
     * by a plain power cycle at rest, because nothing is ever persisted (the
     * commit below is skipped too). Flight/OFF boots use the stored buffer.
     *
     * The fill is first reduced to its FARTHEST cluster (see the
     * DEMO_GROUND_CLUSTER_FT rationale in config.h): on the rig the samples are
     * flat enough that robust_mean's MAD hits 0 and stops rejecting outliers,
     * and a hand crossing the beam during the fill can only read NEARER than
     * the true surface — so keeping the far cluster discards the intrusion.     */
    if (s_demo_scale) {
        /* Farthest plausible (junk-capped) reading in the fill. */
        float floor_ft = -1.0f;
        for (size_t i = 0; i < n_fill; ++i) {
            if (ground_reads[i] <= MAX_VALID_FT && ground_reads[i] > floor_ft) {
                floor_ft = ground_reads[i];
            }
        }
        /* Keep only the samples clustered at that farthest surface. */
        size_t kept = 0;
        for (size_t i = 0; i < n_fill; ++i) {
            if (ground_reads[i] <= MAX_VALID_FT &&
                ground_reads[i] >= floor_ft - DEMO_GROUND_CLUSTER_FT) {
                stored[kept].range_ft = ground_reads[i];
                stored[kept].marker   = 0;   /* fresh, never-persisted entries   */
                ++kept;
            }
        }
        n_stored = kept;   /* 0 == all junk -> resolve falls back as usual       */
    }
#endif

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

    /* Sanity-blend the boot reading through the robust estimator (median/MAD
     * over the fresh fill + the current reading). The airborne/armed seeding
     * below trusts current_ft, and a single corrupt sample here used to be
     * able to seed ST_ARMED on a parked aircraft: with the fill in the vote, a
     * lone flier is rejected outright, while a genuine in-flight reboot — where
     * the fill itself was captured airborne — passes straight through.          */
    if (n_fill > 0) {
        bool est_ok = false;
        float est = robust_estimate(ground_reads, n_fill, current_ft, &est_ok);
        if (est_ok) {
            current_ft = est;
        }
    }

    /* Whether current_ft is REAL sensor data (a live read, or the ground-fill
     * fallback above) rather than the 0.0f initialiser of a totally silent
     * sensor. resolve() must know the difference: a fabricated 0.0 would pass
     * its ground-plausibility band and be adopted as a perfect ground with the
     * calibration warning suppressed — in exactly the dead-sensor case that
     * warning exists to catch.                                                 */
    bool have_current = cur_valid || (n_fill > 0);

    boot_result_t br;
    boot_buffer_resolve(stored, n_stored, current_ft, have_current, &br);
    s_ground_ref_ft = br.ground_ref_ft;

    ESP_LOGI(TAG, "ground_ref=%.2f ft  boot_agl=%.1f ft  airborne=%d calib_err=%d",
             br.ground_ref_ft, br.boot_agl_ft, br.airborne, br.calib_error);

    /* Sensor-health annunciation. If the ENTIRE ground fill came up empty AND
     * the latest read was invalid, the LiDAR never spoke this boot — dead unit,
     * loose connector, or a sensor still configured for the wrong serial mode.
     * Without this chirp a dead-sensor boot is audibly IDENTICAL to a healthy
     * parked one (a normal flight boot plays no audio at all): the classic
     * failed-silent trap for a landing aid whose pilot is trained to expect
     * callouts. The chirp says "not healthy"; the console says why. When there
     * is also no stored reference, the calib_error path below additionally
     * speaks its instruction — that is fine, more signal for a sick boot.       */
    if (n_fill == 0 && !cur_valid) {
        ESP_LOGW(TAG, "SENSOR SILENT at boot: no LiDAR data during the ground "
                      "fill (dead/miswired sensor, or wrong serial config?)");
        audio_play_chirp();
    }

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
    /*  calib_error additionally vetoes the write: a boot that had to fall back
     *  to the emergency offset has, by definition, no trustworthy notion of
     *  where the ground is, so nothing it captured belongs in NVS.              */
    bool persist_ground = !br.airborne && n_fill > 0 && !br.calib_error;
#if DEMO_MODE
    /* Demo boots never write the ground buffer (runtime-only, like sim mode):
     * the rig's 2 ft of travel sits inside GROUND_DEV_FT, so a reboot with the
     * target hoisted would otherwise be misread as "on ground" and persist a
     * poisoned reference that silently kills every later demo run.              */
    if (s_demo_scale) {
        persist_ground = false;
    }
#endif
    if (persist_ground) {
        boot_buffer_commit(ground_reads, n_fill, (uint32_t)(esp_timer_get_time() / 1000));

        /* Fold TODAY's surface into the operative reference. The resolve above
         * builds the ground ref from the PREVIOUS boot's stored fill only, so a
         * modest surface change (grass tie-down -> pavement, strut compression)
         * used to offset the whole session's AGL by a few feet — enough to hold
         * idle taxi AGL above the parked band and block the 30 s taxi-back
         * disarm entirely. When this boot is judged on-ground and the fresh
         * fill's robust mean agrees with the stored reference within
         * GROUND_DEV_FT, the fresh value IS the ground we are sitting on — use
         * it. (persist_ground is never set on demo boots, which self-calibrate
         * through their own cluster path above.)                                */
        {
            bool fresh_ok = false;
            float fresh_ref = robust_mean(ground_reads, n_fill, &fresh_ok);
            if (fresh_ok) {
                /* The commit above just made THIS fill the persisted truth, so
                 * flying the session on anything else is indefensible. The old
                 * agree-within-GROUND_DEV_FT gate here kept a poisoned-HIGH
                 * stored reference operative for the entire flight that had
                 * already CORRECTED it in NVS — every AGL cue tens of feet low
                 * ("ten" spoken at a true 40 ft, flare fade above the runway),
                 * healed only by a power cycle. We are on the ground (the
                 * airborne gate above) holding a robust fresh fill: use it, and
                 * just log loudly when it contradicts the old reference.        */
                if (fabsf(fresh_ref - br.ground_ref_ft) > GROUND_DEV_FT) {
                    ESP_LOGW(TAG, "stored ground ref %.2f ft contradicts this "
                                  "boot's on-ground fill %.2f ft; trusting the fill",
                             (double)br.ground_ref_ft, (double)fresh_ref);
                }
                s_ground_ref_ft = fresh_ref;
                ESP_LOGI(TAG, "ground ref refreshed from this boot's fill: %.2f ft",
                         (double)fresh_ref);
            }
        }
    }

    /* 8. Seed the state machine from the reconstructed AGL. */
    static sm_state_t initial_state;
    initial_state = sm_initial_state(br.boot_agl_ft, !br.calib_error || br.airborne,
                                     g_profile);

    /* An in-flight reboot BELOW ARM_FT — a brownout on short final, anywhere in
     * the 10..100 ft band — used to fall through sm_initial_state's
     * boot_agl <= ARM_FT test into a DISARMED ST_GROUND, silencing every
     * remaining callout, the tone and the flare fade for exactly the landing
     * this box exists to call. The resolver has already computed a trustworthy
     * "we are airborne" verdict (current sits more than GROUND_DEV_FT above the
     * learned ground); thread it into the seeding so ANY airborne boot starts
     * ARMED. sm_init() arms the full ladder for ARMED seeds, and callouts still
     * edge-trigger only on a genuine downward crossing, so this can never blurt
     * numbers on the first tick.                                               */
    if (br.airborne && !br.calib_error && initial_state == ST_GROUND) {
        initial_state = ST_ARMED;
        ESP_LOGI(TAG, "airborne at boot (%.1f ft) -> seeding ARMED, ladder hot",
                 (double)br.boot_agl_ft);
    }
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
