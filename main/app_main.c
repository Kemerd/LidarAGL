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

/*  Bench HIL: set during the boot attach probe when the connected bench tool
 *  asked to open the config menu this boot. Runtime-only (never persisted).      */
static volatile bool s_sim_want_config = false;

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
 *    4. LEVEL 3 — master volume offset. Speak "Volume Adjustment", then a TAP
 *       cycles 0 dB down to VOLUME_OFFSET_DB_MIN in VOLUME_OFFSET_DB_STEP steps.
 *       Each step previews the chosen level by ear as "tone .. <number> .. tone"
 *       (a 1 kHz burst, one spoken number, the burst again) AT the selected
 *       offset, so the pilot trims tone+voice together against the cockpit. The
 *       offset is applied live so every prompt afterward reflects it; DOUBLE-TAP
 *       / timeout confirms. This level always runs (it trims the tone too).
 *    5. A chirp marks each confirm. Persist all three values and reboot.
 *
 *  This function does not return — it always ends in esp_restart().
 * ------------------------------------------------------------------------- */
#define CONFIG_COMMIT_MS    5000   /* idle time with no tap that auto-confirms   */
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

/*  Preview the currently-selected master volume at the level it will actually
 *  run: "tone .. <number> .. tone". The 1 kHz bursts and the spoken number all
 *  pass through the same master offset (already applied via audio_set_master_db),
 *  so the pilot judges tone-vs-voice balance and absolute loudness in one listen.
 *  The number clip is fixed (CO_THIRTY) just to give the voice something to say;
 *  its value carries no meaning here — only its loudness against the tone does.   */
static void preview_volume(void)
{
    audio_play_tone_blocking(VOLUME_PREVIEW_HZ, VOLUME_PREVIEW_MS, VOLUME_PREVIEW_DB);
    vTaskDelay(pdMS_TO_TICKS(90));
    audio_play_clip_blocking(callout_clip(CO_THIRTY));
    vTaskDelay(pdMS_TO_TICKS(90));
    audio_play_tone_blocking(VOLUME_PREVIEW_HZ, VOLUME_PREVIEW_MS, VOLUME_PREVIEW_DB);
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
    config_wipe_volume_offset();

    /* Start the menu at 0 dB (no cut) so the prompts/previews play un-attenuated
     * until the pilot chooses a level in LEVEL 3 below.                          */
    audio_set_master_db(0.0f);

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

    /* ---- LEVEL 3: master volume offset ------------------------------------ */
    /* A pilot-set master trim layered on the analog pot, cycling 0 dB down to
     * VOLUME_OFFSET_DB_MIN in VOLUME_OFFSET_DB_STEP steps. Always runs (it trims
     * the presence tone too, not only the voice). Each tap applies the offset
     * LIVE — so the very next preview, and every prompt after — plays at that
     * level — then previews it as "tone .. number .. tone".                     */
    {
        float vol_db = 0.0f;                /* start at no cut (the default)     */
        audio_set_master_db(vol_db);

        audio_play_clip_blocking(config_clip_piece(CFG_PIECE_VOLUME_ADJ));
        vTaskDelay(pdMS_TO_TICKS(120));
        preview_volume();                   /* preview the starting (0 dB) level */

        idle_ms = 0;
        for (;;) {
            tap_event_t ev = wait_tap_event(&idle_ms);
            if (ev == TAP_DOUBLE || ev == TAP_TIMEOUT) {
                break;                      /* confirm this volume               */
            }
            if (ev == TAP_SINGLE) {
                /* Step DOWN one increment; wrap back to 0 dB past the floor. */
                vol_db -= VOLUME_OFFSET_DB_STEP;
                if (vol_db < VOLUME_OFFSET_DB_MIN - 0.001f) {
                    vol_db = 0.0f;          /* wrap to no-cut */
                }
                ESP_LOGI(TAG, "config: volume offset -> %.0f dB", vol_db);
                audio_set_master_db(vol_db);   /* apply live for the preview */
                preview_volume();
            }
        }
        audio_play_clip_blocking(config_clip_chirp());   /* confirm chirp */
        config_save_volume_offset(vol_db);
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

        sm_out_t out;
        sm_step(&sm, agl, dt_s, g_profile, &out);

        /* Fire the callout, if any, mapping the profile height -> clip id.
         * Suppress any callout ABOVE the configured start-altitude cap so the
         * pilot only hears numbers from their chosen ceiling down (the tone is
         * unaffected). With the cap at the profile top this never suppresses.   */
        if (out.fired_callout >= 0) {
            float ft = g_profile->callouts[out.fired_callout];
            if (ft <= s_start_alt_ft) {
                callout_id_t cid = callout_id_for_ft(ft);
                if (cid != CO_COUNT) {
                    audio_request_callout(cid);
                }
                ESP_LOGI(TAG, "callout %.0f ft (state=%d)", ft, out.state);
            } else {
                ESP_LOGI(TAG, "callout %.0f ft suppressed (cap %.0f ft)",
                         ft, s_start_alt_ft);
            }
        }

        /* Publish tone params + poll cadence. */
        audio_set_params(out.tone_agl, out.tone_active);
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
        sf30c_set_bench_ctrl_cb(bench_ctrl_handler);
        sf30c_sim_enable();
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

    /* Apply the saved master volume offset (0 dB == no cut). Done after the menu
     * branch so a freshly-committed value is the one in effect this boot.        */
    float vol_db = config_load_volume_offset();
    audio_set_master_db(vol_db);
    ESP_LOGI(TAG, "master volume offset: %.0f dB", vol_db);

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
