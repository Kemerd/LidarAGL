/**
 * @file    sf30c.c
 * @brief   SF30 UART driver, ASCII + LWNX parsers, autodetect, sensor task.
 *
 * @details HARDWARE module (ESP-IDF). The pure framing/CRC lives in lwnx.c; the
 *          pure profile selection lives in sensor_profile.c. This file is the
 *          thin layer that talks to the UART and turns bytes into feet.
 */

#include "sf30c.h"
#include "config.h"
#include "lwnx.h"
#include "range_filter.h"
#include "shared.h"

#include <string.h>
#include <math.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"   /* bench raw-byte dump throttle + filter dt           */
#include "esp_task_wdt.h" /* sensor_task subscribes to the TWDT                */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "sf30c";

/* ---- Module state -------------------------------------------------------- */

/*  The robust range pipeline (median-of-drain -> Hampel gate -> re-acquire ->
 *  time-corrected EMA; see range_filter.c). It replaces the old bare EMA +
 *  hold-last-good pair, both of which live inside it now.                      */
static range_filter_t   s_rf;

/*  Carried pairing state of the legacy 2-byte decoder (pure, range_filter.c).  */
static sf30_ascii_ctx_t s_ascii;

/*  UART event queue: the RX hardware flags framing/parity errors and FIFO
 *  overflows on the exact garbled bytes a DFS glitch or light-sleep wake edge
 *  produces — we listen so a corrupt drain can be DISCARDED instead of parsed. */
static QueueHandle_t    s_uart_evq = NULL;

/*  Wall-clock stamp of the previous rf_finalize(), for the filter's dt.        */
static int64_t          s_last_fin_us = 0;

/*  Monotonic sample sequence for the consumer's dt sanity. */
static uint32_t s_seq = 0;

/*  Persistent LWNX parser (binary mode). */
static lwnx_parser_t s_parser;

/*  Bench HIL simulation: when set, the read path drains the SIMULATED LWNX
 *  stream from the USB-Serial-JTAG instead of UART1. Runtime-only (never NVS).  */
static bool                  s_sim_mode = false;
static sf30c_bench_ctrl_cb_t s_bench_cb = NULL;

/*  Bench diagnostic: when set, dump the raw bytes drained from the REAL sensor
 *  UART (throttled) so we can see exactly what the SF30 is putting on the wire.  */
static bool                  s_raw_debug = false;

void sf30c_enable_raw_debug(void)
{
    s_raw_debug = true;
}

/* ---- UART setup ---------------------------------------------------------- */

void sf30c_init(void)
{
    /* The robust range pipeline is used by BOTH the real-sensor and sim paths;
     * seed its ceiling gate with the default profile until autodetect runs.    */
    rf_init(&s_rf, SF30C_PROFILE.max_range_ft);
    sf30_ascii_reset(&s_ascii);

    /* Bench sim mode: there is no real sensor on UART1 — the read path pulls
     * simulated frames from the USB-Serial-JTAG instead. Skip the UART bring-up
     * entirely (sf30c_sim_enable() already reset the parser).                   */
    if (s_sim_mode) {
        ESP_LOGI(TAG, "sim mode: skipping UART1 init (LiDAR not used)");
        return;
    }

    const uart_config_t cfg = {
        .baud_rate  = SF30C_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        /* XTAL, NOT the (default) APB clock. The APB clock is scaled 80->40 MHz
         * by DFS whenever no PM lock is held — which is exactly the on-ground
         * states, since the audio path releases its lock there — and a halved
         * source clock turns the whole 460800-baud stream into framing-error
         * garbage (the "CE-heavy ~328 ft" failure class this file has met
         * before). The 40 MHz XTAL is immune to DFS, so RX stays clean at any
         * CPU frequency. See the ESP-IDF power-management docs.                 */
        .source_clk = UART_SCLK_XTAL,
    };

    /* Install WITH an event queue: framing/parity/overflow errors are detected
     * by the RX hardware on the exact bytes a light-sleep wake edge corrupts,
     * and the read path uses those events to discard the poisoned drain.
     * Depth 64: the 78 Hz stream can post an RX-timeout UART_DATA event per
     * 2-byte burst (~58 per 750 ms GROUND poll), and a full queue silently
     * drops later events — including the error flags this sweep exists to see.
     * (Even a dropped error is not fatal: the median/Hampel layers downstream
     * still gate whatever the corrupt bytes decode to — defence in depth.)     */
    ESP_ERROR_CHECK(uart_driver_install(SF30C_UART_NUM,
                                        SF30C_UART_RX_BUF, SF30C_UART_TX_BUF,
                                        64, &s_uart_evq, 0));
    ESP_ERROR_CHECK(uart_param_config(SF30C_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(SF30C_UART_NUM,
                                 PIN_SF30C_TX, PIN_SF30C_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    lwnx_parser_reset(&s_parser);

    ESP_LOGI(TAG, "UART%d up @ %d 8N1 XTAL-clocked (RX=%d TX=%d), mode=%s",
             SF30C_UART_NUM, SF30C_BAUD, PIN_SF30C_RX, PIN_SF30C_TX,
             (SF30C_MODE == SF30C_BINARY) ? "BINARY" : "ASCII");
}

/* ===========================================================================
 *  Bench HIL simulation (USB-Serial-JTAG side-door)
 * ---------------------------------------------------------------------------
 *  sf30c_sim_enable() installs the USB-Serial-JTAG driver purely to READ the
 *  host's simulated stream; we deliberately do NOT route the console VFS through
 *  the driver, so ESP_LOG keeps using its default (direct-FIFO) path and logging
 *  is unchanged. The driver itself takes no power-management lock (the system
 *  connection-monitor owns the NO_LIGHT_SLEEP lock and follows USB connect /
 *  disconnect), so installing it never blocks in-flight light-sleep.
 * ===========================================================================*/

void sf30c_set_bench_ctrl_cb(sf30c_bench_ctrl_cb_t cb)
{
    s_bench_cb = cb;
}

bool sf30c_sim_active(void)
{
    return s_sim_mode;
}

/*  Hand a decoded BENCH_CTRL frame to the registered callback (opcode + args). */
static void sim_dispatch_frame(const lwnx_frame_t *f)
{
    if (f->cmd != LWNX_CMD_BENCH_CTRL || s_bench_cb == NULL) {
        return;
    }
    uint8_t        op   = (f->plen > 0) ? f->payload[0] : 0;
    const uint8_t *arg  = (f->plen > 1) ? &f->payload[1] : NULL;
    size_t         alen = (f->plen > 1) ? (f->plen - 1) : 0;
    s_bench_cb(op, arg, alen);
}

void sf30c_sim_enable(void)
{
    /* Install the RX driver once (idempotent — the boot probe may already have
     * installed it). We only read with it; console TX stays on its default path.*/
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t cfg = {
            .tx_buffer_size = SIM_USJ_TX_BUF,
            .rx_buffer_size = SIM_USJ_RX_BUF,
        };
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    }
    lwnx_parser_reset(&s_parser);
    /* Fresh filter state for the simulated stream (sim_enable can run before
     * sf30c_init on a bench boot, so it must be self-sufficient).              */
    rf_init(&s_rf, SF30C_PROFILE.max_range_ft);
    sf30_ascii_reset(&s_ascii);
    s_sim_mode = true;
    ESP_LOGW(TAG, "BENCH SIM MODE: reading the LWNX distance stream from USB-Serial-JTAG");
}

void sf30c_sim_read_drain(void)
{
    uint8_t rx[128];
    int n = usb_serial_jtag_read_bytes(rx, sizeof rx, 0);
    for (int i = 0; i < n; ++i) {
        lwnx_frame_t f;
        if (lwnx_feed(&s_parser, rx[i], &f)) {
            sim_dispatch_frame(&f);   /* distance frames are simply ignored here */
        }
    }
}

/* ---- UART hardware error sweep -------------------------------------------- */
/*  Drain the driver's event queue and report whether the hardware flagged any
 *  framing/parity error or FIFO/ring overflow since the last poll. Those flags
 *  fire on precisely the corruption a light-sleep wake edge (or any glitch)
 *  produces — bytes that LOOK like data but aren't. On any hit the caller
 *  throws the whole drain away rather than parsing poisoned bytes.             */
static bool uart_errors_pending(void)
{
    bool hit = false;
    if (s_uart_evq == NULL) {
        return false;
    }
    uart_event_t ev;
    while (xQueueReceive(s_uart_evq, &ev, 0) == pdTRUE) {
        switch (ev.type) {
            case UART_FRAME_ERR:
            case UART_PARITY_ERR:
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                hit = true;
                break;
            default:
                break;                     /* UART_DATA etc — not our business  */
        }
    }
    return hit;
}

/* ---- Drain + parse into the robust filter --------------------------------- */

bool sf30c_read_latest_ft(float *range_ft_out, bool *valid)
{
    /* Keep the filter's ceiling gate synced to the autodetected sensor (the
     * profile is resolved once at boot; this is a cheap idempotent refresh).    */
    if (g_profile != NULL) {
        rf_set_max_range(&s_rf, g_profile->max_range_ft);
    }

    /* Elapsed wall-clock since the previous finalize — drives the filter's
     * physical slew allowance and its time-corrected EMA bandwidth.             */
    int64_t now_us = esp_timer_get_time();
    float dt_s = (s_last_fin_us > 0)
                 ? (float)(now_us - s_last_fin_us) / 1e6f
                 : 0.02f;
    s_last_fin_us = now_us;

    /*  Bound the elapsed time before it reaches the filter. dt_s drives the
     *  Hampel slew allowance and the EMA bandwidth, and both misbehave on a
     *  degenerate value: a non-positive dt (a clock that did not advance, or a
     *  64-bit read torn across a wrap) makes alpha zero or negative, and an
     *  unboundedly large one is harmless to the capped gate but meaningless as
     *  a rate. Neither is data — they are timekeeping artefacts — so clamp to
     *  the range the poll cadences can actually produce. The generous ceiling
     *  still lets a genuinely long gap collapse the EMA to a passthrough, which
     *  is the correct behaviour: state that old deserves no weight.            */
    if (!(dt_s > 0.0f)) {
        dt_s = 0.001f;
    } else if (dt_s > SENSOR_MAX_DT_S) {
        dt_s = SENSOR_MAX_DT_S;
    }

    /* Hardware flagged corrupt bytes since the last poll (real sensor only):
     * flush everything, resync the pair decoder, and hold last-good — parsing
     * a drain the UART itself called garbage is how phantom callouts happen.    */
    if (!s_sim_mode && uart_errors_pending()) {
        static int64_t s_err_log_us = 0;
        if (now_us - s_err_log_us >= 5 * 1000 * 1000) {   /* throttle ~5 s */
            s_err_log_us = now_us;
            ESP_LOGW(TAG, "UART errors flagged; discarding drain (sleep wake / noise)");
        }
        uart_flush_input(SF30C_UART_NUM);
        sf30_ascii_reset(&s_ascii);
        rf_drain_abort(&s_rf);
        bool have = rf_finalize(&s_rf, dt_s, range_ft_out, valid);
        *valid = false;                    /* nothing fresh survived this poll  */
        return have;
    }

    uint8_t rx[256];
    /* Non-blocking drain of whatever is buffered. In bench sim mode the bytes
     * come from the USB-Serial-JTAG (the simulated sensor) instead of UART1.    */
    int n = s_sim_mode
            ? usb_serial_jtag_read_bytes(rx, sizeof rx, 0)
            : uart_read_bytes(SF30C_UART_NUM, rx, sizeof rx, 0);

    /* Bench diagnostic: dump the raw wire bytes (up to 16, ~1 Hz) so we can see
     * exactly what the real sensor is sending — distinguishes "sensor reports
     * lost" from a baud/sync problem. Real-sensor path only.                      */
    if (s_raw_debug && !s_sim_mode) {
        static int64_t s_raw_log_us = 0;
        int64_t t = esp_timer_get_time();
        if (t - s_raw_log_us >= 1000 * 1000) {     /* ~1 s, logged even when n==0 */
            s_raw_log_us = t;
            static const char HEXD[] = "0123456789ABCDEF";
            char hex[3 * 16 + 1];
            int m = n; if (m < 0) m = 0; if (m > 16) m = 16;
            int p = 0;
            for (int i = 0; i < m; ++i) {
                hex[p++] = HEXD[(rx[i] >> 4) & 0xF];
                hex[p++] = HEXD[rx[i] & 0xF];
                hex[p++] = ' ';
            }
            hex[p] = '\0';
            /* n==0  -> sensor TX line is silent (wiring, or output type != Serial)
             * n>0   -> bytes ARE arriving; the hex shows the real on-wire format. */
            ESP_LOGW(TAG, "raw UART1 drain: n=%d [%s]", n, hex);
        }
    }

    /* EVERY decoded sample of the drain is pushed into the robust filter —
     * the old code kept only the LAST pair, which let a single trailing
     * garbage pair outvote ~57 good samples at the GROUND cadence. Validity
     * (lost-signal band, negatives, beyond-ceiling junk) is judged inside
     * rf_push_cm(); the median vote + Hampel gate happen at rf_finalize().     */
    for (int i = 0; i < n; ++i) {
        /* SIM mode always carries the LWNX framed stream from the bench tool —
         * regardless of the real-sensor parse mode below — so decode it as LWNX
         * and dispatch any bench-control frames that ride alongside.             */
        if (s_sim_mode) {
            lwnx_frame_t f;
            if (lwnx_feed(&s_parser, rx[i], &f)) {
                if (f.cmd == LWNX_CMD_DISTANCE_DATA) {
                    int16_t cm = 0;
                    /* firstReturnFiltered is the int16 at payload offset 6. */
                    if (lwnx_read_i16(f.payload, f.plen, 6, &cm)) {
                        rf_push_cm(&s_rf, (float)cm);
                    }
                } else if (f.cmd == LWNX_CMD_BENCH_CTRL) {
                    sim_dispatch_frame(&f);
                }
            }
            continue;
        }

        /* REAL sensor: parse per the compile-time protocol. This SF30/C streams
         * the legacy 2-byte high/low pairs ("Distance over Serial"), so SF30C_MODE
         * is SF30C_ASCII; a unit configured for LWNX would use the binary path.   */
#if SF30C_MODE == SF30C_BINARY
        lwnx_frame_t f;
        if (lwnx_feed(&s_parser, rx[i], &f)) {
            if (f.cmd == LWNX_CMD_DISTANCE_DATA) {
                int16_t cm = 0;
                if (lwnx_read_i16(f.payload, f.plen, 6, &cm)) {
                    rf_push_cm(&s_rf, (float)cm);
                }
            } else if (f.cmd == LWNX_CMD_BENCH_CTRL) {
                sim_dispatch_frame(&f);
            }
        }
#else /* SF30C_ASCII — legacy 2-byte high/low pairs */
        int cm;
        if (sf30_ascii_feed(&s_ascii, rx[i], &cm)) {
            rf_push_cm(&s_rf, (float)cm);
        }
#endif
    }

    /* Close the drain: the samples vote, the Hampel gate judges the winner,
     * and the published value is either the fresh smoothed range or the held
     * last-good (empty drain / majority-lost drain / rejected outlier). The
     * *valid contract is unchanged: false whenever nothing fresh landed.        */
    return rf_finalize(&s_rf, dt_s, range_ft_out, valid);
}

/* ---- Binary helpers: send a command, await a specific reply -------------- */
#if SF30C_MODE == SF30C_BINARY

/*  Write a built LWNX frame out the UART. */
static void lwnx_send(uint8_t cmd, const uint8_t *payload, size_t plen, bool write)
{
    uint8_t frame[LWNX_MAX_FRAME];
    size_t n = lwnx_build(frame, sizeof frame, cmd, payload, plen, write);
    if (n > 0) {
        uart_write_bytes(SF30C_UART_NUM, (const char *)frame, n);
    }
}

/*  Write a u32 setting (little-endian payload). */
static void lwnx_write_u32(uint8_t cmd, uint32_t value)
{
    uint8_t p[4] = {
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF),
    };
    lwnx_send(cmd, p, sizeof p, /*write=*/true);
}

/*  Block (briefly) waiting for a framed reply with the given command id.
 *  Copies up to cap-1 payload bytes into 'out' as a NUL-terminated string and
 *  returns true on success. Uses a fresh local parser so it won't disturb the
 *  streaming parser state.                                                     */
static bool lwnx_await_reply(uint8_t want_cmd, char *out, size_t cap,
                             int timeout_ms)
{
    lwnx_parser_t pr;
    lwnx_parser_reset(&pr);

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        uint8_t rx[128];
        int n = uart_read_bytes(SF30C_UART_NUM, rx, sizeof rx, pdMS_TO_TICKS(20));
        for (int i = 0; i < n; ++i) {
            lwnx_frame_t f;
            if (lwnx_feed(&pr, rx[i], &f) && f.cmd == want_cmd) {
                size_t copy = (f.plen < cap - 1) ? f.plen : cap - 1;
                if (f.payload && copy > 0) {
                    memcpy(out, f.payload, copy);
                }
                out[copy] = '\0';
                return true;
            }
        }
    }
    return false;
}
#endif /* SF30C_MODE == SF30C_BINARY */

const sensor_profile_t *sf30c_detect_profile(void)
{
    if (s_sim_mode) {
        /* No sensor to interrogate on the bench — use the default profile so the
         * config menu still cycles a real callout ladder.                       */
        const sensor_profile_t *p = sensor_profile_for_model(DEFAULT_SENSOR_MODEL);
        ESP_LOGI(TAG, "sim mode: using default profile %s (no autodetect)", p->name);
        return p;
    }
#if SF30C_MODE == SF30C_BINARY
    /* Ask the sensor who it is (cmd 0, read). Retry a couple of times since the
     * unit may still be booting when we first ask.                            */
    char name[64] = {0};
    for (int attempt = 0; attempt < 3; ++attempt) {
        lwnx_send(LWNX_CMD_PRODUCT_NAME, NULL, 0, /*write=*/false);
        if (lwnx_await_reply(LWNX_CMD_PRODUCT_NAME, name, sizeof name, 300)) {
            const sensor_profile_t *p = sensor_profile_from_name(name);
            ESP_LOGI(TAG, "autodetect: product name \"%s\" -> %s", name, p->name);
            return p;
        }
    }
    ESP_LOGW(TAG, "autodetect: no product-name reply; falling back to default");
#endif
    const sensor_profile_t *p = sensor_profile_for_model(DEFAULT_SENSOR_MODEL);
    ESP_LOGI(TAG, "using default profile %s", p->name);
    return p;
}

void sf30c_configure_stream(uint32_t rate_code)
{
    if (s_sim_mode) {
        /* No sensor to configure — the bench free-streams cmd-44 frames. */
        ESP_LOGI(TAG, "sim mode: skipping sensor stream configuration");
        return;
    }
#if SF30C_MODE == SF30C_BINARY
    /* Order matches the LightWare sample: rate, then enable all distance
     * fields, then start streaming distance data.                            */
    lwnx_write_u32(LWNX_CMD_UPDATE_RATE, rate_code);
    lwnx_write_u32(LWNX_CMD_DISTANCE_CFG, 0xFFFFFFFFu);
    lwnx_write_u32(LWNX_CMD_STREAM, 5u);
    ESP_LOGI(TAG, "stream configured (rate_code=%u)", (unsigned)rate_code);
#else /* SF30C_ASCII — legacy SF30/C "Distance over Serial" (product guide §10) */
    /* Configure the sensor over serial exactly the way LightWare's own SF30 Linux
     * sample (sf30_raspberrypi_c_serial) does — now that we talk at the sensor's
     * real baud these actually land. We pin the sampling rate (#R) and serial
     * output rate (#U), zero the offset (#Z0), and start the laser (#Y). Rate
     * codes are a single digit 0..9 (8 == 78 readings/s — under the 500/s line the
     * datasheet rates at +-5 cm, plenty for the flare).
     *
     * The ONLY thing that still needs LightWare Studio is the one-time Output type
     * = "Distance over Serial" + the serial baud (no '#' command sets either).     */
    char digit = (char)('0' + (int)(rate_code % 10));
    char r_cmd[5] = { '#', 'R', digit, ':', '\0' };   /* sampling update rate      */
    char u_cmd[5] = { '#', 'U', digit, ':', '\0' };   /* serial port output rate   */
    uart_write_bytes(SF30C_UART_NUM, "#Y:", 3);       /* laser ON                  */
    uart_write_bytes(SF30C_UART_NUM, r_cmd, 4);
    uart_write_bytes(SF30C_UART_NUM, u_cmd, 4);
    uart_write_bytes(SF30C_UART_NUM, "#Z0:", 4);      /* zero offset = 0 m         */
    ESP_LOGI(TAG, "ASCII config: #Y + #R%c/#U%c + #Z0 (rate code %u, baud %d)",
             digit, digit, (unsigned)rate_code, SF30C_BAUD);
#endif
}

/* ---- Sensor task --------------------------------------------------------- */

void sensor_task(void *arg)
{
    (void)arg;

    /*  Task-watchdog subscription. Every reading the box reasons about comes
     *  through this loop, and it blocks only on bounded UART reads plus its own
     *  cadence delay — so if it stops feeding, it is genuinely wedged (a driver
     *  call that never returns), not merely idle. Without this the box would go
     *  permanently blind with no reset and nothing audible to tell the pilot.  */
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();

        float range_ft;
        bool  valid;
        if (sf30c_read_latest_ft(&range_ft, &valid)) {
            range_sample_t s = {
                .range_ft = range_ft,
                .valid    = valid,
                .seq      = ++s_seq,
                /*  Carry the filter's track-break verdict to the logic task. It
                 *  is true for exactly the one sample that re-acquired onto a
                 *  discontinuous level, so the consumer can re-anchor instead of
                 *  reading the jump as flown motion (see rf_track_broken).      */
                .track_break = rf_track_broken(&s_rf),
                /*  The sensor's own verdict on whether it can see the ground.
                 *  The logic task's power policy keys off THIS rather than off
                 *  altitude (see rf_tracking / the sleep block in logic_task). */
                .tracking    = rf_tracking(&s_rf),
            };
            /* Overwrite so the logic task always sees the freshest sample with
             * no backlog lag — essential for on-time callouts.                */
            xQueueOverwrite(q_range_latest, &s);
        }

        /* Cadence is owned by the state machine via this shared scalar. */
        uint32_t period = g_poll_period_ms;
        if (period == 0) {
            period = POLL_MS_ARMED;
        }
        /* Bench sim streams continuously at ~78 Hz, so we must drain the USB RX
         * briskly even in states whose normal poll is slow (GROUND is 750 ms) —
         * otherwise the RX ring overflows between reads and we drop frames,
         * including one-shot control frames (e.g. reboot-to-config). Flight is
         * unaffected: this only caps the period while sim mode is active.        */
        if (s_sim_mode && period > SIM_POLL_MS) {
            period = SIM_POLL_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}
