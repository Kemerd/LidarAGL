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
#include "shared.h"

#include <string.h>
#include <math.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sf30c";

/* ---- Module state -------------------------------------------------------- */

/*  EMA-smoothed range in feet for the state/voice path. Seeded on first sample. */
static float  s_range_ema_ft = 0.0f;
static bool   s_have_ema      = false;

/*  Last good range, held across lost-signal samples so we never publish junk.  */
static float  s_last_good_ft  = 0.0f;
static bool   s_have_good     = false;

/*  Monotonic sample sequence for the consumer's dt sanity. */
static uint32_t s_seq = 0;

/*  Persistent LWNX parser (binary mode). */
static lwnx_parser_t s_parser;

/*  Bench HIL simulation: when set, the read path drains the SIMULATED LWNX
 *  stream from the USB-Serial-JTAG instead of UART1. Runtime-only (never NVS).  */
static bool                  s_sim_mode = false;
static sf30c_bench_ctrl_cb_t s_bench_cb = NULL;

/* ---- UART setup ---------------------------------------------------------- */

void sf30c_init(void)
{
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
        .source_clk = UART_SCLK_DEFAULT,        /* v5 idiom */
    };

    ESP_ERROR_CHECK(uart_driver_install(SF30C_UART_NUM,
                                        SF30C_UART_RX_BUF, SF30C_UART_TX_BUF,
                                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SF30C_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(SF30C_UART_NUM,
                                 PIN_SF30C_TX, PIN_SF30C_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    lwnx_parser_reset(&s_parser);

    ESP_LOGI(TAG, "UART%d up @ %d 8N1 (RX=%d TX=%d), mode=%s",
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

/* ---- Helpers ------------------------------------------------------------- */

/*  cm -> ft, the SINGLE place this conversion happens. */
static inline float cm_to_ft(float cm)
{
    return cm * CM_TO_FT;
}

/*  Apply the light EMA used by the state/voice path and record last-good. */
static void accept_range_ft(float ft)
{
    if (!s_have_ema) {
        s_range_ema_ft = ft;
        s_have_ema     = true;
    } else {
        s_range_ema_ft += RANGE_EMA_ALPHA * (ft - s_range_ema_ft);
    }
    s_last_good_ft = s_range_ema_ft;
    s_have_good    = true;
}

/* ---- ASCII path: high-bit-flagged 2-byte pairs --------------------------- */
/*  The legacy SF30 stream sends distance as pairs: the first byte has bit 7
 *  set and carries the upper 7 bits; the second byte (bit 7 clear) carries the
 *  lower 7 bits. distance_cm = (high << 7) | low. 16000 cm == lost signal.     */
static bool s_ascii_have_high = false;
static uint8_t s_ascii_high   = 0;

static bool ascii_feed(uint8_t b, float *out_cm, bool *out_valid)
{
    if (b & 0x80) {
        s_ascii_high      = (uint8_t)(b & 0x7F);
        s_ascii_have_high = true;
        return false;
    }
    if (!s_ascii_have_high) {
        return false;                       /* low byte with no preceding high */
    }
    s_ascii_have_high = false;
    int dist_cm = ((int)s_ascii_high << 7) | (int)(b & 0x7F);

    *out_valid = (dist_cm != SF30_LOST_SIGNAL_CM);
    *out_cm    = (float)dist_cm;
    return true;
}

/* ---- Drain + parse the freshest sample ----------------------------------- */

bool sf30c_read_latest_ft(float *range_ft_out, bool *valid)
{
    uint8_t rx[256];
    /* Non-blocking drain of whatever is buffered. In bench sim mode the bytes
     * come from the USB-Serial-JTAG (the simulated sensor) instead of UART1.    */
    int n = s_sim_mode
            ? usb_serial_jtag_read_bytes(rx, sizeof rx, 0)
            : uart_read_bytes(SF30C_UART_NUM, rx, sizeof rx, 0);

    bool got_sample = false;
    bool last_valid = true;
    float last_cm   = 0.0f;

    for (int i = 0; i < n; ++i) {
#if SF30C_MODE == SF30C_BINARY
        lwnx_frame_t f;
        if (lwnx_feed(&s_parser, rx[i], &f)) {
            if (f.cmd == LWNX_CMD_DISTANCE_DATA) {
                int16_t cm = 0;
                /* firstReturnFiltered is the int16 at payload offset 6. */
                if (lwnx_read_i16(f.payload, f.plen, 6, &cm)) {
                    last_cm    = (float)cm;
                    last_valid = (cm != (int16_t)SF30_LOST_SIGNAL_CM) && (cm >= 0);
                    got_sample = true;
                }
            } else if (f.cmd == LWNX_CMD_BENCH_CTRL) {
                /* Bench-control frames ride the same stream while in sim mode. */
                sim_dispatch_frame(&f);
            }
        }
#else /* SF30C_ASCII */
        float cm; bool v;
        if (ascii_feed(rx[i], &cm, &v)) {
            last_cm    = cm;
            last_valid = v;
            got_sample = true;
        }
#endif
    }

    if (got_sample && last_valid) {
        accept_range_ft(cm_to_ft(last_cm));
    }

    /* Report the freshest good range; hold last-good across a lost return so we
     * never feed the state machine a garbage number.                          */
    if (s_have_good) {
        *range_ft_out = s_last_good_ft;
        *valid        = got_sample ? last_valid : false;
        return true;
    }
    return false;
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
#else
    (void)rate_code;   /* ASCII firmware free-streams; nothing to configure. */
#endif
}

/* ---- Sensor task --------------------------------------------------------- */

void sensor_task(void *arg)
{
    (void)arg;

    for (;;) {
        float range_ft;
        bool  valid;
        if (sf30c_read_latest_ft(&range_ft, &valid)) {
            range_sample_t s = {
                .range_ft = range_ft,
                .valid    = valid,
                .seq      = ++s_seq,
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
