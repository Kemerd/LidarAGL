/**
 * @file    sf30c.h
 * @brief   LightWare SF30/C | SF30/D UART driver, parser, and sensor task.
 *
 * @details Owns UART1. Supports two parse modes behind SF30C_MODE:
 *            - SF30C_ASCII : the legacy continuous 2-byte distance stream
 *                            (high-bit-flagged pairs) — handy for bring-up.
 *            - SF30C_BINARY: the LWNX framed protocol — preferred for production;
 *                            also enables the product-name autodetect.
 *
 *          Range is converted from centimetres to feet in exactly one place
 *          (this module). The mount/ground offset is applied later, by the
 *          logic task, against the learned ground reference — NOT here.
 */
#ifndef LIDARAGL_SF30C_H
#define LIDARAGL_SF30C_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "sensor_profile.h"

/**
 * @brief Configure UART1 (pins, baud 8N1) and install the driver.
 */
void sf30c_init(void);

/**
 * @brief Identify the attached sensor and return its profile.
 *
 * @details On the binary path, issues an LWNX product-name read (cmd 0) and
 *          matches the returned string. On the ASCII path, or if no valid name
 *          arrives within a short timeout, falls back to DEFAULT_SENSOR_MODEL.
 *          Always returns a usable profile (never NULL).
 */
const sensor_profile_t *sf30c_detect_profile(void);

/**
 * @brief Put the sensor into the streaming configuration (binary path only).
 *
 * @param rate_code  LWNX update-rate code (cmd 76 argument). Ignored in ASCII
 *                   mode, where the sensor free-streams.
 */
void sf30c_configure_stream(uint32_t rate_code);

/**
 * @brief Read the freshest available range in FEET.
 *
 * @details Drains whatever the UART has buffered, parses it per the active mode,
 *          and returns the most recent valid range. On a lost-signal sample the
 *          last good range is held and @p *valid is set false so the caller can
 *          avoid emitting garbage callouts.
 *
 * @param[out] range_ft_out  Latest range in feet (mount offset NOT applied).
 * @param[out] valid         True if the latest sample was a real return.
 * @return                   True if any sample (valid or held) is available.
 */
bool sf30c_read_latest_ft(float *range_ft_out, bool *valid);

/**
 * @brief FreeRTOS entry point for the sensor task (core 0).
 *
 * @details Polls at g_poll_period_ms, publishes each freshest sample to
 *          q_range_latest via xQueueOverwrite, and applies the light EMA used
 *          by the state/voice path.
 */
void sensor_task(void *arg);

/* ===========================================================================
 *  Bench HIL simulation (USB-Serial-JTAG side-door)
 * ---------------------------------------------------------------------------
 *  In sim mode the read path drains a SIMULATED LWNX stream from the native
 *  USB-Serial-JTAG instead of UART1, feeding the SAME parser/CRC/distance decode
 *  so the host's frames exercise the real production path. Runtime-only: nothing
 *  is persisted, so a power cycle returns to the real sensor. See config.h and
 *  the tools/bench_sim Python app.
 * ===========================================================================*/

/**
 * @brief Enter bench simulation mode (install the USB-Serial-JTAG driver and
 *        flip the read source to it). Idempotent.
 */
void sf30c_sim_enable(void);

/** @brief True while bench simulation mode is active. */
bool sf30c_sim_active(void);

/**
 * @brief Enable a throttled hex dump of the raw bytes drained from the REAL
 *        sensor UART (bench diagnostics only; the sim stream is unaffected).
 *
 * @details Lets a developer see exactly what the SF30 puts on the wire, to tell
 *          "sensor reports lost / out-of-range" apart from a baud/sync problem.
 */
void sf30c_enable_raw_debug(void);

/**
 * @brief Callback invoked when a BENCH_CTRL frame is decoded on the sim stream.
 *
 * @param opcode  OP_* control opcode (payload byte 0).
 * @param arg     Remaining payload bytes (little-endian args), or NULL.
 * @param arglen  Number of arg bytes.
 *
 * @note Runs in whichever task drains the stream (the boot/menu pump, or the
 *       sensor task once running). Keep it short.
 */
typedef void (*sf30c_bench_ctrl_cb_t)(uint8_t opcode, const uint8_t *arg, size_t arglen);

/** @brief Register the bench-control callback (see sf30c_bench_ctrl_cb_t). */
void sf30c_set_bench_ctrl_cb(sf30c_bench_ctrl_cb_t cb);

/**
 * @brief Drain the sim stream once (non-blocking): feed the parser and dispatch
 *        any BENCH_CTRL frames via the registered callback.
 *
 * @details Lets boot-time code and the config menu pump control frames before
 *          the sensor task exists. Distance frames are parsed but ignored here.
 */
void sf30c_sim_read_drain(void);

#endif /* LIDARAGL_SF30C_H */
