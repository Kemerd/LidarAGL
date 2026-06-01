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

#endif /* LIDARAGL_SF30C_H */
