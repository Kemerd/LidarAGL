/**
 * @file    lwnx.h
 * @brief   LightWare LWNX binary protocol: packet build, parse, and CRC.
 *
 * @details The LWNX framing used by the SF30/D (and recent SF30/C firmware) is:
 *
 *            byte 0      : 0xAA              start byte
 *            bytes 1..2  : flags (u16 LE)    flags = (payloadLen << 6) | write
 *            byte 3      : command id        (counted inside payloadLen)
 *            bytes 4..   : payload data      (payloadLen - 1 bytes)
 *            last 2      : CRC-16 (u16 LE)   over bytes [0 .. last payload byte]
 *
 *          payloadLen counts the command-id byte plus the data bytes. The CRC is
 *          a MSB-first 0x1021 (CCITT) variant with init 0 — this is NOT the same
 *          as ESP-IDF's reflected esp_rom_crc16_le(), so we implement it here
 *          exactly as the LightWare reference does (see lwnx_crc16).
 *
 *          PURE module: standard library only. Host-testable. The byte-feed
 *          parser is allocation-free and re-syncs automatically on garbage.
 */
#ifndef LIDARAGL_LWNX_H
#define LIDARAGL_LWNX_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/** LWNX start-of-frame byte. */
#define LWNX_START_BYTE   0xAAu

/** Largest frame we will assemble (matches LightWare's 1 KiB working buffer). */
#define LWNX_MAX_FRAME    256

/* ---- Well-known command IDs (subset we use) ----------------------------- */
#define LWNX_CMD_PRODUCT_NAME  0u   /**< read: product-name string (autodetect) */
#define LWNX_CMD_DISTANCE_CFG  29u  /**< write u32: which distance fields stream */
#define LWNX_CMD_STREAM        30u  /**< write u32: 5 = stream distance data     */
#define LWNX_CMD_DISTANCE_DATA 44u  /**< stream: distance frame (cm)             */
#define LWNX_CMD_UPDATE_RATE   76u  /**< write u32: output rate code             */

/**
 * @brief Compute the LWNX CRC-16 over a byte span.
 *
 * @details Direct port of LightWare's lwnxCreateCrc: init 0, MSB-first, poly
 *          0x1021 expressed via the shift-and-xor form used in their sample.
 *
 * @param data  Bytes to checksum (start byte through last payload byte).
 * @param len   Number of bytes.
 * @return      The 16-bit CRC.
 */
uint16_t lwnx_crc16(const uint8_t *data, size_t len);

/**
 * @brief Build a complete LWNX frame into @p out.
 *
 * @param out      Destination buffer.
 * @param cap      Capacity of @p out in bytes.
 * @param cmd      Command id.
 * @param payload  Data bytes following the command id (may be NULL if @p plen 0).
 * @param plen     Number of data bytes (NOT counting the command id).
 * @param write    True for a write command (sets the write flag bit).
 * @return         Total frame length written, or 0 if it would not fit.
 */
size_t lwnx_build(uint8_t *out, size_t cap, uint8_t cmd,
                  const uint8_t *payload, size_t plen, bool write);

/** A fully parsed, CRC-validated inbound frame (points into the parser buffer). */
typedef struct {
    uint8_t        cmd;      /**< Command id of the frame.                     */
    const uint8_t *payload;  /**< Payload bytes (data after the command id).   */
    size_t         plen;     /**< Number of payload data bytes.                */
} lwnx_frame_t;

/** Incremental byte-feed parser state. Zero-initialise or call lwnx_parser_reset. */
typedef struct {
    int      state;                  /**< internal parse phase                 */
    uint16_t flags;                  /**< accumulated flags field              */
    size_t   payload_len;            /**< expected cmd+data length              */
    size_t   idx;                    /**< bytes collected into buf              */
    uint8_t  buf[LWNX_MAX_FRAME];    /**< working frame buffer                  */
} lwnx_parser_t;

/** @brief Reset a parser to look for a fresh start byte. */
void lwnx_parser_reset(lwnx_parser_t *p);

/**
 * @brief Feed one received byte into the parser.
 *
 * @param p      Parser state.
 * @param b      The received byte.
 * @param[out] out  Filled in and valid ONLY when this call returns true.
 * @return       True when a complete, CRC-valid frame has just been assembled.
 */
bool lwnx_feed(lwnx_parser_t *p, uint8_t b, lwnx_frame_t *out);

/* ---- Small payload helpers (little-endian, bounds-checked) --------------- */

/**
 * @brief Read a little-endian int16 from a payload at a byte offset.
 * @param payload  Payload bytes.
 * @param plen     Payload length.
 * @param offset   Byte offset of the value.
 * @param[out] out  Receives the value when in range.
 * @return         True if @p offset+2 <= plen and @p out was written.
 */
bool lwnx_read_i16(const uint8_t *payload, size_t plen, size_t offset, int16_t *out);

#endif /* LIDARAGL_LWNX_H */
