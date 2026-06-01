/**
 * @file    lwnx.c
 * @brief   LWNX binary protocol implementation (build/parse/CRC).
 *
 * @details PURE: standard library only. The CRC is ported verbatim from the
 *          LightWare SampleLibrary so on-wire frames match the sensor exactly.
 */

#include "lwnx.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 *  CRC-16 — exact port of LightWare's lwnxCreateCrc().
 *
 *  Init 0, processed MSB-first. The shift-and-xor sequence below is the
 *  unrolled form of a poly-0x1021 (CCITT) update and is what the sensor uses to
 *  validate frames. Do NOT substitute esp_rom_crc16_le(): that ROM routine is
 *  the *reflected* (LSB-first) variant and produces different checksums, which
 *  the sensor would reject.
 * ------------------------------------------------------------------------- */
uint16_t lwnx_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        uint16_t code = (uint16_t)(crc >> 8);
        code ^= data[i];
        code ^= (uint16_t)(code >> 4);
        crc = (uint16_t)(crc << 8);
        crc ^= code;
        code = (uint16_t)(code << 5);
        crc ^= code;
        code = (uint16_t)(code << 7);
        crc ^= code;
    }
    return crc;
}

size_t lwnx_build(uint8_t *out, size_t cap, uint8_t cmd,
                  const uint8_t *payload, size_t plen, bool write)
{
    /* payloadLen counts the command-id byte plus the data bytes. */
    size_t payload_len = plen + 1;

    /* Frame = start(1) + flags(2) + cmd(1) + data(plen) + crc(2). */
    size_t frame_len = 1 + 2 + 1 + plen + 2;
    if (frame_len > cap) {
        return 0;   /* caller's buffer is too small */
    }

    /* flags = (payloadLen << 6) | (write ? 1 : 0), stored little-endian. */
    uint16_t flags = (uint16_t)((payload_len << 6) | (write ? 1u : 0u));

    out[0] = LWNX_START_BYTE;
    out[1] = (uint8_t)(flags & 0xFF);
    out[2] = (uint8_t)(flags >> 8);
    out[3] = cmd;
    if (plen > 0 && payload != NULL) {
        memcpy(out + 4, payload, plen);
    }

    /* CRC covers everything up to and including the last payload byte. */
    size_t crc_span = 4 + plen;
    uint16_t crc = lwnx_crc16(out, crc_span);
    out[crc_span + 0] = (uint8_t)(crc & 0xFF);
    out[crc_span + 1] = (uint8_t)(crc >> 8);

    return frame_len;
}

/* ---- Incremental parser -------------------------------------------------- */

/*  Parse phases. We collect the full frame (start..crc) into buf, then verify
 *  the CRC at the end. On any inconsistency we reset and re-scan for a start
 *  byte, so a noisy line resynchronises on its own.                           */
enum {
    PS_START = 0,   /* waiting for 0xAA                         */
    PS_FLAGS_LO,    /* reading flags low byte                  */
    PS_FLAGS_HI,    /* reading flags high byte                 */
    PS_BODY         /* reading cmd + data + crc                */
};

void lwnx_parser_reset(lwnx_parser_t *p)
{
    p->state       = PS_START;
    p->flags       = 0;
    p->payload_len = 0;
    p->idx         = 0;
}

bool lwnx_feed(lwnx_parser_t *p, uint8_t b, lwnx_frame_t *out)
{
    switch (p->state) {
        case PS_START:
            if (b == LWNX_START_BYTE) {
                p->buf[0] = b;
                p->idx    = 1;
                p->state  = PS_FLAGS_LO;
            }
            return false;

        case PS_FLAGS_LO:
            p->buf[p->idx++] = b;
            p->flags = b;                 /* low byte */
            p->state = PS_FLAGS_HI;
            return false;

        case PS_FLAGS_HI:
            p->buf[p->idx++] = b;
            p->flags |= (uint16_t)b << 8; /* high byte */

            /* payloadLen = flags >> 6 (counts cmd + data). Sanity-check it so a
             * corrupt flags field can't make us wait for a giant frame.        */
            p->payload_len = (size_t)(p->flags >> 6);
            if (p->payload_len < 1 ||
                /* total frame = start+flags(3) + payload_len + crc(2) */
                (3 + p->payload_len + 2) > LWNX_MAX_FRAME) {
                lwnx_parser_reset(p);     /* implausible -> resync */
            } else {
                p->state = PS_BODY;
            }
            return false;

        case PS_BODY: {
            p->buf[p->idx++] = b;

            /* Total expected length: start(1)+flags(2)+payload_len+crc(2). */
            size_t total = 3 + p->payload_len + 2;
            if (p->idx < total) {
                return false;             /* still collecting */
            }

            /* Full frame in buf. Validate CRC over [0 .. last payload byte]. */
            size_t crc_span = 3 + p->payload_len;
            uint16_t want = lwnx_crc16(p->buf, crc_span);
            uint16_t got  = (uint16_t)(p->buf[crc_span] |
                                       ((uint16_t)p->buf[crc_span + 1] << 8));

            bool valid = (want == got);
            if (valid) {
                /* cmd is the first byte of the payload region (index 3). */
                out->cmd     = p->buf[3];
                out->payload = (p->payload_len > 1) ? &p->buf[4] : NULL;
                out->plen    = p->payload_len - 1;   /* minus the cmd byte */
            }
            lwnx_parser_reset(p);
            return valid;
        }

        default:
            lwnx_parser_reset(p);
            return false;
    }
}

bool lwnx_read_i16(const uint8_t *payload, size_t plen, size_t offset, int16_t *out)
{
    if (payload == NULL || offset + 2 > plen) {
        return false;
    }
    uint16_t u = (uint16_t)(payload[offset] |
                            ((uint16_t)payload[offset + 1] << 8));
    *out = (int16_t)u;
    return true;
}
