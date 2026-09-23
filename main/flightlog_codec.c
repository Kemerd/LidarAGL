/**
 * @file    flightlog_codec.c
 * @brief   Implementation of the flight recorder's on-flash format (see header).
 *
 * @details PURE: C standard library only. Compiled into both the firmware and
 *          the host test suite (test/test_flightlog_codec.c), and mirrored by the
 *          Python decoder in tools/flightlog/flightlog.py.
 */

#include "flightlog_codec.h"

#include <string.h>

/* ===========================================================================
 *  CRC-8
 * ===========================================================================*/

uint8_t flog_crc8(const uint8_t *data, size_t n)
{
    /*  Plain bitwise CRC-8/SMBUS (poly 0x07, init 0). A table would be faster,
     *  but records are tiny and written at tens of hertz, so the 256-byte table
     *  is not worth its RAM. A NULL span with a non-zero length is a caller
     *  bug; treat it as empty rather than dereference it.                      */
    uint8_t crc = 0u;
    if (data == NULL) {
        return crc;
    }
    for (size_t i = 0; i < n; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ===========================================================================
 *  Record framing
 * ===========================================================================*/

size_t flog_encode_record(uint8_t *out, size_t cap, uint8_t type, uint32_t t_ms,
                          const void *payload, size_t len)
{
    /* --- Reject anything that cannot be framed ----------------------------
     *  Type 0x00 is reserved and 0xFF is what erased flash reads as — a record
     *  with either would be indistinguishable from "end of sector".           */
    if (out == NULL || type == 0x00u || type == 0xFFu) {
        return 0;
    }
    if (len > FLOG_REC_MAX_PAYLOAD || (payload == NULL && len > 0u)) {
        return 0;
    }
    size_t total = FLOG_REC_OVERHEAD + len;
    if (cap < total) {
        return 0;
    }

    /* --- Header: type, len, little-endian timestamp ----------------------- */
    out[0] = type;
    out[1] = (uint8_t)len;
    out[2] = (uint8_t)(t_ms & 0xFFu);
    out[3] = (uint8_t)((t_ms >> 8) & 0xFFu);
    out[4] = (uint8_t)((t_ms >> 16) & 0xFFu);
    out[5] = (uint8_t)((t_ms >> 24) & 0xFFu);

    /* --- Payload, then the CRC over everything before it ------------------ */
    if (len > 0u) {
        memcpy(&out[FLOG_REC_HDR_SIZE], payload, len);
    }
    out[FLOG_REC_HDR_SIZE + len] = flog_crc8(out, FLOG_REC_HDR_SIZE + len);
    return total;
}

int flog_decode_record(const uint8_t *in, size_t avail, uint8_t *type,
                       uint32_t *t_ms, const uint8_t **payload, size_t *len)
{
    /* --- End of data: nothing left, or the erased 0xFF fill -------------- */
    if (in == NULL || avail < FLOG_REC_OVERHEAD || in[0] == 0xFFu) {
        return 0;
    }

    /* --- Structural checks before trusting the length -------------------- */
    size_t plen = in[1];
    if (in[0] == 0x00u || plen > FLOG_REC_MAX_PAYLOAD) {
        return -1;
    }
    size_t total = FLOG_REC_OVERHEAD + plen;
    if (total > avail) {
        return -1;                     /* runs off the end of the sector      */
    }

    /* --- Integrity: a torn write fails here ------------------------------ */
    if (flog_crc8(in, FLOG_REC_HDR_SIZE + plen) != in[FLOG_REC_HDR_SIZE + plen]) {
        return -1;
    }

    /* --- Hand the fields back (outputs are optional) --------------------- */
    if (type) {
        *type = in[0];
    }
    if (t_ms) {
        *t_ms = (uint32_t)in[2] | ((uint32_t)in[3] << 8) |
                ((uint32_t)in[4] << 16) | ((uint32_t)in[5] << 24);
    }
    if (payload) {
        *payload = &in[FLOG_REC_HDR_SIZE];
    }
    if (len) {
        *len = plen;
    }
    return (int)total;
}

bool flog_sector_hdr_valid(const flog_sector_hdr_t *h)
{
    return (h != NULL) && (h->magic == FLOG_MAGIC);
}

/* ===========================================================================
 *  RAW entry encoder
 * ===========================================================================*/

void flog_raw_reset(flog_raw_enc_t *e)
{
    if (e == NULL) {
        return;
    }
    e->n         = 0u;
    e->last      = 0u;
    e->have_last = false;
    e->rep       = 0u;
}

size_t flog_raw_room(const flog_raw_enc_t *e)
{
    /*  One slot is permanently reserved so flog_raw_take() can always flush a
     *  pending REPEAT, whatever state the encoder was left in.                */
    if (e == NULL) {
        return 0u;
    }
    const size_t usable = FLOG_RAW_MAX_ENTRIES - 1u;
    return (e->n >= usable) ? 0u : (usable - e->n);
}

/*  Emit the pending REPEAT run, if any, as one entry. */
static void raw_flush_repeat(flog_raw_enc_t *e)
{
    if (e->rep > 0u && e->n < FLOG_RAW_MAX_ENTRIES) {
        e->entries[e->n++] = (uint16_t)(FLOG_RAW_TAG_REPEAT |
                                        (e->rep & FLOG_RAW_VALUE_MASK));
    }
    e->rep = 0u;
}

bool flog_raw_sample(flog_raw_enc_t *e, int cm)
{
    /*  Require two entries of room even for a sample that would only extend a
     *  run: the run may have to be flushed and followed by a literal, and the
     *  caller should take the record now rather than discover that mid-drain. */
    if (flog_raw_room(e) < 2u) {
        return false;
    }

    /* --- Clamp into the 14-bit field (the wire cannot exceed it anyway) --- */
    if (cm < 0) {
        cm = 0;
    } else if (cm > (int)FLOG_RAW_VALUE_MASK) {
        cm = (int)FLOG_RAW_VALUE_MASK;
    }
    uint16_t v = (uint16_t)cm;

    /* --- Same value as the last literal: just lengthen the run ------------ */
    if (e->have_last && v == e->last && e->rep < FLOG_RAW_VALUE_MASK) {
        e->rep++;
        return true;
    }

    /* --- New value: close any run, then write the literal ----------------- */
    raw_flush_repeat(e);
    e->entries[e->n++] = v;
    e->last      = v;
    e->have_last = true;
    return true;
}

bool flog_raw_marker(flog_raw_enc_t *e, uint32_t t_ms, bool aborted)
{
    if (flog_raw_room(e) < 2u) {
        return false;
    }

    /*  The run must be closed BEFORE the marker so its samples stay inside the
     *  drain they belong to — drain boundaries are what a replay needs most.  */
    raw_flush_repeat(e);
    uint16_t m = (uint16_t)(FLOG_RAW_MARKER_BIT | (t_ms & FLOG_RAW_VALUE_MASK));
    if (aborted) {
        m |= FLOG_RAW_ABORT_BIT;
    }
    e->entries[e->n++] = m;
    return true;
}

size_t flog_raw_take(flog_raw_enc_t *e, uint8_t *out)
{
    if (e == NULL || out == NULL) {
        return 0u;
    }

    /* --- Close the run into the reserved slot, serialise, reset ----------- */
    raw_flush_repeat(e);
    size_t bytes = 0u;
    for (size_t i = 0; i < e->n; ++i) {
        out[bytes++] = (uint8_t)(e->entries[i] & 0xFFu);
        out[bytes++] = (uint8_t)(e->entries[i] >> 8);
    }
    flog_raw_reset(e);
    return bytes;
}
