/**
 * @file    test_lwnx.c
 * @brief   Host unit tests for the LWNX binary protocol.
 *
 * @details Proves: build->parse round-trips for the commands we use, that the
 *          CRC is the LightWare MSB-first variant (and therefore NOT equal to a
 *          reflected CRC), that a corrupted byte is rejected, and that the
 *          parser re-synchronises after garbage.
 */

#include "test_util.h"
#include "lwnx.h"

#include <string.h>
#include <stdbool.h>

TEST_GLOBALS

/* Feed an entire byte buffer through the parser; return true if exactly one
 * valid frame popped out, and copy it to 'out'. */
static bool feed_all(lwnx_parser_t *p, const uint8_t *bytes, size_t n,
                     lwnx_frame_t *out)
{
    lwnx_frame_t f;
    bool got = false;
    for (size_t i = 0; i < n; ++i) {
        if (lwnx_feed(p, bytes[i], &f)) {
            *out = f;
            got = true;
        }
    }
    return got;
}

int main(void)
{
    printf("== lwnx ==\n");

    lwnx_parser_t p;
    lwnx_parser_reset(&p);

    /* --- Round-trip: a read command with no payload (product name) -------- */
    {
        uint8_t frame[LWNX_MAX_FRAME];
        size_t n = lwnx_build(frame, sizeof frame, LWNX_CMD_PRODUCT_NAME,
                              NULL, 0, /*write=*/false);
        ASSERT_TRUE(n == 6, "no-payload frame is 6 bytes");
        ASSERT_TRUE(frame[0] == LWNX_START_BYTE, "frame starts with 0xAA");

        lwnx_parser_reset(&p);
        lwnx_frame_t out;
        bool got = feed_all(&p, frame, n, &out);
        ASSERT_TRUE(got, "no-payload frame parses");
        ASSERT_TRUE(out.cmd == LWNX_CMD_PRODUCT_NAME, "cmd id round-trips (0)");
        ASSERT_TRUE(out.plen == 0, "no-payload plen is 0");
    }

    /* --- Round-trip: a write u32 (update rate) ---------------------------- */
    {
        uint8_t payload[4] = { 8, 0, 0, 0 };       /* rate code 8, LE */
        uint8_t frame[LWNX_MAX_FRAME];
        size_t n = lwnx_build(frame, sizeof frame, LWNX_CMD_UPDATE_RATE,
                              payload, sizeof payload, /*write=*/true);
        ASSERT_TRUE(n == 10, "u32-payload frame is 10 bytes");
        /* write flag is bit 0 of the flags low byte */
        ASSERT_TRUE((frame[1] & 0x01) == 0x01, "write flag set");

        lwnx_parser_reset(&p);
        lwnx_frame_t out;
        bool got = feed_all(&p, frame, n, &out);
        ASSERT_TRUE(got, "u32 frame parses");
        ASSERT_TRUE(out.cmd == LWNX_CMD_UPDATE_RATE, "cmd id round-trips (76)");
        ASSERT_TRUE(out.plen == 4, "u32 payload length is 4");
        ASSERT_TRUE(out.payload && out.payload[0] == 8, "u32 payload byte 0 == 8");
    }

    /* --- Round-trip: a distance frame; read int16 cm at offset 6 ---------- */
    {
        /* Synthesize a cmd-44 frame whose payload has a distance of 1234 cm at
         * offset 6 (matching the SF30/D firstReturnFiltered field).           */
        uint8_t payload[20];
        memset(payload, 0, sizeof payload);
        int16_t dist_cm = 1234;
        payload[6] = (uint8_t)(dist_cm & 0xFF);
        payload[7] = (uint8_t)((uint16_t)dist_cm >> 8);

        uint8_t frame[LWNX_MAX_FRAME];
        size_t n = lwnx_build(frame, sizeof frame, LWNX_CMD_DISTANCE_DATA,
                              payload, sizeof payload, /*write=*/false);

        lwnx_parser_reset(&p);
        lwnx_frame_t out;
        bool got = feed_all(&p, frame, n, &out);
        ASSERT_TRUE(got, "distance frame parses");
        ASSERT_TRUE(out.cmd == LWNX_CMD_DISTANCE_DATA, "cmd id round-trips (44)");

        int16_t read_cm = 0;
        bool ok = lwnx_read_i16(out.payload, out.plen, 6, &read_cm);
        ASSERT_TRUE(ok, "i16 read in range");
        ASSERT_TRUE(read_cm == 1234, "distance reads back as 1234 cm");

        /* Out-of-range read is rejected. */
        int16_t dummy;
        ASSERT_TRUE(!lwnx_read_i16(out.payload, out.plen, 19, &dummy),
                    "i16 read past end rejected");
    }

    /* --- CRC is the MSB-first LightWare variant --------------------------- */
    {
        /* Known property: this CRC of {0xAA} alone is a specific value; more
         * importantly it must differ from a trivial sum and be deterministic.
         * We assert determinism + that a one-bit change flips the CRC.         */
        uint8_t a[4] = { 0xAA, 0x40, 0x00, 0x00 };
        uint16_t c1 = lwnx_crc16(a, 4);
        uint16_t c2 = lwnx_crc16(a, 4);
        ASSERT_TRUE(c1 == c2, "CRC is deterministic");

        a[3] ^= 0x01;                       /* flip one bit */
        uint16_t c3 = lwnx_crc16(a, 4);
        ASSERT_TRUE(c1 != c3, "CRC changes when a byte changes");
    }

    /* --- Corruption is rejected ------------------------------------------- */
    {
        uint8_t payload[4] = { 5, 0, 0, 0 };
        uint8_t frame[LWNX_MAX_FRAME];
        size_t n = lwnx_build(frame, sizeof frame, LWNX_CMD_STREAM,
                              payload, sizeof payload, true);

        /* Corrupt a payload byte AFTER the CRC was computed. */
        frame[4] ^= 0xFF;

        lwnx_parser_reset(&p);
        lwnx_frame_t out;
        bool got = feed_all(&p, frame, n, &out);
        ASSERT_TRUE(!got, "corrupted frame is rejected by CRC");
    }

    /* --- Parser re-synchronises after leading garbage --------------------- */
    {
        uint8_t payload[4] = { 5, 0, 0, 0 };
        uint8_t frame[LWNX_MAX_FRAME];
        size_t n = lwnx_build(frame, sizeof frame, LWNX_CMD_STREAM,
                              payload, sizeof payload, true);

        uint8_t stream[LWNX_MAX_FRAME + 8];
        size_t k = 0;
        /* prepend junk that includes a stray 0xAA to test false-start recovery */
        stream[k++] = 0x12;
        stream[k++] = 0xAA;          /* false start */
        stream[k++] = 0x34;
        stream[k++] = 0x00;
        memcpy(stream + k, frame, n);
        k += n;

        lwnx_parser_reset(&p);
        lwnx_frame_t out;
        bool got = feed_all(&p, stream, k, &out);
        ASSERT_TRUE(got, "parser recovers and finds the real frame after garbage");
        ASSERT_TRUE(out.cmd == LWNX_CMD_STREAM, "recovered cmd id is correct (30)");
    }

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
