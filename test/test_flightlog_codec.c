/**
 * @file    test_flightlog_codec.c
 * @brief   Host unit tests for the flight recorder's on-flash format.
 *
 * @details The recorder is only useful if what it wrote can be read back
 *          exactly, including after the pilot kills the master switch mid-write.
 *          These tests prove: record framing round-trips; the struct layouts the
 *          Python decoder mirrors have the byte sizes it expects; a torn or
 *          corrupted record is REJECTED rather than decoded as garbage; erased
 *          flash reads as a clean end of data; and the RAW run-length encoder
 *          reproduces every sample and every drain boundary, in order.
 */

#include "test_util.h"
#include "flightlog_codec.h"

#include <string.h>
#include <stdbool.h>

TEST_GLOBALS

/* ---------------------------------------------------------------------------
 *  Reference RAW decoder — the same rules tools/flightlog/flightlog.py applies.
 *  Expands entries back into a flat stream where a drain marker is encoded as
 *  -1 - (t_ms & 0x3FFF) and an aborted marker as -100000 - (t & 0x3FFF).
 * ------------------------------------------------------------------------- */
static size_t raw_expand(const uint8_t *bytes, size_t nbytes, long *out, size_t cap)
{
    size_t k = 0;
    long   last = -1;
    for (size_t i = 0; i + 1 < nbytes; i += 2) {
        uint16_t e = (uint16_t)(bytes[i] | (bytes[i + 1] << 8));
        if (e & FLOG_RAW_MARKER_BIT) {
            long t = (long)(e & FLOG_RAW_VALUE_MASK);
            if (k < cap) out[k++] = (e & FLOG_RAW_ABORT_BIT) ? (-100000 - t) : (-1 - t);
        } else if ((e & FLOG_RAW_TAG_MASK) == FLOG_RAW_TAG_REPEAT) {
            for (uint16_t r = 0; r < (e & FLOG_RAW_VALUE_MASK) && k < cap; ++r) {
                out[k++] = last;
            }
        } else {
            last = (long)(e & FLOG_RAW_VALUE_MASK);
            if (k < cap) out[k++] = last;
        }
    }
    return k;
}

/* ===========================================================================*/
static void test_struct_sizes(void)
{
    printf("\n-- on-flash struct sizes (mirrored by flightlog.py) --\n");
    ASSERT_TRUE(sizeof(flog_sector_hdr_t) == 16, "sector header is 16 bytes");
    ASSERT_TRUE(sizeof(flog_decision_t)   == 22, "decision payload is 22 bytes");
    ASSERT_TRUE(sizeof(flog_event_t)      ==  9, "event payload is 9 bytes");
    ASSERT_TRUE(sizeof(flog_audio_t)      == 12, "audio payload is 12 bytes");
    ASSERT_TRUE(FLOG_REC_MAX_SIZE <= 255,        "a whole record fits a u8-sized frame");
}

/* ===========================================================================*/
static void test_record_roundtrip(void)
{
    printf("\n-- record framing round-trip --\n");
    uint8_t buf[FLOG_REC_MAX_SIZE];
    const char *msg = "I (1234) app: callout 50 ft (state=4)";

    size_t n = flog_encode_record(buf, sizeof buf, FLOG_REC_TEXT, 0xA1B2C3D4u,
                                  msg, strlen(msg));
    ASSERT_TRUE(n == FLOG_REC_OVERHEAD + strlen(msg), "encoded size is header+payload+crc");

    uint8_t type = 0; uint32_t t = 0; const uint8_t *p = NULL; size_t len = 0;
    int used = flog_decode_record(buf, n, &type, &t, &p, &len);
    ASSERT_TRUE(used == (int)n,                    "decoder consumes the whole record");
    ASSERT_TRUE(type == FLOG_REC_TEXT,             "type survives");
    ASSERT_TRUE(t == 0xA1B2C3D4u,                  "timestamp survives (little-endian)");
    ASSERT_TRUE(len == strlen(msg) && p && memcmp(p, msg, len) == 0,
                "payload survives byte-for-byte");

    /* An empty payload is legal (e.g. a bare marker-style record). */
    n = flog_encode_record(buf, sizeof buf, FLOG_REC_EVENT, 7u, NULL, 0);
    ASSERT_TRUE(n == FLOG_REC_OVERHEAD, "zero-length payload frames to overhead only");
    ASSERT_TRUE(flog_decode_record(buf, n, NULL, NULL, NULL, NULL) == (int)n,
                "zero-length record decodes (outputs optional)");
}

/* ===========================================================================*/
static void test_record_rejects(void)
{
    printf("\n-- encoder refuses what cannot be framed --\n");
    uint8_t buf[FLOG_REC_MAX_SIZE + 8];
    uint8_t big[FLOG_REC_MAX_PAYLOAD + 1];
    memset(big, 0x5A, sizeof big);

    ASSERT_TRUE(flog_encode_record(buf, sizeof buf, 0xFF, 0, "x", 1) == 0,
                "type 0xFF refused (reads as erased flash)");
    ASSERT_TRUE(flog_encode_record(buf, sizeof buf, 0x00, 0, "x", 1) == 0,
                "type 0x00 refused (reserved)");
    ASSERT_TRUE(flog_encode_record(buf, sizeof buf, FLOG_REC_TEXT, 0, big, sizeof big) == 0,
                "oversized payload refused");
    ASSERT_TRUE(flog_encode_record(buf, 5, FLOG_REC_TEXT, 0, "x", 1) == 0,
                "too-small destination refused");
    ASSERT_TRUE(flog_encode_record(NULL, sizeof buf, FLOG_REC_TEXT, 0, "x", 1) == 0,
                "NULL destination refused");
    ASSERT_TRUE(flog_encode_record(buf, sizeof buf, FLOG_REC_TEXT, 0, NULL, 3) == 0,
                "NULL payload with non-zero length refused");
}

/* ===========================================================================*/
static void test_torn_and_erased(void)
{
    printf("\n-- torn writes and erased flash --\n");
    uint8_t buf[64];
    memset(buf, 0xFF, sizeof buf);

    /* Erased flash is a clean end, not an error. */
    ASSERT_TRUE(flog_decode_record(buf, sizeof buf, NULL, NULL, NULL, NULL) == 0,
                "0xFF type byte == end of data");
    ASSERT_TRUE(flog_decode_record(buf, 3, NULL, NULL, NULL, NULL) == 0,
                "fewer bytes than a header == end of data");

    /* A record whose last bytes never made it (power cut mid-write). */
    const char *msg = "hello";
    size_t n = flog_encode_record(buf, sizeof buf, FLOG_REC_TEXT, 42, msg, 5);
    uint8_t torn[64];
    memcpy(torn, buf, n);
    torn[n - 1] = 0xFF;                      /* CRC byte still erased          */
    ASSERT_TRUE(flog_decode_record(torn, n, NULL, NULL, NULL, NULL) == -1,
                "torn record (CRC never written) is rejected");

    /* A single flipped payload bit. */
    memcpy(torn, buf, n);
    torn[FLOG_REC_HDR_SIZE] ^= 0x04;
    ASSERT_TRUE(flog_decode_record(torn, n, NULL, NULL, NULL, NULL) == -1,
                "bit-flipped payload is rejected by the CRC");

    /* A length that runs off the end of the available bytes. */
    memcpy(torn, buf, n);
    ASSERT_TRUE(flog_decode_record(torn, n - 2, NULL, NULL, NULL, NULL) == -1,
                "record longer than the bytes left is rejected");
}

/* ===========================================================================*/
static void test_sector_header(void)
{
    printf("\n-- sector header --\n");
    flog_sector_hdr_t h = { FLOG_MAGIC, 1, 1, 0 };
    ASSERT_TRUE(flog_sector_hdr_valid(&h), "magic header is valid");
    memset(&h, 0xFF, sizeof h);
    ASSERT_TRUE(!flog_sector_hdr_valid(&h), "erased header is not valid");
    ASSERT_TRUE(!flog_sector_hdr_valid(NULL), "NULL header is not valid");

    const uint8_t *b = (const uint8_t *)&(flog_sector_hdr_t){ FLOG_MAGIC, 0, 0, 0 };
    ASSERT_TRUE(b[0] == 'F' && b[1] == 'L' && b[2] == 'G' && b[3] == '1',
                "magic reads \"FLG1\" in a hex dump");
}

/* ===========================================================================*/
static void test_raw_roundtrip(void)
{
    printf("\n-- RAW encoder: samples, runs, markers --\n");
    flog_raw_enc_t e;
    flog_raw_reset(&e);

    /* A parked drain with jitter, a marker, a blind stretch of sentinels,
     * an aborted drain, then an in-range value again.                        */
    const int drain1[] = { 305, 305, 306, 305, 305, 305 };
    for (size_t i = 0; i < sizeof drain1 / sizeof drain1[0]; ++i) {
        ASSERT_TRUE(flog_raw_sample(&e, drain1[i]), "sample accepted");
    }
    ASSERT_TRUE(flog_raw_marker(&e, 20000u, false), "marker accepted");
    for (int i = 0; i < 40; ++i) {
        (void)flog_raw_sample(&e, 16000);
    }
    ASSERT_TRUE(flog_raw_marker(&e, 20500u, true), "aborted marker accepted");
    (void)flog_raw_sample(&e, 9000);

    uint8_t bytes[FLOG_RAW_MAX_ENTRIES * 2];
    size_t nb = flog_raw_take(&e, bytes);

    /* 40 identical sentinels must collapse to a literal + one REPEAT entry. */
    ASSERT_TRUE(nb / 2 <= 12, "run-length encoding keeps a blind stretch tiny");

    long out[128];
    size_t k = raw_expand(bytes, nb, out, 128);
    size_t expect_n = 6 + 1 + 40 + 1 + 1;
    ASSERT_TRUE(k == expect_n, "expanded entry count matches everything pushed");

    bool ok = (k == expect_n);
    for (size_t i = 0; ok && i < 6; ++i) ok = (out[i] == drain1[i]);
    ASSERT_TRUE(ok, "drain samples reproduce in order");
    ASSERT_TRUE(k > 6 && out[6] == -1 - (long)(20000u & FLOG_RAW_VALUE_MASK),
                "drain marker carries t_ms mod 16384");
    ok = true;
    for (size_t i = 7; ok && i < 47; ++i) ok = (out[i] == 16000);
    ASSERT_TRUE(ok, "all 40 sentinels reproduce");
    ASSERT_TRUE(k > 47 && out[47] == -100000 - (long)(20500u & FLOG_RAW_VALUE_MASK),
                "aborted marker flag survives");
    ASSERT_TRUE(k > 48 && out[48] == 9000, "value after the marker reproduces");

    /* take() resets: the next record starts with a literal, never a REPEAT. */
    (void)flog_raw_sample(&e, 9000);
    nb = flog_raw_take(&e, bytes);
    ASSERT_TRUE(nb == 2 && (bytes[1] & 0xC0) == 0,
                "first entry after a record boundary is a literal SAMPLE");
}

/* ===========================================================================*/
static void test_raw_capacity(void)
{
    printf("\n-- RAW encoder capacity + clamping --\n");
    flog_raw_enc_t e;
    flog_raw_reset(&e);

    /* Alternating values defeat RLE, so the buffer genuinely fills. */
    size_t pushed = 0;
    while (flog_raw_sample(&e, (int)(pushed & 1u) + 100)) {
        ++pushed;
        if (pushed > 10000) break;           /* runaway guard                 */
    }
    ASSERT_TRUE(pushed > 0 && pushed < FLOG_RAW_MAX_ENTRIES,
                "encoder refuses once fewer than 2 entries of room remain");
    ASSERT_TRUE(!flog_raw_marker(&e, 1u, false),
                "marker also refused when full (caller must take first)");

    uint8_t bytes[FLOG_RAW_MAX_ENTRIES * 2];
    size_t nb = flog_raw_take(&e, bytes);
    ASSERT_TRUE(nb <= FLOG_REC_MAX_PAYLOAD, "a full take still fits one record");

    /* A long run ending exactly at a full buffer still flushes its REPEAT. */
    flog_raw_reset(&e);
    while (flog_raw_room(&e) > 2u) {
        (void)flog_raw_sample(&e, (int)(flog_raw_room(&e) & 1u) + 7);
    }
    for (int i = 0; i < 50; ++i) {
        (void)flog_raw_sample(&e, 5);        /* may only extend the run       */
    }
    nb = flog_raw_take(&e, bytes);
    long out[400];
    size_t k = raw_expand(bytes, nb, out, 400);
    ASSERT_TRUE(k > 0 && out[k - 1] == 5, "pending run is flushed into the reserved slot");

    /* Out-of-range values clamp into the 14-bit field instead of corrupting tags. */
    flog_raw_reset(&e);
    (void)flog_raw_sample(&e, -5);
    (void)flog_raw_sample(&e, 99999);
    nb = flog_raw_take(&e, bytes);
    k = raw_expand(bytes, nb, out, 400);
    ASSERT_TRUE(k == 2 && out[0] == 0 && out[1] == 16383,
                "negative clamps to 0, oversize clamps to 16383");
}

int main(void)
{
    printf("== flight recorder codec ==\n");
    test_struct_sizes();
    test_record_roundtrip();
    test_record_rejects();
    test_torn_and_erased();
    test_sector_header();
    test_raw_roundtrip();
    test_raw_capacity();
    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
