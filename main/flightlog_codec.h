/**
 * @file    flightlog_codec.h
 * @brief   PURE on-flash format of the flight recorder: record framing, sector
 *          headers, and the run-length raw-sensor encoding.
 *
 * @details The flight recorder (flightlog.c) exists because two flights came
 *          back silent and every theory since has been reconstructed from first
 *          principles instead of observed. It records what the box actually saw
 *          and decided — every raw SF30 sample, the logic task's decision state,
 *          audio-path events, and every console line — into a dedicated flash
 *          partition, so the next flight can be replayed on the bench.
 *
 *          This header is the SINGLE definition of that format. It includes no
 *          ESP-IDF headers, so the host test suite exercises the exact encoder
 *          the firmware ships, and tools/flightlog/flightlog.py mirrors it
 *          byte-for-byte for decoding. Change one, change the other.
 *
 *          ----------------------------------------------------------------------
 *          PARTITION LAYOUT
 *          ----------------------------------------------------------------------
 *          The partition is a ring of FLOG_SECTOR_SIZE sectors. Each sector that
 *          holds data starts with a flog_sector_hdr_t, followed by back-to-back
 *          records. The unused tail of a sector stays erased (0xFF), which the
 *          decoder reads as "end of this sector". Sectors are ordered by their
 *          monotonic header sequence number, so the ring's wrap point is found
 *          without any separate index that could itself be corrupted.
 *
 *          ----------------------------------------------------------------------
 *          RECORD FRAMING (all little-endian)
 *          ----------------------------------------------------------------------
 *              offset  size  field
 *              0       1     type     (flog_rec_type_t; 0xFF == erased/end)
 *              1       1     len      (payload bytes, <= FLOG_REC_MAX_PAYLOAD)
 *              2       4     t_ms     (milliseconds since this boot)
 *              6       len   payload
 *              6+len   1     crc8     (poly 0x07 over bytes 0 .. 5+len)
 *
 *          The CRC makes a record torn by a power cut (the pilot killing the
 *          master switch mid-write) detectable instead of decoding as garbage.
 */
#ifndef LIDARAGL_FLIGHTLOG_CODEC_H
#define LIDARAGL_FLIGHTLOG_CODEC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---- Packing ------------------------------------------------------------ */
/*  Every on-flash struct is byte-packed so its layout is identical everywhere
 *  it is read. MinGW (the host test compiler on Windows) defaults to the MSVC
 *  struct layout, under which plain `packed` still pads some members, so it
 *  also needs gcc_struct; that attribute is x86-only, hence the guard.        */
#if defined(__MINGW32__) || defined(__MINGW64__)
#define FLOG_PACKED __attribute__((packed, gcc_struct))
#else
#define FLOG_PACKED __attribute__((packed))
#endif

/* ---- Geometry ----------------------------------------------------------- */

/** Flash erase unit; every on-flash structure is sized around it. */
#define FLOG_SECTOR_SIZE      4096u

/** Sector header magic: ASCII "FLG1" read as a little-endian u32. */
#define FLOG_MAGIC            0x31474C46u

/** Bytes before the payload: type + len + t_ms. */
#define FLOG_REC_HDR_SIZE     6u

/** Header plus the trailing CRC byte — the fixed cost of every record. */
#define FLOG_REC_OVERHEAD     (FLOG_REC_HDR_SIZE + 1u)

/** Largest payload a record may carry (len is a u8; margin kept below 255). */
#define FLOG_REC_MAX_PAYLOAD  240u

/** Largest complete record, for sizing stack buffers. */
#define FLOG_REC_MAX_SIZE     (FLOG_REC_MAX_PAYLOAD + FLOG_REC_OVERHEAD)

/* ---- Record types ------------------------------------------------------- */

/** What a record's payload holds. Values are part of the on-flash format. */
typedef enum {
    FLOG_REC_TEXT     = 1,  /**< One console line (ESP_LOG output), no newline. */
    FLOG_REC_RAW      = 2,  /**< Raw SF30 samples + drain markers (u16 entries). */
    FLOG_REC_DECISION = 3,  /**< flog_decision_t — the logic task's view.       */
    FLOG_REC_EVENT    = 4,  /**< flog_event_t — a discrete occurrence.          */
    FLOG_REC_AUDIO    = 5,  /**< flog_audio_t — 1 Hz audio-engine snapshot.     */
} flog_rec_type_t;

/* ---- Sector header ------------------------------------------------------ */

/**
 * @brief Header written at the start of every sector that holds data.
 *
 * @details seq increases by one for every sector ever started, across boots,
 *          so the newest sector is simply the one with the highest seq — the
 *          ring needs no separate write pointer. boot_id increases once per
 *          power-up so the decoder can split the log into sessions; t_ms is the
 *          boot-relative time the sector was opened.
 */
typedef struct FLOG_PACKED {
    uint32_t magic;    /**< FLOG_MAGIC, or anything else == unused sector.  */
    uint32_t seq;      /**< Monotonic sector sequence number.               */
    uint32_t boot_id;  /**< Power-up session this sector belongs to.        */
    uint32_t t_ms;     /**< Boot-relative time the sector was opened.       */
} flog_sector_hdr_t;

/* ---- DECISION payload --------------------------------------------------- */

/*  flog_decision_t.flags — one bit per boolean the logic task decided on. */
#define FLOG_F_VALID        0x01u  /**< Sample was a fresh, accepted range.      */
#define FLOG_F_FRESH        0x02u  /**< Decision ran on a new sample seq.        */
#define FLOG_F_TRACK_BREAK  0x04u  /**< Filter reported a broken track.         */
#define FLOG_F_TRACKING     0x08u  /**< Filter says the sensor sees the ground. */
#define FLOG_F_ARMED        0x10u  /**< Callout ladder armed (sm.armed).         */
#define FLOG_F_TONE_ACTIVE  0x20u  /**< State machine wants the tone.           */
#define FLOG_F_TONE_ON      0x40u  /**< Tone actually published (after mute).   */
#define FLOG_F_SLEEP        0x80u  /**< Light-sleep / audio suspend permitted.  */

/*  flog_decision_t.flags2 */
#define FLOG_F2_STALE       0x01u  /**< Data older than LOST_SIGNAL_MUTE_MS.     */
#define FLOG_F2_POSRATE     0x02u  /**< "Positive rate" fired this tick.        */
#define FLOG_F2_STALE_KICK  0x04u  /**< Decision forced by the stale watchdog.  */
#define FLOG_F2_TRK_SHIFT   3u     /**< Bits 3-4: the range tracker's state:     */
#define FLOG_F2_TRK_MASK    0x18u  /**< rf_state_t SEARCH/TRACK/COAST/LOST 0..3. */
#define FLOG_F2_REENTRY     0x20u  /**< The break was a flyable re-entry.       */
#define FLOG_F2_LATE_RUNG   0x40u  /**< The fired rung came from the late-rung  */
                                   /**< window (sm_reanchor), not a crossing.   */

/**
 * @brief One logic-task decision tick, exactly as it was taken.
 *
 * @details Everything needed to answer "why did the box say nothing?" from a
 *          log: the published range and the AGL it became, the state and the
 *          per-rung armed mask, what fired, and whether the tone and the sleep
 *          policy were on. 22 bytes.
 */
typedef struct FLOG_PACKED {
    uint32_t seq;         /**< Sensor sample sequence number.                  */
    float    range_ft;    /**< Filtered range published by the sensor task.    */
    float    agl_ft;      /**< AGL handed to sm_step (after ground / scaling). */
    int16_t  trend_dfps;  /**< Smoothed vertical rate, tenths of ft/s (+up).   */
    uint16_t armed_mask;  /**< Per-rung armed bits (LSB == callouts[0]).       */
    uint16_t dt_ms;       /**< Elapsed time credited to this decision.         */
    uint8_t  state;       /**< sm_state_t after the step.                      */
    int8_t   fired;       /**< Callout index fired this tick, or -1.           */
    uint8_t  flags;       /**< FLOG_F_* bits.                                  */
    uint8_t  flags2;      /**< FLOG_F2_* bits.                                 */
} flog_decision_t;

/* ---- EVENT payload ------------------------------------------------------ */

/** Discrete occurrences worth a timestamp. Values are part of the format. */
typedef enum {
    FLOG_EV_BOOT              = 1,  /**< a = esp_reset_reason(), b = boot_id.     */
    FLOG_EV_CALLOUT_DEQUEUED  = 2,  /**< a = callout id, b = 1 if it was STARTED. */
    FLOG_EV_CALLOUT_DISCARDED = 3,  /**< a = id; dropped while channel suspended.*/
    FLOG_EV_CLIP_END          = 4,  /**< a = samples played.                     */
    FLOG_EV_AUDIO_SUSPEND     = 5,  /**< Audio channel torn down (light-sleep).  */
    FLOG_EV_AUDIO_RESUME      = 6,  /**< Audio channel brought back up.          */
    FLOG_EV_ALERT_START       = 7,  /**< Sensor-failure chirp armed.             */
    FLOG_EV_LOG_DROPPED       = 8,  /**< a = bytes the RAM ring had to drop.     */
    FLOG_EV_CLIP_MISSING      = 9,  /**< a = id of a clip with no audio data.    */
    FLOG_EV_CALLOUT_STALE     = 10, /**< a = stale id skipped, b = newer id kept.*/
} flog_event_code_t;

/** A discrete event with two free integer arguments. 9 bytes. */
typedef struct FLOG_PACKED {
    uint8_t code;  /**< flog_event_code_t. */
    int32_t a;     /**< First argument (meaning per code). */
    int32_t b;     /**< Second argument (meaning per code). */
} flog_event_t;

/* ---- AUDIO payload ------------------------------------------------------ */

/*  flog_audio_t.flags */
#define FLOG_A_RUNNING      0x01u  /**< I2S channel enabled.                    */
#define FLOG_A_CLIP         0x02u  /**< A voice clip is mid-playback.           */
#define FLOG_A_CALLOUTS_EN  0x04u  /**< Audio mode plays callouts.              */
#define FLOG_A_TONE_EN      0x08u  /**< Audio mode plays the tone.              */
#define FLOG_A_TONE_REQ     0x10u  /**< Logic task asked for the tone.          */
#define FLOG_A_SUSPEND_REQ  0x20u  /**< Logic task asked for a suspend.         */
#define FLOG_A_ALERT        0x40u  /**< Sensor-failure chirp hold active.       */

/** 1 Hz snapshot of the audio engine — proves what reached the DAC. 12 bytes. */
typedef struct FLOG_PACKED {
    uint8_t  flags;             /**< FLOG_A_* bits.                             */
    uint8_t  queue_depth;       /**< Callout ids waiting in q_callouts.         */
    int16_t  tone_agl_dft;      /**< AGL the tone is tracking, tenths of a ft.  */
    uint16_t tone_gain_milli;   /**< Current tone gain x1000 (0 == silent).     */
    uint16_t voice_gain_milli;  /**< Pilot voice-volume gain x1000.             */
    uint16_t write_stalls;      /**< I2S write stalls since boot (saturating).  */
    uint16_t short_writes;      /**< Dropped frame tails since boot (saturating).*/
} flog_audio_t;

/* ---- RAW payload: u16 entries ------------------------------------------- */
/*  Each entry is one little-endian u16, classified by its top two bits. The
 *  SF30/C wire value is 14-bit (0..16383 cm), which leaves those bits free:
 *
 *    0b00xx xxxx xxxx xxxx  SAMPLE  : a decoded distance in cm (0..16383).
 *    0b01xx xxxx xxxx xxxx  REPEAT  : the previous SAMPLE again, N more times.
 *    0b1Axx xxxx xxxx xxxx  MARKER  : the filter FINALIZED its drain here.
 *                                     A = 1 if the drain was discarded for a
 *                                     UART hardware error; low 14 bits are the
 *                                     finalize time's t_ms modulo 16384.
 *
 *  REPEAT keeps a blind cruise (thousands of identical lost-signal sentinels)
 *  nearly free. MARKER keeps the drain boundaries, from which the tracker
 *  derives every sample's timestamp (a drain's samples are spread uniformly
 *  across its interval), so a log can be replayed through range_filter.c and
 *  reproduce the firmware's decisions to the millisecond. Its time is recovered by
 *  unwrapping against the record's own t_ms (records are emitted well inside
 *  the 16.4 s window). Each record is self-contained: the first sample after a
 *  record boundary is always a literal SAMPLE, never a REPEAT.                 */
#define FLOG_RAW_TAG_MASK     0xC000u
#define FLOG_RAW_TAG_SAMPLE   0x0000u
#define FLOG_RAW_TAG_REPEAT   0x4000u
#define FLOG_RAW_MARKER_BIT   0x8000u
#define FLOG_RAW_ABORT_BIT    0x4000u
#define FLOG_RAW_VALUE_MASK   0x3FFFu

/** Entries one RAW record can hold (2 bytes each, within the payload cap). */
#define FLOG_RAW_MAX_ENTRIES  (FLOG_REC_MAX_PAYLOAD / 2u)

/**
 * @brief Streaming encoder state for RAW entries.
 *
 * @details Owned by a single producer (the sensor task). Samples and markers
 *          go in, finished entry arrays come out via flog_raw_take().
 */
typedef struct {
    uint16_t entries[FLOG_RAW_MAX_ENTRIES]; /**< Encoded entries so far.        */
    size_t   n;                             /**< Entries used.                  */
    uint16_t last;                          /**< Last literal SAMPLE value.     */
    bool     have_last;                     /**< False at a record boundary.    */
    uint16_t rep;                           /**< Pending REPEAT count.          */
} flog_raw_enc_t;

/* ---- API ---------------------------------------------------------------- */

/**
 * @brief CRC-8 (polynomial 0x07, init 0x00) over a byte span.
 * @param data  Bytes to checksum (may be NULL only when @p n is 0).
 * @param n     Byte count.
 * @return      The CRC.
 */
uint8_t flog_crc8(const uint8_t *data, size_t n);

/**
 * @brief Frame one record into @p out.
 *
 * @param out      Destination buffer.
 * @param cap      Capacity of @p out in bytes.
 * @param type     flog_rec_type_t value (must not be 0 or 0xFF).
 * @param t_ms     Boot-relative timestamp.
 * @param payload  Payload bytes (may be NULL only when @p len is 0).
 * @param len      Payload length, <= FLOG_REC_MAX_PAYLOAD.
 * @return         Total bytes written, or 0 if any argument is unusable.
 */
size_t flog_encode_record(uint8_t *out, size_t cap, uint8_t type, uint32_t t_ms,
                          const void *payload, size_t len);

/**
 * @brief Parse the record at the start of @p in.
 *
 * @param in       Bytes to parse.
 * @param avail    Bytes available at @p in.
 * @param[out] type     Record type.
 * @param[out] t_ms     Timestamp.
 * @param[out] payload  Points INTO @p in at the payload.
 * @param[out] len      Payload length.
 * @return  Bytes consumed (> 0) on success; 0 at the erased end of a sector
 *          (0xFF type byte, or too few bytes left); -1 if the bytes are not a
 *          valid record (bad length or CRC) — the caller should stop reading
 *          this sector, since framing can no longer be trusted.
 */
int flog_decode_record(const uint8_t *in, size_t avail, uint8_t *type,
                       uint32_t *t_ms, const uint8_t **payload, size_t *len);

/**
 * @brief Whether a sector header marks a sector that holds recorder data.
 * @param h  Header read from flash (may be NULL).
 * @return   True for a FLOG_MAGIC header.
 */
bool flog_sector_hdr_valid(const flog_sector_hdr_t *h);

/** @brief Reset a RAW encoder to empty (the next sample is a literal). */
void flog_raw_reset(flog_raw_enc_t *e);

/**
 * @brief Entries that can still be appended before the record must be taken.
 * @details Every append can emit up to TWO entries (a pending REPEAT flush plus
 *          the new one), so callers flush when this drops below 2.
 */
size_t flog_raw_room(const flog_raw_enc_t *e);

/**
 * @brief Append one decoded distance.
 * @param e   Encoder.
 * @param cm  Wire value in cm; clamped into the 14-bit field.
 * @return    False (and nothing appended) if fewer than 2 entries of room.
 */
bool flog_raw_sample(flog_raw_enc_t *e, int cm);

/**
 * @brief Append a drain-finalize marker.
 * @param e        Encoder.
 * @param t_ms     Boot-relative time of the finalize.
 * @param aborted  True if the drain was discarded for a UART hardware error.
 * @return         False (and nothing appended) if fewer than 2 entries of room.
 */
bool flog_raw_marker(flog_raw_enc_t *e, uint32_t t_ms, bool aborted);

/**
 * @brief Flush any pending REPEAT, copy the entries out as little-endian bytes,
 *        and reset the encoder for the next record.
 * @param e    Encoder.
 * @param out  Destination (>= FLOG_RAW_MAX_ENTRIES * 2 bytes).
 * @return     Bytes written to @p out (0 if the encoder was empty).
 */
size_t flog_raw_take(flog_raw_enc_t *e, uint8_t *out);

#endif /* LIDARAGL_FLIGHTLOG_CODEC_H */
