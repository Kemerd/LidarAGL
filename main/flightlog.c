/**
 * @file    flightlog.c
 * @brief   Flight recorder implementation: RAM ring, flash ring, writer task,
 *          and the ESP_LOG capture hook. See flightlog.h for the safety contract.
 */

#include "flightlog.h"
#include "flightlog_codec.h"
#include "config.h"
#include "audio.h"          /* audio_is_quiet(): erase only while nothing sounds */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"     /* esp_reset_reason() for the boot event */

static const char *TAG = "flog";

/*  The RAM ring indexes with a mask, so its size must be a power of two. */
_Static_assert((FLOG_RING_BYTES & (FLOG_RING_BYTES - 1u)) == 0u,
               "FLOG_RING_BYTES must be a power of two");
_Static_assert(FLOG_STAGE_BYTES >= FLOG_REC_MAX_SIZE,
               "the flash staging buffer must hold at least one whole record");
_Static_assert(FLOG_TEXT_MAX <= FLOG_REC_MAX_PAYLOAD,
               "a captured console line must fit in one record");

/* ===========================================================================
 *  State
 * ===========================================================================*/

/* ---- Flash ring (writer task only, after init) --------------------------- */
static const esp_partition_t *s_part      = NULL;  /* NULL == recorder disabled */
static uint32_t               s_n_sectors = 0;
static uint32_t               s_cur       = 0;     /* sector being written      */
static bool                   s_cur_open  = false; /* s_cur has a header yet?   */
static uint32_t               s_cur_off   = 0;     /* next free byte in s_cur   */
static uint32_t               s_next_seq  = 1;     /* seq for the next header   */
static uint32_t               s_boot_id   = 1;     /* this power-up's session   */
static uint32_t               s_runway    = 0;     /* verified-blank sectors    */
                                                   /* after the current one     */

/* ---- Staging: bytes waiting to be written at s_stage_addr ---------------- */
static uint8_t  s_stage[FLOG_STAGE_BYTES];
static uint32_t s_stage_n    = 0;
static uint32_t s_stage_addr = 0;
static int64_t  s_stage_t0   = 0;   /* when the oldest staged byte arrived (us) */

/* ---- A record popped from the RAM ring that has not found a home yet ----- */
static uint8_t  s_hold[FLOG_REC_MAX_SIZE];
static size_t   s_hold_n = 0;

/* ---- Scratch for blank-checking a sector (writer task only) -------------- */
static uint8_t  s_scratch[256];

/* ---- RAM ring (any task, under s_mux) ------------------------------------ */
static uint8_t       s_ring[FLOG_RING_BYTES];
static uint32_t      s_ring_head = 0;       /* monotonic: bytes ever written   */
static uint32_t      s_ring_tail = 0;       /* monotonic: bytes ever consumed  */
static uint32_t      s_dropped   = 0;       /* bytes refused since last report */
static portMUX_TYPE  s_mux       = portMUX_INITIALIZER_UNLOCKED;

/* ---- Lifecycle ----------------------------------------------------------- */
static volatile bool s_ring_live     = false;  /* producers may append         */
static volatile bool s_writer_armed  = false;  /* writer may touch flash       */

/* ---- RAW encoder (single producer: the sensor read path) ----------------- */
static flog_raw_enc_t s_raw;
static uint32_t       s_raw_t0_ms   = 0;
static bool           s_raw_have_t0 = false;

/* ---- The console writer we chain to after capturing a line -------------- */
static vprintf_like_t s_orig_vprintf = NULL;

/* ===========================================================================
 *  Small helpers
 * ===========================================================================*/

/*  Boot-relative milliseconds, the timestamp every record carries. */
static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ===========================================================================
 *  RAM ring — the only structure producers touch
 * ===========================================================================*/

/*  Append one complete, already-framed record. Never blocks: a full ring drops
 *  the record and counts it. The copy is short (<= FLOG_REC_MAX_SIZE bytes),
 *  so the spinlock is held for a few microseconds at most.                   */
static void ring_push(const uint8_t *rec, size_t n)
{
    if (!s_ring_live || rec == NULL || n == 0u || n > FLOG_REC_MAX_SIZE) {
        return;
    }

    taskENTER_CRITICAL(&s_mux);
    uint32_t used = s_ring_head - s_ring_tail;
    if ((uint32_t)FLOG_RING_BYTES - used < n) {
        s_dropped += (uint32_t)n;            /* full: drop, never wait         */
    } else {
        /* Copy in at most two pieces around the wrap point. */
        uint32_t at    = s_ring_head & (FLOG_RING_BYTES - 1u);
        uint32_t first = (uint32_t)FLOG_RING_BYTES - at;
        if (first > n) {
            first = (uint32_t)n;
        }
        memcpy(&s_ring[at], rec, first);
        if (n > first) {
            memcpy(&s_ring[0], rec + first, n - first);
        }
        s_ring_head += (uint32_t)n;
    }
    taskEXIT_CRITICAL(&s_mux);
}

/*  Pop the oldest whole record into @p out; returns its size, 0 if empty.
 *  Records are pushed atomically, so a partial record is never visible.      */
static size_t ring_pop_record(uint8_t *out)
{
    size_t got = 0;

    taskENTER_CRITICAL(&s_mux);
    uint32_t used = s_ring_head - s_ring_tail;
    if (used >= FLOG_REC_OVERHEAD) {
        uint32_t tail  = s_ring_tail & (FLOG_RING_BYTES - 1u);
        size_t   plen  = s_ring[(tail + 1u) & (FLOG_RING_BYTES - 1u)];
        size_t   total = FLOG_REC_OVERHEAD + plen;
        if (total > FLOG_REC_MAX_SIZE) {
            /* Cannot happen (every push is a framed record), but if the ring
             * is ever inconsistent, resynchronise rather than emit garbage.  */
            s_ring_tail = s_ring_head;
        } else if (used >= total) {
            uint32_t first = (uint32_t)FLOG_RING_BYTES - tail;
            if (first > total) {
                first = (uint32_t)total;
            }
            memcpy(out, &s_ring[tail], first);
            if (total > first) {
                memcpy(out + first, &s_ring[0], total - first);
            }
            s_ring_tail += (uint32_t)total;
            got = total;
        }
    }
    taskEXIT_CRITICAL(&s_mux);

    return got;
}

/*  Frame a payload and append it. The frame is built on the caller's stack
 *  (<= FLOG_REC_MAX_SIZE bytes) so any task can call this concurrently.      */
static void push_record(uint8_t type, const void *payload, size_t len)
{
    if (!s_ring_live) {
        return;
    }
    uint8_t rec[FLOG_REC_MAX_SIZE];
    size_t n = flog_encode_record(rec, sizeof rec, type, now_ms(), payload, len);
    if (n > 0u) {
        ring_push(rec, n);
    }
}

/* ===========================================================================
 *  ESP_LOG capture
 * ===========================================================================*/

/*  Installed with esp_log_set_vprintf(). With LOG_VERSION_1 every ESP_LOGx line
 *  arrives here as ONE call, already carrying its "I (1234) tag: " prefix, so a
 *  line maps to exactly one TEXT record. The console still gets every line: we
 *  format a private copy, then chain to the original writer untouched.       */
static int flog_vprintf(const char *fmt, va_list ap)
{
    if (s_ring_live && fmt != NULL && !xPortInIsrContext()) {
        char line[FLOG_TEXT_MAX + 1];
        va_list cp;
        va_copy(cp, ap);
        int n = vsnprintf(line, sizeof line, fmt, cp);
        va_end(cp);

        if (n > 0) {
            size_t len = ((size_t)n < sizeof line) ? (size_t)n : sizeof line - 1u;
            /* The decoder prints one record per line; drop the line ending. */
            while (len > 0u && (line[len - 1u] == '\n' || line[len - 1u] == '\r')) {
                --len;
            }
            if (len > 0u) {
                push_record(FLOG_REC_TEXT, line, len);
            }
        }
    }
    return (s_orig_vprintf != NULL) ? s_orig_vprintf(fmt, ap) : vprintf(fmt, ap);
}

/* ===========================================================================
 *  Flash side — writer task only
 * ===========================================================================*/

/*  Write the staged bytes. On a flash error the bytes are abandoned rather
 *  than retried forever: losing a second of log beats wedging the writer.    */
static void stage_flush(void)
{
    if (s_stage_n == 0u || s_part == NULL) {
        return;
    }
    (void)esp_partition_write(s_part, s_stage_addr, s_stage, s_stage_n);
    s_stage_n  = 0;
    s_stage_t0 = 0;
}

/*  Queue bytes for the current sector at s_cur_off. The caller guarantees
 *  they fit inside the sector, so staged bytes are always contiguous.        */
static void stage_append(const uint8_t *b, size_t n)
{
    if (s_stage_n + n > sizeof s_stage) {
        stage_flush();
    }
    if (s_stage_n == 0u) {
        s_stage_addr = s_cur * FLOG_SECTOR_SIZE + s_cur_off;
        s_stage_t0   = esp_timer_get_time();
    }
    memcpy(&s_stage[s_stage_n], b, n);
    s_stage_n += (uint32_t)n;
    s_cur_off += (uint32_t)n;
}

/*  True when the whole sector reads erased (0xFF). Reading is cheap; it lets
 *  a sector that is already blank skip its erase entirely.                   */
static bool sector_is_blank(uint32_t idx)
{
    uint32_t base = idx * FLOG_SECTOR_SIZE;
    for (uint32_t off = 0; off < FLOG_SECTOR_SIZE; off += sizeof s_scratch) {
        if (esp_partition_read(s_part, base + off, s_scratch, sizeof s_scratch) != ESP_OK) {
            return false;
        }
        for (size_t i = 0; i < sizeof s_scratch; ++i) {
            if (s_scratch[i] != 0xFFu) {
                return false;
            }
        }
    }
    return true;
}

/*  Make a sector writable: verify blank, else erase. False on flash error. */
static bool sector_prepare(uint32_t idx)
{
    if (sector_is_blank(idx)) {
        return true;
    }
    return esp_partition_erase_range(s_part, idx * FLOG_SECTOR_SIZE,
                                      FLOG_SECTOR_SIZE) == ESP_OK;
}

/*  Index of the sector the recorder will open next. */
static inline uint32_t next_sector_index(void)
{
    return s_cur_open ? (s_cur + 1u) % s_n_sectors : s_cur;
}

/*  Extend the pre-erased runway by one sector. Called only while audio is
 *  quiet. Never laps the sector currently being written.                     */
static void runway_extend_one(void)
{
    if (s_runway >= s_n_sectors - 1u) {
        return;
    }
    uint32_t idx = (next_sector_index() + s_runway) % s_n_sectors;
    if (s_cur_open && idx == s_cur) {
        return;
    }
    if (sector_prepare(idx)) {
        s_runway++;
    }
}

/*  Close the current sector and open the next one with a fresh header.
 *  @p may_erase says whether an erase is acceptable right now; without a
 *  runway sector and without permission the call fails and the caller waits. */
static bool sector_open_next(bool may_erase)
{
    stage_flush();

    uint32_t next = next_sector_index();
    if (s_runway > 0u) {
        s_runway--;                          /* pre-erased: free to use       */
    } else if (!may_erase || !sector_prepare(next)) {
        return false;                        /* wait for a quieter moment     */
    }

    s_cur      = next;
    s_cur_open = true;
    s_cur_off  = 0;

    flog_sector_hdr_t h = {
        .magic   = FLOG_MAGIC,
        .seq     = s_next_seq++,
        .boot_id = s_boot_id,
        .t_ms    = now_ms(),
    };
    stage_append((const uint8_t *)&h, sizeof h);
    return true;
}

/* ---- The writer task ----------------------------------------------------- */
static void flightlog_writer_task(void *arg)
{
    (void)arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(FLOG_WRITER_PERIOD_MS));

        /* Until the flight tasks are running, only buffer (see flightlog.h). */
        if (!s_writer_armed || s_part == NULL) {
            continue;
        }

        /* One read of the audio state per pass: every erase decision below is
         * made against the same answer.                                      */
        bool quiet = audio_is_quiet();

        /* --- Report any drops (the report itself goes through the ring) --- */
        uint32_t dropped;
        taskENTER_CRITICAL(&s_mux);
        dropped   = s_dropped;
        s_dropped = 0;
        taskEXIT_CRITICAL(&s_mux);
        if (dropped > 0u) {
            flightlog_event(FLOG_EV_LOG_DROPPED, (int32_t)dropped, 0);
        }

        /* --- Move whole records from RAM to the staging buffer ------------ */
        for (;;) {
            if (s_hold_n == 0u) {
                s_hold_n = ring_pop_record(s_hold);
                if (s_hold_n == 0u) {
                    break;                           /* ring drained          */
                }
            }
            /* Records never straddle sectors: open a new one when it won't fit. */
            if (!s_cur_open || s_cur_off + s_hold_n > FLOG_SECTOR_SIZE) {
                if (!sector_open_next(quiet)) {
                    break;                           /* keep s_hold for later */
                }
            }
            stage_append(s_hold, s_hold_n);
            s_hold_n = 0;
        }

        /* --- Bound how long bytes sit in RAM (power can vanish any time) --- */
        if (s_stage_n > 0u &&
            esp_timer_get_time() - s_stage_t0 >= (int64_t)FLOG_FLUSH_MS * 1000) {
            stage_flush();
        }

        /* --- Grow the runway, one sector per pass, only while silent ------ */
        if (quiet && s_runway < FLOG_RUNWAY_SECTORS) {
            runway_extend_one();
        }
    }
}

/* ===========================================================================
 *  Public API
 * ===========================================================================*/

void flightlog_init(void)
{
    /* --- Find the partition; without it the recorder is simply off -------- */
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_ANY,
                                      FLOG_PARTITION_LABEL);
    if (s_part == NULL) {
        ESP_LOGW(TAG, "no '%s' partition — flight recorder DISABLED",
                 FLOG_PARTITION_LABEL);
        return;
    }
    s_n_sectors = s_part->size / FLOG_SECTOR_SIZE;
    if (s_n_sectors < 2u) {
        ESP_LOGW(TAG, "'%s' partition too small — flight recorder DISABLED",
                 FLOG_PARTITION_LABEL);
        s_part = NULL;
        return;
    }

    /* --- Resume after the newest sector any earlier session wrote --------
     *  Headers are the whole index: the highest seq is the newest sector,
     *  compared with wrap-safe signed arithmetic. A sector without the magic
     *  is unused or stale and is ignored.                                    */
    bool     have     = false;
    uint32_t newest   = 0;
    uint32_t max_seq  = 0;
    uint32_t max_boot = 0;
    for (uint32_t i = 0; i < s_n_sectors; ++i) {
        flog_sector_hdr_t h;
        if (esp_partition_read(s_part, i * FLOG_SECTOR_SIZE, &h, sizeof h) != ESP_OK ||
            !flog_sector_hdr_valid(&h)) {
            continue;
        }
        if (!have || (int32_t)(h.seq - max_seq) > 0) {
            max_seq = h.seq;
            newest  = i;
        }
        if (!have || (int32_t)(h.boot_id - max_boot) > 0) {
            max_boot = h.boot_id;
        }
        have = true;
    }
    s_cur      = have ? (newest + 1u) % s_n_sectors : 0u;
    s_cur_open = false;
    s_next_seq = have ? max_seq + 1u : 1u;
    s_boot_id  = have ? max_boot + 1u : 1u;
    s_runway   = 0;

    /* --- Open the RAM side and start capturing --------------------------- */
    flog_raw_reset(&s_raw);
    s_ring_live    = true;
    s_orig_vprintf = esp_log_set_vprintf(flog_vprintf);

    flightlog_event(FLOG_EV_BOOT, (int32_t)esp_reset_reason(), (int32_t)s_boot_id);

    /* --- The writer idles until flightlog_start_writer() ----------------- */
    if (xTaskCreatePinnedToCore(flightlog_writer_task, "flog", FLOG_TASK_STACK,
                                NULL, FLOG_TASK_PRIO, NULL, FLOG_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "writer task create failed — recording to RAM only");
    }

    ESP_LOGI(TAG, "flight recorder: %u KB, %u sectors, session #%u, resuming at "
                  "sector %u (seq %u)",
             (unsigned)(s_part->size / 1024u), (unsigned)s_n_sectors,
             (unsigned)s_boot_id, (unsigned)s_cur, (unsigned)s_next_seq);
}

void flightlog_start_writer(void)
{
    s_writer_armed = true;
}

/*  Emit the RAW entries gathered so far as one record. */
static void raw_emit(void)
{
    uint8_t buf[FLOG_RAW_MAX_ENTRIES * 2u];
    size_t n = flog_raw_take(&s_raw, buf);
    if (n > 0u) {
        push_record(FLOG_REC_RAW, buf, n);
    }
    s_raw_have_t0 = false;
}

void flightlog_raw_sample(int cm)
{
    if (!s_ring_live) {
        return;
    }
    if (flog_raw_room(&s_raw) < 2u) {
        raw_emit();                          /* a very long drain: split it   */
    }
    if (!s_raw_have_t0) {
        s_raw_t0_ms   = now_ms();
        s_raw_have_t0 = true;
    }
    (void)flog_raw_sample(&s_raw, cm);
}

void flightlog_raw_finalize(bool aborted)
{
    if (!s_ring_live) {
        return;
    }
    uint32_t t = now_ms();
    if (flog_raw_room(&s_raw) < 2u) {
        raw_emit();
    }
    if (!s_raw_have_t0) {
        s_raw_t0_ms   = t;
        s_raw_have_t0 = true;
    }
    (void)flog_raw_marker(&s_raw, t, aborted);

    /*  Emit on a drain boundary once the record is old enough or nearly full,
     *  so almost every record ends on a marker and replays cleanly.          */
    if (t - s_raw_t0_ms >= FLOG_RAW_EMIT_MS ||
        flog_raw_room(&s_raw) < FLOG_RAW_MAX_ENTRIES / 4u) {
        raw_emit();
    }
}

void flightlog_decision(const flog_decision_t *d)
{
    if (d != NULL) {
        push_record(FLOG_REC_DECISION, d, sizeof *d);
    }
}

void flightlog_event(flog_event_code_t code, int32_t a, int32_t b)
{
    flog_event_t e = { .code = (uint8_t)code, .a = a, .b = b };
    push_record(FLOG_REC_EVENT, &e, sizeof e);
}

void flightlog_audio(const flog_audio_t *a)
{
    if (a != NULL) {
        push_record(FLOG_REC_AUDIO, a, sizeof *a);
    }
}
