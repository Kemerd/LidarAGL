/**
 * @file    flightlog.h
 * @brief   In-flight black-box recorder: raw sensor samples, logic decisions,
 *          audio-path events and every console line, written to a dedicated
 *          flash partition as a ring that keeps the most recent flying.
 *
 * @details HARDWARE module (ESP-IDF). The on-flash format lives in the PURE
 *          flightlog_codec.h; this file owns the partition, a lock-protected
 *          RAM ring that every task can append to without blocking, and a
 *          low-priority writer task that moves the ring to flash.
 *
 *          ----------------------------------------------------------------------
 *          SAFETY CONTRACT — the recorder must never cost a callout
 *          ----------------------------------------------------------------------
 *          - Producers NEVER block. An append is a short spinlocked copy into
 *            RAM; when the ring is full the record is dropped and counted
 *            (FLOG_EV_LOG_DROPPED), never waited on.
 *          - Flash work happens only in the writer task, at priority 1 — below
 *            every flight task — so it only runs in time the box has spare.
 *          - A sector ERASE freezes flash-resident code on both cores for tens
 *            of milliseconds. Erases are therefore done AHEAD of need, only
 *            while audio_is_quiet() says nothing is sounding, and a runway of
 *            FLOG_RUNWAY_SECTORS pre-erased sectors carries the recorder
 *            through an approach without erasing at all. If that runway ever
 *            runs dry while audio is live, the writer simply waits (the RAM
 *            ring absorbs it) rather than erase under a callout.
 *          - Page WRITES are batched to FLOG_STAGE_BYTES (a few ms of freeze),
 *            well inside the I2S DMA's ~90 ms of buffered audio.
 *          - The recorder is optional. A missing partition, a failed flash
 *            operation or a full ring degrades to "not recording" — nothing in
 *            the flight path waits on it or checks its result.
 *
 *          Retrieve and decode with tools/flightlog/flightlog.py (USB, esptool).
 */
#ifndef LIDARAGL_FLIGHTLOG_H
#define LIDARAGL_FLIGHTLOG_H

#include <stdint.h>
#include <stdbool.h>
#include "flightlog_codec.h"

/**
 * @brief Bring the recorder up. Call FIRST in app_main.
 *
 * @details Locates the partition, scans the sector headers to find where the
 *          previous session stopped, starts a new boot session, hooks the
 *          ESP_LOG output so every console line is captured from here on, and
 *          creates the writer task. Until flightlog_start_writer() is called the
 *          writer touches no flash: boot-time lines simply accumulate in RAM,
 *          so the config menu's prompts can never be disturbed by an erase.
 *          Safe to call when the partition is absent (recording is disabled).
 */
void flightlog_init(void);

/**
 * @brief Allow the writer to start using flash. Call once the flight tasks
 *        are running (end of app_main).
 */
void flightlog_start_writer(void);

/**
 * @brief Record one decoded SF30 distance, exactly as it came off the wire.
 * @details SINGLE PRODUCER: only the sensor read path may call this (the boot
 *          ground-fill and the sensor task, which never run concurrently).
 * @param cm  Decoded wire value in centimetres.
 */
void flightlog_raw_sample(int cm);

/**
 * @brief Record that the range filter finalized its drain (a poll boundary).
 * @details Same single-producer rule as flightlog_raw_sample().
 * @param aborted  True when the drain was discarded for a UART hardware error.
 */
void flightlog_raw_finalize(bool aborted);

/** @brief Record one logic-task decision tick. Any task; never blocks. */
void flightlog_decision(const flog_decision_t *d);

/**
 * @brief Record a discrete event. Any task; never blocks.
 * @param code  flog_event_code_t.
 * @param a     First argument (meaning per code).
 * @param b     Second argument (meaning per code).
 */
void flightlog_event(flog_event_code_t code, int32_t a, int32_t b);

/** @brief Record an audio-engine snapshot. Any task; never blocks. */
void flightlog_audio(const flog_audio_t *a);

#endif /* LIDARAGL_FLIGHTLOG_H */
