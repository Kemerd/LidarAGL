#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flightlog.py -- pull and decode the LidarAGL black-box recorder.

The firmware (main/flightlog.c) writes every raw SF30 sample, the logic task's
decisions, audio-path events and every console line into the 'flightlog' flash
partition. This tool reads that partition over USB and turns it into files you
can read, grep and plot.

Usage
-----
    python tools/flightlog/flightlog.py pull              # auto-find the box, read, decode
    python tools/flightlog/flightlog.py pull --port COM4
    python tools/flightlog/flightlog.py decode dump.bin   # decode a saved image
    python tools/flightlog/flightlog.py decode dump.bin --sessions 0   # every session

Output (one folder per run, next to the .bin):
    summary.txt     per power-up session: duration, reset reason, config lines,
                    every callout the logic FIRED vs what the audio task STARTED
    timeline.txt    everything, in order: console lines, events, state changes
    decisions.csv   logic-task decisions (10 Hz + every discrete change)
    raw.csv         every raw SF30 sample and drain boundary (replayable)
    audio.csv       1 Hz audio-engine snapshots
    events.csv      discrete events

The on-flash format is defined ONCE in main/flightlog_codec.h; the constants
below mirror it byte-for-byte. Change one, change the other.

Needs: pyserial + esptool (both ship in the ESP-IDF Python env). Pulling reboots
the box (esptool resets it into the bootloader and back) -- harmless on the
ground; the reboot simply starts a new session in the log.
"""

import argparse
import csv
import io
import os
import re
import struct
import subprocess
import sys
import time

# --- Windows console: force UTF-8 so log text can never crash the printer ---
try:
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")
except Exception:
    pass

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# =============================================================================
#  Format constants -- mirror of main/flightlog_codec.h
# =============================================================================
SECTOR_SIZE = 4096
MAGIC = 0x31474C46                 # "FLG1"
REC_HDR = 6                        # type u8, len u8, t_ms u32
REC_OVERHEAD = REC_HDR + 1         # + crc8
REC_MAX_PAYLOAD = 240

REC_TEXT, REC_RAW, REC_DECISION, REC_EVENT, REC_AUDIO = 1, 2, 3, 4, 5

HDR_FMT = "<IIII"                  # magic, seq, boot_id, t_ms           (16 B)
DECISION_FMT = "<IffhHHBbBB"       # seq range agl trend mask dt state fired flags flags2 (22 B)
EVENT_FMT = "<Bii"                 # code a b                            (9 B)
AUDIO_FMT = "<BBhHHHH"             # flags qdepth tone_agl gain voice stalls short (12 B)
assert struct.calcsize(HDR_FMT) == 16
assert struct.calcsize(DECISION_FMT) == 22
assert struct.calcsize(EVENT_FMT) == 9
assert struct.calcsize(AUDIO_FMT) == 12

F_VALID, F_FRESH, F_TRACK_BREAK, F_TRACKING = 0x01, 0x02, 0x04, 0x08
F_ARMED, F_TONE_ACTIVE, F_TONE_ON, F_SLEEP = 0x10, 0x20, 0x40, 0x80
F2_STALE, F2_POSRATE, F2_STALE_KICK = 0x01, 0x02, 0x04

A_RUNNING, A_CLIP, A_CALLOUTS_EN, A_TONE_EN = 0x01, 0x02, 0x04, 0x08
A_TONE_REQ, A_SUSPEND_REQ, A_ALERT = 0x10, 0x20, 0x40

RAW_MARKER, RAW_ABORT, RAW_TAG_MASK, RAW_REPEAT, RAW_VALUE = 0x8000, 0x4000, 0xC000, 0x4000, 0x3FFF

EVENTS = {
    1: "BOOT", 2: "CALLOUT_DEQUEUED", 3: "CALLOUT_DISCARDED(suspended)",
    4: "CLIP_END", 5: "AUDIO_SUSPEND", 6: "AUDIO_RESUME", 7: "ALERT_START",
    8: "LOG_DROPPED", 9: "CLIP_MISSING", 10: "CALLOUT_STALE_SKIPPED",
}
RESET_REASONS = [
    "UNKNOWN", "POWERON", "EXT", "SW", "PANIC", "INT_WDT", "TASK_WDT", "WDT",
    "DEEPSLEEP", "BROWNOUT", "SDIO", "USB", "JTAG", "EFUSE", "PWR_GLITCH", "CPU_LOCKUP",
]
STATES = ["GROUND", "CLIMB", "ARMED", "CRUISE", "DESCENT"]
# callouts.h callout_id_t order
CALLOUT_IDS = ["10", "20", "30", "40", "50", "100", "200", "300", "400", "500", "600",
               "check gear", "positive rate", "sink rate", "climb rate"]
# sensor_profile.c ladders, indexed by the decision's 'fired' field
LADDER_SF30C = [300, 200, 100, 50, 40, 30, 20, 10]
LADDER_SF30D = [600, 500, 400, 300, 200, 100, 50, 40, 30, 20, 10]


def crc8(data):
    """CRC-8, poly 0x07, init 0 -- identical to flog_crc8()."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def callout_name(i):
    return CALLOUT_IDS[i] if 0 <= i < len(CALLOUT_IDS) else f"id{i}"


def reset_name(i):
    return RESET_REASONS[i] if 0 <= i < len(RESET_REASONS) else f"reason{i}"


def flags_str(f, f2):
    """Compact human-readable flag list for the timeline."""
    out = []
    if f & F_ARMED:
        out.append("ARMED")
    if f & F_TRACKING:
        out.append("tracking")
    else:
        out.append("DARK")
    if f & F_TONE_ACTIVE:
        out.append("tone-wanted")
    if f & F_TONE_ON:
        out.append("TONE")
    if f & F_SLEEP:
        out.append("sleep")
    if f & F_TRACK_BREAK:
        out.append("TRACK-BREAK")
    if f2 & F2_STALE:
        out.append("STALE")
    if f2 & F2_POSRATE:
        out.append("POSRATE")
    return " ".join(out)


# =============================================================================
#  Partition location (from partitions.csv, so the tool follows the firmware)
# =============================================================================
def find_partition(csv_path=os.path.join(REPO, "partitions.csv"), label="flightlog"):
    with open(csv_path, encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            cols = [c.strip() for c in line.split(",")]
            if len(cols) >= 5 and cols[0] == label:
                return int(cols[3], 0), int(cols[4], 0)
    raise SystemExit(f"no '{label}' partition in {csv_path}")


# =============================================================================
#  Decoding
# =============================================================================
def parse_image(img):
    """Return a list of sessions: {boot_id, sectors, records[(t, type, payload)]}."""
    sectors = []
    for i in range(len(img) // SECTOR_SIZE):
        base = i * SECTOR_SIZE
        magic, seq, boot_id, t0 = struct.unpack_from(HDR_FMT, img, base)
        if magic != MAGIC:
            continue
        recs, off, end = [], base + 16, base + SECTOR_SIZE
        torn = False
        while off + REC_OVERHEAD <= end:
            typ = img[off]
            if typ == 0xFF:
                break                                   # erased tail: end of sector
            plen = img[off + 1]
            total = REC_OVERHEAD + plen
            if typ == 0 or plen > REC_MAX_PAYLOAD or off + total > end:
                torn = True
                break
            if crc8(img[off:off + REC_HDR + plen]) != img[off + REC_HDR + plen]:
                torn = True                             # power cut mid-write
                break
            t_ms = struct.unpack_from("<I", img, off + 2)[0]
            recs.append((t_ms, typ, bytes(img[off + REC_HDR:off + REC_HDR + plen])))
            off += total
        sectors.append((seq, boot_id, t0, recs, torn))

    # Newest-last by the monotonic sector sequence (wrap-safe for our lifetime).
    sectors.sort(key=lambda s: s[0])
    sessions = []
    for seq, boot_id, t0, recs, torn in sectors:
        if not sessions or sessions[-1]["boot_id"] != boot_id:
            sessions.append({"boot_id": boot_id, "records": [], "torn": 0, "sectors": 0})
        sessions[-1]["records"].extend(recs)
        sessions[-1]["sectors"] += 1
        sessions[-1]["torn"] += 1 if torn else 0
    return sessions


def expand_raw(t_rec, payload):
    """RAW entries -> list of ('s', cm) samples and ('m', t_ms, aborted) markers."""
    out, last = [], None
    for (e,) in struct.iter_unpack("<H", payload[: len(payload) // 2 * 2]):
        if e & RAW_MARKER:
            m = e & RAW_VALUE
            t = t_rec - ((t_rec - m) & RAW_VALUE)      # unwrap t_ms mod 16384
            out.append(("m", t, bool(e & RAW_ABORT)))
        elif (e & RAW_TAG_MASK) == RAW_REPEAT:
            if last is not None:
                out.extend([("s", last)] * (e & RAW_VALUE))
        else:
            last = e & RAW_VALUE
            out.append(("s", last))
    return out


def fmt_t(ms):
    s = ms / 1000.0
    return f"{int(s // 60):3d}:{s % 60:06.3f}"


def decode(bin_path, sessions_wanted):
    with open(bin_path, "rb") as f:
        img = f.read()
    sessions = parse_image(img)
    if not sessions:
        print("No recorder data found (the partition is empty or was never written).")
        return None
    if sessions_wanted > 0:
        sessions = sessions[-sessions_wanted:]

    out_dir = os.path.splitext(bin_path)[0] + "_decoded"
    os.makedirs(out_dir, exist_ok=True)

    tl = open(os.path.join(out_dir, "timeline.txt"), "w", encoding="utf-8")
    sm = open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8")
    dcsv = csv.writer(open(os.path.join(out_dir, "decisions.csv"), "w", newline="", encoding="utf-8"))
    rcsv = csv.writer(open(os.path.join(out_dir, "raw.csv"), "w", newline="", encoding="utf-8"))
    acsv = csv.writer(open(os.path.join(out_dir, "audio.csv"), "w", newline="", encoding="utf-8"))
    ecsv = csv.writer(open(os.path.join(out_dir, "events.csv"), "w", newline="", encoding="utf-8"))
    dcsv.writerow(["session", "t_ms", "seq", "range_ft", "agl_ft", "trend_fps", "state",
                   "armed_mask", "fired_idx", "fired_ft", "dt_ms", "valid", "fresh",
                   "track_break", "tracking", "armed", "tone_active", "tone_on", "sleep",
                   "stale", "posrate", "stale_kick"])
    rcsv.writerow(["session", "t_ms", "kind", "cm", "aborted"])
    acsv.writerow(["session", "t_ms", "running", "clip", "callouts_en", "tone_en",
                   "tone_req", "suspend_req", "alert", "queue", "tone_agl_ft",
                   "tone_gain", "voice_gain", "stalls", "short_writes"])
    ecsv.writerow(["session", "t_ms", "event", "a", "b"])

    def both(line):
        print(line)
        sm.write(line + "\n")

    for s in sessions:
        sid = s["boot_id"]
        recs = s["records"]
        ladder = LADDER_SF30C
        dur = (recs[-1][0] - recs[0][0]) / 1000.0 if recs else 0.0
        fired, dequeued, discarded, missing = [], [], [], []
        texts_key, reset, drops = [], None, 0
        prev = None
        armed_ever = tone_ever = False
        pending_raw = []

        tl.write(f"\n{'=' * 78}\nSESSION #{sid}  ({len(recs)} records, {s['sectors']} sectors"
                 f"{', %d torn' % s['torn'] if s['torn'] else ''})\n{'=' * 78}\n")

        for t, typ, p in recs:
            if typ == REC_TEXT:
                line = p.decode("utf-8", "replace")
                tl.write(f"{fmt_t(t)}  | {line}\n")
                if "SF30/D" in line and "active profile" in line:
                    ladder = LADDER_SF30D
                if any(k in line for k in ("firmware", "I2S up", "start-altitude cap",
                                           "gear-check", "positive-rate", "vario blip",
                                           "tone-start", "volume offsets", "ground_ref",
                                           "initial state", "active profile", "suppressed",
                                           "MISSING", "SENSOR SILENT", "calibration",
                                           "no ground reference", "light-sleep",
                                           "flight recorder")):
                    texts_key.append(f"{fmt_t(t)}  {line}")

            elif typ == REC_DECISION and len(p) == 22:
                (seq, rng, agl, trend, mask, dt, st, fi, fl, f2) = struct.unpack(DECISION_FMT, p)
                fired_ft = ladder[fi] if 0 <= fi < len(ladder) else ""
                dcsv.writerow([sid, t, seq, f"{rng:.2f}", f"{agl:.2f}", trend / 10.0,
                               STATES[st] if st < len(STATES) else st, f"0x{mask:04X}",
                               fi, fired_ft, dt,
                               int(bool(fl & F_VALID)), int(bool(fl & F_FRESH)),
                               int(bool(fl & F_TRACK_BREAK)), int(bool(fl & F_TRACKING)),
                               int(bool(fl & F_ARMED)), int(bool(fl & F_TONE_ACTIVE)),
                               int(bool(fl & F_TONE_ON)), int(bool(fl & F_SLEEP)),
                               int(bool(f2 & F2_STALE)), int(bool(f2 & F2_POSRATE)),
                               int(bool(f2 & F2_STALE_KICK))])
                armed_ever |= bool(fl & F_ARMED)
                tone_ever |= bool(fl & F_TONE_ON)
                if fi >= 0:
                    fired.append((t, fired_ft, agl))
                    tl.write(f"{fmt_t(t)}  * LOGIC FIRED rung {fired_ft} ft at AGL {agl:.1f}\n")
                # Timeline shows only discrete CHANGES; the CSV has every record.
                key = (st, mask, fl & ~(F_FRESH | F_VALID), f2)
                if key != prev:
                    tl.write(f"{fmt_t(t)}  > {STATES[st] if st < len(STATES) else st:<7} "
                             f"agl {agl:7.1f}  range {rng:7.1f}  trend {trend / 10.0:+6.1f}  "
                             f"mask 0x{mask:04X}  {flags_str(fl, f2)}\n")
                    prev = key

            elif typ == REC_EVENT and len(p) == 9:
                code, a, b = struct.unpack(EVENT_FMT, p)
                name = EVENTS.get(code, f"EV{code}")
                ecsv.writerow([sid, t, name, a, b])
                if code == 1:
                    reset = reset_name(a)
                    tl.write(f"{fmt_t(t)}  ! BOOT  reset reason = {reset}\n")
                elif code == 2:
                    dequeued.append((t, callout_name(a), b))
                    tl.write(f"{fmt_t(t)}  ! audio took '{callout_name(a)}' off the queue -> "
                             f"{'PLAYED' if b else 'NOT PLAYED (callouts disabled in this mode?)'}\n")
                elif code == 3:
                    discarded.append((t, callout_name(a)))
                    tl.write(f"{fmt_t(t)}  ! audio DISCARDED '{callout_name(a)}' (channel suspended)\n")
                elif code == 9:
                    missing.append((t, callout_name(a)))
                    tl.write(f"{fmt_t(t)}  ! clip for '{callout_name(a)}' has NO AUDIO DATA\n")
                elif code == 10:
                    tl.write(f"{fmt_t(t)}  ! skipped stale '{callout_name(a)}' -> "
                             f"'{callout_name(b)}' (descending faster than words)
")
                elif code == 8:
                    drops += a
                    tl.write(f"{fmt_t(t)}  ! recorder dropped {a} bytes (RAM ring full)\n")
                else:
                    tl.write(f"{fmt_t(t)}  ! {name} {a if a else ''}\n")

            elif typ == REC_AUDIO and len(p) == 12:
                fl, q, tagl, g, vg, stl, sh = struct.unpack(AUDIO_FMT, p)
                acsv.writerow([sid, t, int(bool(fl & A_RUNNING)), int(bool(fl & A_CLIP)),
                               int(bool(fl & A_CALLOUTS_EN)), int(bool(fl & A_TONE_EN)),
                               int(bool(fl & A_TONE_REQ)), int(bool(fl & A_SUSPEND_REQ)),
                               int(bool(fl & A_ALERT)), q, tagl / 10.0, g / 1000.0,
                               vg / 1000.0, stl, sh])

            elif typ == REC_RAW:
                # Samples are timestamped by the drain they belong to, i.e. the
                # NEXT marker (a long drain can span two records, hence the
                # session-wide pending list). A replay feeds each drain's samples
                # to rf_push_cm, then rf_finalize with dt = marker-to-marker.
                for item in expand_raw(t, p):
                    if item[0] == "s":
                        pending_raw.append(item[1])
                    else:
                        for cm in pending_raw:
                            rcsv.writerow([sid, item[1], "sample", cm, ""])
                        pending_raw.clear()
                        rcsv.writerow([sid, item[1], "drain_end", "", int(item[2])])

        # ---- Per-session summary -------------------------------------------
        both(f"\n=== SESSION #{sid}: {dur / 60:.1f} min, reset reason {reset or '?'}"
             f"{', %d torn sectors' % s['torn'] if s['torn'] else ''}"
             f"{', RECORDER DROPPED %d BYTES' % drops if drops else ''}")
        for k in texts_key:
            both("   " + k)
        both(f"   ladder armed at some point: {'YES' if armed_ever else 'no'}   "
             f"tone sounded: {'YES' if tone_ever else 'no'}")
        both(f"   rungs FIRED by the logic ({len(fired)}): "
             + (", ".join(f"{ft}@{agl:.0f}ft" for _, ft, agl in fired) or "none"))
        both(f"   callouts taken by audio ({len(dequeued)}): "
             + (", ".join(f"{n}{'' if b else '(NOT PLAYED)'}" for _, n, b in dequeued) or "none"))
        if discarded:
            both(f"   callouts DISCARDED while suspended: " + ", ".join(n for _, n in discarded))
        if missing:
            both(f"   clips with NO AUDIO DATA: " + ", ".join(n for _, n in missing))

    for fh in (tl, sm):
        fh.close()
    print(f"\nDecoded -> {out_dir}")
    return out_dir


# =============================================================================
#  Pulling over USB
# =============================================================================
def find_port():
    """The ESP32-S3's native USB-Serial-JTAG is VID 0x303A; COM numbers move."""
    try:
        from serial.tools import list_ports
    except ImportError:
        raise SystemExit("pyserial missing -- run this with the ESP-IDF Python env")
    hits = [p.device for p in list_ports.comports() if p.vid == 0x303A]
    if not hits:
        raise SystemExit("No ESP32-S3 (VID 303A) found. Plug the box in over USB, "
                         "or pass --port COMx.")
    if len(hits) > 1:
        print(f"Several candidates {hits}; using {hits[0]} (pass --port to choose).")
    return hits[0]


def pull(port, out_path):
    off, size = find_partition()
    port = port or find_port()
    print(f"Reading 'flightlog' ({size // 1024} KB at 0x{off:X}) from {port} ...")
    cmd = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "-p", port,
           "--before", "default-reset", "--after", "hard-reset",
           "read-flash", hex(off), hex(size), out_path]
    rc = subprocess.call(cmd)
    if rc != 0:
        raise SystemExit(f"esptool failed (exit {rc}). Is a serial monitor holding the port?")
    return out_path


def main():
    ap = argparse.ArgumentParser(description="Pull and decode the LidarAGL flight recorder.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p1 = sub.add_parser("pull", help="read the recorder over USB, then decode it")
    p1.add_argument("--port", help="serial port (default: auto-detect VID 303A)")
    p1.add_argument("--out", help="where to save the raw image (.bin)")
    p1.add_argument("--sessions", type=int, default=5,
                    help="decode the newest N power-up sessions (0 = all; default 5)")
    p2 = sub.add_parser("decode", help="decode a previously pulled .bin image")
    p2.add_argument("image")
    p2.add_argument("--sessions", type=int, default=5,
                    help="decode the newest N power-up sessions (0 = all; default 5)")
    a = ap.parse_args()

    if a.cmd == "pull":
        out = a.out or os.path.join(os.getcwd(),
                                    time.strftime("flightlog_%Y%m%d_%H%M%S.bin"))
        decode(pull(a.port, out), a.sessions)
    else:
        decode(a.image, a.sessions)


if __name__ == "__main__":
    main()
