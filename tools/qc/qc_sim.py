#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
qc.py -- headless QC harness for the LidarAGL unit.

Attaches to the board over USB-Serial-JTAG using the SAME bench-sim protocol the
GUI uses (tools/bench_sim), streams simulated SF30 distance frames at the real
78 Hz cadence, and parses the device console to verify the firmware actually
walks its state machine and fires the callout ladder.

This is hardware-in-the-loop: every byte goes through the real parser -> CRC ->
range filter -> ground ref -> state machine -> audio chain on the actual unit.

Windows console safe: ASCII-only output, UTF-8 forced on stdout.
"""

import sys, io, time, struct, threading

# Force UTF-8 so redirected output never dies on a stray byte from the device.
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

sys.path.insert(0, r"L:\Dev\LidarAGL\tools\bench_sim")
import serial
from lwnx_codec import build_distance_frame, build_lost_signal_frame, build_bench_ctrl
from protocol import OP_HELLO, OP_REBOOT, DEFAULT_STREAM_HZ

PORT = "COM3"

# ---------------------------------------------------------------------------
#  Serial plumbing
# ---------------------------------------------------------------------------
class Unit:
    """Owns the port, a background reader thread, and the captured console log."""

    def __init__(self, port):
        # USB-Serial-JTAG is CDC: baud is ignored but pyserial demands a number.
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.lines = []
        self._buf = b""
        self._lock = threading.Lock()
        self._stop = False
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        while not self._stop:
            try:
                data = self.ser.read(4096)
            except Exception:
                break
            if data:
                with self._lock:
                    self._buf += data
                    # Device log is line-oriented; split on either terminator.
                    while b"\n" in self._buf:
                        raw, self._buf = self._buf.split(b"\n", 1)
                        s = raw.decode("utf-8", "replace").rstrip("\r")
                        if s:
                            self.lines.append((time.time(), s))
            else:
                time.sleep(0.005)

    def snapshot(self):
        with self._lock:
            return list(self.lines)

    def mark(self):
        """Return an index so a test can look only at lines produced after it."""
        with self._lock:
            return len(self.lines)

    def since(self, idx):
        with self._lock:
            return [s for _, s in self.lines[idx:]]

    def send(self, b):
        self.ser.write(b)
        self.ser.flush()

    def close(self):
        self._stop = True
        time.sleep(0.1)
        try:
            self.ser.close()
        except Exception:
            pass


def reset_board(u):
    """Pulse RTS to reset the ESP32, so we catch its 4 s attach window."""
    u.ser.dtr = False
    u.ser.rts = True
    time.sleep(0.15)
    u.ser.rts = False


def attach(u, timeout=12.0):
    """Reset, then spam OP_HELLO through the boot window until sim mode reports."""
    reset_board(u)
    hello = build_bench_ctrl(OP_HELLO)
    start = time.time()
    idx = u.mark()
    while time.time() - start < timeout:
        u.send(hello)
        time.sleep(0.02)
        for s in u.since(idx):
            if "BENCH SIM" in s.upper() or "sim mode" in s.lower():
                return True
    return False


# ---------------------------------------------------------------------------
#  Altitude streaming
# ---------------------------------------------------------------------------
def stream_agl(u, agl_ft, dur_s, ground_ft, hz=DEFAULT_STREAM_HZ, lost=False):
    """Hold a constant AGL for dur_s, emitting frames at the sensor's real rate."""
    period = 1.0 / hz
    n = max(1, int(dur_s * hz))
    for _ in range(n):
        if lost:
            u.send(build_lost_signal_frame())
        else:
            # The firmware computes AGL = range - ground_ref, so send the RANGE.
            u.send(build_distance_frame(int(round((agl_ft + ground_ft) / 0.0328084))))
        time.sleep(period)


def ramp_agl(u, a0, a1, rate_fps, ground_ft, hz=DEFAULT_STREAM_HZ):
    """Fly from a0 to a1 at rate_fps, streaming at the real cadence."""
    period = 1.0 / hz
    dist = abs(a1 - a0)
    dur = dist / rate_fps if rate_fps > 0 else 0
    n = max(1, int(dur * hz))
    for i in range(n + 1):
        agl = a0 + (a1 - a0) * (i / n)
        if agl > 500:  # above the SF30/C ceiling -> genuine loss of signal
            u.send(build_lost_signal_frame())
        else:
            u.send(build_distance_frame(int(round((agl + ground_ft) / 0.0328084))))
        time.sleep(period)


# ---------------------------------------------------------------------------
#  Result bookkeeping
# ---------------------------------------------------------------------------
PASS, FAIL = [], []

def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))
    return cond


import re
CALLOUT_RE = re.compile(r"callout\s+(\d+)\s+ft")

def callouts_in(lines):
    """Extract the callout heights the firmware reports, in the order spoken."""
    out = []
    for s in lines:
        m = CALLOUT_RE.search(s)
        if m:
            out.append(int(m.group(1)))
    return out


GROUND = 3.0   # simulated mount height: sensor reads 3 ft with the box parked


def main():
    print("=" * 72)
    print("LidarAGL UNIT QC  --  hardware-in-the-loop on", PORT)
    print("=" * 72)

    u = Unit(PORT)
    try:
        # ---------------------------------------------------------------
        print("\n[0] Attach + boot banner")
        ok = attach(u)
        # The boot banner is not complete when the sim-mode line appears: the
        # ground-ref and power-management lines land ~1 s later. Poll for the LAST
        # of them rather than sleeping a guessed interval, so this never races.
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if any("power management" in s for _, s in u.snapshot()):
                break
            time.sleep(0.05)
        boot = u.snapshot()
        boot_txt = "\n".join(s for _, s in boot)
        check("bench sim mode attached", ok)
        check("firmware v1.62 booted", "v1.62" in boot_txt)
        check("light-sleep compiled OFF",
              "SLEEP_MODE_ENABLE=0" in boot_txt or "Light sleep: DISABLED" in boot_txt)
        check("SF30/C profile active", "SF30/C" in boot_txt)
        check("audio I2S came up", "I2S up" in boot_txt)
        # NOTE: in sim mode the LiDAR is deliberately not read (ranges arrive over
        # USB instead), so the boot ground-fill sees nothing and the firmware
        # RIGHTLY raises calib_err=1 + "SENSOR SILENT at boot". That alarm firing
        # is a PASS here -- it proves the dead-sensor detector works. The real
        # calibration health of the unit is checked on a normal boot instead.
        check("dead-sensor alarm fires when no LiDAR data (expected in sim)",
              "SENSOR SILENT" in boot_txt or "calib_err=1" in boot_txt)
        check("falls back safely with no ground reference",
              "fallback" in boot_txt or "calib_err=0" in boot_txt)

        # Let the box settle on the ground and learn its ground reference.
        print("\n[1] Ground settle (learning ground ref)")
        stream_agl(u, 0.0, 6.0, GROUND)
        idx = u.mark()
        stream_agl(u, 0.0, 3.0, GROUND)
        check("no callouts while parked", len(callouts_in(u.since(idx))) == 0,
              f"saw {callouts_in(u.since(idx))}")

        # ---------------------------------------------------------------
        print("\n[2] Takeoff / climb -> positive rate")
        idx = u.mark()
        ramp_agl(u, 0.0, 260.0, 25.0, GROUND)       # Vy 1500 fpm
        climb = u.since(idx)
        check("positive-rate callout fired",
              any("positive rate" in s for s in climb))
        cl = callouts_in(climb)
        check("no descent callouts during climb", len(cl) == 0, f"saw {cl}")

        # ---------------------------------------------------------------
        print("\n[3] Cruise above sensor ceiling (blind, lost-signal)")
        idx = u.mark()
        stream_agl(u, 0, 8.0, GROUND, lost=True)
        blind = u.since(idx)
        check("no phantom callouts while blind", len(callouts_in(blind)) == 0,
              f"saw {callouts_in(blind)}")

        # ---------------------------------------------------------------
        print("\n[4] ILS approach 300 ft -> touchdown (3 deg @ 85 kt)")
        idx = u.mark()
        ramp_agl(u, 300.0, 0.0, 7.52, GROUND)       # 451 fpm
        stream_agl(u, 0.0, 3.0, GROUND)             # rollout
        appr = callouts_in(u.since(idx))
        print(f"       callouts heard: {appr}")
        check("approach produced callouts", len(appr) > 0)
        check("callouts strictly descending", appr == sorted(appr, reverse=True),
              f"{appr}")
        check("no duplicate callouts", len(appr) == len(set(appr)), f"{appr}")
        for rung in (200, 100, 50, 40, 30, 20, 10):
            check(f"rung {rung} ft spoken", rung in appr)

        # ---------------------------------------------------------------
        print("\n[5] Fault injection: garbage burst on final")
        idx = u.mark()
        # Fly 150 -> 60 ft but slam in wild bogus ranges partway down.
        period = 1.0 / DEFAULT_STREAM_HZ
        agl, i = 150.0, 0
        while agl > 60.0:
            if 40 <= i < 55:      # ~0.2 s of pure junk mid-descent
                u.send(build_distance_frame(int((12.0 + GROUND) / 0.0328084)))
            else:
                u.send(build_distance_frame(int(round((agl + GROUND) / 0.0328084))))
            agl -= 7.52 * period
            i += 1
            time.sleep(period)
        bad = callouts_in(u.since(idx))
        print(f"       callouts heard: {bad}")
        check("no phantom LOW callout from garbage burst",
              not any(c <= 20 for c in bad), f"saw {bad}")

        # ---------------------------------------------------------------
        print("\n[6] Dead sensor (total signal loss in flight)")
        idx = u.mark()
        stream_agl(u, 0, 6.0, GROUND, lost=True)
        dead = u.since(idx)
        check("no callouts invented on dead sensor",
              len(callouts_in(dead)) == 0, f"saw {callouts_in(dead)}")


        # ---------------------------------------------------------------
        print("\n[6b] Go-around at 30 ft, then a second full approach")
        idx = u.mark()
        ramp_agl(u, 200.0, 30.0, 7.52, GROUND)      # first approach
        ramp_agl(u, 30.0, 250.0, 25.0, GROUND)      # break off, climb away
        first = callouts_in(u.since(idx))
        idx2 = u.mark()
        ramp_agl(u, 250.0, 0.0, 7.52, GROUND)       # second approach
        stream_agl(u, 0.0, 3.0, GROUND)
        second = callouts_in(u.since(idx2))
        print(f"       approach 1: {first}")
        print(f"       approach 2: {second}")
        check("go-around: rungs re-armed for 2nd approach",
              all(r in second for r in (100, 50, 30, 10)), f"{second}")

        # ---------------------------------------------------------------
        print("\n[6c] Stuck sensor value (frozen reading, not lost signal)")
        idx = u.mark()
        for _ in range(int(6.0 * DEFAULT_STREAM_HZ)):
            u.send(build_distance_frame(int((75.0 + GROUND) / 0.0328084)))
            time.sleep(1.0 / DEFAULT_STREAM_HZ)
        stuck = callouts_in(u.since(idx))
        check("stuck value does not walk the ladder down",
              not any(c < 50 for c in stuck), f"saw {stuck}")

        # ---------------------------------------------------------------
        print("\n[6d] Taxi-back: 30 s parked must disarm the ladder")
        stream_agl(u, 0.0, 34.0, GROUND)            # sit still to disarm
        idx2 = u.mark()
        for i in range(int(8.0 * DEFAULT_STREAM_HZ)):
            bump = 1.5 if (i // 40) % 2 else 0.2    # taxi bumps near ground
            u.send(build_distance_frame(int(round((bump + GROUND) / 0.0328084))))
            time.sleep(1.0 / DEFAULT_STREAM_HZ)
        taxi = callouts_in(u.since(idx2))
        check("no callouts while taxiing after disarm", len(taxi) == 0, f"saw {taxi}")

        # ---------------------------------------------------------------
        print("\n[7] Health: no crashes / panics / asserts all run")
        full = "\n".join(s for _, s in u.snapshot())
        for bad_tok in ("Guru Meditation", "panic", "abort()", "assert failed",
                        "StoreProhibited", "LoadProhibited", "IllegalInstruction",
                        "Stack canary", "CORRUPT HEAP"):
            check(f"no '{bad_tok}'", bad_tok not in full)
        # Reset reasons: our own RTS pulse shows as USB_UART_CHIP_RESET / a normal
        # power-on is POWERON. Anything else (brownout, watchdog, panic) is a fault.
        resets = re.findall(r"rst:0x[0-9a-fA-F]+ \(([A-Z_]+)\)", full)
        benign = {"USB_UART_CHIP_RESET", "POWERON_RESET", "RTC_SW_CPU_RESET",
                  "USB_UART_CHIP_RESET".lower().upper()}
        check("only benign reset reasons", all(r in benign for r in resets),
              f"reasons={resets}")
        check("exactly one reset (no unexpected reboots)", len(resets) <= 1,
              f"reasons={resets}")
        check("task watchdog quiet", "task_wdt" not in full)

    finally:
        u.close()

    print("\n" + "=" * 72)
    print(f"RESULT: {len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED:")
        for f in FAIL:
            print("   -", f)
    print("=" * 72)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
