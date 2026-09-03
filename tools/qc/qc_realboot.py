#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
qc_realboot.py -- normal-boot health check (NO bench sim attached).

Complements qc.py. Here the unit boots exactly as it will in the aircraft: the
real SF30/C on UART1, no USB bench stream. This is the only configuration in
which the ground reference and calibration verdict are meaningful, so the
calibration checks live here rather than in the sim-mode suite.
"""

import sys, io, time, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
sys.path.insert(0, r"L:\Dev\LidarAGL\tools\bench_sim")
import serial

PORT = "COM3"
PASS, FAIL = [], []

def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def main():
    print("=" * 72)
    print("LidarAGL REAL-BOOT QC  --  no bench attached, real sensor path")
    print("=" * 72)

    p = serial.Serial(PORT, 115200, timeout=0.05)
    # Reset and capture the whole boot WITHOUT ever sending a bench hello, so the
    # firmware takes its ordinary flight path (sim attach window times out).
    p.dtr = False
    p.rts = True
    time.sleep(0.15)
    p.rts = False

    buf = b""
    t0 = time.time()
    while time.time() - t0 < 12:
        buf += p.read(4096)
    p.close()
    txt = buf.decode("utf-8", "replace")

    print("\n--- boot log (filtered) ---")
    for line in txt.splitlines():
        if any(k in line for k in ("app:", "sf30c:", "audio:", "pm:")):
            print("   ", line.strip())

    print("\n--- checks ---")
    check("did NOT enter bench sim mode", "BENCH SIM" not in txt.upper())
    check("firmware v1.62", "v1.62" in txt)
    check("light-sleep OFF", "SLEEP_MODE_ENABLE=0" in txt or "Light sleep: DISABLED" in txt)

    # Calibration: the numbers that decide whether AGL is trustworthy in flight.
    m = re.search(r"ground_ref=([\d.]+) ft\s+boot_agl=([-\d.]+) ft\s+airborne=(\d)\s+calib_err=(\d)", txt)
    if m:
        gref, bagl, air, cerr = float(m.group(1)), float(m.group(2)), int(m.group(3)), int(m.group(4))
        print(f"    ground_ref={gref} ft  boot_agl={bagl} ft  airborne={air}  calib_err={cerr}")
        check("calibration error clear", cerr == 0, f"calib_err={cerr}")
        check("not falsely 'airborne' at boot", air == 0, f"airborne={air}")
        check("ground ref is a plausible mount height", 0.5 <= gref <= 15.0, f"{gref} ft")
        check("boot AGL sits on the ground", abs(bagl) <= 3.0, f"{bagl} ft")
    else:
        check("ground reference line present", False, "no ground_ref line in boot log")

    check("sensor NOT reported silent", "SENSOR SILENT" not in txt)
    check("no ground-ref fallback needed", "no ground reference" not in txt)
    check("UART up at 460800 XTAL-clocked", "460800" in txt and "XTAL" in txt)
    check("SF30/C profile detected", "SF30/C" in txt)
    check("audio I2S up", "I2S up" in txt)
    check("PM locks held", "no-light-sleep=HELD" in txt)
    check("reached running state", "app: running" in txt)

    for tok in ("Guru Meditation", "panic", "assert failed", "StoreProhibited",
                "LoadProhibited", "task_wdt"):
        check(f"no '{tok}'", tok not in txt)

    print("\n" + "=" * 72)
    print(f"RESULT: {len(PASS)} passed, {len(FAIL)} failed")
    for f in FAIL:
        print("   -", f)
    print("=" * 72)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
