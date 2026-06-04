#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py -- launch the LidarAGL hardware-in-the-loop bench GUI.

    python tools/bench_sim/main.py

Streams byte-accurate simulated SF30/C LWNX frames to a real ESP32-S3 running
the LidarAGL firmware over its USB-Serial-JTAG port, so you can bench-test the
actual firmware (callouts + presence tone on headphones) with no LiDAR fitted.

Deps: pyserial, customtkinter  (see requirements.txt).
"""

import os
import sys

# Windows-console-safe output.
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

# Run-from-anywhere: put this folder on the import path so the flat sibling
# modules (protocol, lwnx_codec, ...) import cleanly.
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)


def _fail(missing):
    print("Missing dependency: %s" % missing)
    print("Install the bench-sim requirements with:")
    print("    pip install -r tools/bench_sim/requirements.txt")
    sys.exit(1)


def main():
    try:
        import serial  # noqa: F401  (pyserial)
    except ImportError:
        _fail("pyserial")
    try:
        import customtkinter  # noqa: F401
    except ImportError:
        _fail("customtkinter")

    import app_ctk
    app_ctk.run()


if __name__ == "__main__":
    main()
