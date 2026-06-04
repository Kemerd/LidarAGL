#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_worker.py -- thread-safe pyserial wrapper for the bench.

The device's native USB-Serial-JTAG is FULL DUPLEX: the bench sends simulated
LWNX frames (host->device) while the firmware's ESP_LOG output streams back
(device->host). This worker owns the port:

    * send()         -- thread-safe writes (the sim engine calls this ~78x/sec,
                        the GUI calls it for control frames). A lock serialises
                        writers so frames never interleave on the wire.
    * a reader thread -- continuously drains RX and hands decoded text lines to
                        an on_rx callback, so the GUI can show the device log.

pyserial is the only third-party dependency here.
"""

import threading

import serial                       # pyserial
import serial.tools.list_ports

import protocol as P


def list_ports():
    """Return [(device, description), ...] for every serial port on the system.

    The ESP32-S3 native port shows up as "USB Serial Device" / "USB JTAG/serial
    debug unit" with VID 303A -- we surface the description so the user can pick
    the right COM port.
    """
    out = []
    for p in serial.tools.list_ports.comports():
        out.append((p.device, p.description or ""))
    return out


class SerialWorker:
    """Owns one serial connection; safe to call send() from any thread."""

    def __init__(self, on_rx=None, on_status=None):
        # on_rx(text)    -- called with each decoded text line from the device.
        # on_status(msg) -- called on connect/disconnect/errors for the UI.
        self._on_rx = on_rx
        self._on_status = on_status

        self._ser = None
        self._tx_lock = threading.Lock()
        self._reader = None
        self._stop = threading.Event()

    # --- connection ----------------------------------------------------------

    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect(self, port: str, baud: int = P.NOMINAL_BAUD) -> bool:
        """Open `port`. Returns True on success. Starts the RX reader thread."""
        self.disconnect()                       # ensure a clean slate
        try:
            # Short read timeout so the reader loop stays responsive and can be
            # stopped promptly; write_timeout guards against a stalled host pipe.
            self._ser = serial.Serial(port, baudrate=baud, timeout=0.05,
                                      write_timeout=0.5)
        except Exception as e:                  # noqa: BLE001 -- surface to UI
            self._status("open failed: %s" % e)
            self._ser = None
            return False

        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._status("connected: %s @ %d" % (port, baud))
        return True

    def disconnect(self):
        """Stop the reader and close the port. Safe to call repeatedly."""
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    # --- I/O -----------------------------------------------------------------

    def send(self, data: bytes):
        """Write raw bytes to the device. Thread-safe; silent no-op if closed."""
        if not self.is_open():
            return
        with self._tx_lock:
            try:
                self._ser.write(data)
            except Exception as e:              # noqa: BLE001
                self._status("write failed: %s" % e)

    def pulse_reset(self):
        """Best-effort hardware reset via the classic DTR/RTS toggle.

        On many ESP32-S3 boards the native USB-Serial-JTAG honours the same
        reset wiring esptool uses. This is a convenience for the FIRST attach
        (so you needn't reach for the on-board EN button); if your board doesn't
        wire it up, just tap EN instead -- the bench will still catch the boot.
        """
        if not self.is_open():
            return
        try:
            # EN low (RTS), GPIO0 high (DTR not asserted) -> reset into run mode.
            self._ser.setDTR(False)
            self._ser.setRTS(True)
            import time
            time.sleep(0.05)
            self._ser.setRTS(False)
            self._status("sent reset pulse (DTR/RTS)")
        except Exception as e:                  # noqa: BLE001
            self._status("reset pulse failed: %s" % e)

    # --- internals -----------------------------------------------------------

    def _read_loop(self):
        """Drain RX, split into text lines, and forward them to on_rx."""
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception:
                break                           # port went away
            if not chunk:
                continue
            buf.extend(chunk)
            # The device emits newline-terminated log lines; emit complete lines.
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                text = line.decode("utf-8", errors="replace").rstrip("\r")
                if text and self._on_rx:
                    self._on_rx(text)

    def _status(self, msg: str):
        if self._on_status:
            self._on_status(msg)
