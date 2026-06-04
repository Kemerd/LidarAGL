#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_worker.py -- resilient pyserial wrapper for the bench.

The ESP32-S3's native USB-Serial-JTAG drops off the USB bus whenever the chip
RESETS -- and entering sim mode requires a reboot. So this worker is built to
RIDE those resets: one manager thread keeps (re)opening the port and reading the
device log, while send() simply no-ops during the brief gap the port is gone.
That makes "click Enter Sim -> chip reboots -> port re-enumerates -> hello caught"
work without the user babysitting the connection.

The device's USB RX is only drained while the firmware is in its boot sim-window
or already in sim mode; the rest of the time host writes will time out (the chip
just isn't listening). We treat that as normal and report it once, not per-frame.
"""

import threading
import time

import serial                       # pyserial
import serial.tools.list_ports

import protocol as P

# Espressif's USB vendor id. The ESP32-S3 native USB-Serial-JTAG enumerates under
# this VID (PID 0x1001) -- the only reliable way to tell it apart from other COM
# devices (a Steam controller, say, also shows up as a generic "USB Serial Device").
ESP_VID = 0x303A


def list_ports():
    """Return [{device, desc, vid, pid, is_esp}, ...] for every serial port."""
    out = []
    for p in serial.tools.list_ports.comports():
        out.append({
            "device": p.device,
            "desc": p.description or "",
            "vid": p.vid,
            "pid": p.pid,
            "is_esp": (p.vid == ESP_VID),
        })
    return out


class SerialWorker:
    def __init__(self, on_rx=None, on_status=None):
        # on_rx(text)    -- one decoded device-log line.
        # on_status(msg) -- connect / reconnect / error notices for the UI.
        self._on_rx = on_rx
        self._on_status = on_status

        self._port = None
        self._baud = P.NOMINAL_BAUD

        self._ser = None
        self._ser_lock = threading.Lock()   # guards the handle swap
        self._tx_lock = threading.Lock()    # serialises concurrent writers

        self._want = False                  # user intends to be connected
        self._stop = threading.Event()
        self._thread = None
        self._write_ok = True               # for throttling write-fail spam

    # --- state ---------------------------------------------------------------

    def is_active(self) -> bool:
        """True between connect() and disconnect() (drives the UI button)."""
        return self._want

    def is_open(self) -> bool:
        with self._ser_lock:
            return self._ser is not None and self._ser.is_open

    # --- connection ----------------------------------------------------------

    def connect(self, port: str, baud: int = P.NOMINAL_BAUD) -> bool:
        """Begin maintaining a connection to `port` (auto-reopens across resets)."""
        self.disconnect()
        self._port = port
        self._baud = baud
        self._want = True
        self._write_ok = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def disconnect(self):
        self._want = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None
        self._close()

    # --- I/O -----------------------------------------------------------------

    def send(self, data: bytes):
        """Write bytes; thread-safe. No-ops (not an error) while the port is gone."""
        with self._ser_lock:
            ser = self._ser
        if ser is None:
            return
        try:
            with self._tx_lock:
                ser.write(data)
            self._write_ok = True
        except Exception as e:              # noqa: BLE001
            # Report only the FIRST failure in a run -- the device simply isn't
            # draining yet (it's running normally, not in its sim window).
            if self._write_ok:
                self._status("device isn't accepting data yet (%s) -- click "
                             "Enter Sim / tap EN so it opens its sim window" % e)
                self._write_ok = False
            self._close()                   # likely a reset -> let the manager reopen

    def pulse_reset(self):
        """Reboot the chip via the RTS line -- the same pin esptool resets with."""
        with self._ser_lock:
            ser = self._ser
        if ser is None:
            return
        try:
            ser.setRTS(True)                # EN low  -> chip held in reset
            time.sleep(0.1)
            ser.setRTS(False)               # EN high -> boot
            self._status("reset pulse sent (RTS) -- catching the sim window…")
        except Exception as e:              # noqa: BLE001
            self._status("reset pulse failed (%s) -- tap the board's EN button" % e)

    # --- internals -----------------------------------------------------------

    def _open(self) -> bool:
        try:
            s = serial.Serial(self._port, self._baud, timeout=0.05,
                              write_timeout=0.3)
        except Exception:
            return False
        with self._ser_lock:
            self._ser = s
        self._write_ok = True
        self._status("connected: %s @ %d" % (self._port, self._baud))
        return True

    def _close(self):
        with self._ser_lock:
            s = self._ser
            self._ser = None
        if s is not None:
            try:
                s.close()
            except Exception:
                pass

    def _run(self):
        """Manager loop: keep the port open and stream RX lines; reopen on drop."""
        buf = bytearray()
        announced_wait = False
        while self._want and not self._stop.is_set():
            if not self.is_open():
                if self._open():
                    announced_wait = False
                else:
                    if not announced_wait:
                        self._status("waiting for %s (rebooting / unplugged?)…"
                                     % self._port)
                        announced_wait = True
                    time.sleep(0.3)         # device mid-reset; retry shortly
                    continue

            with self._ser_lock:
                ser = self._ser
            if ser is None:
                continue
            try:
                chunk = ser.read(256)
            except Exception:
                self._status("connection lost — reconnecting…")
                self._close()
                continue
            if not chunk:
                continue

            buf.extend(chunk)
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                text = line.decode("utf-8", errors="replace").rstrip("\r")
                if text and self._on_rx:
                    self._on_rx(text)

        self._close()

    def _status(self, msg: str):
        if self._on_status:
            self._on_status(msg)
