#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
controller.py -- bench-control sender + boot-window "attach" spammer.

The firmware enters sim mode only if it catches a BENCH_CTRL hello during a
short window right after boot. Since opening the USB-Serial-JTAG port does NOT
reset the chip, we keep RESENDING the attach opcode in the background so that
whenever the device (re)boots -- a power cycle, an EN tap, or an OP_REBOOT -- the
window is caught immediately. The GUI stops the spam once it sees the firmware's
"BENCH SIM MODE" banner come back on the log.
"""

import threading
import time

import lwnx_codec as codec
import protocol as P


class BenchController:
    def __init__(self, serial):
        self.serial = serial
        self._spam_thread = None
        self._spam_stop = threading.Event()

    # --- one-shot control frames ---------------------------------------------

    def _send(self, opcode: int):
        self.serial.send(codec.build_bench_ctrl(opcode))

    def reboot(self):
        """Ask a running sim-mode device to restart (ignored if not in sim)."""
        self._send(P.OP_REBOOT)

    def enter_config(self):
        """Ask an in-sim device to SOFTWARE-reboot into the boot config menu.

        Sent a few times since it only takes effect while the device is already
        in sim mode (draining USB). The caller should then attach with HELLO so
        the device is caught back into sim and the menu runs.
        """
        frame = codec.build_bench_ctrl(P.OP_ENTER_CONFIG)
        for _ in range(3):
            self.serial.send(frame)
            time.sleep(0.02)

    def menu_next(self):
        """Config menu: cycle the current selection (single tap)."""
        self._send(P.OP_MENU_NEXT)

    def menu_confirm(self):
        """Config menu: confirm the current selection (double tap)."""
        self._send(P.OP_MENU_CONFIRM)

    # --- attach spammer ------------------------------------------------------

    def start_attach(self, opcode: int = P.OP_HELLO, interval: float = 0.04):
        """Repeatedly send `opcode` until stop_attach() (catches the boot window).

        opcode is OP_HELLO for a normal sim attach, or OP_ENTER_CONFIG to bring
        the box up straight into the config menu.
        """
        self.stop_attach()
        self._spam_stop.clear()
        self._spam_thread = threading.Thread(
            target=self._spam, args=(opcode, interval), daemon=True)
        self._spam_thread.start()

    def stop_attach(self):
        self._spam_stop.set()
        if self._spam_thread is not None:
            self._spam_thread.join(timeout=1.0)
            self._spam_thread = None

    def is_attaching(self) -> bool:
        return self._spam_thread is not None and self._spam_thread.is_alive()

    def _spam(self, opcode: int, interval: float):
        frame = codec.build_bench_ctrl(opcode)
        while not self._spam_stop.is_set():
            self.serial.send(frame)
            time.sleep(interval)
