#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_engine.py -- the streaming heart of the bench.

A background thread ties the pieces together at the sensor's real cadence:

    altitude_model -> + ground offset + noise -> ft->cm -> LWNX cmd-44 frame -> serial

Streaming continuously (default AGL 0 = on the ground) during the device's boot
also feeds its ground-fill, so the firmware LEARNS the mount/ground offset for
real -- AGL = range - ground_ref is then exercised exactly as in flight.
"""

import random
import threading
import time

import lwnx_codec as codec
import noise
import protocol as P


class SimEngine:
    def __init__(self, serial, model):
        self.serial = serial
        self.model = model

        # Live, GUI-tunable parameters (plain attributes; the GIL makes the
        # float reads from the stream thread safe enough for a bench tool).
        self.rate_hz = P.DEFAULT_STREAM_HZ      # ~78 Hz, matches the real sensor
        self.ground_offset_ft = 3.0             # simulated mount height
        self.sigma_ft = 0.0                     # gaussian range jitter
        self.dropout_prob = 0.0                 # per-frame lost-signal probability
        self.max_range_ft = 328.0               # sensor max range (SF30/C ~100 m);
                                                # beyond it the bench sends lost-signal

        # Read-back for the UI.
        self.last_agl_ft = 0.0
        self.last_cm = 0

        self.streaming = False
        self._thread = None
        self._stop = threading.Event()
        # Fixed seed: repeatable noise across runs (no wall-clock dependence).
        self._rng = random.Random(0xA61D)

    def start(self):
        if self.streaming:
            return
        self._stop.clear()
        self.streaming = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.streaming = False

    def _loop(self):
        """Emit one distance frame per tick, pacing to rate_hz with drift catch-up."""
        last = time.monotonic()
        next_t = last
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - last
            last = now

            # Advance the altitude source and synthesise the sensor reading.
            agl = self.model.update(dt)
            cm = noise.distance_cm(agl, self.ground_offset_ft,
                                   self.sigma_ft, self.dropout_prob, self._rng,
                                   self.max_range_ft)
            self.serial.send(codec.build_distance_frame(cm))

            self.last_agl_ft = agl
            self.last_cm = cm

            # Pace to the configured rate; if we fell behind, resync rather than
            # spin to catch up (keeps the stream smooth, like the real sensor).
            period = 1.0 / max(1, self.rate_hz)
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()
