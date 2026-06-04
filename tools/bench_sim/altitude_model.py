#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
altitude_model.py -- the AGL source the bench feeds to the firmware.

Two modes:
    'manual' -- the AGL is whatever the GUI slider / drag last set. This is the
                "fly it by hand" mode for poking specific heights.
    'ils'    -- a constant-glideslope approach: the aircraft descends a straight
                glideslope at a fixed groundspeed, so the firmware runs the full
                CLIMB -> ARMED -> DESCENT sequence and fires the callout ladder
                with realistic timing.

The model produces AGL in FEET. The sim engine adds the mount/ground offset and
converts to the sensor's centimetres -- see sim_engine.py.
"""

import math

# knots -> feet per second (1 kt = 1.68781 ft/s)
KT_TO_FPS = 1.68781


class AltitudeModel:
    def __init__(self):
        # Current source mode.
        self.mode = "manual"            # 'manual' | 'ils'

        # Manual mode: the directly-commanded AGL (set by the slider / drag).
        self.manual_ft = 0.0

        # ILS parameters (sensible Glasair-ish approach defaults).
        self.glideslope_deg = 3.0       # standard ILS glideslope
        self.groundspeed_kt = 70.0      # approach groundspeed
        self.start_alt_ft   = 342.0     # AGL the approach begins at (default ILS start)

        # ILS run state.
        self.running = False            # is the approach currently advancing?
        self._dist_ft = 0.0             # horizontal distance to the aim point
        self._agl = 0.0                 # last computed AGL (for read-back)

    # --- manual --------------------------------------------------------------

    def set_manual(self, ft: float):
        """Set the hand-flown AGL (clamped non-negative)."""
        self.manual_ft = max(0.0, float(ft))

    # --- ILS -----------------------------------------------------------------

    def _slope_tan(self) -> float:
        return math.tan(math.radians(self.glideslope_deg))

    def arm_ils(self):
        """Place the aircraft at start_alt_ft on the glideslope, paused."""
        self.mode = "ils"
        slope = self._slope_tan()
        # AGL = dist * tan(slope)  ->  dist = AGL / tan(slope)
        self._dist_ft = (self.start_alt_ft / slope) if slope > 1e-6 else 0.0
        self._agl = self.start_alt_ft
        self.running = False

    def play_ils(self):
        """Begin (or resume) advancing the approach."""
        if self.mode != "ils":
            self.arm_ils()
        self.running = True

    def pause_ils(self):
        self.running = False

    def is_ils_running(self) -> bool:
        return self.mode == "ils" and self.running

    # --- step ----------------------------------------------------------------

    def update(self, dt: float) -> float:
        """Advance the model by dt seconds and return the current AGL in feet."""
        if self.mode == "ils" and self.running:
            # Fly the aircraft toward the aim point at the set groundspeed.
            self._dist_ft -= self.groundspeed_kt * KT_TO_FPS * dt
            if self._dist_ft <= 0.0:
                self._dist_ft = 0.0
                self.running = False        # touchdown -- hold at ground
            self._agl = self._dist_ft * self._slope_tan()
            return self._agl

        # Manual (or a paused ILS holds its last AGL via manual_ft sync).
        return self.manual_ft

    def current_agl(self) -> float:
        """Last AGL without advancing (for UI read-back)."""
        if self.mode == "ils":
            return self._agl
        return self.manual_ft
