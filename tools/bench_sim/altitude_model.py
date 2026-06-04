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
import random

# knots -> feet per second (1 kt = 1.68781 ft/s)
KT_TO_FPS = 1.68781

# --- Hand-flying error model tunables (used only when error_rate > 0) --------
# A real pilot never tracks the glideslope perfectly: they wander, OVER-correct,
# and occasionally flatten out ("partial level-off") before re-intercepting. We
# model the vertical DEVIATION from the ideal glideslope as an UNDER-damped spring
# -- the pilot chasing the needle, so a correction overshoots and has to be caught
# again (== over-correction) -- driven by continuous control jitter, with random
# level-off impulses layered on top. error_rate (0..0.25) scales the whole mess;
# 0 == a perfect on-rails glideslope. Deviations taper to zero near the ground
# (you fly tightest in the flare) so the touchdown stays clean and repeatable.
ERR_NAT_FREQ      = 0.55          # rad/s  -- needle-chase rate (~11 s period)
ERR_DAMPING       = 0.40          # < 1 -> overshoot / over-correction
ERR_GUST_SIGMA    = 18.0          # ft/s^2 of control jitter per unit error_rate
ERR_LEVELOFF_HZ   = 0.9           # expected level-offs/sec per unit error_rate
ERR_LEVELOFF_FRAC = (0.4, 1.1)    # impulse as a fraction of the nominal descent
ERR_DEV_CAP_FT    = 50.0          # hard bound on the vertical deviation (ft)
ERR_TAPER_FT      = 50.0          # below this AGL the deviation fades to 0


class AltitudeModel:
    def __init__(self):
        # Current source mode.
        self.mode = "manual"            # 'manual' | 'ils'

        # Manual mode: the directly-commanded AGL (set by the slider / drag).
        self.manual_ft = 0.0

        # ILS parameters (sensible Glasair-ish approach defaults).
        self.glideslope_deg = 3.0       # standard ILS glideslope
        self.groundspeed_kt = 70.0      # approach groundspeed
        self.start_alt_ft   = 400.0     # AGL the approach begins at (default ILS start)

        # Hand-flying error: 0 == perfect on-rails glideslope, up to 0.25 == sloppy
        # (wander + over-corrections + partial level-offs). See the ERR_* tunables.
        self.error_rate = 0.0
        self._dev   = 0.0               # vertical deviation from the glideslope (ft)
        self._dev_v = 0.0               # its rate of change (ft/s)

        # ILS run state.
        self.running = False            # is the approach currently advancing?
        self._dist_ft = 0.0             # horizontal distance to the aim point
        self._agl = 0.0                 # last computed AGL (for read-back)

    # --- manual --------------------------------------------------------------

    def set_manual(self, ft: float):
        """Set the hand-flown AGL (clamped non-negative)."""
        self.manual_ft = max(0.0, float(ft))

    def set_error_rate(self, r: float):
        """Set the ILS hand-flying error (0 = perfect glideslope .. 0.25 = sloppy)."""
        self.error_rate = min(0.25, max(0.0, float(r)))

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
        # Start every approach perfectly on the needle; the error builds as it flies.
        self._dev = 0.0
        self._dev_v = 0.0

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

    def _step_error(self, dt: float, nominal: float) -> float:
        """Advance the hand-flying deviation by dt; return the (tapered) ft to add.

        The deviation is an under-damped spring chasing zero (the glideslope): the
        pilot corrects toward the needle but OVERSHOOTS, then catches it again, which
        reads as over-correction. White control jitter drives the wander, and random
        impulses momentarily flatten the descent (a partial level-off the pilot has
        to re-correct from). The result is tapered to zero near the ground so the
        flare stays clean. All scaled by error_rate (0..0.25).
        """
        r = self.error_rate
        if dt <= 0.0:
            return 0.0

        # Under-damped restoring dynamics. The control jitter is white ACCELERATION
        # noise, scaled by 1/sqrt(dt) so its felt amplitude is frame-rate independent
        # (the stream rate slider must not change how sloppy the approach looks).
        wn = ERR_NAT_FREQ
        gust  = random.gauss(0.0, ERR_GUST_SIGMA * r / math.sqrt(dt))
        accel = -(wn * wn) * self._dev - (2.0 * ERR_DAMPING * wn) * self._dev_v + gust

        # Occasional partial level-off: push the deviation UP (stop descending as
        # fast for a beat), which the spring then has to pull back down through.
        if random.random() < ERR_LEVELOFF_HZ * r * dt:
            nominal_vs = self.groundspeed_kt * KT_TO_FPS * self._slope_tan()
            self._dev_v += nominal_vs * random.uniform(*ERR_LEVELOFF_FRAC)

        # Integrate, then clamp the deviation (and bleed the velocity that pushed it
        # past the cap so it doesn't slam the rail and stick).
        self._dev_v += accel * dt
        self._dev   += self._dev_v * dt
        if self._dev > ERR_DEV_CAP_FT:
            self._dev = ERR_DEV_CAP_FT
            self._dev_v = min(self._dev_v, 0.0)
        elif self._dev < -ERR_DEV_CAP_FT:
            self._dev = -ERR_DEV_CAP_FT
            self._dev_v = max(self._dev_v, 0.0)

        # Fade the deviation out as the ground nears so the touchdown is precise.
        taper = min(1.0, nominal / ERR_TAPER_FT) if ERR_TAPER_FT > 0.0 else 1.0
        return self._dev * taper

    def update(self, dt: float) -> float:
        """Advance the model by dt seconds and return the current AGL in feet."""
        if self.mode == "ils" and self.running:
            # Fly the aircraft toward the aim point at the set groundspeed.
            self._dist_ft -= self.groundspeed_kt * KT_TO_FPS * dt
            if self._dist_ft <= 0.0:
                self._dist_ft = 0.0
                self.running = False        # touchdown -- hold at ground
            nominal = self._dist_ft * self._slope_tan()

            # Layer the hand-flying deviation onto the ideal glideslope when enabled,
            # else fly the perfect on-rails approach (and keep the error state zeroed).
            if self.error_rate > 1e-6 and self.running:
                self._agl = max(0.0, nominal + self._step_error(dt, nominal))
            else:
                self._dev = 0.0
                self._dev_v = 0.0
                self._agl = nominal
            return self._agl

        # Manual (or a paused ILS holds its last AGL via manual_ft sync).
        return self.manual_ft

    def current_agl(self) -> float:
        """Last AGL without advancing (for UI read-back)."""
        if self.mode == "ils":
            return self._agl
        return self.manual_ft
