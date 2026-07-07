#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
noise.py -- realistic sensor-error injection for the bench.

A real SF30 doesn't report a perfect number: there's a little range jitter, and
over a dark/wet/over-water surface it occasionally returns its lost-signal
sentinel. Injecting both lets us prove the firmware behaves on the bench exactly
as it must in the air:

    * gaussian jitter  -> exercises the runtime range filter's smoothing
                          (range_filter.c) so the tone pitch and callout timing
                          stay smooth.
    * lost-signal drops -> exercises the firmware's last-good HOLD, so a dropout
                          never produces a garbage callout.

On top of the realistic model, FaultInjector (below) deliberately breaks the
stream -- spikes, random garbage, corruption bursts, stuck values -- to prove
the v1.59 robust pipeline (median-of-drain + Hampel gate + re-acquire) holds
the line: a parked/taxiing box must stay SILENT through all of it.
"""

import lwnx_codec as codec
import protocol as P


def distance_cm(agl_ft: float, ground_offset_ft: float,
                sigma_ft: float, dropout_prob: float, rng,
                max_range_ft: float = 0.0, edge_band_ft: float = 30.0) -> int:
    """Turn a desired AGL into the centimetre value the sensor would report.

    The firmware computes AGL = range - ground_ref, so the bench sends RANGE:
        range_ft = agl_ft + ground_offset_ft   (+ gaussian jitter)

    Out-of-range modelling: a real SF30 can't see the ground past its max range
    (SF30/C ~328 ft / 100 m), so beyond it the bench emits the lost-signal
    sentinel just like the sensor would. We ramp into it across the last
    `edge_band_ft` (returns get progressively flakier near the limit) and go to
    100% lost above it -- which exercises the firmware's last-good HOLD exactly
    as a real out-of-range climb would.

    `rng` is a random.Random instance (passed in so the engine controls seeding).
    """
    true_ft = agl_ft + ground_offset_ft

    if max_range_ft > 0.0:
        if true_ft >= max_range_ft:
            return P.SF30_LOST_SIGNAL_CM            # past range: always lost
        if true_ft > max_range_ft - edge_band_ft:
            # In the edge band the lost-signal probability ramps 0 -> 1.
            t = (true_ft - (max_range_ft - edge_band_ft)) / edge_band_ft
            if rng.random() < t:
                return P.SF30_LOST_SIGNAL_CM

    if dropout_prob > 0.0 and rng.random() < dropout_prob:
        return P.SF30_LOST_SIGNAL_CM

    rng_ft = true_ft
    if sigma_ft > 0.0:
        rng_ft += rng.gauss(0.0, sigma_ft)
    if rng_ft < 0.0:
        rng_ft = 0.0
    return codec.ft_to_cm(rng_ft)


# The legacy 2-byte wire encoding tops out at 14 bits, so ANY corruption --
# whatever its physical source -- decodes somewhere in 0..16383 cm. Random
# fault values are drawn from that whole space (including the lost-signal
# sentinel band the firmware must gate), exactly like real garbage would land.
WIRE_MAX_CM = 16384


class FaultInjector:
    """Deliberate stream corruption, layered AFTER the realistic sensor model.

    Four independent fault modes, mirroring the real-world corruption classes
    found in the taxi-incident audit (value-level -- the LWNX CRC on this HIL
    transport means byte-level garbage never reaches the decoder, so we corrupt
    the VALUES, which is what survives the checksum-free ASCII path in flight):

      * SPIKE  (one-shot) : the next N frames report a fixed wrong range.
                            1 frame == the classic isolated outlier; the
                            median-of-drain or Hampel gate must eat it.
      * RANDOM (continuous): each frame has probability p of reporting a
                            uniformly random wire value. Uncorrelated garbage
                            must never re-acquire, whatever p.
      * BURST  (periodic)  : every period_s, corrupt burst_len consecutive
                            frames with random values -- the light-sleep
                            wake-edge signature. The output must HOLD through
                            each burst and resume cleanly.
      * STUCK  (toggle)    : report one fixed wrong value continuously. This is
                            the correlated-corruption worst case: the filter's
                            re-acquire logic will (BY DESIGN) accept it after
                            RANGE_REACQUIRE_N agreeing polls, exactly as it
                            must accept a genuine terrain step. Expect the box
                            to follow it after ~3 polls -- the test proves the
                            re-acquire threshold, not silence.

    All attributes are plain floats/bools tuned live from the GUI thread; the
    stream thread only reads them (same GIL contract as SimEngine's knobs).
    """

    def __init__(self):
        # RANDOM mode.
        self.random_prob = 0.0        # per-frame corruption probability (0..1)

        # BURST mode.
        self.burst_enabled = False
        self.burst_period_s = 5.0     # one burst every this many seconds
        self.burst_len = 12           # frames per burst (~150 ms at 78 Hz)

        # STUCK mode.
        self.stuck_enabled = False
        self.stuck_ft = 250.0         # the wrong range the sensor "sees"

        # SPIKE one-shot state (armed by inject_spike()).
        self._spike_cm = 0
        self._spike_left = 0

        # BURST scheduling state (owned by the stream thread).
        self._burst_left = 0
        self._next_burst_t = None

        # Total corrupted frames, for the GUI read-out.
        self.corrupted = 0

    def inject_spike(self, ft, frames):
        """Arm a one-shot spike: the next `frames` frames report `ft`."""
        self._spike_cm = codec.ft_to_cm(max(0.0, float(ft)))
        self._spike_left = max(1, int(frames))

    def any_active(self):
        """True when any fault mode is live (drives the GUI status label)."""
        return (self.random_prob > 0.0 or self.burst_enabled or
                self.stuck_enabled or self._spike_left > 0)

    def apply(self, cm, rng, now_s):
        """Transform one clean frame value; returns the (possibly) corrupted cm.

        Priority order: STUCK > SPIKE > BURST > RANDOM -- deterministic when
        modes are stacked, and the most aggressive fault wins.
        """
        if self.stuck_enabled:
            self.corrupted += 1
            return codec.ft_to_cm(max(0.0, self.stuck_ft))

        if self._spike_left > 0:
            self._spike_left -= 1
            self.corrupted += 1
            return self._spike_cm

        if self.burst_enabled:
            if self._next_burst_t is None:
                # First tick with bursts on: schedule the next one a full
                # period out so enabling the switch doesn't corrupt instantly.
                self._next_burst_t = now_s + max(0.5, self.burst_period_s)
            if self._burst_left > 0:
                self._burst_left -= 1
                self.corrupted += 1
                return rng.randrange(0, WIRE_MAX_CM)
            if now_s >= self._next_burst_t:
                self._next_burst_t = now_s + max(0.5, self.burst_period_s)
                self._burst_left = max(1, int(self.burst_len)) - 1
                self.corrupted += 1
                return rng.randrange(0, WIRE_MAX_CM)
        else:
            # Switch off mid-burst: kill the remainder and forget the schedule.
            self._burst_left = 0
            self._next_burst_t = None

        if self.random_prob > 0.0 and rng.random() < self.random_prob:
            self.corrupted += 1
            return rng.randrange(0, WIRE_MAX_CM)

        return cm
