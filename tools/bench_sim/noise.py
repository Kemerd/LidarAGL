#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
noise.py -- realistic sensor-error injection for the bench.

A real SF30 doesn't report a perfect number: there's a little range jitter, and
over a dark/wet/over-water surface it occasionally returns its lost-signal
sentinel. Injecting both lets us prove the firmware behaves on the bench exactly
as it must in the air:

    * gaussian jitter  -> exercises the range EMA (config.h RANGE_EMA_ALPHA) so
                          the tone pitch and callout timing stay smooth.
    * lost-signal drops -> exercises the firmware's last-good HOLD (sf30c.c), so
                          a dropout never produces a garbage callout.
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
