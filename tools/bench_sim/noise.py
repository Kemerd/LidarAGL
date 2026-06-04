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
                sigma_ft: float, dropout_prob: float, rng) -> int:
    """Turn a desired AGL into the centimetre value the sensor would report.

    The firmware computes AGL = range - ground_ref, so the bench sends RANGE:
        range_ft = agl_ft + ground_offset_ft   (+ gaussian jitter)
    With probability `dropout_prob` we instead emit the lost-signal sentinel.

    `rng` is a random.Random instance (passed in so the engine controls seeding).
    """
    if dropout_prob > 0.0 and rng.random() < dropout_prob:
        return P.SF30_LOST_SIGNAL_CM

    rng_ft = agl_ft + ground_offset_ft
    if sigma_ft > 0.0:
        rng_ft += rng.gauss(0.0, sigma_ft)
    if rng_ft < 0.0:
        rng_ft = 0.0
    return codec.ft_to_cm(rng_ft)
