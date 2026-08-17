# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Four-wheel (double-track) model.

Re-exports ``include/vehicle_models/double_track.hpp``.

Adds to the single-track model: individual Ackermann wheel angles, both
longitudinal and lateral load transfer, per-wheel tire saturation and combined
slip. This is where understeer at the limit, inner-wheel lift and the yaw
moment from a left/right longitudinal force difference show up -- the
single-track model cannot reproduce any of them.

Load transfer is evaluated with a one-pass predictor (static loads -> a_x/a_y
-> transferred loads -> forces). No iteration, deterministic execution time.

The state and input layouts are the same as ``dynamic_bicycle``.
"""

from __future__ import annotations

from ._core import DoubleTrackForces, DoubleTrackModel, DoubleTrackParams

# Wheel order of every per-wheel array in DoubleTrackForces.
FL, FR, RL, RR = 0, 1, 2, 3
WHEEL_NAMES = ("FL", "FR", "RL", "RR")

__all__ = ["DoubleTrackForces", "DoubleTrackModel", "DoubleTrackParams", "FL",
           "FR", "RL", "RR", "WHEEL_NAMES"]
