# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Numeric helpers shared by every model.

Direct port of ``include/vehicle_models/types.hpp``. The sign conventions and
the guarded division are what keep the Python results bit-comparable (to
double precision) with the C++ library.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

PI = 3.14159265358979323846
GRAVITY = 9.80665  # [m/s^2]


def deg2rad(deg: float) -> float:
    return deg * PI / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / PI


def normalize_angle(angle: float) -> float:
    """Wrap an angle into [-pi, pi)."""
    x = math.fmod(angle + PI, 2.0 * PI)
    if x < 0.0:
        x += 2.0 * PI
    return x - PI


def clamp_value(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def signum(v: float) -> int:
    return (v > 0.0) - (v < 0.0)


def guard_denominator(v: float, eps: float) -> float:
    """Keep ``|v| >= eps`` while preserving the sign.

    Used to avoid the ``1/v_x`` singularity of the dynamic models at
    standstill, so the derivative stays finite over the whole operating range.
    """
    if abs(v) >= eps:
        return v
    return -eps if v < 0.0 else eps


@dataclass
class Pose2D:
    """Planar pose, the common exchange type between models."""

    x: float = 0.0  # [m]
    y: float = 0.0  # [m]
    yaw: float = 0.0  # [rad]
