# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Nonlinear single-track model, its linear 2-DOF reduction and the blend.

Re-exports ``include/vehicle_models/dynamic_bicycle.hpp``::

    m (vx_dot - vy*r) = Fx - Fyf sin(d) - Fres
    m (vy_dot + vx*r) = Fyf cos(d) + Fyr
    Iz r_dot          = l_f Fyf cos(d) - l_r Fyr

``BicycleForces.ax`` / ``.ay`` are the body-frame accelerations an IMU at the
CoG would measure, not the state derivatives: the state derivative adds the
rotating-frame terms back (``vx_dot = ax + r*vy``, ``vy_dot = ay - r*vx``).

The tire type is chosen at construction. Both axles must use the same tire
class, because the C++ model is a template over a single tire type.
"""

from __future__ import annotations

import numpy as np

from ._core import (BicycleForces, BlendedBicycleModel, DynamicBicycleModel,
                    LinearLateralBicycleModel, Pose2D, dynamic_side_slip,
                    dynamic_speed)

# State layout [x, y, yaw, vx, vy, r] -- velocities in the body frame.
X, Y, YAW, VX, VY, R = 0, 1, 2, 3, 4, 5
# Input layout [F_x, steer], F_x the total longitudinal tire force [N].
FX, STEER = 0, 1
# The linear 2-DOF model keeps [x, y, yaw, vy, r] instead.
LAT_VY, LAT_R = 3, 4

__all__ = ["BicycleForces", "BlendedBicycleModel", "DynamicBicycleModel",
           "FX", "LAT_R", "LAT_VY", "LinearLateralBicycleModel", "R", "STEER",
           "VX", "VY", "X", "Y", "YAW", "dynamic_input", "dynamic_state",
           "lateral_state", "pose_of", "side_slip_of", "speed_of",
           "steer_input"]


def dynamic_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                  vx: float = 0.0, vy: float = 0.0,
                  r: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw, vx, vy, r], dtype=float)


def dynamic_input(fx: float = 0.0, steer: float = 0.0) -> np.ndarray:
    """[F_x, steer]"""
    return np.array([fx, steer], dtype=float)


def lateral_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                  vy: float = 0.0, r: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw, vy, r], dtype=float)


def steer_input(steer: float = 0.0) -> np.ndarray:
    """[steer]"""
    return np.array([steer], dtype=float)


def pose_of(s: np.ndarray) -> Pose2D:
    return Pose2D(s[X], s[Y], s[YAW])


def speed_of(s: np.ndarray) -> float:
    """Total speed hypot(vx, vy) of a 6-state dynamic state vector."""
    return dynamic_speed(s)


def side_slip_of(s: np.ndarray) -> float:
    """Body slip angle beta [rad] of a 6-state dynamic state vector."""
    return dynamic_side_slip(s)
