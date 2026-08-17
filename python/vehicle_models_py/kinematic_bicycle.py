# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Kinematic bicycle model (no tire slip).

Re-exports ``include/vehicle_models/kinematic_bicycle.hpp``.

``ReferencePoint`` selects which point of the vehicle the state ``(x, y)``
refers to; the three forms describe the same vehicle and differ only in the
point being integrated::

    RearAxle          yaw_dot = v tan(delta) / L
    CenterOfGravity   yaw_dot = v cos(beta) tan(delta) / L,  beta = atan(l_r tan(delta) / L)
    FrontAxle         yaw_dot = v sin(delta) / L

Valid while the lateral acceleration stays well below mu*g (rule of thumb:
below ~0.4 g). Above that, use ``DynamicBicycleModel``.
"""

from __future__ import annotations

import numpy as np

from ._core import (KinematicBicycleModel, KinematicBicycleSteerModel, Pose2D,
                    ReferencePoint)

# State layout [x, y, yaw, v], and [x, y, yaw, v, delta] with the actuator.
X, Y, YAW, V = 0, 1, 2, 3
DELTA = 4
# Input layout [acceleration, steer angle]
ACCEL, STEER = 0, 1

__all__ = ["ACCEL", "DELTA", "KinematicBicycleModel",
           "KinematicBicycleSteerModel", "ReferencePoint", "STEER", "V", "X",
           "Y", "YAW", "kinematic_input", "kinematic_state", "pose_of",
           "steer_dynamics_state"]


def kinematic_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                    v: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw, v], dtype=float)


def kinematic_input(accel: float = 0.0, steer: float = 0.0) -> np.ndarray:
    """[acceleration, steer angle]"""
    return np.array([accel, steer], dtype=float)


def steer_dynamics_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                         v: float = 0.0, steer: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw, v, steer], dtype=float)


def pose_of(s: np.ndarray) -> Pose2D:
    return Pose2D(s[X], s[Y], s[YAW])
