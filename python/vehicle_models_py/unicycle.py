# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Unicycle and differential-drive models.

Re-exports ``include/vehicle_models/unicycle.hpp``::

    x_dot   = v * cos(yaw)
    y_dot   = v * sin(yaw)
    yaw_dot = omega

No non-holonomic constraint on the turn radius, so it can rotate in place.
Correct for skid-steer platforms and the usual planner-side abstraction; it is
NOT a valid model for an Ackermann-steered vehicle.

The state and input layouts below are the Python calling convention: the C++
side takes the same numbers in the same order.
"""

from __future__ import annotations

import numpy as np

from ._core import DifferentialDriveModel, DifferentialDriveParams, UnicycleModel
from ._core import Pose2D

# State layout [x, y, yaw]
X, Y, YAW = 0, 1, 2

__all__ = ["DifferentialDriveModel", "DifferentialDriveParams", "UnicycleModel",
           "X", "Y", "YAW", "unicycle_input", "unicycle_pose",
           "unicycle_state", "wheel_rate_input"]


def unicycle_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw], dtype=float)


def unicycle_input(v: float = 0.0, omega: float = 0.0) -> np.ndarray:
    """[v, omega]"""
    return np.array([v, omega], dtype=float)


def wheel_rate_input(left: float = 0.0, right: float = 0.0) -> np.ndarray:
    """[omega_left, omega_right] wheel angular velocities [rad/s]"""
    return np.array([left, right], dtype=float)


def unicycle_pose(s: np.ndarray) -> Pose2D:
    return Pose2D(s[X], s[Y], s[YAW])
