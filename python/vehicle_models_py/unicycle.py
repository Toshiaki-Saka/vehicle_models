# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Unicycle and differential-drive models.

Port of ``include/vehicle_models/unicycle.hpp``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .types import Pose2D, normalize_angle

# State layout [x, y, yaw]
X, Y, YAW = 0, 1, 2


def unicycle_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw], dtype=float)


def unicycle_input(v: float = 0.0, omega: float = 0.0) -> np.ndarray:
    """[v, omega]"""
    return np.array([v, omega], dtype=float)


def unicycle_pose(s: np.ndarray) -> Pose2D:
    return Pose2D(s[X], s[Y], s[YAW])


class UnicycleModel:
    """Unicycle (differential-drive body) model::

        x_dot   = v * cos(yaw)
        y_dot   = v * sin(yaw)
        yaw_dot = omega

    No non-holonomic constraint on the turn radius, so it can rotate in place.
    Correct for skid-steer platforms and the usual planner-side abstraction; it
    is NOT a valid model for an Ackermann-steered vehicle.
    """

    n_states = 3

    def derivative(self, s: np.ndarray, u: np.ndarray) -> np.ndarray:
        return np.array([u[0] * math.cos(s[YAW]),
                         u[0] * math.sin(s[YAW]),
                         u[1]], dtype=float)

    def normalize_state(self, s: np.ndarray) -> np.ndarray:
        s[YAW] = normalize_angle(s[YAW])
        return s


@dataclass
class DifferentialDriveParams:
    wheel_radius: float = 0.15  # [m]
    track: float = 0.50  # distance between the two driven wheels [m]


def wheel_rate_input(left: float = 0.0, right: float = 0.0) -> np.ndarray:
    """[omega_left, omega_right] wheel angular velocities [rad/s]"""
    return np.array([left, right], dtype=float)


@dataclass
class DifferentialDriveModel:
    """Differential drive with wheel angular rates as the input::

        v     = r * (omega_r + omega_l) / 2
        omega = r * (omega_r - omega_l) / track
    """

    params: DifferentialDriveParams = field(default_factory=DifferentialDriveParams)

    n_states = 3

    def to_body_velocity(self, u: np.ndarray) -> np.ndarray:
        v = self.params.wheel_radius * (u[1] + u[0]) * 0.5
        w = self.params.wheel_radius * (u[1] - u[0]) / self.params.track
        return unicycle_input(v, w)

    def to_wheel_rates(self, v: float, omega: float) -> np.ndarray:
        half = 0.5 * omega * self.params.track
        return wheel_rate_input((v - half) / self.params.wheel_radius,
                                (v + half) / self.params.wheel_radius)

    def derivative(self, s: np.ndarray, u: np.ndarray) -> np.ndarray:
        return UnicycleModel().derivative(s, self.to_body_velocity(u))

    def normalize_state(self, s: np.ndarray) -> np.ndarray:
        s[YAW] = normalize_angle(s[YAW])
        return s
