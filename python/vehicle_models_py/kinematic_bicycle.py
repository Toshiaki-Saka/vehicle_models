# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Kinematic bicycle models.

Port of ``include/vehicle_models/kinematic_bicycle.hpp``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from .parameters import VehicleParameters
from .types import Pose2D, clamp_value, normalize_angle

# State layout [x, y, yaw, v]
X, Y, YAW, V = 0, 1, 2, 3
# Steer-dynamics state adds [.., delta]
DELTA = 4
# Input layout [accel, steer]
ACCEL, STEER = 0, 1


class ReferencePoint(Enum):
    """Point of the vehicle the state (x, y) refers to."""

    REAR_AXLE = "RearAxle"
    CENTER_OF_GRAVITY = "CenterOfGravity"
    FRONT_AXLE = "FrontAxle"


def kinematic_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                    v: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw, v], dtype=float)


def kinematic_input(accel: float = 0.0, steer: float = 0.0) -> np.ndarray:
    return np.array([accel, steer], dtype=float)


def steer_dynamics_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                         v: float = 0.0, steer: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw, v, steer], dtype=float)


def pose_of(s: np.ndarray) -> Pose2D:
    return Pose2D(s[X], s[Y], s[YAW])


@dataclass
class KinematicBicycleModel:
    """Kinematic bicycle model (no tire slip).

    RearAxle reference::

        x_dot = v cos(yaw), y_dot = v sin(yaw)
        yaw_dot = v tan(delta) / L,  v_dot = a

    CenterOfGravity reference (``beta = atan(l_r tan(delta) / L)``)::

        x_dot = v cos(yaw + beta), y_dot = v sin(yaw + beta)
        yaw_dot = v cos(beta) tan(delta) / L,  v_dot = a

    FrontAxle reference (``v`` is the front wheel speed)::

        x_dot = v cos(yaw + delta), y_dot = v sin(yaw + delta)
        yaw_dot = v sin(delta) / L,  v_dot = a

    Valid while the lateral acceleration stays well below ``mu*g`` (rule of
    thumb: below ~0.4 g). Above that, use :class:`DynamicBicycleModel`.
    """

    params: VehicleParameters = field(default_factory=VehicleParameters)
    reference: ReferencePoint = ReferencePoint.REAR_AXLE
    apply_limits: bool = True  # clamp steer / accel / speed to the params

    n_states = 4

    def side_slip(self, steer: float) -> float:
        """Body slip angle at the CoG for the current steer angle."""
        return math.atan(self.params.l_r * math.tan(steer)
                         / self.params.wheel_base())

    def derivative(self, s: np.ndarray, u: np.ndarray) -> np.ndarray:
        p = self.params
        delta = (clamp_value(u[STEER], -p.steer_max, p.steer_max)
                 if self.apply_limits else u[STEER])
        accel = (clamp_value(u[ACCEL], p.accel_min, p.accel_max)
                 if self.apply_limits else u[ACCEL])
        if self.apply_limits:
            # Stop integrating speed once the envelope is reached.
            if s[V] >= p.speed_max and accel > 0.0:
                accel = 0.0
            if s[V] <= -p.speed_max and accel < 0.0:
                accel = 0.0

        L = p.wheel_base()
        d = np.empty(4)
        if self.reference is ReferencePoint.CENTER_OF_GRAVITY:
            beta = self.side_slip(delta)
            d[0] = s[V] * math.cos(s[YAW] + beta)
            d[1] = s[V] * math.sin(s[YAW] + beta)
            d[2] = s[V] * math.cos(beta) * math.tan(delta) / L
        elif self.reference is ReferencePoint.FRONT_AXLE:
            d[0] = s[V] * math.cos(s[YAW] + delta)
            d[1] = s[V] * math.sin(s[YAW] + delta)
            d[2] = s[V] * math.sin(delta) / L
        else:  # REAR_AXLE
            d[0] = s[V] * math.cos(s[YAW])
            d[1] = s[V] * math.sin(s[YAW])
            d[2] = s[V] * math.tan(delta) / L
        d[3] = accel
        return d

    def normalize_state(self, s: np.ndarray) -> np.ndarray:
        s[YAW] = normalize_angle(s[YAW])
        if self.apply_limits:
            s[V] = clamp_value(s[V], -self.params.speed_max,
                               self.params.speed_max)
        return s

    def lateral_acceleration(self, s: np.ndarray, steer: float) -> float:
        """Lateral acceleration implied by the kinematics, ``v^2 / R``."""
        return s[V] * s[V] * math.tan(steer) / self.params.wheel_base()


@dataclass
class KinematicBicycleSteerModel:
    """Kinematic bicycle whose steering actuator has a first order lag and a
    rate limit -- the difference that matters when validating a path-tracking
    controller against a real EPS::

        delta_dot = clamp((delta_cmd - delta) / tau, +-rate_max)

    State ``[x, y, yaw, v, delta]``, input ``[accel, steer command]``.
    """

    base: KinematicBicycleModel = field(default_factory=KinematicBicycleModel)

    n_states = 5

    @property
    def params(self) -> VehicleParameters:
        return self.base.params

    def derivative(self, s: np.ndarray, u: np.ndarray) -> np.ndarray:
        core = self.base.derivative(s[:4], kinematic_input(u[ACCEL], s[DELTA]))
        p = self.base.params
        cmd = clamp_value(u[STEER], -p.steer_max, p.steer_max)
        rate = clamp_value((cmd - s[DELTA]) / p.steer_time_constant,
                           -p.steer_rate_max, p.steer_rate_max)
        d = np.empty(5)
        d[:4] = core
        d[4] = rate
        return d

    def normalize_state(self, s: np.ndarray) -> np.ndarray:
        p = self.base.params
        s[YAW] = normalize_angle(s[YAW])
        if self.base.apply_limits:
            s[V] = clamp_value(s[V], -p.speed_max, p.speed_max)
            s[DELTA] = clamp_value(s[DELTA], -p.steer_max, p.steer_max)
        return s
