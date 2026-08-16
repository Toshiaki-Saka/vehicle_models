# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Single-track (bicycle) dynamic models.

Port of ``include/vehicle_models/dynamic_bicycle.hpp``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from . import tires as tire
from .parameters import VehicleParameters
from .types import (GRAVITY, Pose2D, clamp_value, guard_denominator,
                    normalize_angle)

# State layout [x, y, yaw, v_x, v_y, yaw_rate] -- velocities in the body frame
X, Y, YAW, VX, VY, R = 0, 1, 2, 3, 4, 5
# Input layout [F_x, steer]; F_x is the total longitudinal tire force [N]
FX, STEER = 0, 1
# Lateral (2-DOF) state layout [x, y, yaw, v_y, yaw_rate]
LAT_VY, LAT_R = 3, 4


def dynamic_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                  vx: float = 0.0, vy: float = 0.0,
                  yaw_rate: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw, vx, vy, yaw_rate], dtype=float)


def dynamic_input(fx: float = 0.0, steer: float = 0.0) -> np.ndarray:
    return np.array([fx, steer], dtype=float)


def lateral_state(x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                  vy: float = 0.0, yaw_rate: float = 0.0) -> np.ndarray:
    return np.array([x, y, yaw, vy, yaw_rate], dtype=float)


def steer_input(steer: float = 0.0) -> np.ndarray:
    return np.array([steer], dtype=float)


def pose_of(s: np.ndarray) -> Pose2D:
    return Pose2D(s[X], s[Y], s[YAW])


def speed_of(s: np.ndarray) -> float:
    return math.hypot(s[VX], s[VY])


def side_slip_of(s: np.ndarray) -> float:
    """Body slip angle beta [rad]."""
    return math.atan2(s[VY], guard_denominator(s[VX], 1e-6))


@dataclass
class BicycleForces:
    """Intermediate quantities of one derivative evaluation.

    Useful for logging, for plausibility monitors and for comparing models
    against each other.
    """

    slip_front: float = 0.0  # [rad]
    slip_rear: float = 0.0  # [rad]
    fz_front: float = 0.0  # [N]
    fz_rear: float = 0.0  # [N]
    fy_front: float = 0.0  # [N]
    fy_rear: float = 0.0  # [N]
    f_resist: float = 0.0  # aero + rolling resistance [N]
    ax: float = 0.0  # body longitudinal acceleration [m/s^2]
    ay: float = 0.0  # body lateral acceleration [m/s^2]


def _set_tire_stiffness(t: Any, c: float, fz: float) -> None:
    """Mirror of the C++ overload set used by ``syncTiresFromParams()``."""
    if isinstance(t, (tire.LinearTire, tire.FialaTire)):
        t.cornering_stiffness = c
    elif isinstance(t, tire.PacejkaTire):
        t.B = c / (t.C * t.friction * max(fz, 1.0))


def _set_tire_friction(t: Any, mu: float) -> None:
    if isinstance(t, (tire.LinearTire, tire.FialaTire, tire.PacejkaTire)):
        t.friction = mu


@dataclass
class DynamicBicycleModel:
    """Nonlinear 3-DOF single-track (bicycle) model::

        m (vx_dot - vy*r) = Fx - Fyf sin(d) - Fres
        m (vy_dot + vx*r) = Fyf cos(d) + Fyr
        Iz r_dot          = l_f Fyf cos(d) - l_r Fyr

    Slip angles use a guarded ``1/v_x``, so the model stays finite at
    standstill, but below ``low_speed_guard`` its lateral behaviour is not
    trustworthy -- use :class:`BlendedBicycleModel` there.

    ``tire_front`` / ``tire_rear`` may be any tire model from
    :mod:`vehicle_models_py.tires`.
    """

    params: VehicleParameters = field(default_factory=VehicleParameters)
    tire_front: Any = field(default_factory=tire.LinearTire)
    tire_rear: Any = field(default_factory=tire.LinearTire)
    longitudinal_load_transfer: bool = True

    n_states = 6

    def __post_init__(self) -> None:
        self.sync_tires_from_params()

    def sync_tires_from_params(self, sync_friction: bool = False) -> None:
        """Copy the axle cornering stiffness of ``params`` into the tires.

        ``sync_friction`` is an addition over the C++ library, whose
        ``syncTiresFromParams()`` only propagates the stiffness: the tire
        friction there stays at its own default. Pass ``True`` when the
        single-track model has to run at the same road friction as
        :class:`~vehicle_models_py.double_track.DoubleTrackModel`, which does
        propagate ``params.friction``.
        """
        if sync_friction:
            _set_tire_friction(self.tire_front, self.params.friction)
            _set_tire_friction(self.tire_rear, self.params.friction)
        _set_tire_stiffness(self.tire_front, self.params.cornering_stiffness_front,
                            self.params.static_load_front())
        _set_tire_stiffness(self.tire_rear, self.params.cornering_stiffness_rear,
                            self.params.static_load_rear())

    def compute_forces(self, s: np.ndarray, u: np.ndarray) -> BicycleForces:
        p = self.params
        f = BicycleForces()
        delta = clamp_value(u[STEER], -p.steer_max, p.steer_max)
        L = p.wheel_base()
        vx_g = guard_denominator(s[VX], p.low_speed_guard)

        f.slip_front = delta - math.atan((s[VY] + p.l_f * s[R]) / vx_g)
        f.slip_rear = -math.atan((s[VY] - p.l_r * s[R]) / vx_g)

        # Quasi-static longitudinal load transfer, using the commanded Fx.
        ax_cmd = u[FX] / p.mass
        dfz = (p.mass * ax_cmd * p.cg_height / L
               if self.longitudinal_load_transfer else 0.0)
        f.fz_front = max(0.0, p.static_load_front() - dfz)
        f.fz_rear = max(0.0, p.static_load_rear() + dfz)

        f.fy_front = self.tire_front.lateral_force(f.slip_front, f.fz_front)
        f.fy_rear = self.tire_rear.lateral_force(f.slip_rear, f.fz_rear)

        f.f_resist = (p.drag_area * s[VX] * abs(s[VX])
                      + p.rolling_resistance * p.mass * GRAVITY
                      * math.tanh(s[VX] / 0.1))

        f.ax = (u[FX] - f.fy_front * math.sin(delta) - f.f_resist) / p.mass
        f.ay = (f.fy_front * math.cos(delta) + f.fy_rear) / p.mass
        return f

    def derivative(self, s: np.ndarray, u: np.ndarray) -> np.ndarray:
        p = self.params
        delta = clamp_value(u[STEER], -p.steer_max, p.steer_max)
        f = self.compute_forces(s, u)

        d = np.empty(6)
        d[0] = s[VX] * math.cos(s[YAW]) - s[VY] * math.sin(s[YAW])
        d[1] = s[VX] * math.sin(s[YAW]) + s[VY] * math.cos(s[YAW])
        d[2] = s[R]
        d[3] = f.ax + s[VY] * s[R]
        d[4] = f.ay - s[VX] * s[R]
        d[5] = (p.l_f * f.fy_front * math.cos(delta)
                - p.l_r * f.fy_rear) / p.inertia_z
        return d

    def normalize_state(self, s: np.ndarray) -> np.ndarray:
        s[YAW] = normalize_angle(s[YAW])
        return s

    def measured_lateral_acceleration(self, s: np.ndarray,
                                      u: np.ndarray) -> float:
        """Lateral acceleration as an IMU at the CoG would measure it."""
        return self.compute_forces(s, u).ay

    def input_from_acceleration(self, ax: float, steer: float) -> np.ndarray:
        """Build the input from a desired longitudinal acceleration."""
        return dynamic_input(self.params.mass * ax, steer)


@dataclass
class LinearLateralBicycleModel:
    """Linear 2-DOF lateral model at constant ``v_x``.

    The classic handling model, and the plant most lateral controllers are
    designed against::

        [vy_dot]   [a11 a12][vy]   [b1]
        [ r_dot] = [a21 a22][ r] + [b2] delta

    State ``[x, y, yaw, v_y, r]``, input ``[delta]``.
    """

    params: VehicleParameters = field(default_factory=VehicleParameters)
    longitudinal_speed: float = 10.0  # [m/s]

    n_states = 5

    def state_matrix(self) -> np.ndarray:
        """Continuous-time A matrix of ``[v_y, r]``."""
        p = self.params
        m, iz = p.mass, p.inertia_z
        cf, cr = p.cornering_stiffness_front, p.cornering_stiffness_rear
        lf, lr = p.l_f, p.l_r
        vx = guard_denominator(self.longitudinal_speed, p.low_speed_guard)
        return np.array([
            [-(cf + cr) / (m * vx), -vx - (lf * cf - lr * cr) / (m * vx)],
            [-(lf * cf - lr * cr) / (iz * vx),
             -(lf * lf * cf + lr * lr * cr) / (iz * vx)],
        ], dtype=float)

    def input_matrix(self) -> np.ndarray:
        """Continuous-time B matrix of ``[v_y, r]`` for the steer input."""
        p = self.params
        cf = p.cornering_stiffness_front
        return np.array([cf / p.mass, p.l_f * cf / p.inertia_z], dtype=float)

    def derivative(self, s: np.ndarray, u: np.ndarray) -> np.ndarray:
        p = self.params
        delta = clamp_value(u[0], -p.steer_max, p.steer_max)
        a = self.state_matrix()
        b = self.input_matrix()
        vx = self.longitudinal_speed

        d = np.empty(5)
        d[0] = vx * math.cos(s[YAW]) - s[LAT_VY] * math.sin(s[YAW])
        d[1] = vx * math.sin(s[YAW]) + s[LAT_VY] * math.cos(s[YAW])
        d[2] = s[LAT_R]
        d[3] = a[0][0] * s[LAT_VY] + a[0][1] * s[LAT_R] + b[0] * delta
        d[4] = a[1][0] * s[LAT_VY] + a[1][1] * s[LAT_R] + b[1] * delta
        return d

    def normalize_state(self, s: np.ndarray) -> np.ndarray:
        s[YAW] = normalize_angle(s[YAW])
        return s


@dataclass
class BlendedBicycleModel:
    """Kinematic / dynamic blended model.

    Below ``blend_speed_low`` the lateral states are driven toward the values
    the kinematic bicycle would produce (first order, time constant
    ``blend_time_constant``); above ``blend_speed_high`` the model is purely
    dynamic. This removes the low-speed singularity of the single-track model
    while keeping the dynamic behaviour where it matters -- the standard
    arrangement for shuttle / valet speed ranges.
    """

    dynamic: DynamicBicycleModel = field(default_factory=DynamicBicycleModel)
    blend_speed_low: float = 1.0  # [m/s]
    blend_speed_high: float = 4.0  # [m/s]
    blend_time_constant: float = 0.10  # [s]

    n_states = 6

    @property
    def params(self) -> VehicleParameters:
        return self.dynamic.params

    def blend_factor(self, vx: float) -> float:
        a = abs(vx)
        if self.blend_speed_high <= self.blend_speed_low:
            return 1.0 if a >= self.blend_speed_high else 0.0
        return clamp_value(
            (a - self.blend_speed_low)
            / (self.blend_speed_high - self.blend_speed_low), 0.0, 1.0)

    def compute_forces(self, s: np.ndarray, u: np.ndarray) -> BicycleForces:
        return self.dynamic.compute_forces(s, u)

    def derivative(self, s: np.ndarray, u: np.ndarray) -> np.ndarray:
        p = self.dynamic.params
        delta = clamp_value(u[STEER], -p.steer_max, p.steer_max)
        lam = self.blend_factor(s[VX])
        d = self.dynamic.derivative(s, u)
        if lam >= 1.0:
            return d

        # Kinematic reference for the lateral states.
        L = p.wheel_base()
        beta_kin = math.atan(p.l_r * math.tan(delta) / L)
        r_kin = s[VX] * math.cos(beta_kin) * math.tan(delta) / L
        vy_kin = s[VX] * math.tan(beta_kin)
        tau = max(self.blend_time_constant, 1e-3)

        vy_dot_kin = (vy_kin - s[VY]) / tau
        r_dot_kin = (r_kin - s[R]) / tau
        f_resist = (p.drag_area * s[VX] * abs(s[VX])
                    + p.rolling_resistance * p.mass * GRAVITY
                    * math.tanh(s[VX] / 0.1))
        vx_dot_kin = (u[FX] - f_resist) / p.mass

        d[3] = lam * d[3] + (1.0 - lam) * vx_dot_kin
        d[4] = lam * d[4] + (1.0 - lam) * vy_dot_kin
        d[5] = lam * d[5] + (1.0 - lam) * r_dot_kin
        return d

    def normalize_state(self, s: np.ndarray) -> np.ndarray:
        s[YAW] = normalize_angle(s[YAW])
        return s

    def input_from_acceleration(self, ax: float, steer: float) -> np.ndarray:
        return dynamic_input(self.params.mass * ax, steer)
