# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Four-wheel (double-track) model.

Port of ``include/vehicle_models/double_track.hpp``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, List

import numpy as np

from . import tires as tire
from .ackermann import AckermannGeometry, WheelAngles, road_wheel_angles
from .dynamic_bicycle import (FX, R, STEER, VX, VY, YAW, _set_tire_friction,
                              _set_tire_stiffness, dynamic_input)
from .parameters import VehicleParameters
from .types import clamp_value, guard_denominator, normalize_angle

# Wheel order used by every per-wheel array below.
FL, FR, RL, RR = 0, 1, 2, 3
WHEEL_NAMES = ("FL", "FR", "RL", "RR")


@dataclass
class DoubleTrackForces:
    slip_angle: List[float] = field(default_factory=lambda: [0.0] * 4)  # [rad]
    normal_load: List[float] = field(default_factory=lambda: [0.0] * 4)  # [N]
    lateral: List[float] = field(default_factory=lambda: [0.0] * 4)  # [N]
    longitudinal: List[float] = field(default_factory=lambda: [0.0] * 4)  # [N]
    steer: WheelAngles = field(default_factory=WheelAngles)  # front wheels [rad]
    ax: float = 0.0  # [m/s^2]
    ay: float = 0.0  # [m/s^2]

    def load_sum(self) -> float:
        return sum(self.normal_load)


@dataclass
class DoubleTrackParams:
    """Additional parameters that only the double-track model needs."""

    front_roll_stiffness_ratio: float = 0.55  # lateral transfer share, front axle
    front_drive_ratio: float = 0.0  # 0 = RWD, 1 = FWD, 0.5 = AWD
    front_brake_ratio: float = 0.65  # brake force share on the front axle
    combined_slip: bool = True  # apply the friction ellipse


@dataclass
class DoubleTrackModel:
    """3-DOF double-track (four-wheel) model.

    Adds to the single-track model: individual Ackermann wheel angles,
    longitudinal and lateral load transfer, per-wheel tire saturation and
    combined slip. This is where understeer at the limit, inner-wheel lift and
    the difference between ideal and real Ackermann actually show up -- the
    single-track model cannot reproduce any of them.

    Load transfer is evaluated with a one-pass predictor: forces are first
    computed on the static loads to estimate ``a_x`` / ``a_y``, then recomputed
    on the transferred loads. No iteration, deterministic execution time.

    State ``[x, y, yaw, vx, vy, r]``, input ``[Fx total, steer]``.
    """

    params: VehicleParameters = field(default_factory=VehicleParameters)
    dt_params: DoubleTrackParams = field(default_factory=DoubleTrackParams)
    tire_front: Any = field(default_factory=tire.FialaTire)  # per wheel
    tire_rear: Any = field(default_factory=tire.FialaTire)

    n_states = 6

    def __post_init__(self) -> None:
        self.sync_tires_from_params()

    def sync_tires_from_params(self) -> None:
        """Split the axle cornering stiffness of ``params`` over the two wheels,
        so the double-track and single-track models describe the same vehicle."""
        p = self.params
        _set_tire_friction(self.tire_front, p.friction)
        _set_tire_friction(self.tire_rear, p.friction)
        _set_tire_stiffness(self.tire_front, 0.5 * p.cornering_stiffness_front,
                            0.5 * p.static_load_front())
        _set_tire_stiffness(self.tire_rear, 0.5 * p.cornering_stiffness_rear,
                            0.5 * p.static_load_rear())

    def geometry(self) -> AckermannGeometry:
        return AckermannGeometry.from_params(self.params)

    def compute_forces(self, s: np.ndarray, u: np.ndarray) -> DoubleTrackForces:
        p = self.params
        delta = clamp_value(u[STEER], -p.steer_max, p.steer_max)
        tf = p.track_front
        tr = p.track_rear

        f = DoubleTrackForces()
        f.steer = road_wheel_angles(self.geometry(), delta)

        # --- slip angles, per wheel ----------------------------------------
        vy = s[VY]
        r = s[R]
        eps = p.low_speed_guard
        vx_fl = guard_denominator(s[VX] - 0.5 * tf * r, eps)
        vx_fr = guard_denominator(s[VX] + 0.5 * tf * r, eps)
        vx_rl = guard_denominator(s[VX] - 0.5 * tr * r, eps)
        vx_rr = guard_denominator(s[VX] + 0.5 * tr * r, eps)

        f.slip_angle[FL] = f.steer.left - math.atan((vy + p.l_f * r) / vx_fl)
        f.slip_angle[FR] = f.steer.right - math.atan((vy + p.l_f * r) / vx_fr)
        f.slip_angle[RL] = -math.atan((vy - p.l_r * r) / vx_rl)
        f.slip_angle[RR] = -math.atan((vy - p.l_r * r) / vx_rr)

        # --- longitudinal force distribution --------------------------------
        front_share = (self.dt_params.front_drive_ratio if u[FX] >= 0.0
                       else self.dt_params.front_brake_ratio)
        fx_front = 0.5 * front_share * u[FX]
        fx_rear = 0.5 * (1.0 - front_share) * u[FX]
        f.longitudinal[FL] = fx_front
        f.longitudinal[FR] = fx_front
        f.longitudinal[RL] = fx_rear
        f.longitudinal[RR] = fx_rear

        # --- pass 1: static loads, to estimate a_x / a_y ---------------------
        ax_est = u[FX] / p.mass
        tmp = DoubleTrackForces(slip_angle=list(f.slip_angle),
                                normal_load=list(f.normal_load),
                                lateral=list(f.lateral),
                                longitudinal=list(f.longitudinal),
                                steer=f.steer)
        self._distribute_loads(tmp, ax_est, 0.0)
        self._evaluate_tires(tmp)
        ay_est = self._lateral_acceleration(tmp, f.steer)

        # --- pass 2: transferred loads --------------------------------------
        self._distribute_loads(f, ax_est, ay_est)
        self._evaluate_tires(f)
        f.ax = self._longitudinal_acceleration(f, f.steer)
        f.ay = self._lateral_acceleration(f, f.steer)
        return f

    def derivative(self, s: np.ndarray, u: np.ndarray) -> np.ndarray:
        p = self.params
        f = self.compute_forces(s, u)
        tf = p.track_front
        tr = p.track_rear

        def fx_body(i: int, steer_angle: float) -> float:
            return (f.longitudinal[i] * math.cos(steer_angle)
                    - f.lateral[i] * math.sin(steer_angle))

        def fy_body(i: int, steer_angle: float) -> float:
            return (f.longitudinal[i] * math.sin(steer_angle)
                    + f.lateral[i] * math.cos(steer_angle))

        # Yaw moment from all four contact patches.
        mz = (p.l_f * (fy_body(FL, f.steer.left) + fy_body(FR, f.steer.right))
              - p.l_r * (fy_body(RL, 0.0) + fy_body(RR, 0.0))
              + 0.5 * tf * (fx_body(FR, f.steer.right) - fx_body(FL, f.steer.left))
              + 0.5 * tr * (fx_body(RR, 0.0) - fx_body(RL, 0.0)))

        d = np.empty(6)
        d[0] = s[VX] * math.cos(s[YAW]) - s[VY] * math.sin(s[YAW])
        d[1] = s[VX] * math.sin(s[YAW]) + s[VY] * math.cos(s[YAW])
        d[2] = s[R]
        d[3] = f.ax + s[VY] * s[R]
        d[4] = f.ay - s[VX] * s[R]
        d[5] = mz / p.inertia_z
        return d

    def normalize_state(self, s: np.ndarray) -> np.ndarray:
        s[YAW] = normalize_angle(s[YAW])
        return s

    def input_from_acceleration(self, ax: float, steer: float) -> np.ndarray:
        return dynamic_input(self.params.mass * ax, steer)

    # -- internals ----------------------------------------------------------

    def _distribute_loads(self, f: DoubleTrackForces, ax: float,
                          ay: float) -> None:
        p = self.params
        L = p.wheel_base()
        h = p.cg_height
        m = p.mass

        fz_front_axle = max(0.0, p.static_load_front() - m * ax * h / L)
        fz_rear_axle = max(0.0, p.static_load_rear() + m * ax * h / L)

        kf = clamp_value(self.dt_params.front_roll_stiffness_ratio, 0.0, 1.0)
        d_front = m * ay * h * kf / p.track_front
        d_rear = m * ay * h * (1.0 - kf) / p.track_rear

        # Positive a_y (left turn) unloads the left (inner) wheels.
        f.normal_load[FL] = max(0.0, 0.5 * fz_front_axle - d_front)
        f.normal_load[FR] = max(0.0, 0.5 * fz_front_axle + d_front)
        f.normal_load[RL] = max(0.0, 0.5 * fz_rear_axle - d_rear)
        f.normal_load[RR] = max(0.0, 0.5 * fz_rear_axle + d_rear)

    def _evaluate_tires(self, f: DoubleTrackForces) -> None:
        for i in range(4):
            t = self.tire_front if i < 2 else self.tire_rear
            fy = t.lateral_force(f.slip_angle[i], f.normal_load[i])
            if self.dt_params.combined_slip:
                fy *= tire.friction_ellipse_scale(f.longitudinal[i],
                                                  self.params.friction,
                                                  f.normal_load[i])
            f.lateral[i] = fy

    def _longitudinal_acceleration(self, f: DoubleTrackForces,
                                   steer: WheelAngles) -> float:
        fx = 0.0
        fx += (f.longitudinal[FL] * math.cos(steer.left)
               - f.lateral[FL] * math.sin(steer.left))
        fx += (f.longitudinal[FR] * math.cos(steer.right)
               - f.lateral[FR] * math.sin(steer.right))
        fx += f.longitudinal[RL]
        fx += f.longitudinal[RR]
        return fx / self.params.mass

    def _lateral_acceleration(self, f: DoubleTrackForces,
                              steer: WheelAngles) -> float:
        fy = 0.0
        fy += (f.longitudinal[FL] * math.sin(steer.left)
               + f.lateral[FL] * math.cos(steer.left))
        fy += (f.longitudinal[FR] * math.sin(steer.right)
               + f.lateral[FR] * math.cos(steer.right))
        fy += f.lateral[RL]
        fy += f.lateral[RR]
        return fy / self.params.mass
