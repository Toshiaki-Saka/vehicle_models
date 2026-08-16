# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Closed-form results of the linear single-track model.

Port of ``include/vehicle_models/linear_analysis.hpp``.

These are the reference the simulation is checked against, and the kind of
quantity a plausibility monitor compares a measured yaw rate with.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .dynamic_bicycle import LinearLateralBicycleModel
from .parameters import VehicleParameters
from .types import GRAVITY, guard_denominator, rad2deg

INF = float("inf")


def understeer_gradient(p: VehicleParameters) -> float:
    """Understeer gradient K [rad/(m/s^2)]: ``delta = L/R + K * a_y``.

    ``K > 0`` understeer, ``K = 0`` neutral steer, ``K < 0`` oversteer.
    """
    return (p.mass / p.wheel_base()
            * (p.l_r / p.cornering_stiffness_front
               - p.l_f / p.cornering_stiffness_rear))


def understeer_gradient_deg_per_g(p: VehicleParameters) -> float:
    """The same quantity in deg/g, the unit used in most test reports."""
    return rad2deg(understeer_gradient(p) * GRAVITY)


def characteristic_speed(p: VehicleParameters) -> float:
    """Characteristic speed [m/s] of an understeering vehicle.

    The yaw rate gain peaks here at half the neutral-steer value. Infinite if
    the vehicle is not understeering.
    """
    k = understeer_gradient(p)
    if k <= 1e-12:
        return INF
    return math.sqrt(p.wheel_base() / k)


def critical_speed(p: VehicleParameters) -> float:
    """Critical speed [m/s] of an oversteering vehicle (divergent above it)."""
    k = understeer_gradient(p)
    if k >= -1e-12:
        return INF
    return math.sqrt(-p.wheel_base() / k)


def neutral_steer_point(p: VehicleParameters) -> float:
    """Distance of the neutral steer point behind the front axle [m]."""
    cf = p.cornering_stiffness_front
    cr = p.cornering_stiffness_rear
    return p.wheel_base() * cr / (cf + cr)


def static_margin(p: VehicleParameters) -> float:
    """Static margin [-] = ``(x_NSP - l_f) / L``. Positive means understeer."""
    return (neutral_steer_point(p) - p.l_f) / p.wheel_base()


def yaw_rate_gain(p: VehicleParameters, vx: float) -> float:
    """Steady state yaw rate per steer angle [1/s]."""
    L = p.wheel_base()
    k = understeer_gradient(p)
    den = L + k * vx * vx
    if abs(den) < 1e-12:
        return INF
    return vx / den


def lateral_acceleration_gain(p: VehicleParameters, vx: float) -> float:
    """Steady state lateral acceleration per steer angle [m/s^2/rad]."""
    return vx * yaw_rate_gain(p, vx)


def required_steer_angle(p: VehicleParameters, radius: float,
                         vx: float) -> float:
    """Steer angle needed to hold a radius at a speed (Ackermann + slip term)."""
    if not math.isfinite(radius) or abs(radius) < 1e-9:
        return 0.0
    ay = vx * vx / radius
    return p.wheel_base() / radius + understeer_gradient(p) * ay


@dataclass
class SteadyState:
    yaw_rate: float = 0.0  # [rad/s]
    lateral_accel: float = 0.0  # [m/s^2]
    side_slip: float = 0.0  # body slip angle beta [rad]
    radius: float = 0.0  # [m]
    slip_front: float = 0.0  # front axle slip angle [rad]
    slip_rear: float = 0.0  # rear axle slip angle [rad]


def steady_state_cornering(p: VehicleParameters, vx: float,
                           delta: float) -> SteadyState:
    """Closed-form steady state cornering response of the linear model."""
    s = SteadyState()
    s.yaw_rate = yaw_rate_gain(p, vx) * delta
    s.lateral_accel = vx * s.yaw_rate
    s.radius = INF if abs(s.yaw_rate) < 1e-12 else vx / s.yaw_rate
    # beta = l_r/R - m*l_f*v^2 / (L*C_r*R)
    inv_r = (1.0 / s.radius
             if (math.isfinite(s.radius) and abs(s.radius) > 1e-12) else 0.0)
    s.side_slip = ((p.l_r - p.mass * p.l_f * vx * vx
                    / (p.wheel_base() * p.cornering_stiffness_rear)) * inv_r)
    s.slip_rear = s.side_slip - p.l_r * s.yaw_rate / guard_denominator(vx, 1e-6)
    s.slip_front = delta - (s.side_slip
                            + p.l_f * s.yaw_rate / guard_denominator(vx, 1e-6))
    return s


@dataclass
class YawMode:
    natural_frequency: float = 0.0  # [rad/s]
    damping_ratio: float = 0.0  # [-]
    real_1: float = 0.0  # eigenvalue 1, real part
    imag_1: float = 0.0  # eigenvalue 1, imaginary part
    real_2: float = 0.0
    imag_2: float = 0.0
    stable: bool = False


def yaw_mode(p: VehicleParameters, vx: float) -> YawMode:
    """Eigenvalues of the ``[v_y, r]`` system -- the yaw/sideslip mode."""
    model = LinearLateralBicycleModel(p, vx)
    a = model.state_matrix()
    tr = a[0][0] + a[1][1]
    det = a[0][0] * a[1][1] - a[0][1] * a[1][0]

    m = YawMode()
    m.natural_frequency = math.sqrt(det) if det > 0.0 else 0.0
    m.damping_ratio = -tr / (2.0 * math.sqrt(det)) if det > 0.0 else 0.0

    disc = tr * tr - 4.0 * det
    if disc >= 0.0:
        root = math.sqrt(disc)
        m.real_1 = 0.5 * (tr + root)
        m.real_2 = 0.5 * (tr - root)
    else:
        root = math.sqrt(-disc)
        m.real_1 = m.real_2 = 0.5 * tr
        m.imag_1 = 0.5 * root
        m.imag_2 = -0.5 * root
    m.stable = (m.real_1 < 0.0) and (m.real_2 < 0.0)
    return m


def max_lateral_acceleration(p: VehicleParameters) -> float:
    """Largest lateral acceleration the axles can support (simple mu*g bound)."""
    return p.friction * GRAVITY
