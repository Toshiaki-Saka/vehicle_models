# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Steering geometry of a two-axle vehicle.

Port of ``include/vehicle_models/ackermann.hpp``.

Sign convention: ``delta > 0`` means a left turn (counter-clockwise, positive
yaw rate). The "bicycle" steer angle delta is the angle of the virtual single
front wheel on the vehicle centre line.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .parameters import VehicleParameters
from .types import clamp_value, guard_denominator

INF = float("inf")


@dataclass
class AckermannGeometry:
    wheel_base: float = 2.70  # [m]
    track_front: float = 1.55  # [m]
    track_rear: float = 1.55  # [m]
    steering_ratio: float = 16.0  # handwheel angle / road wheel angle [-]
    ackermann_ratio: float = 1.0  # 1 = ideal Ackermann, 0 = parallel steer

    @staticmethod
    def from_params(p: VehicleParameters) -> "AckermannGeometry":
        return AckermannGeometry(
            wheel_base=p.wheel_base(),
            track_front=p.track_front,
            track_rear=p.track_rear,
            steering_ratio=p.steering_ratio,
            ackermann_ratio=p.ackermann_ratio,
        )


@dataclass
class WheelAngles:
    left: float = 0.0  # [rad]
    right: float = 0.0  # [rad]


@dataclass
class WheelSpeeds:
    front_left: float = 0.0  # [m/s]
    front_right: float = 0.0
    rear_left: float = 0.0
    rear_right: float = 0.0


def turn_radius(g: AckermannGeometry, delta: float) -> float:
    """Signed turn radius at the rear axle centre. +inf when driving straight."""
    t = math.tan(delta)
    if abs(t) < 1e-12:
        return INF
    return g.wheel_base / t


def steer_angle_for_radius(g: AckermannGeometry, radius: float) -> float:
    """Steer angle of the virtual bicycle wheel producing a given radius."""
    if not math.isfinite(radius) or abs(radius) < 1e-12:
        return 0.0
    return math.atan(g.wheel_base / radius)


def road_wheel_angles(g: AckermannGeometry, delta: float) -> WheelAngles:
    """Split a bicycle steer angle into the two front road wheel angles.

    Ideal Ackermann satisfies ``cot(delta_outer) - cot(delta_inner) = T / L``.
    With ``ackermann_ratio`` k the result is linearly blended toward parallel
    steering, which is how real racks with a finite Ackermann percentage are
    usually approximated.
    """
    w = WheelAngles()
    if abs(delta) < 1e-9:
        return w

    cot = 1.0 / math.tan(delta)
    half = 0.5 * g.track_front / g.wheel_base
    ideal_left = math.atan(1.0 / (cot - half))
    ideal_right = math.atan(1.0 / (cot + half))

    k = clamp_value(g.ackermann_ratio, 0.0, 1.0)
    w.left = delta + k * (ideal_left - delta)
    w.right = delta + k * (ideal_right - delta)
    return w


def bicycle_angle_from_wheels(g: AckermannGeometry, left: float,
                              right: float) -> float:
    """Equivalent bicycle angle from measured road wheel angles.

    Averaged in cotangent space, which is exact for ideal Ackermann.
    """
    if abs(left) < 1e-9 and abs(right) < 1e-9:
        return 0.0
    cot_l = 1.0 / math.tan(guard_denominator(left, 1e-9))
    cot_r = 1.0 / math.tan(guard_denominator(right, 1e-9))
    return math.atan(2.0 / (cot_l + cot_r))


def handwheel_to_road_wheel(g: AckermannGeometry, handwheel: float) -> float:
    return handwheel / g.steering_ratio


def road_wheel_to_handwheel(g: AckermannGeometry, road_wheel: float) -> float:
    return road_wheel * g.steering_ratio


def ackermann_error(g: AckermannGeometry, delta: float) -> float:
    """Deviation of the outer wheel from ideal Ackermann [rad].

    Positive = the outer wheel is steered more than ideal, i.e. toward
    parallel / anti-Ackermann.
    """
    if abs(delta) < 1e-9:
        return 0.0
    actual = road_wheel_angles(g, delta)
    inner = actual.left if delta > 0.0 else actual.right
    outer = actual.right if delta > 0.0 else actual.left
    ratio = g.track_front / g.wheel_base
    cot_inner = 1.0 / math.tan(guard_denominator(inner, 1e-9))
    cot_ideal_outer = cot_inner + (ratio if delta > 0.0 else -ratio)
    ideal_outer = math.atan(1.0 / cot_ideal_outer)
    return abs(outer) - abs(ideal_outer)


def wheel_speeds(g: AckermannGeometry, v: float, yaw_rate: float) -> WheelSpeeds:
    """Longitudinal speed of each wheel centre for a rigid-body motion.

    ``v`` is the speed at the rear axle centre, ``yaw_rate`` the body yaw rate.
    The front wheel speeds are projected onto their own steered direction,
    which is the quantity a wheel-speed sensor sees.
    """
    s = WheelSpeeds()
    s.rear_left = v - 0.5 * g.track_rear * yaw_rate
    s.rear_right = v + 0.5 * g.track_rear * yaw_rate

    vy_front = g.wheel_base * yaw_rate
    vxl = v - 0.5 * g.track_front * yaw_rate
    vxr = v + 0.5 * g.track_front * yaw_rate
    radius = INF if abs(yaw_rate) < 1e-9 else v / yaw_rate
    delta = steer_angle_for_radius(g, radius)
    wa = road_wheel_angles(g, delta)
    s.front_left = vxl * math.cos(wa.left) + vy_front * math.sin(wa.left)
    s.front_right = vxr * math.cos(wa.right) + vy_front * math.sin(wa.right)
    return s


def wheel_angular_rates(speeds: WheelSpeeds, wheel_radius: float) -> WheelSpeeds:
    """Wheel angular rates [rad/s] from the wheel centre speeds."""
    r = wheel_radius if wheel_radius > 1e-9 else 1e-9
    return WheelSpeeds(
        front_left=speeds.front_left / r,
        front_right=speeds.front_right / r,
        rear_left=speeds.rear_left / r,
        rear_right=speeds.rear_right / r,
    )


def minimum_turn_radius(g: AckermannGeometry, steer_max: float) -> float:
    """Minimum turn radius at the outer front wheel at full lock [m]."""
    wa = road_wheel_angles(g, steer_max)
    outer = wa.right if steer_max > 0.0 else wa.left
    return abs(g.wheel_base / math.sin(guard_denominator(outer, 1e-9)))
