# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Steering geometry of a two-axle vehicle.

Re-exports ``include/vehicle_models/ackermann.hpp``.

Sign convention: ``delta > 0`` is a left turn (counter-clockwise, positive yaw
rate). The "bicycle" steer angle is the angle of the virtual single front
wheel on the vehicle centre line.
"""

from __future__ import annotations

from ._core import (AckermannGeometry, WheelAngles, WheelSpeeds,
                    ackermann_error, bicycle_angle_from_wheels,
                    handwheel_to_road_wheel, minimum_turn_radius,
                    road_wheel_angles, road_wheel_to_handwheel,
                    steer_angle_for_radius, turn_radius, wheel_angular_rates,
                    wheel_speeds)

INF = float("inf")

__all__ = ["INF", "AckermannGeometry", "WheelAngles", "WheelSpeeds",
           "ackermann_error", "bicycle_angle_from_wheels",
           "handwheel_to_road_wheel", "minimum_turn_radius",
           "road_wheel_angles", "road_wheel_to_handwheel",
           "steer_angle_for_radius", "turn_radius", "wheel_angular_rates",
           "wheel_speeds"]
