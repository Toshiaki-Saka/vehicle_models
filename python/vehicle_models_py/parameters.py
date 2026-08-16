# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Vehicle parameter set shared by every model.

Port of ``include/vehicle_models/vehicle_parameters.hpp``. One struct keeps a
single vehicle definition consistent across the kinematic, dynamic and
double-track models, which is what makes cross-model plausibility checks
meaningful.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import List

from .types import GRAVITY, PI, deg2rad


@dataclass
class VehicleParameters:
    # --- geometry ----------------------------------------------------------
    l_f: float = 1.20  # CoG to front axle [m]
    l_r: float = 1.50  # CoG to rear axle [m]
    track_front: float = 1.55  # front track width [m]
    track_rear: float = 1.55  # rear track width [m]
    cg_height: float = 0.55  # CoG height above ground [m]
    wheel_radius: float = 0.32  # effective rolling radius [m]

    # --- inertia -----------------------------------------------------------
    mass: float = 1600.0  # total mass [kg]
    inertia_z: float = 2600.0  # yaw moment of inertia [kg m^2]

    # --- tires (per axle, i.e. both wheels combined) -----------------------
    cornering_stiffness_front: float = 90000.0  # [N/rad]
    cornering_stiffness_rear: float = 110000.0  # [N/rad]
    friction: float = 1.0  # peak road friction [-]

    # --- resistances -------------------------------------------------------
    drag_area: float = 0.40  # 0.5*rho*Cd*A [N/(m/s)^2]
    rolling_resistance: float = 0.012  # [-]

    # --- steering ----------------------------------------------------------
    steering_ratio: float = 16.0  # handwheel / road wheel [-]
    ackermann_ratio: float = 1.0  # 1 = ideal, 0 = parallel
    steer_max: float = deg2rad(35.0)  # road wheel limit [rad]
    steer_rate_max: float = deg2rad(90.0)  # road wheel rate limit [rad/s]
    steer_time_constant: float = 0.06  # 1st order actuator lag [s]

    # --- actuation limits --------------------------------------------------
    accel_max: float = 2.0  # [m/s^2]
    accel_min: float = -5.0  # [m/s^2]
    speed_max: float = 20.0  # [m/s]

    # --- numerics ----------------------------------------------------------
    low_speed_guard: float = 1.0  # |v_x| floor of the dynamic models [m/s]

    def wheel_base(self) -> float:
        return self.l_f + self.l_r

    def static_load_front(self) -> float:
        return self.mass * GRAVITY * self.l_r / self.wheel_base()

    def static_load_rear(self) -> float:
        return self.mass * GRAVITY * self.l_f / self.wheel_base()

    def copy(self) -> "VehicleParameters":
        return replace(self)

    def validate(self) -> List[str]:
        """Return the list of violated constraints; empty means usable."""
        errors: List[str] = []

        def require(ok: bool, msg: str) -> None:
            if not ok:
                errors.append(msg)

        require(self.l_f > 0.0, "l_f must be positive")
        require(self.l_r > 0.0, "l_r must be positive")
        require(self.track_front > 0.0, "track_front must be positive")
        require(self.track_rear > 0.0, "track_rear must be positive")
        require(self.cg_height >= 0.0, "cg_height must be non-negative")
        require(self.wheel_radius > 0.0, "wheel_radius must be positive")
        require(self.mass > 0.0, "mass must be positive")
        require(self.inertia_z > 0.0, "inertia_z must be positive")
        require(self.cornering_stiffness_front > 0.0,
                "front cornering stiffness must be positive")
        require(self.cornering_stiffness_rear > 0.0,
                "rear cornering stiffness must be positive")
        require(self.friction > 0.0, "friction must be positive")
        require(self.drag_area >= 0.0, "drag_area must be non-negative")
        require(self.rolling_resistance >= 0.0,
                "rolling_resistance must be non-negative")
        require(self.steering_ratio > 0.0, "steering_ratio must be positive")
        require(0.0 <= self.ackermann_ratio <= 1.0,
                "ackermann_ratio must be within [0, 1]")
        require(0.0 < self.steer_max < 0.5 * PI,
                "steer_max must be within (0, pi/2)")
        require(self.steer_rate_max > 0.0, "steer_rate_max must be positive")
        require(self.steer_time_constant > 0.0,
                "steer_time_constant must be positive")
        require(self.accel_max > 0.0, "accel_max must be positive")
        require(self.accel_min < 0.0, "accel_min must be negative")
        require(self.speed_max > 0.0, "speed_max must be positive")
        require(self.low_speed_guard > 0.0, "low_speed_guard must be positive")
        return errors


def make_passenger_car_parameters() -> VehicleParameters:
    """Mid-size passenger car (the defaults of VehicleParameters)."""
    return VehicleParameters()


def make_shuttle_parameters() -> VehicleParameters:
    """Compact low-speed automated shuttle (~6 m class)."""
    p = VehicleParameters()
    p.l_f = 1.35
    p.l_r = 1.65
    p.track_front = 1.48
    p.track_rear = 1.48
    p.cg_height = 0.85
    p.wheel_radius = 0.33
    p.mass = 3000.0
    p.inertia_z = 6500.0
    p.cornering_stiffness_front = 130000.0
    p.cornering_stiffness_rear = 160000.0
    p.steering_ratio = 18.0
    p.steer_max = deg2rad(30.0)
    p.accel_max = 1.0
    p.accel_min = -2.5
    p.speed_max = 5.6  # 20 km/h
    return p


def make_buggy_parameters() -> VehicleParameters:
    """Small off-road buggy / test mule."""
    p = VehicleParameters()
    p.l_f = 0.80
    p.l_r = 0.90
    p.track_front = 1.20
    p.track_rear = 1.20
    p.cg_height = 0.45
    p.wheel_radius = 0.28
    p.mass = 450.0
    p.inertia_z = 220.0
    p.cornering_stiffness_front = 25000.0
    p.cornering_stiffness_rear = 28000.0
    p.friction = 0.7
    p.drag_area = 0.25
    p.steering_ratio = 10.0
    p.steer_max = deg2rad(40.0)
    p.speed_max = 11.0
    return p


def make_oversteer_car_parameters() -> VehicleParameters:
    """Passenger car with the axle stiffness swapped, i.e. oversteering.

    Not part of the C++ presets; kept here because a finite critical speed is
    the most instructive case in the handling analysis view.
    """
    p = VehicleParameters()
    p.cornering_stiffness_front = 130000.0
    p.cornering_stiffness_rear = 70000.0
    return p


PRESETS = {
    "Passenger car": make_passenger_car_parameters,
    "Low-speed shuttle": make_shuttle_parameters,
    "Off-road buggy": make_buggy_parameters,
    "Oversteering car": make_oversteer_car_parameters,
}
