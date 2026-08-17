# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Python interface to the ``vehicle_models`` header-only C++ library.

The equations of motion live in ``include/vehicle_models/*.hpp`` and are
reached through the ``_core`` pybind11 extension module. The modules below are
re-export shims over it -- they add the numpy calling convention (state and
input as 1-D arrays plus index constants) and a few GUI conveniences, and no
physics::

    types.py             <- vehicle_models/types.hpp
    parameters.py        <- vehicle_models/vehicle_parameters.hpp
    tires.py             <- vehicle_models/tire/tire_models.hpp
    ackermann.py         <- vehicle_models/ackermann.hpp
    integrator.py        <- vehicle_models/integrator.hpp
    unicycle.py          <- vehicle_models/unicycle.hpp
    kinematic_bicycle.py <- vehicle_models/kinematic_bicycle.hpp
    dynamic_bicycle.py   <- vehicle_models/dynamic_bicycle.hpp
    double_track.py      <- vehicle_models/double_track.hpp
    linear_analysis.py   <- vehicle_models/linear_analysis.hpp

Everything on top of that (``maneuvers``, ``runner``, ``performance``,
``route``, ``animation``, ``gui``) is simulation infrastructure that exists
only on the Python side.

``_core`` is a compiled extension: build it with ``pip install .`` from the
repository root, or with ``-DVEHICLE_MODELS_BUILD_PYTHON=ON``.
"""

from .ackermann import (AckermannGeometry, WheelAngles, WheelSpeeds,
                        ackermann_error, bicycle_angle_from_wheels,
                        handwheel_to_road_wheel, minimum_turn_radius,
                        road_wheel_angles, road_wheel_to_handwheel,
                        steer_angle_for_radius, turn_radius,
                        wheel_angular_rates, wheel_speeds)
from .double_track import (DoubleTrackForces, DoubleTrackModel,
                           DoubleTrackParams, FL, FR, RL, RR, WHEEL_NAMES)
from .dynamic_bicycle import (BicycleForces, BlendedBicycleModel,
                              DynamicBicycleModel, LinearLateralBicycleModel,
                              dynamic_input, dynamic_state, lateral_state,
                              side_slip_of, speed_of, steer_input)
from .integrator import (IntegratorType, simulate, step, step_euler, step_heun,
                         step_rk4)
from .kinematic_bicycle import (KinematicBicycleModel,
                                KinematicBicycleSteerModel, ReferencePoint,
                                kinematic_input, kinematic_state,
                                steer_dynamics_state)
from .parameters import (PRESETS, VehicleParameters, make_buggy_parameters,
                         make_oversteer_car_parameters,
                         make_passenger_car_parameters,
                         make_shuttle_parameters)
from .tires import (FialaTire, LinearTire, PacejkaTire, TIRE_TYPES,
                    friction_ellipse_scale, make_tire)
from .types import (GRAVITY, PI, Pose2D, clamp_value, deg2rad,
                    guard_denominator, normalize_angle, rad2deg, signum)
from .unicycle import (DifferentialDriveModel, DifferentialDriveParams,
                       UnicycleModel, unicycle_input, unicycle_state,
                       wheel_rate_input)

__version__ = "0.1.0"

__all__ = [name for name in dir() if not name.startswith("_")]
