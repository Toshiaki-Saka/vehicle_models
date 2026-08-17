# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Vehicle parameter set shared by every model.

Re-exports ``include/vehicle_models/vehicle_parameters.hpp``. The field names,
the defaults and ``validate()`` all come from the C++ struct, so a parameter
set that the GUI accepts is one the C++ library accepts.

Only the GUI conveniences are defined here: an extra oversteering preset and
the name -> factory mapping the preset dropdown iterates over.
"""

from __future__ import annotations

from ._core import (VehicleParameters, make_buggy_parameters,
                    make_passenger_car_parameters, make_shuttle_parameters)

__all__ = ["PRESETS", "VehicleParameters", "make_buggy_parameters",
           "make_oversteer_car_parameters", "make_passenger_car_parameters",
           "make_shuttle_parameters"]


def make_oversteer_car_parameters() -> VehicleParameters:
    """Passenger car with the axle stiffness swapped, i.e. oversteering.

    Not one of the C++ presets; kept here because a finite critical speed is
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
