# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Tire models.

Re-exports ``include/vehicle_models/tire/tire_models.hpp``.

Sign convention used by every tire model here::

    positive slip angle alpha  ->  positive lateral force Fy
    alpha_front = delta - atan((v_y + l_f*r) / v_x)
    alpha_rear  =       - atan((v_y - l_r*r) / v_x)

The C++ templates are parameterised on a single tire type, so a model's two
axles must use the same tire class; mixing them raises ``TypeError``.
"""

from __future__ import annotations

from ._core import FialaTire, LinearTire, PacejkaTire, friction_ellipse_scale

__all__ = ["FialaTire", "LinearTire", "PacejkaTire", "TIRE_TYPES",
           "friction_ellipse_scale", "make_tire"]

TIRE_TYPES = {
    "Linear": LinearTire,
    "Fiala": FialaTire,
    "Pacejka": PacejkaTire,
}


def make_tire(kind: str, cornering_stiffness: float, nominal_load: float,
              friction: float) -> object:
    """Create a tire of ``kind`` matched to a cornering stiffness and load.

    Convenience used by the GUI so the three tire models describe the same
    axle: identical slope at ``alpha = 0`` and identical peak ``mu*Fz``.
    """
    if kind == "Linear":
        return LinearTire(cornering_stiffness=cornering_stiffness,
                          friction=friction)
    if kind == "Fiala":
        return FialaTire(cornering_stiffness=cornering_stiffness,
                         friction=friction)
    if kind == "Pacejka":
        return PacejkaTire.from_cornering_stiffness(
            cornering_stiffness, nominal_load, friction)
    raise ValueError("unknown tire model: " + str(kind))
