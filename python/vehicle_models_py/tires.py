# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Tire models.

Port of ``include/vehicle_models/tire/tire_models.hpp``.

Sign convention used by every tire model here::

    positive slip angle alpha  ->  positive lateral force Fy
    alpha_front = delta - atan((v_y + l_f*r) / v_x)
    alpha_rear  =       - atan((v_y - l_r*r) / v_x)

A tire model only has to provide ``lateral_force(slip_angle, normal_force)``
and ``cornering_stiffness(normal_force)``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .types import clamp_value


@dataclass
class LinearTire:
    """Linear tire with a hard ``mu*Fz`` saturation.

    The reference model for the closed-form linear handling analysis.
    """

    cornering_stiffness: float = 90000.0  # [N/rad]
    friction: float = 1.0  # [-]

    def lateral_force(self, slip_angle: float, normal_force: float) -> float:
        f_max = self.friction * max(normal_force, 0.0)
        return clamp_value(self.cornering_stiffness * slip_angle, -f_max, f_max)

    def cornering_stiffness_at(self, normal_force: float = 0.0) -> float:
        return self.cornering_stiffness


@dataclass
class FialaTire:
    """Fiala brush model.

    Cubic build-up to full sliding at ``alpha_sl = atan(3*mu*Fz / C)``.
    Physically shaped and only two parameters.
    """

    cornering_stiffness: float = 90000.0  # [N/rad]
    friction: float = 1.0  # [-]

    def lateral_force(self, slip_angle: float, normal_force: float) -> float:
        fz = max(normal_force, 0.0)
        mu_fz = self.friction * fz
        if mu_fz <= 0.0 or self.cornering_stiffness <= 0.0:
            return 0.0

        alpha_sl = math.atan(3.0 * mu_fz / self.cornering_stiffness)
        if abs(slip_angle) >= alpha_sl:
            return mu_fz if slip_angle >= 0.0 else -mu_fz

        t = math.tan(slip_angle)
        c = self.cornering_stiffness
        f = (c * t
             - (c * c) / (3.0 * mu_fz) * abs(t) * t
             + (c * c * c) / (27.0 * mu_fz * mu_fz) * t * t * t)
        return clamp_value(f, -mu_fz, mu_fz)

    def cornering_stiffness_at(self, normal_force: float = 0.0) -> float:
        return self.cornering_stiffness


@dataclass
class PacejkaTire:
    """Pacejka Magic Formula, pure lateral slip.

    ``Fy = D * sin(C * atan(B*a - E*(B*a - atan(B*a))))``, ``D = mu * Fz``.
    """

    B: float = 10.0  # stiffness factor
    C: float = 1.9  # shape factor
    E: float = 0.97  # curvature factor
    friction: float = 1.0  # peak factor, D = friction * Fz

    def lateral_force(self, slip_angle: float, normal_force: float) -> float:
        d = self.friction * max(normal_force, 0.0)
        ba = self.B * slip_angle
        return d * math.sin(self.C * math.atan(ba - self.E * (ba - math.atan(ba))))

    def cornering_stiffness_at(self, normal_force: float) -> float:
        """dFy/dalpha at alpha = 0, i.e. B*C*D."""
        return self.B * self.C * self.friction * max(normal_force, 0.0)

    @staticmethod
    def from_cornering_stiffness(cornering_stiffness: float,
                                 nominal_load: float,
                                 friction_coeff: float = 1.0) -> "PacejkaTire":
        """Build a Pacejka set matching a cornering stiffness at a given load,
        so it can be swapped in for :class:`LinearTire`."""
        t = PacejkaTire()
        t.friction = friction_coeff
        t.C = 1.9
        t.E = 0.97
        d = friction_coeff * max(nominal_load, 1.0)
        t.B = cornering_stiffness / (t.C * d)
        return t


def friction_ellipse_scale(fx: float, friction: float,
                           normal_force: float) -> float:
    """Lateral-force scale factor once ``fx`` is already used at the same patch."""
    f_max = friction * max(normal_force, 0.0)
    if f_max <= 0.0:
        return 0.0
    ratio = clamp_value(abs(fx) / f_max, 0.0, 1.0)
    return math.sqrt(max(0.0, 1.0 - ratio * ratio))


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
