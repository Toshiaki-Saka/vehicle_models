# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Closed-form results of the linear single-track model.

Re-exports ``include/vehicle_models/linear_analysis.hpp``. These are the
reference the simulation is checked against in the unit tests, and the kind of
quantity a plausibility monitor compares a measured yaw rate with::

    K   = m/L * (l_r/C_f - l_f/C_r)        understeer gradient
    d   = L/R + K a_y                      steer angle for a radius
    r/d = v / (L + K v^2)                  yaw rate gain
    V_ch = sqrt(L/K),  V_cr = sqrt(-L/K)   characteristic / critical speed
"""

from __future__ import annotations

from ._core import (SteadyState, YawMode, characteristic_speed, critical_speed,
                    lateral_acceleration_gain, max_lateral_acceleration,
                    neutral_steer_point, required_steer_angle, static_margin,
                    steady_state_cornering, understeer_gradient,
                    understeer_gradient_deg_per_g, yaw_mode, yaw_rate_gain)

INF = float("inf")

__all__ = ["INF", "SteadyState", "YawMode", "characteristic_speed",
           "critical_speed", "lateral_acceleration_gain",
           "max_lateral_acceleration", "neutral_steer_point",
           "required_steer_angle", "static_margin", "steady_state_cornering",
           "understeer_gradient", "understeer_gradient_deg_per_g", "yaw_mode",
           "yaw_rate_gain"]
