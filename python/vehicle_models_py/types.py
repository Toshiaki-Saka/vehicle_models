# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Numeric helpers shared by every model.

Re-exports ``include/vehicle_models/types.hpp`` through the ``_core``
extension module. Nothing is computed in Python here: ``normalize_angle``,
``clamp_value`` and ``guard_denominator`` are the C++ functions the models
themselves call, so a Python-side check and a model-side check can never
disagree.
"""

from __future__ import annotations

from ._core import (GRAVITY, PI, Pose2D, clamp_value, deg2rad,
                    guard_denominator, normalize_angle, rad2deg, signum)

__all__ = ["GRAVITY", "PI", "Pose2D", "clamp_value", "deg2rad",
           "guard_denominator", "normalize_angle", "rad2deg", "signum"]
