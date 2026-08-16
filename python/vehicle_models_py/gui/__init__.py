# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Tkinter + matplotlib front end for the vehicle models.

``VehicleModelsApp`` and ``main`` are resolved on first access rather than at
import time, so ``from .gui import theme`` -- which is all the animation
renderer needs -- costs nothing and works on a machine without tkinter.
"""

from typing import Any

__all__ = ["VehicleModelsApp", "main"]


def __getattr__(name: str) -> Any:  # PEP 562
    if name in __all__:
        from . import app
        return getattr(app, name)
    raise AttributeError("module %r has no attribute %r" % (__name__, name))


def __dir__():
    return sorted(list(globals()) + __all__)
