# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Fixed-step integrators.

For every model in this package the integration runs in C++
(``include/vehicle_models/integrator.hpp``): the models are type-erased behind
a common base, so the library's own Euler / Heun / RK4 code is what advances
the state.

The pure-Python branch below exists for one case only -- a model written in
Python, which the documented extension point in ``docs_*/python-api.md``
allows and which the C++ integrator cannot call. It is a transcription of the
same three formulas; if you add a model here, prefer adding it to the C++
library instead so both sides stay in step.
"""

from __future__ import annotations

from enum import Enum

import numpy as np

from . import _core
from ._core import ModelBase

__all__ = ["IntegratorType", "simulate", "step", "step_euler", "step_heun",
           "step_rk4"]


class IntegratorType(Enum):
    """Selectable method, named by the string the GUI shows.

    Kept as a Python enum rather than re-exporting the C++ one: the GUI
    iterates over it and looks members up by their ``value``, which
    pybind11 enums do not support. It maps 1:1 onto ``_core.IntegratorType``.
    """

    EULER = "Euler"
    HEUN = "Heun"
    RK4 = "RK4"


_NATIVE_METHOD = {
    IntegratorType.EULER: _core.IntegratorType.EULER,
    IntegratorType.HEUN: _core.IntegratorType.HEUN,
    IntegratorType.RK4: _core.IntegratorType.RK4,
}


def _is_native(model) -> bool:
    return isinstance(model, ModelBase)


def step_euler(model, x: np.ndarray, u, dt: float) -> np.ndarray:
    if _is_native(model):
        return _core.step_euler(model, x, u, dt)
    nxt = x + model.derivative(x, u) * dt
    return model.normalize_state(nxt)


def step_heun(model, x: np.ndarray, u, dt: float) -> np.ndarray:
    """Explicit trapezoidal (Heun) method, 2nd order."""
    if _is_native(model):
        return _core.step_heun(model, x, u, dt)
    k1 = model.derivative(x, u)
    k2 = model.derivative(x + k1 * dt, u)
    nxt = x + (k1 + k2) * (0.5 * dt)
    return model.normalize_state(nxt)


def step_rk4(model, x: np.ndarray, u, dt: float) -> np.ndarray:
    """Classical Runge-Kutta, 4th order. Default for all simulations here."""
    if _is_native(model):
        return _core.step_rk4(model, x, u, dt)
    k1 = model.derivative(x, u)
    k2 = model.derivative(x + k1 * (0.5 * dt), u)
    k3 = model.derivative(x + k2 * (0.5 * dt), u)
    k4 = model.derivative(x + k3 * dt, u)
    nxt = x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (dt / 6.0)
    return model.normalize_state(nxt)


_METHODS = {
    IntegratorType.EULER: step_euler,
    IntegratorType.HEUN: step_heun,
    IntegratorType.RK4: step_rk4,
}


def step(model, x: np.ndarray, u, dt: float,
         method: IntegratorType = IntegratorType.RK4) -> np.ndarray:
    """Zero-order-hold step with a selectable method."""
    if _is_native(model):
        return _core.step(model, x, u, dt, _NATIVE_METHOD[method])
    return _METHODS[method](model, x, u, dt)


def simulate(model, x0: np.ndarray, u, duration: float, dt: float,
             method: IntegratorType = IntegratorType.RK4) -> np.ndarray:
    """Integrate over ``duration`` with fixed sub-steps of ``dt``.

    The final step is shortened so the run ends exactly at ``duration``.
    """
    if _is_native(model):
        return _core.simulate(model, x0, u, duration, dt,
                              _NATIVE_METHOD[method])
    x = np.array(x0, dtype=float)
    t = 0.0
    while t < duration - 1e-12:
        h = (duration - t) if (duration - t < dt) else dt
        x = step(model, x, u, h, method)
        t += h
    return x
