# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Fixed-step integrators.

Port of ``include/vehicle_models/integrator.hpp``.

Every model in this package satisfies the same implicit interface::

    derivative(state, input) -> ndarray
    normalize_state(state)   -> ndarray   # wrap angles, clamp, ...

which is all the integrators below need, so a model added later works with
them unchanged.
"""

from __future__ import annotations

from enum import Enum

import numpy as np


class IntegratorType(Enum):
    EULER = "Euler"
    HEUN = "Heun"
    RK4 = "RK4"


def step_euler(model, x: np.ndarray, u, dt: float) -> np.ndarray:
    nxt = x + model.derivative(x, u) * dt
    return model.normalize_state(nxt)


def step_heun(model, x: np.ndarray, u, dt: float) -> np.ndarray:
    """Explicit trapezoidal (Heun) method, 2nd order."""
    k1 = model.derivative(x, u)
    k2 = model.derivative(x + k1 * dt, u)
    nxt = x + (k1 + k2) * (0.5 * dt)
    return model.normalize_state(nxt)


def step_rk4(model, x: np.ndarray, u, dt: float) -> np.ndarray:
    """Classical Runge-Kutta, 4th order. Default for all simulations here."""
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
    return _METHODS[method](model, x, u, dt)


def simulate(model, x0: np.ndarray, u, duration: float, dt: float,
             method: IntegratorType = IntegratorType.RK4) -> np.ndarray:
    """Integrate over ``duration`` with fixed sub-steps of ``dt``."""
    x = np.array(x0, dtype=float)
    t = 0.0
    while t < duration - 1e-12:
        h = (duration - t) if (duration - t < dt) else dt
        x = step(model, x, u, h, method)
        t += h
    return x
