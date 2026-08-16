# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Performance experiments built on top of the models.

Straight-line acceleration and braking, the limit handling curve from a ramp
steer, and a simulated g-g envelope. All of them are plain simulations of the
library models -- nothing here adds physics, it only asks the models the
questions a vehicle test engineer would ask.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from . import dynamic_bicycle as db
from .double_track import DoubleTrackModel, DoubleTrackParams
from .integrator import IntegratorType, step
from .linear_analysis import (max_lateral_acceleration, understeer_gradient,
                              yaw_rate_gain)
from .parameters import VehicleParameters
from .tires import make_tire
from .types import GRAVITY, clamp_value

Progress = Optional[Callable[[float], None]]


def _single_track(params: VehicleParameters, tire_kind: str
                  ) -> db.DynamicBicycleModel:
    front = make_tire(tire_kind, params.cornering_stiffness_front,
                      params.static_load_front(), params.friction)
    rear = make_tire(tire_kind, params.cornering_stiffness_rear,
                     params.static_load_rear(), params.friction)
    model = db.DynamicBicycleModel(params, front, rear)
    model.sync_tires_from_params(sync_friction=True)
    return model


def _double_track(params: VehicleParameters, tire_kind: str,
                  dt_params: Optional[DoubleTrackParams] = None
                  ) -> DoubleTrackModel:
    front = make_tire(tire_kind, 0.5 * params.cornering_stiffness_front,
                      0.5 * params.static_load_front(), params.friction)
    rear = make_tire(tire_kind, 0.5 * params.cornering_stiffness_rear,
                     0.5 * params.static_load_rear(), params.friction)
    return DoubleTrackModel(params, dt_params or DoubleTrackParams(),
                            front, rear)


def build_model(params: VehicleParameters, tire_kind: str, four_wheel: bool):
    return (_double_track(params, tire_kind) if four_wheel
            else _single_track(params, tire_kind))


# --------------------------------------------------------------------------
# straight line
# --------------------------------------------------------------------------

@dataclass
class StraightLineResult:
    time: np.ndarray
    speed: np.ndarray
    distance: np.ndarray
    accel: np.ndarray
    metrics: Dict[str, float] = field(default_factory=dict)


def acceleration_run(params: VehicleParameters, tire_kind: str = "Fiala",
                     four_wheel: bool = True, dt: float = 0.01,
                     t_max: float = 60.0) -> StraightLineResult:
    """Full-throttle run from standstill.

    ``Fx = m * accel_max`` is held; the achieved acceleration falls off with
    speed because aerodynamic drag and rolling resistance are subtracted
    inside the model.
    """
    model = build_model(params, tire_kind, four_wheel)
    x = db.dynamic_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    u = db.dynamic_input(params.mass * params.accel_max, 0.0)
    v_target = params.speed_max

    t_list, v_list, s_list, a_list = [0.0], [0.0], [0.0], [params.accel_max]
    t = 0.0
    while t < t_max and x[db.VX] < v_target - 1e-3:
        x = step(model, x, u, dt, IntegratorType.RK4)
        t += dt
        t_list.append(t)
        v_list.append(float(x[db.VX]))
        s_list.append(float(x[db.X]))
        a_list.append(float(model.compute_forces(x, u).ax))

    res = StraightLineResult(np.asarray(t_list), np.asarray(v_list),
                             np.asarray(s_list), np.asarray(a_list))
    res.metrics["v_reached"] = float(res.speed[-1])
    res.metrics["t_total"] = float(res.time[-1])
    res.metrics["s_total"] = float(res.distance[-1])
    for mark_kmh in (30.0, 50.0, 100.0):
        mark = mark_kmh / 3.6
        if res.speed[-1] >= mark:
            idx = int(np.searchsorted(res.speed, mark))
            res.metrics["t_%d_kmh" % int(mark_kmh)] = float(res.time[idx])
    return res


def braking_run(params: VehicleParameters, v0: float, tire_kind: str = "Fiala",
                four_wheel: bool = True, dt: float = 0.005
                ) -> StraightLineResult:
    """Straight-line braking from ``v0`` to standstill at ``accel_min``."""
    model = build_model(params, tire_kind, four_wheel)
    x = db.dynamic_state(0.0, 0.0, 0.0, v0, 0.0, 0.0)
    u = db.dynamic_input(params.mass * params.accel_min, 0.0)

    t_list, v_list, s_list, a_list = [0.0], [v0], [0.0], [params.accel_min]
    t = 0.0
    while t < 30.0 and x[db.VX] > 1e-3:
        x = step(model, x, u, dt, IntegratorType.RK4)
        if x[db.VX] < 0.0:
            x[db.VX] = 0.0
        t += dt
        t_list.append(t)
        v_list.append(float(x[db.VX]))
        s_list.append(float(x[db.X]))
        a_list.append(float(model.compute_forces(x, u).ax))

    res = StraightLineResult(np.asarray(t_list), np.asarray(v_list),
                             np.asarray(s_list), np.asarray(a_list))
    res.metrics["v0"] = v0
    res.metrics["t_stop"] = float(res.time[-1])
    res.metrics["s_stop"] = float(res.distance[-1])
    res.metrics["mu_required"] = abs(params.accel_min) / GRAVITY
    return res


# --------------------------------------------------------------------------
# limit handling
# --------------------------------------------------------------------------

@dataclass
class RampSteerResult:
    time: np.ndarray
    steer: np.ndarray  # road wheel angle [rad]
    lateral_accel: np.ndarray  # [m/s^2]
    yaw_rate: np.ndarray  # [rad/s]
    side_slip: np.ndarray  # [rad]
    speed: np.ndarray  # [m/s]
    understeer_angle: np.ndarray  # delta - L/R [rad]
    metrics: Dict[str, float] = field(default_factory=dict)


def ramp_steer_run(params: VehicleParameters, speed: float,
                   tire_kind: str = "Fiala", four_wheel: bool = True,
                   ramp_rate: float = 0.02, duration: float = 20.0,
                   dt: float = 0.005, hold_speed: bool = True
                   ) -> RampSteerResult:
    """Slowly increasing steer at constant speed -- the handling-diagram test.

    ``ramp_rate`` is in rad/s at the road wheel. Slow enough (a few deg/s) the
    vehicle stays quasi-steady, so the recorded ``delta - L/R`` versus ``a_y``
    curve is the classic understeer characteristic: its initial slope is the
    understeer gradient K and its end is the friction limit.
    """
    model = build_model(params, tire_kind, four_wheel)
    x = db.dynamic_state(0.0, 0.0, 0.0, speed, 0.0, 0.0)
    L = params.wheel_base()

    n = int(duration / dt)
    t_list, steer_list, ay_list = [], [], []
    r_list, beta_list, v_list, ua_list = [], [], [], []
    integral = 0.0

    for i in range(n + 1):
        t = i * dt
        delta = clamp_value(ramp_rate * t, -params.steer_max, params.steer_max)
        vx = float(x[db.VX])
        if hold_speed:
            err = speed - vx
            raw = 1.2 * err + 0.6 * integral
            ax_cmd = clamp_value(raw, params.accel_min, params.accel_max)
            if abs(ax_cmd - raw) < 1e-12:
                integral += err * dt
        else:
            ax_cmd = 0.0
        u = db.dynamic_input(params.mass * ax_cmd, delta)
        f = model.compute_forces(x, u)

        r = float(x[db.R])
        curvature = r / vx if abs(vx) > 1e-6 else 0.0
        t_list.append(t)
        steer_list.append(delta)
        ay_list.append(f.ay)
        r_list.append(r)
        beta_list.append(db.side_slip_of(x))
        v_list.append(vx)
        ua_list.append(delta - L * curvature)

        x = step(model, x, u, dt, IntegratorType.RK4)
        if delta >= params.steer_max - 1e-9 and t > 2.0:
            # Full lock reached; a few more seconds add nothing.
            if t > duration * 0.5:
                break

    res = RampSteerResult(np.asarray(t_list), np.asarray(steer_list),
                          np.asarray(ay_list), np.asarray(r_list),
                          np.asarray(beta_list), np.asarray(v_list),
                          np.asarray(ua_list))
    ay_abs = np.abs(res.lateral_accel)
    peak = int(np.argmax(ay_abs))
    res.metrics["ay_max"] = float(ay_abs[peak])
    res.metrics["ay_max_g"] = float(ay_abs[peak] / GRAVITY)
    res.metrics["steer_at_ay_max"] = float(res.steer[peak])
    res.metrics["beta_at_ay_max"] = float(res.side_slip[peak])
    res.metrics["mu_bound"] = max_lateral_acceleration(params)
    res.metrics["k_linear"] = understeer_gradient(params)

    # Fit the understeer gradient over the linear part (|ay| < 0.3 * ay_max).
    mask = ay_abs < 0.3 * max(ay_abs[peak], 1e-6)
    if np.count_nonzero(mask) > 10:
        fit = np.polyfit(res.lateral_accel[mask], res.understeer_angle[mask], 1)
        res.metrics["k_measured"] = float(fit[0])
    return res


def max_cornering_speed(ay_max: float, radii: np.ndarray) -> np.ndarray:
    """``v_max = sqrt(a_y,max * R)`` -- the speed a radius can be taken at."""
    return np.sqrt(np.maximum(ay_max, 0.0) * radii)


# --------------------------------------------------------------------------
# g-g envelope
# --------------------------------------------------------------------------

@dataclass
class GGResult:
    ax: np.ndarray  # achieved longitudinal acceleration [m/s^2]
    ay: np.ndarray  # achieved lateral acceleration [m/s^2]
    envelope: np.ndarray  # (N, 2) upper boundary, sorted by ax
    friction_circle: float  # mu * g [m/s^2]


def gg_diagram(params: VehicleParameters, speed: float,
               tire_kind: str = "Fiala", four_wheel: bool = True,
               n_ax: int = 7, n_steer: int = 14, settle: float = 1.5,
               dt: float = 0.01, progress: Progress = None) -> GGResult:
    """Simulated g-g envelope at one speed.

    For a grid of longitudinal demands and steer angles the model is run for
    ``settle`` seconds and the resulting ``(a_x, a_y)`` recorded. The upper
    boundary of the cloud is the combined-slip envelope the tire model and the
    load transfer actually allow -- for a point-mass it would be a circle of
    radius ``mu*g``.
    """
    model = build_model(params, tire_kind, four_wheel)
    ax_cmds = np.linspace(params.accel_min, params.accel_max, n_ax)
    steers = np.linspace(0.0, params.steer_max, n_steer)

    points: List[Tuple[float, float]] = []
    total = len(ax_cmds) * len(steers)
    done = 0
    for ax_cmd in ax_cmds:
        for delta in steers:
            x = db.dynamic_state(0.0, 0.0, 0.0, speed, 0.0, 0.0)
            u = db.dynamic_input(params.mass * float(ax_cmd), float(delta))
            steps = int(settle / dt)
            for _ in range(steps):
                x = step(model, x, u, dt, IntegratorType.RK4)
                if x[db.VX] < 0.5:  # braked to a crawl, stop the run
                    break
            f = model.compute_forces(x, u)
            points.append((f.ax, f.ay))
            done += 1
            if progress is not None and done % 5 == 0:
                progress(done / total)

    arr = np.asarray(points)
    ax_vals, ay_vals = arr[:, 0], arr[:, 1]

    # Upper boundary: the largest |ay| reached in each a_x bin.
    bins = np.linspace(ax_vals.min(), ax_vals.max(), max(n_ax, 5))
    envelope = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        mask = (ax_vals >= lo) & (ax_vals <= hi)
        if np.any(mask):
            idx = int(np.argmax(np.abs(ay_vals[mask])))
            envelope.append((float(ax_vals[mask][idx]),
                             float(np.abs(ay_vals[mask][idx]))))
    envelope.sort(key=lambda p: p[0])

    if progress is not None:
        progress(1.0)
    return GGResult(ax_vals, ay_vals, np.asarray(envelope),
                    params.friction * GRAVITY)


# --------------------------------------------------------------------------
# summary table
# --------------------------------------------------------------------------

def steady_state_table(params: VehicleParameters,
                       speeds: np.ndarray) -> Dict[str, np.ndarray]:
    """Closed-form yaw-rate and lateral-acceleration gains over a speed sweep."""
    gains = np.array([yaw_rate_gain(params, float(v)) for v in speeds])
    return {
        "speed": speeds,
        "yaw_rate_gain": gains,
        "ay_gain": speeds * gains,
        "neutral_gain": speeds / params.wheel_base(),
    }
