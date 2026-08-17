# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Uniform driver for every model in the library.

Each model has its own state layout and its own input (acceleration for the
kinematic models, longitudinal force for the dynamic ones). The adapters below
hide that behind one interface, so the same manoeuvre can be pushed through
all of them and the results compared channel by channel -- which is the whole
point of the GUI.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

from . import dynamic_bicycle as db
from . import kinematic_bicycle as kb
from .double_track import FL, FR, RL, RR, DoubleTrackModel, DoubleTrackParams
from .integrator import IntegratorType, step
from .maneuvers import Maneuver, ManeuverConfig
from .parameters import VehicleParameters
from .tires import make_tire
from .types import Pose2D, clamp_value

# Channels every adapter fills. Missing values are NaN, so a plot of a channel
# a model cannot produce simply shows nothing instead of a wrong zero.
CHANNELS = (
    "x", "y", "yaw", "vx", "vy", "v", "r", "beta", "ax", "ay",
    "steer_cmd", "steer", "alpha_f", "alpha_r", "curvature",
    "fz_fl", "fz_fr", "fz_rl", "fz_rr",
    "fy_f", "fy_r", "steer_l", "steer_r",
)

NAN = float("nan")


# --------------------------------------------------------------------------
# speed controller
# --------------------------------------------------------------------------

@dataclass
class SpeedController:
    """PI controller holding the target speed during a lateral manoeuvre.

    Without it the dynamic models bleed speed through cornering drag while the
    kinematic ones do not, and the comparison would be between two different
    operating points rather than between two models.
    """

    kp: float = 1.2
    ki: float = 0.6
    integral: float = 0.0

    def update(self, v_target: float, v: float, dt: float,
               accel_min: float, accel_max: float) -> float:
        err = v_target - v
        raw = self.kp * err + self.ki * self.integral
        cmd = clamp_value(raw, accel_min, accel_max)
        if abs(cmd - raw) < 1e-12:  # anti-windup: integrate only when linear
            self.integral += err * dt
        return cmd


# --------------------------------------------------------------------------
# adapters
# --------------------------------------------------------------------------

class ModelAdapter:
    """Common interface: reset, sample the outputs, advance one step."""

    key = "base"
    label = "base"
    has_wheel_loads = False
    # Distance from the rear axle to the point the state (x, y) refers to,
    # along the body x axis. The models disagree on this, and a path tracker
    # has to steer them all from the same point.
    reference_offset = 0.0

    def __init__(self, params: VehicleParameters):
        self.params = params
        self.state = np.zeros(1)

    # -- to be provided by the concrete adapters --------------------------
    def reset(self, v0: float) -> None:
        raise NotImplementedError

    def sample(self, steer_cmd: float, ax_cmd: float) -> Dict[str, float]:
        raise NotImplementedError

    def advance(self, steer_cmd: float, ax_cmd: float, dt: float,
                method: IntegratorType) -> None:
        raise NotImplementedError

    # -- shared -----------------------------------------------------------
    def pose(self) -> Pose2D:
        return Pose2D(self.state[0], self.state[1], self.state[2])

    def pose_for(self, point: str = "state") -> Pose2D:
        """Pose of ``point``: the state point itself, or the rear axle."""
        pose = self.pose()
        if point != "rear_axle" or abs(self.reference_offset) < 1e-12:
            return pose
        d = self.reference_offset
        return Pose2D(pose.x - d * math.cos(pose.yaw),
                      pose.y - d * math.sin(pose.yaw), pose.yaw)

    def speed(self) -> float:
        raise NotImplementedError

    @staticmethod
    def _blank() -> Dict[str, float]:
        return {name: NAN for name in CHANNELS}


class KinematicAdapter(ModelAdapter):
    """Kinematic bicycle, optionally with steering actuator dynamics."""

    def __init__(self, params: VehicleParameters,
                 reference: kb.ReferencePoint = kb.ReferencePoint.REAR_AXLE,
                 with_actuator: bool = False, key: str = "kinematic",
                 label: str = "Kinematic bicycle"):
        super().__init__(params)
        self.key = key
        self.label = label
        self.with_actuator = with_actuator
        base = kb.KinematicBicycleModel(params, reference)
        self.model = kb.KinematicBicycleSteerModel(base) if with_actuator else base
        self.base = base
        self.reference_offset = {
            kb.ReferencePoint.REAR_AXLE: 0.0,
            kb.ReferencePoint.CENTER_OF_GRAVITY: params.l_r,
            kb.ReferencePoint.FRONT_AXLE: params.wheel_base(),
        }[reference]

    def reset(self, v0: float) -> None:
        if self.with_actuator:
            self.state = kb.steer_dynamics_state(0.0, 0.0, 0.0, v0, 0.0)
        else:
            self.state = kb.kinematic_state(0.0, 0.0, 0.0, v0)

    def speed(self) -> float:
        return float(self.state[kb.V])

    def _road_wheel(self, steer_cmd: float) -> float:
        if self.with_actuator:
            return float(self.state[kb.DELTA])
        return clamp_value(steer_cmd, -self.params.steer_max,
                           self.params.steer_max)

    def sample(self, steer_cmd: float, ax_cmd: float) -> Dict[str, float]:
        p = self.params
        s = self.state
        delta = self._road_wheel(steer_cmd)
        v = float(s[kb.V])
        L = p.wheel_base()
        beta = 0.0
        # `==`, not `is`: ReferencePoint now comes from the C++ enum, and
        # pybind11 hands back a fresh object each time instead of the interned
        # singleton a Python Enum would return.
        if self.base.reference == kb.ReferencePoint.CENTER_OF_GRAVITY:
            beta = self.base.side_slip(delta)
            r = v * math.cos(beta) * math.tan(delta) / L
        elif self.base.reference == kb.ReferencePoint.FRONT_AXLE:
            r = v * math.sin(delta) / L
        else:
            r = v * math.tan(delta) / L

        out = self._blank()
        out.update(
            x=float(s[kb.X]), y=float(s[kb.Y]), yaw=float(s[kb.YAW]),
            v=v, vx=v * math.cos(beta), vy=v * math.sin(beta),
            r=r, beta=beta, ay=v * r, ax=ax_cmd,
            steer_cmd=steer_cmd, steer=delta,
            alpha_f=0.0, alpha_r=0.0,
            curvature=(r / v if abs(v) > 1e-6 else 0.0),
        )
        return out

    def advance(self, steer_cmd: float, ax_cmd: float, dt: float,
                method: IntegratorType) -> None:
        u = kb.kinematic_input(ax_cmd, steer_cmd)
        self.state = step(self.model, self.state, u, dt, method)


class LinearLateralAdapter(ModelAdapter):
    """Linear 2-DOF lateral model. Longitudinal speed is frozen by design."""

    key = "linear2dof"
    label = "Linear 2-DOF"

    def __init__(self, params: VehicleParameters):
        super().__init__(params)
        self.reference_offset = params.l_r
        self.model = db.LinearLateralBicycleModel(params, 10.0)

    def reset(self, v0: float) -> None:
        self.model.longitudinal_speed = v0
        self.state = db.lateral_state(0.0, 0.0, 0.0, 0.0, 0.0)

    def speed(self) -> float:
        return self.model.longitudinal_speed

    def sample(self, steer_cmd: float, ax_cmd: float) -> Dict[str, float]:
        p = self.params
        s = self.state
        vx = self.model.longitudinal_speed
        delta = clamp_value(steer_cmd, -p.steer_max, p.steer_max)
        vy = float(s[db.LAT_VY])
        r = float(s[db.LAT_R])
        a = self.model.state_matrix()
        b = self.model.input_matrix()
        vy_dot = a[0][0] * vy + a[0][1] * r + b[0] * delta
        vxg = vx if abs(vx) > 1e-6 else 1e-6

        out = self._blank()
        out.update(
            x=float(s[0]), y=float(s[1]), yaw=float(s[2]),
            vx=vx, vy=vy, v=math.hypot(vx, vy), r=r,
            beta=math.atan2(vy, vxg),
            ax=0.0, ay=vy_dot + vx * r,
            steer_cmd=steer_cmd, steer=delta,
            alpha_f=delta - (vy + p.l_f * r) / vxg,
            alpha_r=-(vy - p.l_r * r) / vxg,
            curvature=r / vxg,
            fy_f=p.cornering_stiffness_front * (delta - (vy + p.l_f * r) / vxg),
            fy_r=p.cornering_stiffness_rear * (-(vy - p.l_r * r) / vxg),
        )
        return out

    def advance(self, steer_cmd: float, ax_cmd: float, dt: float,
                method: IntegratorType) -> None:
        self.state = step(self.model, self.state,
                          db.steer_input(steer_cmd), dt, method)


class SingleTrackAdapter(ModelAdapter):
    """Nonlinear single-track model, plain or kinematic-blended."""

    def __init__(self, params: VehicleParameters, tire_kind: str = "Linear",
                 blended: bool = False, key: str = "dynamic",
                 label: str = "Dynamic bicycle"):
        super().__init__(params)
        self.key = key
        self.label = label
        self.reference_offset = params.l_r
        front = make_tire(tire_kind, params.cornering_stiffness_front,
                          params.static_load_front(), params.friction)
        rear = make_tire(tire_kind, params.cornering_stiffness_rear,
                         params.static_load_rear(), params.friction)
        core = db.DynamicBicycleModel(params, front, rear)
        # The C++ syncTiresFromParams() only propagates the stiffness; propagate
        # the road friction too so every model here runs on the same surface.
        core.sync_tires_from_params(sync_friction=True)
        self.model = db.BlendedBicycleModel(core) if blended else core
        self.core = core

    def reset(self, v0: float) -> None:
        self.state = db.dynamic_state(0.0, 0.0, 0.0, v0, 0.0, 0.0)

    def speed(self) -> float:
        return float(math.hypot(self.state[db.VX], self.state[db.VY]))

    def sample(self, steer_cmd: float, ax_cmd: float) -> Dict[str, float]:
        p = self.params
        s = self.state
        u = db.dynamic_input(p.mass * ax_cmd, steer_cmd)
        f = self.core.compute_forces(s, u)
        vx, vy, r = float(s[db.VX]), float(s[db.VY]), float(s[db.R])

        out = self._blank()
        out.update(
            x=float(s[db.X]), y=float(s[db.Y]), yaw=float(s[db.YAW]),
            vx=vx, vy=vy, v=math.hypot(vx, vy), r=r,
            beta=db.side_slip_of(s), ax=f.ax, ay=f.ay,
            steer_cmd=steer_cmd,
            steer=clamp_value(steer_cmd, -p.steer_max, p.steer_max),
            alpha_f=f.slip_front, alpha_r=f.slip_rear,
            curvature=(r / vx if abs(vx) > 1e-6 else 0.0),
            fz_fl=0.5 * f.fz_front, fz_fr=0.5 * f.fz_front,
            fz_rl=0.5 * f.fz_rear, fz_rr=0.5 * f.fz_rear,
            fy_f=f.fy_front, fy_r=f.fy_rear,
        )
        return out

    def advance(self, steer_cmd: float, ax_cmd: float, dt: float,
                method: IntegratorType) -> None:
        u = db.dynamic_input(self.params.mass * ax_cmd, steer_cmd)
        self.state = step(self.model, self.state, u, dt, method)


class DoubleTrackAdapter(ModelAdapter):
    """Four-wheel model: per-wheel loads, Ackermann angles and combined slip."""

    has_wheel_loads = True

    def __init__(self, params: VehicleParameters, tire_kind: str = "Fiala",
                 dt_params: Optional[DoubleTrackParams] = None,
                 key: str = "double_track", label: str = "Double track"):
        super().__init__(params)
        self.key = key
        self.label = label
        self.reference_offset = params.l_r
        front = make_tire(tire_kind, 0.5 * params.cornering_stiffness_front,
                          0.5 * params.static_load_front(), params.friction)
        rear = make_tire(tire_kind, 0.5 * params.cornering_stiffness_rear,
                         0.5 * params.static_load_rear(), params.friction)
        self.model = DoubleTrackModel(params, dt_params or DoubleTrackParams(),
                                      front, rear)

    def reset(self, v0: float) -> None:
        self.state = db.dynamic_state(0.0, 0.0, 0.0, v0, 0.0, 0.0)

    def speed(self) -> float:
        return float(math.hypot(self.state[db.VX], self.state[db.VY]))

    def sample(self, steer_cmd: float, ax_cmd: float) -> Dict[str, float]:
        p = self.params
        s = self.state
        u = db.dynamic_input(p.mass * ax_cmd, steer_cmd)
        f = self.model.compute_forces(s, u)
        vx, vy, r = float(s[db.VX]), float(s[db.VY]), float(s[db.R])

        out = self._blank()
        out.update(
            x=float(s[db.X]), y=float(s[db.Y]), yaw=float(s[db.YAW]),
            vx=vx, vy=vy, v=math.hypot(vx, vy), r=r,
            beta=db.side_slip_of(s), ax=f.ax, ay=f.ay,
            steer_cmd=steer_cmd,
            steer=clamp_value(steer_cmd, -p.steer_max, p.steer_max),
            alpha_f=0.5 * (f.slip_angle[FL] + f.slip_angle[FR]),
            alpha_r=0.5 * (f.slip_angle[RL] + f.slip_angle[RR]),
            curvature=(r / vx if abs(vx) > 1e-6 else 0.0),
            fz_fl=f.normal_load[FL], fz_fr=f.normal_load[FR],
            fz_rl=f.normal_load[RL], fz_rr=f.normal_load[RR],
            fy_f=f.lateral[FL] + f.lateral[FR],
            fy_r=f.lateral[RL] + f.lateral[RR],
            steer_l=f.steer.left, steer_r=f.steer.right,
        )
        return out

    def advance(self, steer_cmd: float, ax_cmd: float, dt: float,
                method: IntegratorType) -> None:
        u = db.dynamic_input(self.params.mass * ax_cmd, steer_cmd)
        self.state = step(self.model, self.state, u, dt, method)


# --------------------------------------------------------------------------
# model catalogue
# --------------------------------------------------------------------------

@dataclass
class ModelOption:
    key: str
    label: str
    build: Callable[[VehicleParameters, str], ModelAdapter]
    note: str = ""


MODEL_CATALOG: List[ModelOption] = [
    ModelOption(
        "kin_rear", "Kinematic (rear axle)",
        lambda p, tk: KinematicAdapter(p, kb.ReferencePoint.REAR_AXLE,
                                       False, "kin_rear",
                                       "Kinematic (rear axle)"),
        "No tire slip. Valid below ~0.4 g."),
    ModelOption(
        "kin_cog", "Kinematic (CoG)",
        lambda p, tk: KinematicAdapter(p, kb.ReferencePoint.CENTER_OF_GRAVITY,
                                       False, "kin_cog", "Kinematic (CoG)"),
        "Kinematic side slip beta = atan(l_r tan d / L)."),
    ModelOption(
        "kin_steer", "Kinematic + steer actuator",
        lambda p, tk: KinematicAdapter(p, kb.ReferencePoint.REAR_AXLE,
                                       True, "kin_steer",
                                       "Kinematic + steer actuator"),
        "First order EPS lag plus a rate limit."),
    ModelOption(
        "linear2dof", "Linear 2-DOF (constant vx)",
        lambda p, tk: LinearLateralAdapter(p),
        "The plant most lateral controllers are designed against."),
    ModelOption(
        "dynamic", "Dynamic bicycle",
        lambda p, tk: SingleTrackAdapter(p, tk, False, "dynamic",
                                         "Dynamic bicycle (%s tire)" % tk),
        "Nonlinear single track with the selected tire model."),
    ModelOption(
        "blended", "Blended kinematic/dynamic",
        lambda p, tk: SingleTrackAdapter(p, tk, True, "blended",
                                         "Blended (%s tire)" % tk),
        "Kinematic below the blend speed, dynamic above."),
    ModelOption(
        "double_track", "Double track (4 wheels)",
        lambda p, tk: DoubleTrackAdapter(p, tk),
        "Load transfer, Ackermann per wheel, combined slip."),
]

MODEL_BY_KEY = {m.key: m for m in MODEL_CATALOG}

# Which point of the vehicle each model integrates as its (x, y). Anything that
# overlays two models -- the animation, the tracking error -- has to undo this
# difference first, or a 1.5 m offset would be read as a difference in
# behaviour.
STATE_REFERENCE: Dict[str, str] = {
    "kin_rear": "rear_axle",
    "kin_steer": "rear_axle",
    "kin_cog": "cog",
    "linear2dof": "cog",
    "dynamic": "cog",
    "blended": "cog",
    "double_track": "cog",
}


def rear_axle_track(result: "RunResult",
                    params: VehicleParameters) -> tuple:
    """``(x, y, yaw)`` of the rear axle over a whole run.

    The point a path tracker steers, and therefore the point a cross-track
    error is honestly measured at.
    """
    x, y, yaw = result["x"], result["y"], result["yaw"]
    if STATE_REFERENCE.get(result.key, "cog") == "rear_axle":
        return x, y, yaw
    return (x - params.l_r * np.cos(yaw), y - params.l_r * np.sin(yaw), yaw)


# --------------------------------------------------------------------------
# the run itself
# --------------------------------------------------------------------------

@dataclass
class RunResult:
    key: str
    label: str
    time: np.ndarray
    channels: Dict[str, np.ndarray]
    summary: Dict[str, float] = field(default_factory=dict)
    note: str = ""

    def __getitem__(self, name: str) -> np.ndarray:
        return self.channels[name]


def _summarize(result: RunResult, cfg: ManeuverConfig) -> Dict[str, float]:
    """Peak / steady state metrics that make two models directly comparable."""
    t = result.time
    r = result.channels["r"]
    ay = result.channels["ay"]
    beta = result.channels["beta"]
    v = result.channels["v"]
    out: Dict[str, float] = {}
    if t.size == 0:
        return out

    tail = max(1, int(0.1 * t.size))
    out["r_peak"] = float(np.nanmax(np.abs(r)))
    out["r_final"] = float(np.nanmean(r[-tail:]))
    out["ay_peak"] = float(np.nanmax(np.abs(ay)))
    out["ay_final"] = float(np.nanmean(ay[-tail:]))
    out["beta_peak"] = float(np.nanmax(np.abs(beta)))
    out["beta_final"] = float(np.nanmean(beta[-tail:]))
    out["v_final"] = float(np.nanmean(v[-tail:]))

    # Step response metrics only mean something for a step input.
    r_final = out["r_final"]
    if abs(r_final) > 1e-6:
        out["overshoot"] = (out["r_peak"] - abs(r_final)) / abs(r_final)
        target = 0.9 * abs(r_final)
        idx = np.where(np.abs(r) >= target)[0]
        if idx.size:
            out["t_response"] = float(t[idx[0]] - cfg.t_start)
    return out


def run_maneuver(params: VehicleParameters, cfg: ManeuverConfig,
                 model_keys: Sequence[str], tire_kind: str = "Fiala",
                 method: IntegratorType = IntegratorType.RK4,
                 progress: Optional[Callable[[float], None]] = None,
                 ) -> List[RunResult]:
    """Run one manoeuvre through several models and return their histories."""
    adapters = [MODEL_BY_KEY[k].build(params, tire_kind) for k in model_keys
                if k in MODEL_BY_KEY]
    # One manoeuvre object per model: a closed-loop driver carries state (its
    # place on the route, its speed integrator) that must not be shared.
    maneuvers = [Maneuver(cfg, params) for _ in adapters]
    controllers = [SpeedController() for _ in adapters]
    for a in adapters:
        a.reset(cfg.initial_speed)

    n = cfg.n_steps()
    dt = cfg.dt
    times = np.empty(n + 1)
    logs: List[Dict[str, List[float]]] = [
        {name: [] for name in CHANNELS} for _ in adapters]

    t0 = time.perf_counter()
    for i in range(n + 1):
        t = i * dt
        times[i] = t
        for adapter, log, ctrl, maneuver in zip(adapters, logs, controllers,
                                                maneuvers):
            pose = adapter.pose_for(maneuver.tracking_point)
            steer_cmd, ax_override = maneuver.command(t, pose, adapter.speed())
            if ax_override is None:
                if cfg.hold_speed:
                    ax_cmd = ctrl.update(cfg.initial_speed, adapter.speed(), dt,
                                         params.accel_min, params.accel_max)
                else:
                    ax_cmd = 0.0
            else:
                ax_cmd = ax_override

            sample = adapter.sample(steer_cmd, ax_cmd)
            for name in CHANNELS:
                log[name].append(sample[name])
            if i < n:
                adapter.advance(steer_cmd, ax_cmd, dt, method)
        if progress is not None and (i % 200 == 0):
            progress(i / max(n, 1))

    results = []
    for adapter, log in zip(adapters, logs):
        channels = {name: np.asarray(values, dtype=float)
                    for name, values in log.items()}
        res = RunResult(adapter.key, adapter.label, times, channels,
                        note=MODEL_BY_KEY[adapter.key].note)
        res.summary = _summarize(res, cfg)
        results.append(res)

    if progress is not None:
        progress(1.0)
    for res in results:
        res.summary["wall_time"] = time.perf_counter() - t0
    return results


def to_csv(results: Sequence[RunResult]) -> str:
    """Flatten several runs into one long CSV (one block of columns per model)."""
    if not results:
        return ""
    header = ["t"]
    columns = [results[0].time]
    for res in results:
        for name in CHANNELS:
            data = res.channels[name]
            if np.all(np.isnan(data)):
                continue
            header.append("%s.%s" % (res.key, name))
            columns.append(data)
    rows = [",".join(header)]
    matrix = np.column_stack(columns)
    for row in matrix:
        rows.append(",".join("%.6g" % value for value in row))
    return "\n".join(rows) + "\n"
