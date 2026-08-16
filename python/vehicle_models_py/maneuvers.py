# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Open-loop and closed-loop test manoeuvres.

Simulation infrastructure that only exists on the Python side. A manoeuvre is
anything that can answer::

    command(t, pose, v) -> (steer_command [rad], ax_override [m/s^2] or None)

``ax_override = None`` hands the longitudinal channel to the speed controller
of the runner, which is what keeps a lateral test at a constant speed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np

from .linear_analysis import required_steer_angle
from .parameters import VehicleParameters
from .route import Route, load_route, speed_profile
from .types import Pose2D, clamp_value, deg2rad, normalize_angle

Command = Tuple[float, Optional[float]]

STEP_STEER = "Step steer (J-turn)"
RAMP_STEER = "Ramp steer"
SINE_STEER = "Sine steer"
SINE_DWELL = "Sine with dwell"
CONSTANT_RADIUS = "Constant radius"
BRAKE_IN_TURN = "Braking in a turn"
STRAIGHT_LINE = "Straight-line accel / brake"
SLALOM = "Slalom (path following)"
LANE_CHANGE = "Double lane change (path following)"
ROUTE = "Reference route (path following)"

MANEUVER_KINDS = (STEP_STEER, RAMP_STEER, SINE_STEER, SINE_DWELL,
                  CONSTANT_RADIUS, BRAKE_IN_TURN, STRAIGHT_LINE, SLALOM,
                  LANE_CHANGE, ROUTE)

# Paths expressed as ``y = f(x)``: they own the lane offset / section length.
LATERAL_PATH_MANEUVERS = (SLALOM, LANE_CHANGE)
# Everything a driver model steers along, however the path is described.
PATH_MANEUVERS = (SLALOM, LANE_CHANGE, ROUTE)


@dataclass
class ManeuverConfig:
    """Everything the GUI can set about a test run."""

    kind: str = STEP_STEER
    duration: float = 8.0  # [s]
    dt: float = 0.002  # [s]

    initial_speed: float = 20.0  # [m/s]
    hold_speed: bool = True  # close a PI loop on the initial speed

    steer_amplitude: float = deg2rad(3.0)  # road wheel [rad]
    t_start: float = 1.0  # [s], step / sine onset
    frequency: float = 0.5  # [Hz], sine manoeuvres
    ramp_rate: float = deg2rad(2.0)  # [rad/s], ramp steer
    radius: float = 50.0  # [m], constant radius
    brake_accel: float = -4.0  # [m/s^2]
    brake_start: float = 3.0  # [s]

    # path-following manoeuvres
    lane_offset: float = 3.5  # [m]
    section_length: float = 30.0  # [m]
    lookahead_base: float = 4.0  # [m]
    lookahead_gain: float = 0.5  # [s]

    # reference-route following. ``route`` defaults to data/reference_route.csv
    # and ``route_speed`` to the curvature-limited profile of that route; both
    # are filled in on first use and cached here, so every model of one run
    # drives exactly the same road at exactly the same target speed.
    route: Optional[Route] = None
    route_speed: Optional[np.ndarray] = None  # [m/s] per route point
    route_ay_ratio: float = 0.35  # a_y limit of the profile, in mu*g
    route_preview: float = 2.0  # braking look-ahead [s]
    lookahead_min: float = 3.0  # [m]
    lookahead_max: float = 18.0  # [m]

    # only used to turn a radius into a steer angle
    params: Optional[VehicleParameters] = None

    def n_steps(self) -> int:
        return int(round(self.duration / self.dt))


# --------------------------------------------------------------------------
# open-loop steering profiles
# --------------------------------------------------------------------------

def _step(cfg: ManeuverConfig, t: float) -> Command:
    return (cfg.steer_amplitude if t >= cfg.t_start else 0.0), None


def _ramp(cfg: ManeuverConfig, t: float) -> Command:
    if t < cfg.t_start:
        return 0.0, None
    return cfg.ramp_rate * (t - cfg.t_start), None


def _sine(cfg: ManeuverConfig, t: float) -> Command:
    if t < cfg.t_start:
        return 0.0, None
    return cfg.steer_amplitude * math.sin(
        2.0 * math.pi * cfg.frequency * (t - cfg.t_start)), None


def _sine_dwell(cfg: ManeuverConfig, t: float) -> Command:
    """NHTSA-style sine with a 500 ms dwell at the second peak."""
    if t < cfg.t_start:
        return 0.0, None
    period = 1.0 / max(cfg.frequency, 1e-6)
    tau = t - cfg.t_start
    dwell = 0.5  # [s]
    if tau < 0.75 * period:
        return cfg.steer_amplitude * math.sin(2.0 * math.pi * cfg.frequency * tau), None
    if tau < 0.75 * period + dwell:
        return -cfg.steer_amplitude, None
    tau -= dwell
    if tau < period:
        return cfg.steer_amplitude * math.sin(2.0 * math.pi * cfg.frequency * tau), None
    return 0.0, None


def _constant_radius(cfg: ManeuverConfig, t: float) -> Command:
    p = cfg.params or VehicleParameters()
    delta = required_steer_angle(p, cfg.radius, cfg.initial_speed)
    if t < cfg.t_start:
        return 0.0, None
    # Ramp in over 1 s so the transient does not dominate the plot.
    ramp = clamp_value((t - cfg.t_start) / 1.0, 0.0, 1.0)
    return ramp * delta, None


def _brake_in_turn(cfg: ManeuverConfig, t: float) -> Command:
    steer = cfg.steer_amplitude if t >= cfg.t_start else 0.0
    if t >= cfg.brake_start:
        return steer, cfg.brake_accel
    return steer, None


def _straight_line(cfg: ManeuverConfig, t: float) -> Command:
    p = cfg.params or VehicleParameters()
    if t < cfg.brake_start:
        return 0.0, p.accel_max
    return 0.0, cfg.brake_accel


OPEN_LOOP = {
    STEP_STEER: _step,
    RAMP_STEER: _ramp,
    SINE_STEER: _sine,
    SINE_DWELL: _sine_dwell,
    CONSTANT_RADIUS: _constant_radius,
    BRAKE_IN_TURN: _brake_in_turn,
    STRAIGHT_LINE: _straight_line,
}


# --------------------------------------------------------------------------
# reference paths and the driver that follows them
# --------------------------------------------------------------------------

def lane_change_path(cfg: ManeuverConfig) -> Callable[[float], float]:
    """Smooth double lane change: out one lane, back again.

    ``tanh`` transitions instead of the ISO 3888 cone gates, so the reference
    is differentiable and the driver model stays well behaved.
    """
    s = cfg.section_length
    w = max(0.15 * s, 1.0)
    x1, x2 = s, 2.0 * s

    def y_ref(x: float) -> float:
        return 0.5 * cfg.lane_offset * (math.tanh((x - x1) / w)
                                        - math.tanh((x - x2) / w))

    return y_ref


def slalom_path(cfg: ManeuverConfig) -> Callable[[float], float]:
    """Sinusoidal slalom with one cone every ``section_length``."""
    wavelength = 2.0 * max(cfg.section_length, 1.0)

    def y_ref(x: float) -> float:
        if x <= 0.0:
            return 0.0
        return cfg.lane_offset * math.sin(2.0 * math.pi * x / wavelength)

    return y_ref


def reference_path(cfg: ManeuverConfig, x_end: float = None) -> np.ndarray:
    """Sampled reference path as an ``(N, 2)`` array, for plotting."""
    if cfg.kind == ROUTE:
        return cfg.route.points if cfg.route is not None else np.empty((0, 2))
    if cfg.kind not in LATERAL_PATH_MANEUVERS:
        return np.empty((0, 2))
    y_ref = (lane_change_path(cfg) if cfg.kind == LANE_CHANGE
             else slalom_path(cfg))
    if x_end is None:
        x_end = cfg.initial_speed * cfg.duration
    xs = np.linspace(0.0, max(x_end, 1.0), 400)
    return np.column_stack([xs, [y_ref(x) for x in xs]])


@dataclass
class PurePursuitDriver:
    """Pure pursuit on a ``y = f(x)`` reference.

    ``delta = atan(2 L sin(alpha) / L_d)`` with a speed dependent lookahead
    ``L_d = lookahead_base + lookahead_gain * v``. Deliberately simple: the
    point of the path manoeuvres is to compare vehicle models under the same
    driver, not to tune a controller.
    """

    y_ref: Callable[[float], float]
    wheel_base: float
    lookahead_base: float = 4.0
    lookahead_gain: float = 0.5
    steer_max: float = deg2rad(35.0)

    def steer(self, pose: Pose2D, v: float) -> float:
        ld = max(self.lookahead_base + self.lookahead_gain * abs(v), 1.0)
        # Target point one lookahead ahead along the path abscissa.
        x_t = pose.x + ld * math.cos(pose.yaw)
        y_t = self.y_ref(x_t)
        alpha = normalize_angle(math.atan2(y_t - pose.y, x_t - pose.x) - pose.yaw)
        delta = math.atan(2.0 * self.wheel_base * math.sin(alpha) / ld)
        return clamp_value(delta, -self.steer_max, self.steer_max)


@dataclass
class RouteDriver:
    """Driver model for a :class:`~vehicle_models_py.route.Route`.

    Lateral: the same pure pursuit law as :class:`PurePursuitDriver`, but the
    target point is taken at a fixed arc length ahead of the *projection* of
    the vehicle on the route rather than at ``x + L_d``, which is what lets it
    follow a road that turns through any angle.

    Longitudinal: a PI on the profile speed under the vehicle, plus a braking
    term that scans the profile ahead::

        a_brake = min over the preview window of (v_ref^2 - v^2) / (2 ds)

    so the vehicle brakes for a corner while it is still on the straight,
    exactly as the profile assumes. One instance per model: the projection
    index and the integrator are per-vehicle state.
    """

    route: Route
    profile: np.ndarray
    params: VehicleParameters
    dt: float = 0.002
    lookahead_base: float = 4.0
    lookahead_gain: float = 0.5
    lookahead_min: float = 3.0
    lookahead_max: float = 18.0
    preview_time: float = 2.0
    kp: float = 1.4
    ki: float = 0.5
    goal_tolerance: float = 0.5  # [m] of arc length left when the goal is met

    index: int = 0  # last projection, the search hint for the next one
    integral: float = 0.0
    finished: bool = False
    projection: Optional[object] = None

    def lookahead(self, v: float) -> float:
        return clamp_value(self.lookahead_base + self.lookahead_gain * abs(v),
                           self.lookahead_min, self.lookahead_max)

    def command(self, pose: Pose2D, v: float) -> Command:
        proj = self.route.project(pose.x, pose.y, self.index)
        self.index = proj.index
        self.projection = proj
        if proj.s >= self.route.length - self.goal_tolerance:
            self.finished = True
        if self.finished:
            # Goal reached: hold the wheel straight and brake to a stop, the
            # way a real tracker hands back at the end of its path. Chasing a
            # lookahead point beyond the route instead would steer the dynamic
            # models at v -> 0, where their guarded 1/v_x makes the lateral
            # behaviour meaningless and the vehicle wanders off the goal.
            return 0.0, clamp_value(self.kp * -v, self.params.accel_min,
                                    self.params.accel_max)

        ld = self.lookahead(v)
        tx, ty = self.route.point_at(proj.s + ld)
        alpha = normalize_angle(math.atan2(ty - pose.y, tx - pose.x) - pose.yaw)
        wheel_base = self.params.wheel_base()
        delta = math.atan(2.0 * wheel_base * math.sin(alpha) / ld)
        steer = clamp_value(delta, -self.params.steer_max,
                            self.params.steer_max)
        return steer, self._longitudinal(proj.s, v)

    def _longitudinal(self, s: float, v: float) -> float:
        p = self.params
        # Past the end of the route the profile reads its last value, which is
        # the stop speed, so no special case is needed here.
        v_ref = float(np.interp(s, self.route.s, self.profile))
        error = v_ref - v
        pi = self.kp * error + self.ki * self.integral

        cmd = pi
        preview = max(self.preview_time * abs(v), 8.0)
        lo = self.index + 1
        hi = int(np.searchsorted(self.route.s, s + preview)) + 1
        if hi > lo:
            gap = np.maximum(self.route.s[lo:hi] - s, 0.5)
            brake = float(np.min((self.profile[lo:hi] ** 2 - v * v)
                                 / (2.0 * gap)))
            # The preview only ever adds braking. Letting it cap acceleration
            # too would hold the vehicle just under the target speed on a
            # straight -- the PI would never be in charge, so its integrator
            # would never close the remaining error.
            if brake < 0.0:
                cmd = min(cmd, brake)

        clamped = clamp_value(cmd, p.accel_min, p.accel_max)
        # Integrate only while the PI term is the one in charge and linear.
        if abs(clamped - pi) < 1e-12:
            self.integral += error * self.dt
        return clamped

    @property
    def progress(self) -> float:
        return 0.0 if self.projection is None else self.projection.s


class Maneuver:
    """Uniform front end over the open-loop profiles and the driver models."""

    def __init__(self, cfg: ManeuverConfig, params: VehicleParameters):
        self.cfg = cfg
        self.params = params
        cfg.params = params
        self.driver: Optional[PurePursuitDriver] = None
        self.route_driver: Optional[RouteDriver] = None
        if cfg.kind == ROUTE:
            if cfg.route is None:
                cfg.route = load_route()
            if cfg.route_speed is None:
                cfg.route_speed = speed_profile(cfg.route, params,
                                                ay_ratio=cfg.route_ay_ratio)
            self.route_driver = RouteDriver(
                route=cfg.route,
                profile=cfg.route_speed,
                params=params,
                dt=cfg.dt,
                lookahead_base=cfg.lookahead_base,
                lookahead_gain=cfg.lookahead_gain,
                lookahead_min=cfg.lookahead_min,
                lookahead_max=cfg.lookahead_max,
                preview_time=cfg.route_preview,
            )
        elif cfg.kind in LATERAL_PATH_MANEUVERS:
            y_ref = (lane_change_path(cfg) if cfg.kind == LANE_CHANGE
                     else slalom_path(cfg))
            self.driver = PurePursuitDriver(
                y_ref=y_ref,
                wheel_base=params.wheel_base(),
                lookahead_base=cfg.lookahead_base,
                lookahead_gain=cfg.lookahead_gain,
                steer_max=params.steer_max,
            )

    def command(self, t: float, pose: Pose2D, v: float) -> Command:
        if self.route_driver is not None:
            return self.route_driver.command(pose, v)
        if self.driver is not None:
            return self.driver.steer(pose, v), None
        steer, ax = OPEN_LOOP[self.cfg.kind](self.cfg, t)
        return clamp_value(steer, -self.params.steer_max,
                           self.params.steer_max), ax

    @property
    def closed_loop(self) -> bool:
        return self.driver is not None or self.route_driver is not None

    @property
    def tracking_point(self) -> str:
        """Point of the vehicle the driver steers.

        Pure pursuit is derived at the rear axle, so the route driver asks for
        that pose from every model; without it the models whose state is the
        CoG would be steered from a point 1.5 m further forward and would cut
        every corner by that much.
        """
        return "rear_axle" if self.route_driver is not None else "state"
