# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Reference route: geometry, speed profile and tracking error.

Simulation infrastructure that only exists on the Python side, and the piece
the manoeuvre catalogue was missing: the slalom and the lane change are
``y = f(x)`` curves, which cannot describe a road that turns through 90 deg or
doubles back. A :class:`Route` is a polyline with arc length, so any geometry
works.

Three things live here, and they are deliberately independent of any vehicle
model:

* :func:`load_route` reads the CSV in ``data/`` (columns located by name, so
  extra columns and a different order are fine),
* :func:`speed_profile` turns the curvature into the speed a driver would
  actually carry -- lateral acceleration first, then a backward pass for
  braking and a forward pass for the power available,
* :func:`analyse_tracking` projects a driven trajectory back onto the route to
  get the signed cross-track error, which is what makes two models comparable
  on a course neither of them follows exactly.
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from .parameters import VehicleParameters
from .types import GRAVITY, normalize_angle

# ``python/vehicle_models_py/route.py`` -> repository root -> data/
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_ROUTE_PATH = os.path.join(_ROOT, "data", "reference_route.csv")

# Column names accepted for each quantity, lower cased. Only x and y are
# required; yaw and curvature are derived from the geometry when missing.
COLUMN_ALIASES: Dict[str, Tuple[str, ...]] = {
    "x": ("x_m", "x", "east_m", "x[m]", "pos_x"),
    "y": ("y_m", "y", "north_m", "y[m]", "pos_y"),
    "yaw": ("yaw", "yaw_rad", "heading", "theta", "psi"),
    "curvature": ("curvature", "kappa", "curvature_1pm", "k"),
}


@dataclass
class Projection:
    """Where a point sits relative to the route."""

    index: int = 0  # index of the segment start, usable as the next hint
    s: float = 0.0  # arc length of the foot point [m]
    lateral: float = 0.0  # signed offset, + = left of the direction of travel [m]
    heading: float = 0.0  # route tangent there [rad]
    curvature: float = 0.0  # route curvature there [1/m]
    distance: float = 0.0  # |lateral| except past the ends [m]


@dataclass
class Route:
    """A reference path as a polyline, with arc length and curvature.

    ``yaw`` and ``curvature`` are optional: pass them when the generator knows
    them exactly (the bundled route does, it is built from clothoids), and they
    are differentiated from the geometry otherwise.
    """

    x: np.ndarray
    y: np.ndarray
    yaw: Optional[np.ndarray] = None
    curvature: Optional[np.ndarray] = None
    name: str = "route"

    def __post_init__(self) -> None:
        self.x = np.asarray(self.x, dtype=float).ravel()
        self.y = np.asarray(self.y, dtype=float).ravel()
        if self.x.size != self.y.size:
            raise ValueError("x and y must have the same length")
        if self.x.size < 2:
            raise ValueError("a route needs at least two points")

        self.ds = np.hypot(np.diff(self.x), np.diff(self.y))
        if np.any(self.ds <= 0.0):
            raise ValueError("route contains repeated points")
        self.s = np.concatenate(([0.0], np.cumsum(self.ds)))
        self.points = np.column_stack((self.x, self.y))

        if self.yaw is None:
            heading = np.arctan2(np.diff(self.y), np.diff(self.x))
            self.yaw = np.concatenate((heading, heading[-1:]))
        else:
            self.yaw = np.asarray(self.yaw, dtype=float).ravel()
        # Interpolating a wrapped angle across the +-pi seam gives a spurious
        # 2 pi sweep, so every lookup goes through the unwrapped copy.
        self.yaw_unwrapped = np.unwrap(self.yaw)

        if self.curvature is None:
            self.curvature = np.gradient(self.yaw_unwrapped, self.s)
        else:
            self.curvature = np.asarray(self.curvature, dtype=float).ravel()

        if not (self.yaw.size == self.curvature.size == self.x.size):
            raise ValueError("yaw and curvature must match the number of points")

    # -- geometry -----------------------------------------------------------
    @property
    def length(self) -> float:
        return float(self.s[-1])

    def __len__(self) -> int:
        return int(self.x.size)

    def point_at(self, s: float) -> Tuple[float, float]:
        """Interpolated position at arc length ``s``, extrapolated past the end
        along the closing tangent (so a lookahead never runs out of road)."""
        if s <= 0.0:
            over = s
            return (float(self.x[0] + over * math.cos(self.yaw[0])),
                    float(self.y[0] + over * math.sin(self.yaw[0])))
        if s >= self.length:
            over = s - self.length
            return (float(self.x[-1] + over * math.cos(self.yaw[-1])),
                    float(self.y[-1] + over * math.sin(self.yaw[-1])))
        return (float(np.interp(s, self.s, self.x)),
                float(np.interp(s, self.s, self.y)))

    def heading_at(self, s: float) -> float:
        return normalize_angle(
            float(np.interp(s, self.s, self.yaw_unwrapped)))

    def curvature_at(self, s: float) -> float:
        return float(np.interp(s, self.s, self.curvature))

    # -- projection ---------------------------------------------------------
    def _foot(self, px: float, py: float, j: int) -> Tuple[float, float, float, float]:
        """Foot of the perpendicular on segment ``j -> j+1``.

        Returns ``(t, s, lateral, distance)`` with ``t`` clamped to the
        segment, so a point beyond either end projects onto the end itself.
        """
        ax, ay = self.x[j], self.y[j]
        ex, ey = self.x[j + 1] - ax, self.y[j + 1] - ay
        length2 = ex * ex + ey * ey
        t = ((px - ax) * ex + (py - ay) * ey) / length2
        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
        fx, fy = ax + t * ex, ay + t * ey
        seg = math.sqrt(length2)
        # Cross product of the unit tangent with the offset: positive to the
        # left of the direction of travel.
        lateral = ((ex * (py - fy) - ey * (px - fx)) / seg)
        return t, float(self.s[j] + t * seg), float(lateral), math.hypot(px - fx, py - fy)

    def project(self, px: float, py: float, hint: int = 0,
                back: int = 12, ahead: Optional[int] = 160) -> Projection:
        """Nearest point on the route, searched around ``hint``.

        ``ahead=None`` searches the whole route; the windowed default is both
        faster and immune to a route that passes close to itself.
        """
        n = int(self.x.size)
        if ahead is None:
            lo, hi = 0, n
        else:
            lo = max(0, int(hint) - back)
            hi = min(n, int(hint) + ahead + 1)
            if hi - lo < 2:
                lo, hi = max(0, n - 2), n
        dx = self.x[lo:hi] - px
        dy = self.y[lo:hi] - py
        near = int(np.argmin(dx * dx + dy * dy)) + lo

        best: Optional[Tuple[float, float, float, float, int]] = None
        for j in (near - 1, near):
            if j < 0 or j >= n - 1:
                continue
            t, s, lateral, distance = self._foot(px, py, j)
            if best is None or distance < best[3]:
                best = (t, s, lateral, distance, j)
        if best is None:  # degenerate: a two point route hit at an end
            t, s, lateral, distance = self._foot(px, py, 0)
            best = (t, s, lateral, distance, 0)

        t, s, lateral, distance, j = best
        return Projection(index=j, s=s, lateral=lateral,
                          heading=self.heading_at(s),
                          curvature=self.curvature_at(s), distance=distance)


# --------------------------------------------------------------------------
# loading
# --------------------------------------------------------------------------

def _find_column(header: Sequence[str], names: Sequence[str]) -> int:
    lowered = [cell.strip().lower().lstrip("﻿") for cell in header]
    for name in names:
        if name in lowered:
            return lowered.index(name)
    return -1


def _is_number(cell: str) -> bool:
    try:
        float(cell)
    except ValueError:
        return False
    return True


def load_route(path: Optional[str] = None) -> Route:
    """Read a route CSV.

    Columns are located by header name (``x_m``, ``y_m``, ``yaw``,
    ``curvature`` and the aliases in :data:`COLUMN_ALIASES`), which is the same
    contract the C++ path-tracking loader uses, so extra columns and a
    different column order are accepted. A file without a header row is read as
    ``x, y[, yaw[, curvature]]``.
    """
    path = path or DEFAULT_ROUTE_PATH
    with open(path, newline="", encoding="utf-8-sig") as handle:
        rows = [row for row in csv.reader(handle)
                if row and not row[0].lstrip().startswith("#")]
    if len(rows) < 2:
        raise ValueError("%s holds no route points" % path)

    header = rows[0]
    if _is_number(header[0]):  # headerless: assume the documented order
        columns = {"x": 0, "y": 1}
        if len(header) > 2:
            columns["yaw"] = 2
        if len(header) > 3:
            columns["curvature"] = 3
        body = rows
    else:
        columns = {}
        for key, names in COLUMN_ALIASES.items():
            index = _find_column(header, names)
            if index >= 0:
                columns[key] = index
        missing = [k for k in ("x", "y") if k not in columns]
        if missing:
            raise ValueError("%s has no %s column (looked for %s)"
                             % (path, "/".join(missing),
                                ", ".join(COLUMN_ALIASES[missing[0]])))
        body = rows[1:]

    width = max(columns.values()) + 1
    values = np.array([[float(row[i]) for i in range(width)]
                       for row in body if len(row) >= width], dtype=float)
    if values.shape[0] < 2:
        raise ValueError("%s holds fewer than two usable rows" % path)

    def column(key: str) -> Optional[np.ndarray]:
        return values[:, columns[key]] if key in columns else None

    return Route(x=values[:, columns["x"]], y=values[:, columns["y"]],
                 yaw=column("yaw"), curvature=column("curvature"),
                 name=os.path.splitext(os.path.basename(path))[0])


# --------------------------------------------------------------------------
# speed profile
# --------------------------------------------------------------------------

def speed_profile(route: Route, params: VehicleParameters, *,
                  ay_ratio: float = 0.35, speed_max: Optional[float] = None,
                  accel_max: Optional[float] = None,
                  decel_max: Optional[float] = None,
                  start_speed: Optional[float] = None,
                  end_speed: float = 0.0,
                  min_speed: float = 1.5) -> np.ndarray:
    """Reference speed at every route point.

    Three limits in the order a driver applies them:

    1. ``v = sqrt(a_y,max / |kappa|)`` -- how fast the corner can be taken,
       with ``a_y,max = ay_ratio * mu * g``. The default 0.35 keeps the run
       inside the range where the kinematic models are still defensible, which
       is the comparison this animation exists to show;
    2. a backward pass at ``decel_max`` so braking starts *before* the corner
       rather than in it;
    3. a forward pass at ``accel_max`` so the profile is one the drive train
       can actually produce.

    ``min_speed`` keeps the profile above the ``low_speed_guard`` of the
    dynamic models; ``end_speed`` is the speed asked for at the last point, 0
    meaning the vehicle is brought to a stop on the goal.
    """
    v_max = params.speed_max if speed_max is None else speed_max
    a_acc = params.accel_max if accel_max is None else accel_max
    a_dec = abs(params.accel_min if decel_max is None else decel_max)
    ay_max = max(ay_ratio, 1e-3) * params.friction * GRAVITY

    kappa = np.abs(route.curvature)
    v = np.full(kappa.shape, float(v_max))
    turning = kappa > 1e-6
    v[turning] = np.sqrt(ay_max / kappa[turning])
    v = np.clip(v, min(min_speed, v_max), v_max)

    if start_speed is not None:
        v[0] = min(v[0], float(start_speed))
    v[-1] = min(v[-1], float(end_speed))

    ds = route.ds
    for i in range(v.size - 2, -1, -1):  # braking
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_dec * ds[i]))
    for i in range(1, v.size):  # power
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_acc * ds[i - 1]))
    return v


def travel_time(route: Route, profile: np.ndarray) -> float:
    """Time to drive the profile exactly, ignoring the tracking error."""
    mid = 0.5 * (profile[:-1] + profile[1:])
    return float(np.sum(route.ds / np.maximum(mid, 0.5)))


# --------------------------------------------------------------------------
# tracking error
# --------------------------------------------------------------------------

@dataclass
class TrackingReport:
    """What one driven trajectory did against the route, sample by sample."""

    s: np.ndarray  # arc length reached [m]
    lateral: np.ndarray  # signed cross-track error, + = left [m]
    heading_error: np.ndarray  # yaw minus route tangent [rad]
    curvature_ref: np.ndarray  # route curvature under the vehicle [1/m]
    v_ref: np.ndarray  # speed the profile asked for [m/s]
    summary: Dict[str, float] = field(default_factory=dict)


def analyse_tracking(route: Route, x: np.ndarray, y: np.ndarray,
                     yaw: np.ndarray, *, time: Optional[np.ndarray] = None,
                     profile: Optional[np.ndarray] = None,
                     finish_tolerance: float = 1.0) -> TrackingReport:
    """Project a trajectory onto the route, sample by sample.

    The search hint walks forward with the vehicle, so the report is correct
    even where the route passes close to an earlier part of itself. Pass the
    trajectory of the point the driver steers (the rear axle here), not the
    CoG, or a body slip angle would be read as a tracking error.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    yaw = np.asarray(yaw, dtype=float)
    n = x.size

    s = np.empty(n)
    lateral = np.empty(n)
    heading_error = np.empty(n)
    curvature_ref = np.empty(n)

    hint = 0
    for i in range(n):
        proj = route.project(float(x[i]), float(y[i]), hint)
        hint = proj.index
        s[i] = proj.s
        lateral[i] = proj.lateral
        heading_error[i] = normalize_angle(float(yaw[i]) - proj.heading)
        curvature_ref[i] = proj.curvature

    v_ref = (np.interp(s, route.s, profile) if profile is not None
             else np.full(n, np.nan))

    summary: Dict[str, float] = {
        "lateral_max": float(np.max(np.abs(lateral))) if n else 0.0,
        "lateral_rms": float(np.sqrt(np.mean(lateral ** 2))) if n else 0.0,
        "heading_max": float(np.max(np.abs(heading_error))) if n else 0.0,
        "distance": float(s[-1]) if n else 0.0,
        "progress": float(s[-1] / route.length) if n else 0.0,
    }
    finished = np.where(s >= route.length - finish_tolerance)[0]
    if finished.size and time is not None:
        summary["finish_time"] = float(time[finished[0]])
    return TrackingReport(s=s, lateral=lateral, heading_error=heading_error,
                          curvature_ref=curvature_ref, v_ref=v_ref,
                          summary=summary)
