# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Animated route run: every model on the same road, in one figure.

:class:`~vehicle_models_py.animation.VehicleScene` follows one vehicle and
draws the others as ghosts, which is the right framing for a 100 m manoeuvre.
A 1 km route needs the other framing: no model is the subject, the camera
follows the pack, and a minimap says where on the route the pack currently is.

The three channels are chosen for path tracking rather than for handling:
cross-track error (did it stay on the road), speed against the reference
profile (did it brake in time for the corner), and lateral acceleration
against the friction limit and the 0.4 g line below which the kinematic models
are defensible -- which is exactly where the models start to part company.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .animation import body_outline, frame_indices, pose_at, to_world
from .gui import theme
from .parameters import VehicleParameters
from .route import Route, TrackingReport
from .runner import RunResult
from .types import GRAVITY, rad2deg

# Camera: never tighter than this, never wider than that, and always wide
# enough to hold every model plus a margin.
SPAN_MIN = 20.0
SPAN_MAX = 400.0
DEFAULT_SPAN = 34.0  # the pack is a few metres across; the rest is context
KINEMATIC_VALIDITY_G = 0.4


def _wheel_lines(params: VehicleParameters, res: RunResult, i: int,
                 x: float, y: float, yaw: float) -> List[np.ndarray]:
    """The four wheels as world-frame segments, at their real steer angles."""
    p = params
    steer = float(res["steer"][i])
    steer_l = res["steer_l"][i]
    steer_r = res["steer_r"][i]
    if not np.isfinite(steer_l):
        steer_l = steer_r = steer
    defs = ((p.l_f, 0.5 * p.track_front, steer_l),
            (p.l_f, -0.5 * p.track_front, steer_r),
            (-p.l_r, 0.5 * p.track_rear, 0.0),
            (-p.l_r, -0.5 * p.track_rear, 0.0))
    length = 2.0 * p.wheel_radius
    out = []
    for cx, cy, angle in defs:
        local = np.array([[-0.5 * length, 0.0], [0.5 * length, 0.0]])
        c, s = math.cos(angle), math.sin(angle)
        local = local @ np.array([[c, -s], [s, c]]).T + np.array([cx, cy])
        out.append(to_world(local, x, y, yaw))
    return out


class _VehicleArtists:
    """Everything drawn for one model: trace, body, wheels, map dot, HUD line."""

    def __init__(self, ax_view, ax_map, params: VehicleParameters,
                 result: RunResult, report: TrackingReport, row: int):
        self.params = params
        self.result = result
        self.report = report
        color = theme.model_color(result.key)
        self.color = color

        self.trace = ax_view.plot([], [], color=color, linewidth=1.4,
                                  alpha=0.55, zorder=2)[0]
        self.fill = ax_view.fill([], [], color=color, alpha=0.14, linewidth=0,
                                 zorder=4)[0]
        self.body = ax_view.plot([], [], color=color, linewidth=1.8,
                                 zorder=5)[0]
        self.wheels = [ax_view.plot([], [], color=theme.INK, linewidth=1.8,
                                    zorder=6)[0] for _ in range(4)]
        self.dot = ax_map.plot([], [], "o", color=color, markersize=5,
                               zorder=5)[0]
        # The camera moves under the text, so each line carries its own
        # background rather than relying on whatever is behind it.
        self.hud = ax_view.text(0.014, 0.977 - 0.045 * row, "",
                                transform=ax_view.transAxes, va="top",
                                ha="left", fontsize=8.5, color=color,
                                family="Consolas", zorder=9,
                                bbox=dict(boxstyle="square,pad=0.25",
                                          facecolor=theme.SURFACE,
                                          edgecolor="none", alpha=0.82))

    def draw(self, i: int) -> Tuple[float, float]:
        res = self.result
        x, y, yaw = pose_at(res, self.params, i)
        outline = to_world(body_outline(self.params), x, y, yaw)
        self.body.set_data(outline[:, 0], outline[:, 1])
        self.fill.set_xy(outline)
        for line, seg in zip(self.wheels,
                             _wheel_lines(self.params, res, i, x, y, yaw)):
            line.set_data(seg[:, 0], seg[:, 1])
        self.trace.set_data(res["x"][:i + 1], res["y"][:i + 1])
        self.dot.set_data([x], [y])

        rep = self.report
        self.hud.set_text("%-29.29s v %4.1f  e %+5.2f m  a_y %+5.2f g"
                          % (res.label, float(res["v"][i]),
                             float(rep.lateral[i]),
                             float(res["ay"][i]) / GRAVITY))
        return x, y

    @property
    def artists(self) -> List[object]:
        return [self.trace, self.fill, self.body, self.dot,
                self.hud] + list(self.wheels)


class RouteScene:
    """The animated figure for one route run with several models.

    ``draw(i)`` moves everything to sample ``i``; nothing here touches a
    canvas, so the same object serves a movie writer and an interactive
    playback loop.
    """

    def __init__(self, figure, params: VehicleParameters, route: Route,
                 results: Sequence[RunResult],
                 reports: Sequence[TrackingReport],
                 profile: Optional[np.ndarray] = None,
                 span: float = DEFAULT_SPAN,
                 title: Optional[str] = None):
        self.figure = figure
        self.params = params
        self.route = route
        self.results = list(results)
        self.reports = list(reports)
        self.profile = profile
        self.span = span
        self.vehicles: List[_VehicleArtists] = []
        self.channel_axes: List[tuple] = []
        self._build(title)

    # -- construction -------------------------------------------------------
    def _build(self, title: Optional[str]) -> None:
        fig = self.figure
        fig.clear()
        gs = fig.add_gridspec(4, 2, width_ratios=(1.95, 1.0),
                              height_ratios=(1.45, 1.0, 1.0, 1.0),
                              hspace=0.62, wspace=0.2, left=0.055,
                              right=0.975, top=0.925, bottom=0.065)
        self.ax_view = fig.add_subplot(gs[:, 0])
        self.ax_map = fig.add_subplot(gs[0, 1])
        ax_cte = fig.add_subplot(gs[1, 1])
        ax_speed = fig.add_subplot(gs[2, 1])
        ax_ay = fig.add_subplot(gs[3, 1])

        route = self.route
        theme.style_axes(self.ax_view, title or "Reference route",
                         "x [m]", "y [m]")
        self.ax_view.set_aspect("equal", adjustable="box")
        self.ax_view.plot(route.x, route.y, color=theme.REFERENCE,
                          linestyle="--", linewidth=1.3, zorder=1)

        self._build_map()
        for row, (res, rep) in enumerate(zip(self.results, self.reports)):
            self.vehicles.append(_VehicleArtists(self.ax_view, self.ax_map,
                                                 self.params, res, rep, row))
        self.clock = self.ax_view.text(
            0.986, 0.977, "", transform=self.ax_view.transAxes, va="top",
            ha="right", fontsize=9, color=theme.INK, family="Consolas",
            zorder=9, bbox=dict(boxstyle="square,pad=0.25",
                                facecolor=theme.SURFACE, edgecolor="none",
                                alpha=0.82))

        self._build_channels(ax_cte, ax_speed, ax_ay)

    def _build_map(self) -> None:
        ax = self.ax_map
        route = self.route
        ax.plot(route.x, route.y, color=theme.MUTED, linewidth=1.0, zorder=1)
        ax.plot([route.x[0]], [route.y[0]], "o", color=theme.GOOD,
                markersize=4, zorder=2)
        ax.plot([route.x[-1]], [route.y[-1]], "s", color=theme.CRITICAL,
                markersize=4, zorder=2)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title("Route (%.0f m)" % route.length)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(False)
        for side in ("top", "right", "left", "bottom"):
            ax.spines[side].set_visible(False)
        # The window the main view is showing, so the zoom has a context.
        from matplotlib.patches import Rectangle
        self.camera_box = Rectangle((0, 0), 1, 1, fill=False,
                                    edgecolor=theme.INK_2, linewidth=1.0,
                                    zorder=4)
        ax.add_patch(self.camera_box)

    def _build_channels(self, ax_cte, ax_speed, ax_ay) -> None:
        time = self.results[0].time

        for res, rep in zip(self.results, self.reports):
            ax_cte.plot(res.time, rep.lateral, color=theme.model_color(res.key),
                        linewidth=1.5)
        theme.reference_line(ax_cte, 0.0)
        theme.style_axes(ax_cte, "Cross-track error", "", "e [m]")

        if self.profile is not None:
            ax_speed.plot(time, self.reports[0].v_ref, color=theme.REFERENCE,
                          linestyle="--", linewidth=1.2, label="profile")
            ax_speed.legend(loc="lower right")
        for res in self.results:
            ax_speed.plot(res.time, res["v"], color=theme.model_color(res.key),
                          linewidth=1.5)
        theme.style_axes(ax_speed, "Speed", "", "v [m/s]")

        for res in self.results:
            ax_ay.plot(res.time, res["ay"] / GRAVITY,
                       color=theme.model_color(res.key), linewidth=1.5)
        theme.reference_line(ax_ay, self.params.friction, label="mu")
        theme.reference_line(ax_ay, -self.params.friction)
        theme.reference_line(ax_ay, KINEMATIC_VALIDITY_G,
                             label="kinematic validity", color=theme.WARNING,
                             style=":", align="left")
        theme.reference_line(ax_ay, -KINEMATIC_VALIDITY_G,
                             color=theme.WARNING, style=":")
        theme.style_axes(ax_ay, "Lateral acceleration", "t [s]", "a_y [g]")

        for ax, values in ((ax_cte, [r.lateral for r in self.reports]),
                           (ax_speed, [r["v"] for r in self.results]),
                           (ax_ay, [r["ay"] / GRAVITY for r in self.results])):
            cursor = ax.axvline(0.0, color=theme.INK_2, linewidth=1.0)
            dots = [ax.plot([0], [series[0]], "o",
                            color=theme.model_color(res.key), markersize=4)[0]
                    for res, series in zip(self.results, values)]
            self.channel_axes.append((ax, cursor, dots, values))

    # -- per frame ----------------------------------------------------------
    def draw(self, i: int) -> List[object]:
        time = self.results[0].time
        i = int(min(max(i, 0), time.size - 1))

        positions = [vehicle.draw(i) for vehicle in self.vehicles]
        self._move_camera(positions)

        lead = max(float(rep.s[i]) for rep in self.reports)
        self.clock.set_text("t %6.1f s\ns %6.0f / %.0f m"
                            % (time[i], lead, self.route.length))

        for _ax, cursor, dots, values in self.channel_axes:
            cursor.set_xdata([time[i], time[i]])
            for dot, series in zip(dots, values):
                dot.set_data([time[i]], [series[i]])

        touched: List[object] = [self.clock, self.camera_box]
        for vehicle in self.vehicles:
            touched.extend(vehicle.artists)
        return touched

    def _move_camera(self, positions: Sequence[Tuple[float, float]]) -> None:
        """Frame the pack: centred on it, wide enough to hold all of it."""
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        cx = 0.5 * (min(xs) + max(xs))
        cy = 0.5 * (min(ys) + max(ys))
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
        span = min(max(self.span, spread * 1.35 + 14.0, SPAN_MIN), SPAN_MAX)
        self.ax_view.set_xlim(cx - 0.5 * span, cx + 0.5 * span)
        self.ax_view.set_ylim(cy - 0.5 * span, cy + 0.5 * span)
        self.camera_box.set_bounds(cx - 0.5 * span, cy - 0.5 * span,
                                   span, span)

    # -- text ---------------------------------------------------------------
    def readout_lines(self, i: int) -> List[str]:
        """Instantaneous state of every model, one block each."""
        lines: List[str] = ["t        %7.2f s" % self.results[0].time[i], ""]
        for res, rep in zip(self.results, self.reports):
            lines.append(res.label)
            lines.append("  s      %7.1f m" % rep.s[i])
            lines.append("  e      %7.2f m" % rep.lateral[i])
            lines.append("  psi_e  %7.2f deg"
                         % rad2deg(float(rep.heading_error[i])))
            lines.append("  v      %7.2f m/s (ref %.2f)"
                         % (res["v"][i], rep.v_ref[i]))
            lines.append("  delta  %7.2f deg" % rad2deg(float(res["steer"][i])))
            lines.append("  a_y    %7.3f g" % (res["ay"][i] / GRAVITY))
            lines.append("")
        return lines


# --------------------------------------------------------------------------
# movie
# --------------------------------------------------------------------------

def build_route_animation(params: VehicleParameters, route: Route,
                          results: Sequence[RunResult],
                          reports: Sequence[TrackingReport], *,
                          profile: Optional[np.ndarray] = None,
                          dt: float = 0.005, fps: float = 20.0,
                          rate: float = 4.0, span: float = DEFAULT_SPAN,
                          stop_index: Optional[int] = None,
                          title: Optional[str] = None,
                          figsize: Tuple[float, float] = (11.0, 6.2),
                          dpi: int = 100, repeat: bool = True, figure=None):
    """Build the figure and a ``FuncAnimation`` of the whole route run.

    ``rate`` is the playback speed: a minute of driving is not worth a minute
    of watching, and 4x keeps the corner entries legible. ``stop_index`` ends
    the movie early -- pass the sample where the last model reached the goal,
    and the tail of parked vehicles is left out.

    Returns ``(figure, animation)``. Keep a reference to the animation:
    matplotlib drops the playback as soon as it is collected.
    """
    from matplotlib.animation import FuncAnimation

    if figure is None:
        from matplotlib.figure import Figure
        figure = Figure(figsize=figsize, dpi=dpi)
    figure.patch.set_facecolor(theme.SURFACE)

    scene = RouteScene(figure, params, route, results, reports,
                       profile=profile, span=span, title=title)
    frames = frame_indices(results[0], dt, fps, rate)
    if stop_index is not None:
        frames = [f for f in frames if f <= stop_index] or [0]

    anim = FuncAnimation(figure, scene.draw, frames=frames,
                         interval=int(round(1000.0 / max(fps, 1e-6))),
                         blit=False, repeat=repeat, cache_frame_data=False)
    anim.scene = scene  # type: ignore[attr-defined]
    return figure, anim


# --------------------------------------------------------------------------
# static overview
# --------------------------------------------------------------------------

def route_overview(figure, params: VehicleParameters, route: Route,
                   results: Sequence[RunResult],
                   reports: Sequence[TrackingReport],
                   profile: Optional[np.ndarray] = None,
                   title: Optional[str] = None):
    """The whole run as one still: driven paths plus the channels against
    distance, which is how a route run is read after the fact."""
    figure.clear()
    figure.patch.set_facecolor(theme.SURFACE)
    gs = figure.add_gridspec(3, 2, width_ratios=(1.55, 1.0), hspace=0.6,
                             wspace=0.2, left=0.06, right=0.975, top=0.9,
                             bottom=0.075)
    ax_path = figure.add_subplot(gs[:, 0])
    ax_cte = figure.add_subplot(gs[0, 1])
    ax_speed = figure.add_subplot(gs[1, 1])
    ax_ay = figure.add_subplot(gs[2, 1])

    ax_path.plot(route.x, route.y, color=theme.REFERENCE, linestyle="--",
                 linewidth=1.2, label="reference route", zorder=1)
    for res in results:
        ax_path.plot(res["x"], res["y"], color=theme.model_color(res.key),
                     linewidth=1.6, label=res.label, zorder=2)
    ax_path.plot([route.x[0]], [route.y[0]], "o", color=theme.GOOD,
                 markersize=6, zorder=3)
    ax_path.plot([route.x[-1]], [route.y[-1]], "s", color=theme.CRITICAL,
                 markersize=6, zorder=3)
    theme.style_axes(ax_path, title or "Driven path", "x [m]", "y [m]")
    # True scale on a route that is twice as wide as it is tall leaves the cell
    # part empty; anchoring north keeps the title level with the channels.
    ax_path.set_aspect("equal", adjustable="box")
    ax_path.set_anchor("N")
    ax_path.legend(loc="best", fontsize=8)

    for res, rep in zip(results, reports):
        ax_cte.plot(rep.s, rep.lateral, color=theme.model_color(res.key),
                    linewidth=1.5)
    theme.reference_line(ax_cte, 0.0)
    theme.style_axes(ax_cte, "Cross-track error", "", "e [m]")

    if profile is not None:
        ax_speed.plot(route.s, profile, color=theme.REFERENCE, linestyle="--",
                      linewidth=1.2, label="profile")
        ax_speed.legend(loc="lower right")
    for res, rep in zip(results, reports):
        ax_speed.plot(rep.s, res["v"], color=theme.model_color(res.key),
                      linewidth=1.5)
    theme.style_axes(ax_speed, "Speed", "", "v [m/s]")

    for res, rep in zip(results, reports):
        ax_ay.plot(rep.s, res["ay"] / GRAVITY,
                   color=theme.model_color(res.key), linewidth=1.5)
    theme.reference_line(ax_ay, params.friction, label="mu")
    theme.reference_line(ax_ay, -params.friction)
    theme.reference_line(ax_ay, KINEMATIC_VALIDITY_G,
                         label="kinematic validity", color=theme.WARNING,
                         style=":", align="left")
    theme.reference_line(ax_ay, -KINEMATIC_VALIDITY_G, color=theme.WARNING,
                         style=":")
    theme.style_axes(ax_ay, "Lateral acceleration", "s [m]", "a_y [g]")
    return figure


def finish_index(reports: Sequence[TrackingReport], route: Route,
                 tolerance: float = 1.0) -> Optional[int]:
    """Sample at which the *last* model reached the goal, or None."""
    last = None
    for rep in reports:
        reached = np.where(rep.s >= route.length - tolerance)[0]
        if not reached.size:
            return None
        last = int(reached[0]) if last is None else max(last, int(reached[0]))
    return last
