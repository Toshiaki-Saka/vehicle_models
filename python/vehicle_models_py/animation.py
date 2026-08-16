# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Animated top view of a manoeuvre, with no Tk dependency.

``VehicleScene`` owns every artist that makes a run watchable: the body
outline, the four wheels at their real Ackermann angles, the discs that scale
with the vertical load, the velocity vector, and the time cursors on the
channel plots. It knows nothing about who advances the frame index.

The GUI animation tab drives it from a Tk timer; ``build_animation`` drives the
same object from ``matplotlib.animation``, so the demo GIF in the
documentation is literally the scene the application shows.

Only numpy and matplotlib are needed here, so the command-line demo runs on a
machine that has no tkinter installed at all.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .gui import theme
from .maneuvers import ManeuverConfig, reference_path
from .parameters import VehicleParameters
from .runner import STATE_REFERENCE, RunResult
from .types import GRAVITY, rad2deg

# Which point of the vehicle the state (x, y) refers to, per model.
REAR_AXLE_MODELS = {key for key, point in STATE_REFERENCE.items()
                    if point == "rear_axle"}

CHANNEL_SPECS: Tuple[Tuple[str, str, str], ...] = (
    ("r", "Yaw rate", "r [deg/s]"),
    ("ay", "Lateral acceleration", "a_y [g]"),
    ("beta", "Body slip", "beta [deg]"),
)


def _channel_values(res: RunResult, channel: str) -> np.ndarray:
    """A channel in the unit the animation plots it in."""
    data = res.channels[channel]
    if channel == "ay":
        return data / GRAVITY
    return rad2deg(data)


def pose_at(res: RunResult, params: VehicleParameters,
            i: int) -> Tuple[float, float, float]:
    """Pose of the *centre of gravity* at sample ``i``.

    The kinematic models integrate the rear axle, so their (x, y) has to be
    pushed forward by ``l_r`` before two models can be drawn on top of each
    other.
    """
    i = int(min(max(i, 0), res.time.size - 1))
    x = float(res["x"][i])
    y = float(res["y"][i])
    yaw = float(res["yaw"][i])
    if res.key in REAR_AXLE_MODELS:
        x += params.l_r * math.cos(yaw)
        y += params.l_r * math.sin(yaw)
    return x, y, yaw


def body_outline(params: VehicleParameters) -> np.ndarray:
    """Closed body polygon in body coordinates, nose pointing +x."""
    p = params
    half = 0.5 * max(p.track_front, p.track_rear) + 0.12
    return np.array([[-p.l_r - 0.5, -half], [p.l_f + 0.5, -half],
                     [p.l_f + 0.75, 0.0], [p.l_f + 0.5, half],
                     [-p.l_r - 0.5, half], [-p.l_r - 0.5, -half]])


def to_world(points: np.ndarray, x: float, y: float,
             yaw: float) -> np.ndarray:
    """Rotate body-frame ``(N, 2)`` points by ``yaw`` and translate to (x, y)."""
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([x, y])


class VehicleScene:
    """The animated figure for one run, optionally with other models as ghosts.

    ``draw(i)`` moves everything to sample ``i``. Nothing here touches a
    canvas: the caller decides when to blit, which is what lets the same code
    serve a Tk timer and a movie writer.
    """

    def __init__(self, figure, params: VehicleParameters, result: RunResult,
                 cfg: Optional[ManeuverConfig] = None, span: float = 25.0,
                 show_channels: bool = True,
                 ghosts: Sequence[RunResult] = (),
                 title: Optional[str] = None):
        self.figure = figure
        self.params = params
        self.result = result
        self.cfg = cfg
        self.span = span
        self.ghosts = list(ghosts)
        self.artists: Dict[str, object] = {}
        self.channel_axes: List[tuple] = []
        self._build(show_channels, title)

    # -- construction -------------------------------------------------------
    def _build(self, show_channels: bool, title: Optional[str]) -> None:
        fig = self.figure
        fig.clear()
        if show_channels:
            gs = fig.add_gridspec(3, 2, width_ratios=(2.1, 1.0), hspace=0.55,
                                  wspace=0.22, left=0.06, right=0.975,
                                  top=0.92, bottom=0.07)
            ax_view = fig.add_subplot(gs[:, 0])
        else:
            gs = None
            ax_view = fig.add_subplot(111)
            fig.subplots_adjust(left=0.09, right=0.98, top=0.9, bottom=0.09)

        res = self.result
        self.ax_view = ax_view
        theme.style_axes(ax_view, title or ("Top view - %s" % res.label),
                         "x [m]", "y [m]")
        ax_view.set_aspect("equal", adjustable="box")

        if self.cfg is not None:
            path = reference_path(self.cfg)
            if path.size:
                ax_view.plot(path[:, 0], path[:, 1], color=theme.REFERENCE,
                             linestyle="--", linewidth=1.2, zorder=1,
                             label="reference path")

        # Other models, drawn as outline plus trace only: enough to see them
        # part company with the model in focus, quiet enough not to compete.
        self.ghost_artists: List[Dict[str, object]] = []
        for ghost in self.ghosts:
            gcolor = theme.model_color(ghost.key)
            self.ghost_artists.append({
                "trace": ax_view.plot([], [], color=gcolor, linewidth=1.2,
                                      alpha=0.45, zorder=2)[0],
                "body": ax_view.plot([], [], color=gcolor, linewidth=1.4,
                                     alpha=0.6, linestyle="--", zorder=4,
                                     label=ghost.label)[0],
            })

        color = theme.model_color(res.key)
        self.artists["trace"] = ax_view.plot([], [], color=color, linewidth=1.6,
                                             alpha=0.55, zorder=2)[0]
        self.artists["body_fill"] = ax_view.fill([], [], color=color,
                                                 alpha=0.12, linewidth=0,
                                                 zorder=4)[0]
        self.artists["body"] = ax_view.plot([], [], color=color, linewidth=2.0,
                                            zorder=5, label=res.label)[0]
        self.artists["loads"] = ax_view.scatter([], [], s=[], color=color,
                                                alpha=0.25, linewidths=0,
                                                zorder=3)
        for name in ("wheel0", "wheel1", "wheel2", "wheel3"):
            self.artists[name] = ax_view.plot([], [], color=theme.INK,
                                              linewidth=2.4, zorder=6)[0]
        self.artists["velocity"] = ax_view.annotate(
            "", xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color=theme.CRITICAL, lw=2.0),
            zorder=7)
        self.artists["hud"] = ax_view.text(
            0.02, 0.98, "", transform=ax_view.transAxes, va="top", ha="left",
            fontsize=8.5, color=theme.INK_2, family="Consolas", zorder=8)
        if self.ghosts:
            ax_view.legend(loc="lower right", fontsize=8)

        if gs is None:
            return
        for row, (channel, ch_title, ylabel) in enumerate(CHANNEL_SPECS):
            ax = fig.add_subplot(gs[row, 1])
            values = _channel_values(res, channel)
            for ghost in self.ghosts:
                ax.plot(ghost.time, _channel_values(ghost, channel),
                        color=theme.model_color(ghost.key), linewidth=1.2,
                        alpha=0.55, linestyle="--")
            ax.plot(res.time, values, color=color, linewidth=1.8)
            if channel == "ay":
                theme.reference_line(ax, self.params.friction, label="mu")
                theme.reference_line(ax, -self.params.friction)
            theme.style_axes(ax, ch_title, "t [s]" if row == 2 else "", ylabel)
            cursor = ax.axvline(0.0, color=theme.INK_2, linewidth=1.0)
            dot = ax.plot([0], [values[0] if values.size else 0], "o",
                          color=color, markersize=5)[0]
            self.channel_axes.append((ax, cursor, dot, values))

    # -- per frame ----------------------------------------------------------
    def draw(self, i: int) -> List[object]:
        """Move every artist to sample ``i``; returns the artists touched."""
        res = self.result
        p = self.params
        i = int(min(max(i, 0), res.time.size - 1))

        x, y, yaw = pose_at(res, p, i)
        world_body = to_world(body_outline(p), x, y, yaw)
        self.artists["body"].set_data(world_body[:, 0], world_body[:, 1])
        self.artists["body_fill"].set_xy(world_body)

        steer = float(res["steer"][i])
        steer_l = res["steer_l"][i]
        steer_r = res["steer_r"][i]
        if not np.isfinite(steer_l):
            steer_l = steer_r = steer
        lf, lr = p.l_f, p.l_r
        tf, tr = p.track_front, p.track_rear
        wheel_defs = ((lf, 0.5 * tf, steer_l), (lf, -0.5 * tf, steer_r),
                      (-lr, 0.5 * tr, 0.0), (-lr, -0.5 * tr, 0.0))
        length = 2.0 * p.wheel_radius
        for k, (cx, cy, angle) in enumerate(wheel_defs):
            local = np.array([[-0.5 * length, 0.0], [0.5 * length, 0.0]])
            c, s = math.cos(angle), math.sin(angle)
            local = local @ np.array([[c, -s], [s, c]]).T + np.array([cx, cy])
            world = to_world(local, x, y, yaw)
            self.artists["wheel%d" % k].set_data(world[:, 0], world[:, 1])

        loads = np.array([res["fz_fl"][i], res["fz_fr"][i],
                          res["fz_rl"][i], res["fz_rr"][i]])
        centres = to_world(np.array([[lf, 0.5 * tf], [lf, -0.5 * tf],
                                     [-lr, 0.5 * tr], [-lr, -0.5 * tr]]),
                           x, y, yaw)
        scatter = self.artists["loads"]
        if np.all(np.isfinite(loads)):
            static = 0.25 * p.mass * GRAVITY
            sizes = 240.0 * np.clip(loads / max(static, 1.0), 0.0, 3.0)
            scatter.set_offsets(centres)
            scatter.set_sizes(sizes)
            scatter.set_visible(True)
        else:
            scatter.set_visible(False)

        # velocity vector at the CoG, scaled to the view so it stays readable
        span = max(self.span, 8.0)
        v = float(res["v"][i])
        beta = float(res["beta"][i])
        direction = yaw + (beta if np.isfinite(beta) else 0.0)
        arrow_len = 0.22 * span * min(v / max(p.speed_max, 1e-6), 1.5)
        arrow = self.artists["velocity"]
        arrow.set_position((x, y))
        arrow.xy = (x + arrow_len * math.cos(direction),
                    y + arrow_len * math.sin(direction))

        self.ax_view.set_xlim(x - 0.5 * span, x + 0.5 * span)
        self.ax_view.set_ylim(y - 0.5 * span, y + 0.5 * span)

        self.artists["trace"].set_data(res["x"][:i + 1], res["y"][:i + 1])

        for ghost, art in zip(self.ghosts, self.ghost_artists):
            j = int(min(i, ghost.time.size - 1))
            gx, gy, gyaw = pose_at(ghost, p, j)
            gbody = to_world(body_outline(p), gx, gy, gyaw)
            art["body"].set_data(gbody[:, 0], gbody[:, 1])
            art["trace"].set_data(ghost["x"][:j + 1], ghost["y"][:j + 1])

        self.artists["hud"].set_text(self.hud_text(i))

        for _ax, cursor, dot, values in self.channel_axes:
            cursor.set_xdata([res.time[i], res.time[i]])
            dot.set_data([res.time[i]], [values[i]])

        touched = list(self.artists.values())
        for art in self.ghost_artists:
            touched.extend(art.values())
        return touched

    # -- text ---------------------------------------------------------------
    def hud_text(self, i: int) -> str:
        res = self.result
        return ("t     %6.2f s\n"
                "v     %6.2f m/s\n"
                "delta %6.2f deg\n"
                "r     %6.2f deg/s\n"
                "a_y   %6.3f g\n"
                "beta  %6.2f deg"
                % (res.time[i], float(res["v"][i]),
                   rad2deg(float(res["steer"][i])),
                   rad2deg(float(res["r"][i])),
                   float(res["ay"][i]) / GRAVITY,
                   rad2deg(float(res["beta"][i]))))

    def readout_lines(self, i: int) -> List[str]:
        """The detailed instantaneous state, one ``label value`` line each."""
        res = self.result
        return [
            "t        %7.3f s" % res.time[i],
            "x, y     %7.2f, %.2f m" % (res["x"][i], res["y"][i]),
            "yaw      %7.2f deg" % rad2deg(float(res["yaw"][i])),
            "v_x/v_y  %6.2f / %.2f m/s" % (res["vx"][i], res["vy"][i]),
            "alpha_f  %7.2f deg" % rad2deg(float(res["alpha_f"][i])),
            "alpha_r  %7.2f deg" % rad2deg(float(res["alpha_r"][i])),
            "F_z FL   %7.0f N" % res["fz_fl"][i],
            "F_z FR   %7.0f N" % res["fz_fr"][i],
            "F_z RL   %7.0f N" % res["fz_rl"][i],
            "F_z RR   %7.0f N" % res["fz_rr"][i],
        ]


# --------------------------------------------------------------------------
# movie / live playback
# --------------------------------------------------------------------------

GIF_TICK_MS = 10  # GIF stores the frame delay in centiseconds, nothing finer


def gif_fps(fps: float) -> float:
    """The nearest frame rate a GIF can actually play back.

    Asking for 15 fps writes a 66.7 ms delay that the format rounds to 70 ms,
    so the file would play 5 % slow against its own time stamps. Snapping the
    frame rate first keeps the animation honest about real time.
    """
    ms = max(GIF_TICK_MS,
             int(round(1000.0 / max(fps, 1e-6) / GIF_TICK_MS)) * GIF_TICK_MS)
    return 1000.0 / ms


def frame_indices(res: RunResult, dt: float, fps: float = 20.0,
                  rate: float = 1.0) -> List[int]:
    """Sample indices for ``fps`` frames per second at ``rate`` x real time.

    A simulation runs at 200-500 Hz; a movie needs 20. The stride is what turns
    one into the other, and ``rate`` is the playback speed multiplier the GUI
    calls "Playback rate".
    """
    stride = max(1, int(round(rate / (max(fps, 1) * max(dt, 1e-9)))))
    return list(range(0, res.time.size, stride))


def build_animation(params: VehicleParameters, result: RunResult,
                    cfg: Optional[ManeuverConfig] = None, *,
                    ghosts: Sequence[RunResult] = (), fps: float = 20.0,
                    rate: float = 1.0, span: float = 25.0,
                    show_channels: bool = True,
                    figsize: Optional[Tuple[float, float]] = None,
                    dpi: int = 100, title: Optional[str] = None,
                    repeat: bool = True, figure=None):
    """Build the figure and a ``FuncAnimation`` playing the whole run.

    Pass ``figure`` to draw into a canvas that already exists -- a pyplot
    figure for on-screen playback, for instance; without it a bare ``Figure``
    is created, which is all a movie writer needs.

    Returns ``(figure, animation)``. Keep a reference to the animation:
    matplotlib drops it, and the playback with it, as soon as it is collected.
    """
    from matplotlib.animation import FuncAnimation

    if figure is None:
        from matplotlib.figure import Figure
        if figsize is None:
            figsize = (9.5, 5.4) if show_channels else (6.4, 5.4)
        figure = Figure(figsize=figsize, dpi=dpi)
    figure.patch.set_facecolor(theme.SURFACE)

    scene = VehicleScene(figure, params, result, cfg, span=span,
                         show_channels=show_channels, ghosts=ghosts,
                         title=title)
    dt = cfg.dt if cfg is not None else float(result.time[1] - result.time[0])
    frames = frame_indices(result, dt, fps, rate)

    anim = FuncAnimation(figure, scene.draw, frames=frames,
                         interval=int(round(1000.0 / max(fps, 1e-6))),
                         blit=False, repeat=repeat,
                         cache_frame_data=False)
    anim.scene = scene  # type: ignore[attr-defined]
    return figure, anim


def _quantize_gif(path: str, fps: int, colors: int) -> None:
    """Re-encode a GIF onto one shared palette.

    The camera follows the vehicle, so every frame differs everywhere and GIF's
    inter-frame coding buys nothing; the palette is the only lever left on file
    size, and it is worth roughly a factor of two.
    """
    from PIL import Image, ImageSequence

    quantize = getattr(Image, "Quantize", Image)
    dither = getattr(Image, "Dither", Image)
    with Image.open(path) as src:
        frames = [frame.convert("RGB") for frame in ImageSequence.Iterator(src)]
    if not frames:
        return
    palette = frames[0].quantize(colors=colors, method=quantize.MEDIANCUT)
    reduced = [f.quantize(palette=palette, dither=dither.NONE) for f in frames]
    reduced[0].save(path, save_all=True, append_images=reduced[1:],
                    duration=int(round(1000.0 / gif_fps(fps))), loop=0,
                    optimize=True)


def save_animation(anim, path: str, fps: float = 20.0,
                   colors: int = 128) -> str:
    """Write the animation to ``path``; GIF via Pillow, everything else ffmpeg.

    ``colors`` caps the GIF palette (0 leaves the writer's output alone).
    """
    if path.lower().endswith(".gif"):
        from matplotlib.animation import PillowWriter
        anim.save(path, writer=PillowWriter(fps=fps))
        if colors:
            _quantize_gif(path, fps, colors)
        return path
    from matplotlib.animation import FFMpegWriter
    if not FFMpegWriter.isAvailable():
        raise RuntimeError(
            "ffmpeg is required to write %s. Install ffmpeg, or write a .gif "
            "instead (Pillow only)." % path)
    anim.save(path, writer=FFMpegWriter(fps=fps, bitrate=2400))
    return path
