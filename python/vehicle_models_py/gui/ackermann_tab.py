# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Ackermann tab: steering geometry, drawn and swept."""

from __future__ import annotations

import math
from tkinter import ttk

import numpy as np

from ..ackermann import (AckermannGeometry, ackermann_error,
                         minimum_turn_radius, road_wheel_angles, turn_radius,
                         wheel_speeds)
from ..parameters import VehicleParameters
from ..types import deg2rad, rad2deg
from . import theme
from .widgets import FieldGrid, PlotPanel, Table

WHEEL_COLORS = (theme.SERIES[0], theme.SERIES[1], theme.SERIES[2],
                theme.SERIES[3])


def wheel_polygon(cx: float, cy: float, angle: float, length: float,
                  width: float) -> np.ndarray:
    """Corners of a wheel rectangle centred at (cx, cy), rotated by ``angle``."""
    half_l, half_w = 0.5 * length, 0.5 * width
    corners = np.array([[-half_l, -half_w], [half_l, -half_w],
                        [half_l, half_w], [-half_l, half_w], [-half_l, -half_w]])
    rot = np.array([[math.cos(angle), -math.sin(angle)],
                    [math.sin(angle), math.cos(angle)]])
    return corners @ rot.T + np.array([cx, cy])


class AckermannTab(ttk.Frame):
    def __init__(self, master, app, **kwargs):
        super().__init__(master, **kwargs)
        self.app = app

        controls = ttk.Frame(self)
        controls.pack(side="left", fill="y", padx=(6, 0), pady=6)

        box = ttk.LabelFrame(controls, text="Operating point")
        box.pack(fill="x", pady=(0, 6))
        self.fields = FieldGrid(box)
        self.fields.pack(fill="x", padx=6, pady=4)
        self.fields.add_entry("delta", "Bicycle steer", 20.0, "deg")
        self.fields.add_entry("speed", "Speed", 10.0, "m/s")
        self.fields.add_entry("yaw_rate", "Yaw rate range", 0.6, "rad/s")
        ttk.Button(controls, text="Update", command=self.refresh).pack(fill="x")

        ttk.Label(controls, text=(
            "Ideal Ackermann satisfies\n"
            "cot(outer) - cot(inner) = T/L,\n"
            "so both front wheels roll\n"
            "around one common centre.\n\n"
            "ackermann_ratio k blends\n"
            "linearly toward parallel\n"
            "steering (k = 0), which is\n"
            "what a real rack does."),
            foreground=theme.MUTED, justify="left").pack(anchor="w",
                                                         pady=(10, 0))

        right = ttk.PanedWindow(self, orient="vertical")
        right.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        self.plot = PlotPanel(right, figsize=(9.5, 6.6))
        right.add(self.plot, weight=4)
        table_frame = ttk.Frame(right)
        right.add(table_frame, weight=1)
        ttk.Label(table_frame, text="Steering geometry",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w")
        self.table = Table(table_frame, [
            ("quantity", "Quantity", 250),
            ("value", "Value", 110),
            ("unit", "Unit", 80),
            ("meaning", "Reading", 420),
        ], height=7)
        self.table.pack(fill="both", expand=True)

    def refresh(self) -> None:
        p = self.app.params
        g = AckermannGeometry.from_params(p)
        delta = deg2rad(self.fields.get_float("delta", 20.0))
        delta = max(min(delta, p.steer_max), -p.steer_max)
        speed = self.fields.get_float("speed", 10.0)
        r_max = max(abs(self.fields.get_float("yaw_rate", 0.6)), 1e-3)
        self._plot(p, g, delta, speed, r_max)
        self._fill_table(p, g, delta, speed)

    def _plot(self, p: VehicleParameters, g: AckermannGeometry, delta: float,
              speed: float, r_max: float) -> None:
        fig = self.plot.figure
        fig.clear()
        gs = fig.add_gridspec(2, 2, hspace=0.45, wspace=0.24, left=0.075,
                              right=0.98, top=0.90, bottom=0.09)

        # --- top view -----------------------------------------------------
        ax = fig.add_subplot(gs[0, 0])
        wa = road_wheel_angles(g, delta)
        lf, lr = p.l_f, p.l_r
        tf, tr = p.track_front, p.track_rear
        body = np.array([[-lr - 0.6, -0.5 * max(tf, tr) - 0.15],
                         [lf + 0.6, -0.5 * max(tf, tr) - 0.15],
                         [lf + 0.6, 0.5 * max(tf, tr) + 0.15],
                         [-lr - 0.6, 0.5 * max(tf, tr) + 0.15],
                         [-lr - 0.6, -0.5 * max(tf, tr) - 0.15]])
        ax.plot(body[:, 0], body[:, 1], color=theme.AXIS, linewidth=1.2)
        ax.plot([lf, lf], [-0.5 * tf, 0.5 * tf], color=theme.AXIS,
                linewidth=1.0)
        ax.plot([-lr, -lr], [-0.5 * tr, 0.5 * tr], color=theme.AXIS,
                linewidth=1.0)

        wheels = [(lf, 0.5 * tf, wa.left, "FL"), (lf, -0.5 * tf, wa.right, "FR"),
                  (-lr, 0.5 * tr, 0.0, "RL"), (-lr, -0.5 * tr, 0.0, "RR")]
        for (cx, cy, angle, name), color in zip(wheels, WHEEL_COLORS):
            poly = wheel_polygon(cx, cy, angle, 2.0 * p.wheel_radius, 0.22)
            ax.fill(poly[:, 0], poly[:, 1], color=color, alpha=0.85,
                    linewidth=0)
            ax.plot(poly[:, 0], poly[:, 1], color=theme.SURFACE, linewidth=2.0)

        if abs(delta) > 1e-6:
            radius = turn_radius(g, delta)
            centre = np.array([-lr, radius])
            ax.plot([centre[0]], [centre[1]], "o", color=theme.INK_2,
                    markersize=5)
            for (cx, cy, _angle, _name) in wheels:
                ax.plot([centre[0], cx], [centre[1], cy], color=theme.MUTED,
                        linestyle=":", linewidth=1.0)
            # Short arc only: a full circle would set the scale and shrink the
            # vehicle to a speck.
            arc_half = min(0.25, 2.5 / max(abs(radius), 1e-6))
            arc = np.linspace(-arc_half, arc_half, 60)
            ax.plot(centre[0] + abs(radius) * np.sin(arc),
                    centre[1] - np.sign(radius) * abs(radius) * np.cos(arc),
                    color=theme.REFERENCE, linestyle="--", linewidth=1.2)
            ax.annotate("turn centre\nR = %.2f m" % abs(radius),
                        xy=(centre[0], centre[1]), xytext=(6, -14),
                        textcoords="offset points", fontsize=7.5,
                        color=theme.INK_2)
        ax.annotate("delta_L = %.2f deg" % rad2deg(wa.left),
                    xy=(lf, 0.5 * tf), xytext=(8, 10),
                    textcoords="offset points", fontsize=7.5,
                    color=WHEEL_COLORS[0])
        ax.annotate("delta_R = %.2f deg" % rad2deg(wa.right),
                    xy=(lf, -0.5 * tf), xytext=(8, -16),
                    textcoords="offset points", fontsize=7.5,
                    color=WHEEL_COLORS[1])
        theme.style_axes(ax, "Top view at delta = %.1f deg" % rad2deg(delta),
                         "x [m] (forward)", "y [m] (left)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(False)

        # --- wheel angles vs bicycle steer --------------------------------
        ax = fig.add_subplot(gs[0, 1])
        steers = np.linspace(0.0, p.steer_max, 200)
        ideal = AckermannGeometry.from_params(p)
        ideal.ackermann_ratio = 1.0
        left = np.array([road_wheel_angles(g, float(s)).left for s in steers])
        right = np.array([road_wheel_angles(g, float(s)).right for s in steers])
        left_ideal = np.array([road_wheel_angles(ideal, float(s)).left
                               for s in steers])
        right_ideal = np.array([road_wheel_angles(ideal, float(s)).right
                                for s in steers])
        ax.plot(rad2deg(steers), rad2deg(left), color=theme.SERIES[0],
                linewidth=2.0, label="inner (left)")
        ax.plot(rad2deg(steers), rad2deg(right), color=theme.SERIES[1],
                linewidth=2.0, label="outer (right)")
        ax.plot(rad2deg(steers), rad2deg(left_ideal), color=theme.REFERENCE,
                linestyle="--", linewidth=1.2, label="ideal Ackermann")
        ax.plot(rad2deg(steers), rad2deg(right_ideal), color=theme.REFERENCE,
                linestyle="--", linewidth=1.2)
        ax.plot(rad2deg(steers), rad2deg(steers), color=theme.MUTED,
                linestyle=":", linewidth=1.2, label="parallel steer")
        theme.style_axes(ax, "Road wheel angles (k = %.2f)" % p.ackermann_ratio,
                         "bicycle steer delta [deg]", "wheel angle [deg]")
        ax.legend(loc="upper left")

        # --- Ackermann error over the ratio -------------------------------
        ax = fig.add_subplot(gs[1, 0])
        ratios = (0.0, 0.25, 0.5, 0.75, 1.0)
        colors = theme.sequential_colors(len(ratios))
        for ratio, color in zip(ratios, colors):
            geo = AckermannGeometry.from_params(p)
            geo.ackermann_ratio = ratio
            err = np.array([ackermann_error(geo, float(s)) for s in steers])
            ax.plot(rad2deg(steers), rad2deg(err), color=color, linewidth=2.0,
                    label="k = %.2f" % ratio)
        theme.reference_line(ax, 0.0, label="ideal")
        theme.style_axes(ax, "Ackermann error of the outer wheel",
                         "bicycle steer delta [deg]", "error [deg]")
        ax.legend(loc="upper left", title="ackermann_ratio", title_fontsize=8)

        # --- wheel speeds --------------------------------------------------
        ax = fig.add_subplot(gs[1, 1])
        rates = np.linspace(-r_max, r_max, 200)
        names = ("front left", "front right", "rear left", "rear right")
        speeds = {name: [] for name in names}
        for rate in rates:
            ws = wheel_speeds(g, speed, float(rate))
            speeds["front left"].append(ws.front_left)
            speeds["front right"].append(ws.front_right)
            speeds["rear left"].append(ws.rear_left)
            speeds["rear right"].append(ws.rear_right)
        for name, color in zip(names, WHEEL_COLORS):
            ax.plot(rates, speeds[name], color=color, linewidth=2.0, label=name)
        theme.reference_line(ax, speed, label="body speed")
        theme.style_axes(ax, "Wheel speeds at v = %.1f m/s" % speed,
                         "yaw rate [rad/s]", "wheel speed [m/s]")
        ax.legend(loc="upper left", ncol=2)

        fig.suptitle("Wheelbase %.2f m, front track %.2f m, "
                     "min turn radius %.2f m"
                     % (g.wheel_base, g.track_front,
                        minimum_turn_radius(g, p.steer_max)),
                     x=0.075, ha="left", fontsize=10, color=theme.INK_2, y=0.97)
        self.plot.draw()

    def _fill_table(self, p: VehicleParameters, g: AckermannGeometry,
                    delta: float, speed: float) -> None:
        wa = road_wheel_angles(g, delta)
        radius = turn_radius(g, delta)
        ws = wheel_speeds(g, speed, speed / radius if math.isfinite(radius)
                          else 0.0)
        rows = [
            ["Inner wheel angle", "%.3f" % rad2deg(wa.left), "deg",
             "left wheel in a left turn"],
            ["Outer wheel angle", "%.3f" % rad2deg(wa.right), "deg",
             "difference to the inner wheel %.3f deg"
             % rad2deg(wa.left - wa.right)],
            ["Ackermann error", "%.4f" % rad2deg(ackermann_error(g, delta)),
             "deg", "positive = outer wheel over-steered vs ideal"],
            ["Turn radius (rear axle)",
             "%.3f" % (abs(radius) if math.isfinite(radius) else float("inf")),
             "m", "L / tan(delta)"],
            ["Minimum turn radius", "%.3f" % minimum_turn_radius(g, p.steer_max),
             "m", "outer front wheel at full lock (%.1f deg)"
             % rad2deg(p.steer_max)],
            ["Handwheel angle", "%.1f" % rad2deg(delta * g.steering_ratio),
             "deg", "steering ratio %.1f : 1" % g.steering_ratio],
            ["Wheel speed spread", "%.3f" % (ws.front_right - ws.front_left),
             "m/s", "front L %.2f / R %.2f, rear L %.2f / R %.2f m/s"
             % (ws.front_left, ws.front_right, ws.rear_left, ws.rear_right)],
        ]
        self.table.set_rows(rows)
