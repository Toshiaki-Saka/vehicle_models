# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Handling analysis tab: the closed-form results of the linear model.

Everything here is analytic (``linear_analysis.py``), so it re-renders
instantly whenever a parameter changes -- this is the view that tells you what
kind of vehicle the parameter set actually describes.
"""

from __future__ import annotations

import math
from tkinter import ttk

import numpy as np
from matplotlib.colors import LinearSegmentedColormap

from .. import linear_analysis as analysis
from ..ackermann import AckermannGeometry, minimum_turn_radius
from ..parameters import VehicleParameters
from ..types import GRAVITY, deg2rad, rad2deg
from . import theme
from .widgets import FieldGrid, PlotPanel, Table

BLUE_CMAP = LinearSegmentedColormap.from_list("vm_blue", theme.SEQUENTIAL[1:])


class HandlingTab(ttk.Frame):
    def __init__(self, master, app, **kwargs):
        super().__init__(master, **kwargs)
        self.app = app

        controls = ttk.Frame(self)
        controls.pack(side="left", fill="y", padx=(6, 0), pady=6)

        box = ttk.LabelFrame(controls, text="Sweep")
        box.pack(fill="x", pady=(0, 6))
        self.fields = FieldGrid(box)
        self.fields.pack(fill="x", padx=6, pady=4)
        self.fields.add_entry("v_max", "Speed sweep to", 45.0, "m/s")
        self.fields.add_entry("radius", "Radius", 50.0, "m")
        self.fields.add_entry("v_eval", "Evaluate at", 20.0, "m/s")
        self.fields.add_entry("delta_eval", "Steer", 3.0, "deg")
        ttk.Button(controls, text="Update", command=self.refresh).pack(fill="x")

        ttk.Label(controls, text=(
            "delta = L/R + K a_y\n\n"
            "K > 0 understeer\nK = 0 neutral\nK < 0 oversteer\n\n"
            "The gains below are the\nsteady state response of\n"
            "the linear 2-DOF model."),
            foreground=theme.MUTED, justify="left").pack(
            anchor="w", pady=(10, 0))

        right = ttk.PanedWindow(self, orient="vertical")
        right.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        self.plot = PlotPanel(right, figsize=(9.5, 6.6))
        right.add(self.plot, weight=4)
        table_frame = ttk.Frame(right)
        right.add(table_frame, weight=1)
        ttk.Label(table_frame, text="Handling metrics",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w")
        self.table = Table(table_frame, [
            ("quantity", "Quantity", 260),
            ("value", "Value", 120),
            ("unit", "Unit", 90),
            ("meaning", "Reading", 380),
        ], height=9)
        self.table.pack(fill="both", expand=True)

    # -- rendering --------------------------------------------------------
    def refresh(self) -> None:
        p = self.app.params
        v_max = max(self.fields.get_float("v_max", 45.0), 5.0)
        radius = self.fields.get_float("radius", 50.0)
        v_eval = max(self.fields.get_float("v_eval", 20.0), 0.5)
        delta_eval = deg2rad(self.fields.get_float("delta_eval", 3.0))

        speeds = np.linspace(0.5, v_max, 300)
        self._plot(p, speeds, radius, v_eval)
        self._fill_table(p, v_eval, delta_eval, radius)

    def _plot(self, p: VehicleParameters, speeds: np.ndarray, radius: float,
              v_eval: float) -> None:
        fig = self.plot.figure
        fig.clear()
        gs = fig.add_gridspec(2, 3, hspace=0.5, wspace=0.3, left=0.065,
                              right=0.975, top=0.90, bottom=0.09)

        v_ch = analysis.characteristic_speed(p)
        v_cr = analysis.critical_speed(p)
        L = p.wheel_base()

        # --- yaw rate gain ---
        ax = fig.add_subplot(gs[0, 0])
        gain = np.array([analysis.yaw_rate_gain(p, float(v)) for v in speeds])
        gain = np.where(np.isfinite(gain), gain, np.nan)
        ax.plot(speeds, gain, color=theme.SERIES[0], linewidth=2.0,
                label="this vehicle")
        ax.plot(speeds, speeds / L, color=theme.REFERENCE, linestyle="--",
                linewidth=1.2, label="neutral steer")
        if math.isfinite(v_ch):
            theme.reference_line(ax, v_ch, axis="x", label="V_char")
        if math.isfinite(v_cr):
            theme.reference_line(ax, v_cr, axis="x", label="V_crit",
                                 color=theme.CRITICAL)
            # The gain diverges at V_crit; scale to the neutral-steer line so
            # the approach to the asymptote stays readable.
            ax.set_ylim(0.0, 3.0 * float(speeds[-1]) / L)
        theme.style_axes(ax, "Yaw rate gain", "v [m/s]", "r / delta [1/s]")
        ax.legend(loc="upper left")

        # --- lateral acceleration gain ---
        ax = fig.add_subplot(gs[0, 1])
        ay_gain = speeds * gain
        ax.plot(speeds, ay_gain / GRAVITY, color=theme.SERIES[0], linewidth=2.0)
        theme.reference_line(ax, p.friction, label="mu limit")
        theme.style_axes(ax, "Lateral acceleration gain", "v [m/s]",
                         "a_y / delta [g/rad]")
        if math.isfinite(v_cr):
            ax.set_ylim(0.0, max(1.0, 1.5 * p.friction))

        # --- required steer for a fixed radius ---
        ax = fig.add_subplot(gs[0, 2])
        required = np.array([analysis.required_steer_angle(p, radius, float(v))
                             for v in speeds])
        ay = speeds ** 2 / radius
        valid = ay <= p.friction * GRAVITY
        ax.plot(speeds[valid], rad2deg(required[valid]), color=theme.SERIES[0],
                linewidth=2.0, label="delta required")
        ax.plot(speeds, np.full_like(speeds, rad2deg(L / radius)),
                color=theme.REFERENCE, linestyle="--", linewidth=1.2,
                label="Ackermann L/R")
        if np.any(~valid):
            theme.reference_line(ax, float(speeds[valid][-1]) if np.any(valid)
                                 else float(speeds[0]), axis="x",
                                 label="mu limit", color=theme.CRITICAL,
                                 align="left")
        theme.style_axes(ax, "Steer to hold R = %.0f m" % radius, "v [m/s]",
                         "delta [deg]")
        ax.legend(loc="upper left")

        # The mode quantities blow up as v -> 0 (the 1/v_x terms), so the mode
        # plots start where the model is meaningful instead of squashing the
        # interesting speed range against the axis.
        mode_speeds = speeds[speeds >= 2.0]
        if mode_speeds.size < 5:
            mode_speeds = speeds
        modes = [analysis.yaw_mode(p, float(v)) for v in mode_speeds]

        # --- natural frequency ---
        ax = fig.add_subplot(gs[1, 0])
        wn = np.array([m.natural_frequency for m in modes])
        ax.plot(mode_speeds, wn / (2.0 * math.pi), color=theme.SERIES[0],
                linewidth=2.0)
        theme.style_axes(ax, "Yaw mode frequency (v >= 2 m/s)", "v [m/s]",
                         "f_n [Hz]")

        # --- damping ratio ---
        ax = fig.add_subplot(gs[1, 1])
        zeta = np.array([m.damping_ratio for m in modes])
        ax.plot(mode_speeds, zeta, color=theme.SERIES[0], linewidth=2.0)
        theme.reference_line(ax, 0.7, label="zeta = 0.7", align="left")
        theme.reference_line(ax, 1.0, label="critically damped", style=":",
                             align="left")
        unstable = np.array([not m.stable for m in modes])
        if np.any(unstable):
            ax.fill_between(mode_speeds, 0, 1, where=unstable,
                            color=theme.CRITICAL, alpha=0.10,
                            transform=ax.get_xaxis_transform(),
                            label="unstable")
            ax.legend(loc="upper right")
        theme.style_axes(ax, "Yaw mode damping", "v [m/s]", "zeta [-]")
        # zeta blows up where the mode splits into two real roots; keep the
        # 0 - 1.5 band, which is the one that carries meaning, on screen.
        ax.set_ylim(min(0.0, float(np.nanmin(zeta))),
                    min(max(1.2, 1.15 * float(np.nanmax(zeta))), 3.0))

        # --- eigenvalue locus ---
        ax = fig.add_subplot(gs[1, 2])
        re = np.concatenate([[m.real_1 for m in modes], [m.real_2 for m in modes]])
        im = np.concatenate([[m.imag_1 for m in modes], [m.imag_2 for m in modes]])
        colors = np.concatenate([mode_speeds, mode_speeds])
        scatter = ax.scatter(re, im, c=colors, cmap=BLUE_CMAP, s=9,
                             linewidths=0)
        ax.axvline(0.0, color=theme.CRITICAL, linewidth=1.2, linestyle="--")
        # One very fast pole at the low-speed end would otherwise set the scale.
        left = float(np.percentile(re, 5)) * 1.6
        ax.set_xlim(min(left, -1.0), max(1.0, float(np.max(re)) * 1.4))
        bar = fig.colorbar(scatter, ax=ax, pad=0.02)
        bar.set_label("v [m/s]", color=theme.INK_2, fontsize=8)
        bar.ax.tick_params(labelsize=7, color=theme.MUTED,
                           labelcolor=theme.MUTED)
        bar.outline.set_visible(False)
        theme.style_axes(ax, "Eigenvalues of [v_y, r]", "Re [1/s]", "Im [1/s]")

        title = ("Understeer gradient K = %+.3f deg/g" %
                 analysis.understeer_gradient_deg_per_g(p))
        if math.isfinite(v_ch):
            title += "  |  characteristic speed %.1f m/s" % v_ch
        elif math.isfinite(v_cr):
            title += "  |  CRITICAL speed %.1f m/s - unstable above it" % v_cr
        else:
            title += "  |  neutral steer"
        fig.suptitle(title, x=0.065, ha="left", fontsize=10, color=theme.INK_2,
                     y=0.97)
        self.plot.draw()

    def _fill_table(self, p: VehicleParameters, v_eval: float,
                    delta_eval: float, radius: float) -> None:
        k = analysis.understeer_gradient(p)
        ss = analysis.steady_state_cornering(p, v_eval, delta_eval)
        mode = analysis.yaw_mode(p, v_eval)
        g = AckermannGeometry.from_params(p)
        v_ch = analysis.characteristic_speed(p)
        v_cr = analysis.critical_speed(p)

        def fmt(value: float, digits: int = 3) -> str:
            if not math.isfinite(value):
                return "inf"
            return "%.*f" % (digits, value)

        rows = [
            ["Understeer gradient K", "%+.4f" % k, "rad/(m/s2)",
             "understeer" if k > 0 else ("oversteer" if k < 0 else "neutral")],
            ["Understeer gradient K", "%+.3f" % analysis.understeer_gradient_deg_per_g(p),
             "deg/g", "extra handwheel-equivalent steer per g"],
            ["Static margin", "%+.4f" % analysis.static_margin(p), "-",
             "positive = neutral steer point behind the CoG"],
            ["Neutral steer point", fmt(analysis.neutral_steer_point(p)), "m",
             "behind the front axle (l_f = %.2f m)" % p.l_f],
            ["Characteristic speed", fmt(v_ch, 2), "m/s",
             "yaw rate gain peaks here" if math.isfinite(v_ch)
             else "not understeering"],
            ["Critical speed", fmt(v_cr, 2), "m/s",
             "divergent above this speed" if math.isfinite(v_cr)
             else "stable at every speed"],
            ["Yaw rate gain @ %.1f m/s" % v_eval,
             fmt(analysis.yaw_rate_gain(p, v_eval)), "1/s",
             "neutral-steer value would be %.3f" % (v_eval / p.wheel_base())],
            ["Yaw mode frequency @ %.1f m/s" % v_eval,
             fmt(mode.natural_frequency / (2.0 * math.pi)), "Hz",
             "damping ratio %.3f%s" % (mode.damping_ratio,
                                       "" if mode.stable else "  (UNSTABLE)")],
            ["Steady r @ %.1f deg" % rad2deg(delta_eval),
             fmt(rad2deg(ss.yaw_rate), 2), "deg/s",
             "radius %.1f m, a_y %.2f g" % (ss.radius if math.isfinite(ss.radius)
                                            else float("inf"),
                                            ss.lateral_accel / GRAVITY)],
            ["Steady body slip", fmt(rad2deg(ss.side_slip), 2), "deg",
             "front slip %.2f deg, rear slip %.2f deg"
             % (rad2deg(ss.slip_front), rad2deg(ss.slip_rear))],
            ["Max lateral acceleration", fmt(p.friction * GRAVITY, 2), "m/s2",
             "simple mu*g bound = %.2f g" % p.friction],
            ["Minimum turn radius", fmt(minimum_turn_radius(g, p.steer_max), 2),
             "m", "outer front wheel at full lock"],
            ["Speed for R = %.0f m at the limit" % radius,
             fmt(math.sqrt(p.friction * GRAVITY * radius), 2), "m/s",
             "%.0f km/h" % (3.6 * math.sqrt(p.friction * GRAVITY * radius))],
        ]
        tags = ["" for _ in rows]
        self.table.set_rows(rows, tags)
