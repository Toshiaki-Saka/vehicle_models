# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Tire tab: the three tire models side by side on the same axle."""

from __future__ import annotations

import math
from tkinter import ttk

import numpy as np

from ..parameters import VehicleParameters
from ..tires import friction_ellipse_scale, make_tire
from ..types import deg2rad, rad2deg
from . import theme
from .widgets import FieldGrid, PlotPanel, Table

AXLE_CHOICES = ("Front axle", "Rear axle",
                "Front wheel (half axle)", "Rear wheel (half axle)")
TIRE_ORDER = ("Linear", "Fiala", "Pacejka")
TIRE_COLORS = {"Linear": theme.SERIES[0], "Fiala": theme.SERIES[1],
               "Pacejka": theme.SERIES[2]}


class TireTab(ttk.Frame):
    def __init__(self, master, app, **kwargs):
        super().__init__(master, **kwargs)
        self.app = app

        controls = ttk.Frame(self)
        controls.pack(side="left", fill="y", padx=(6, 0), pady=6)

        box = ttk.LabelFrame(controls, text="Operating point")
        box.pack(fill="x", pady=(0, 6))
        self.fields = FieldGrid(box)
        self.fields.pack(fill="x", padx=6, pady=4)
        self.axle_var = self.fields.add_combo("axle", "Axle", AXLE_CHOICES,
                                              "Front axle", width=16)
        self.fields.add_entry("fz", "Normal load Fz", 0.0, "N")
        self.fields.add_entry("mu", "Friction mu", 1.0, "-")
        self.fields.add_entry("alpha_max", "Slip range", 15.0, "deg")
        self.load_var = self.fields.add_combo(
            "load_model", "Load sweep uses", TIRE_ORDER, "Fiala", width=16)
        ttk.Button(controls, text="Update", command=self.refresh).pack(fill="x")
        ttk.Button(controls, text="Load from vehicle",
                   command=self._load_from_vehicle).pack(fill="x", pady=(4, 0))

        ttk.Label(controls, text=(
            "All three models are matched\n"
            "to the same cornering stiffness\n"
            "at alpha = 0 and the same peak\n"
            "mu*Fz, so the only difference\n"
            "left is the saturation shape."),
            foreground=theme.MUTED, justify="left").pack(anchor="w",
                                                         pady=(10, 0))

        right = ttk.PanedWindow(self, orient="vertical")
        right.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        self.plot = PlotPanel(right, figsize=(9.5, 6.6))
        right.add(self.plot, weight=4)
        table_frame = ttk.Frame(right)
        right.add(table_frame, weight=1)
        ttk.Label(table_frame, text="Tire characteristics",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w")
        self.table = Table(table_frame, [
            ("model", "Tire model", 160),
            ("c", "C_alpha [N/rad]", 130),
            ("peak", "peak Fy [N]", 110),
            ("alpha_peak", "alpha at peak [deg]", 140),
            ("alpha_sl", "full sliding [deg]", 130),
            ("note", "Shape", 320),
        ], height=5)
        self.table.pack(fill="both", expand=True)

    # -- helpers ----------------------------------------------------------
    def _load_from_vehicle(self) -> None:
        p = self.app.params
        self.fields.set_value("mu", p.friction)
        self.fields.set_value("fz", self._nominal_load(p))
        self.refresh()

    def _stiffness(self, p: VehicleParameters) -> float:
        axle = self.axle_var.get()
        if axle.startswith("Front axle"):
            return p.cornering_stiffness_front
        if axle.startswith("Rear axle"):
            return p.cornering_stiffness_rear
        if axle.startswith("Front wheel"):
            return 0.5 * p.cornering_stiffness_front
        return 0.5 * p.cornering_stiffness_rear

    def _nominal_load(self, p: VehicleParameters) -> float:
        axle = self.axle_var.get()
        if axle.startswith("Front axle"):
            return p.static_load_front()
        if axle.startswith("Rear axle"):
            return p.static_load_rear()
        if axle.startswith("Front wheel"):
            return 0.5 * p.static_load_front()
        return 0.5 * p.static_load_rear()

    # -- rendering --------------------------------------------------------
    def refresh(self) -> None:
        p = self.app.params
        c = self._stiffness(p)
        fz = self.fields.get_float("fz", 0.0)
        if fz <= 0.0:
            fz = self._nominal_load(p)
            self.fields.set_value("fz", fz)
        mu = max(self.fields.get_float("mu", p.friction), 1e-3)
        alpha_max = deg2rad(max(self.fields.get_float("alpha_max", 15.0), 1.0))

        tires = {name: make_tire(name, c, fz, mu) for name in TIRE_ORDER}
        alphas = np.linspace(-alpha_max, alpha_max, 501)

        self._plot(tires, alphas, c, fz, mu)
        self._fill_table(tires, c, fz, mu)

    def _plot(self, tires, alphas: np.ndarray, c: float, fz: float,
              mu: float) -> None:
        fig = self.plot.figure
        fig.clear()
        gs = fig.add_gridspec(2, 2, hspace=0.45, wspace=0.24, left=0.075,
                              right=0.98, top=0.90, bottom=0.09)
        alpha_deg = rad2deg(alphas)

        # --- Fy vs slip angle ---
        ax = fig.add_subplot(gs[0, 0])
        for name in TIRE_ORDER:
            fy = np.array([tires[name].lateral_force(float(a), fz)
                           for a in alphas])
            ax.plot(alpha_deg, fy / 1000.0, color=TIRE_COLORS[name],
                    linewidth=2.0, label=name)
        ax.plot(alpha_deg, c * alphas / 1000.0, color=theme.REFERENCE,
                linestyle=":", linewidth=1.2, label="C_alpha * alpha")
        theme.reference_line(ax, mu * fz / 1000.0, label="mu*Fz")
        theme.reference_line(ax, -mu * fz / 1000.0)
        theme.style_axes(ax, "Lateral force at Fz = %.0f N" % fz,
                         "slip angle [deg]", "F_y [kN]")
        # The tangent line runs away; the tire curves are what matters here.
        ax.set_ylim(-1.45 * mu * fz / 1000.0, 1.45 * mu * fz / 1000.0)
        ax.legend(loc="upper left")

        # --- load sensitivity, one model, sequential ramp ---
        ax = fig.add_subplot(gs[0, 1])
        name = self.load_var.get()
        loads = np.linspace(0.4 * fz, 1.6 * fz, 5)
        colors = theme.sequential_colors(len(loads))
        for load, color in zip(loads, colors):
            tire = make_tire(name, c, float(load), mu)
            fy = np.array([tire.lateral_force(float(a), float(load))
                           for a in alphas])
            ax.plot(alpha_deg, fy / 1000.0, color=color, linewidth=2.0,
                    label="%.0f N" % load)
        theme.style_axes(ax, "%s: load sensitivity" % name,
                         "slip angle [deg]", "F_y [kN]")
        ax.legend(loc="upper left", title="Fz", title_fontsize=8)

        # --- normalized shape ---
        ax = fig.add_subplot(gs[1, 0])
        for name in TIRE_ORDER:
            fy = np.array([tires[name].lateral_force(float(a), fz)
                           for a in alphas])
            ax.plot(alpha_deg, fy / (mu * fz), color=TIRE_COLORS[name],
                    linewidth=2.0, label=name)
        theme.reference_line(ax, 1.0, label="mu*Fz")
        theme.style_axes(ax, "Normalized force (saturation shape)",
                         "slip angle [deg]", "F_y / (mu Fz) [-]")
        ax.set_ylim(-1.25, 1.25)
        ax.legend(loc="upper left")

        # --- friction ellipse ---
        ax = fig.add_subplot(gs[1, 1])
        ratios = np.linspace(-1.0, 1.0, 201)
        scale = np.array([friction_ellipse_scale(r * mu * fz, mu, fz)
                          for r in ratios])
        ax.plot(ratios, scale, color=theme.SERIES[0], linewidth=2.0,
                label="available F_y factor")
        ax.fill_between(ratios, 0, scale, color=theme.SERIES[0], alpha=0.10)
        for used in (0.5, 0.8):
            factor = friction_ellipse_scale(used * mu * fz, mu, fz)
            ax.annotate("%.0f%% F_x -> %.0f%% F_y" % (100 * used, 100 * factor),
                        xy=(used, factor), xytext=(-8, 8), ha="right",
                        textcoords="offset points", fontsize=7.5,
                        color=theme.INK_2)
            ax.plot([used], [factor], "o", color=theme.SERIES[0], markersize=5)
        theme.style_axes(ax, "Friction ellipse (combined slip)",
                         "F_x / (mu Fz) [-]", "F_y scale [-]")
        ax.legend(loc="lower center")

        fig.suptitle("C_alpha = %.0f N/rad,  mu = %.2f,  peak F_y = %.0f N"
                     % (c, mu, mu * fz), x=0.075, ha="left", fontsize=10,
                     color=theme.INK_2, y=0.97)
        self.plot.draw()

    def _fill_table(self, tires, c: float, fz: float, mu: float) -> None:
        alphas = np.linspace(0.0, deg2rad(45.0), 2000)
        notes = {
            "Linear": "constant slope, hard clip at mu*Fz",
            "Fiala": "brush model, cubic build-up then full sliding",
            "Pacejka": "Magic Formula, peak above the sliding value",
        }
        rows = []
        for name in ("Linear", "Fiala", "Pacejka"):
            tire = tires[name]
            fy = np.array([tire.lateral_force(float(a), fz) for a in alphas])
            peak_idx = int(np.argmax(fy))
            if name == "Fiala":
                # alpha_sl = atan(3 mu Fz / C)
                sliding = "%.2f" % rad2deg(math.atan(3.0 * mu * fz / c))
            elif name == "Linear":
                # the clip point is simply mu*Fz / C
                sliding = "%.2f" % rad2deg(mu * fz / c if c > 0 else 0.0)
            else:
                sliding = "-"  # the Magic Formula only decays asymptotically
            rows.append([
                name, "%.0f" % c, "%.0f" % fy[peak_idx],
                "%.2f" % rad2deg(alphas[peak_idx]), sliding, notes[name],
            ])
        self.table.set_rows(rows)
