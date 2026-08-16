# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Performance tab: what the vehicle can actually do.

Acceleration, braking, the limit handling curve and a simulated g-g envelope.
Every number here comes out of the same models the manoeuvre tab uses, so the
envelope and the manoeuvre results are always consistent with each other.
"""

from __future__ import annotations

import math
from tkinter import ttk
from typing import Optional

import numpy as np

from .. import linear_analysis as analysis
from ..ackermann import AckermannGeometry, minimum_turn_radius
from ..parameters import VehicleParameters
from ..performance import (acceleration_run, braking_run, gg_diagram,
                           max_cornering_speed, ramp_steer_run)
from ..tires import TIRE_TYPES
from ..types import GRAVITY, deg2rad, rad2deg
from . import theme
from .widgets import FieldGrid, PlotPanel, Table, TaskRunner

MODEL_CHOICES = ("Double track (4 wheels)", "Single track (bicycle)")


class PerformanceTab(ttk.Frame):
    def __init__(self, master, app, **kwargs):
        super().__init__(master, **kwargs)
        self.app = app
        self.tasks = TaskRunner(self)
        self.data: Optional[dict] = None

        controls = ttk.Frame(self)
        controls.pack(side="left", fill="y", padx=(6, 0), pady=6)

        box = ttk.LabelFrame(controls, text="Test setup")
        box.pack(fill="x", pady=(0, 6))
        self.fields = FieldGrid(box)
        self.fields.pack(fill="x", padx=6, pady=4)
        self.fields.add_combo("model", "Model", MODEL_CHOICES,
                              MODEL_CHOICES[0], width=16)
        self.fields.add_combo("tire", "Tire model", list(TIRE_TYPES.keys()),
                              "Fiala", width=16)
        self.fields.add_entry("v_brake", "Braking from", 20.0, "m/s")
        self.fields.add_entry("v_limit", "Limit test speed", 20.0, "m/s")
        self.fields.add_entry("ramp_rate", "Ramp steer rate", 1.5, "deg/s")
        self.fields.add_entry("gg_speed", "g-g speed", 20.0, "m/s")
        self.fields.add_entry("gg_grid", "g-g grid (ax x steer)", 7.0, "x14")

        self.run_button = ttk.Button(controls, text="Run performance suite",
                                     command=self.run)
        self.run_button.pack(fill="x")
        self.progress = ttk.Progressbar(controls, mode="determinate",
                                        maximum=1.0)
        self.progress.pack(fill="x", pady=(6, 0))

        ttk.Label(controls, text=(
            "The ramp steer sweeps the\n"
            "steer angle slowly at a fixed\n"
            "speed. Its initial slope is the\n"
            "understeer gradient K; where\n"
            "a_y stops rising is the tire\n"
            "limit of this parameter set."),
            foreground=theme.MUTED, justify="left").pack(anchor="w",
                                                         pady=(10, 0))

        right = ttk.PanedWindow(self, orient="vertical")
        right.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        self.plot = PlotPanel(right, figsize=(9.5, 6.6))
        right.add(self.plot, weight=4)
        table_frame = ttk.Frame(right)
        right.add(table_frame, weight=1)
        ttk.Label(table_frame, text="Performance figures",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w")
        self.table = Table(table_frame, [
            ("quantity", "Quantity", 260),
            ("value", "Value", 110),
            ("unit", "Unit", 80),
            ("meaning", "Reading", 400),
        ], height=8)
        self.table.pack(fill="both", expand=True)

        self._show_placeholder()

    # -- run ---------------------------------------------------------------
    def run(self) -> None:
        if self.tasks.busy:
            return
        p = self.app.params
        tire = self.fields.get_str("tire")
        four_wheel = self.fields.get_str("model").startswith("Double")
        v_brake = max(self.fields.get_float("v_brake", 20.0), 1.0)
        v_limit = max(self.fields.get_float("v_limit", 20.0), 2.0)
        ramp = deg2rad(max(self.fields.get_float("ramp_rate", 1.5), 0.1))
        gg_speed = max(self.fields.get_float("gg_speed", 20.0), 2.0)
        n_ax = int(max(self.fields.get_float("gg_grid", 7.0), 3.0))

        self.run_button.configure(state="disabled")
        self.progress.configure(value=0.0)
        self.app.set_status("Running the performance suite...")

        def work(progress):
            progress(0.02)
            accel = acceleration_run(p, tire, four_wheel, dt=0.01)
            progress(0.15)
            brake = braking_run(p, v_brake, tire, four_wheel, dt=0.005)
            progress(0.30)
            ramp_res = ramp_steer_run(p, v_limit, tire, four_wheel,
                                      ramp_rate=ramp, duration=25.0, dt=0.005)
            progress(0.50)
            gg = gg_diagram(p, gg_speed, tire, four_wheel, n_ax=n_ax,
                            n_steer=14,
                            progress=lambda f: progress(0.5 + 0.5 * f))
            return {"accel": accel, "brake": brake, "ramp": ramp_res, "gg": gg,
                    "v_brake": v_brake, "v_limit": v_limit,
                    "gg_speed": gg_speed, "tire": tire,
                    "four_wheel": four_wheel}

        def done(data):
            self.run_button.configure(state="normal")
            self.progress.configure(value=1.0)
            self.data = data  # type: ignore[assignment]
            self._plot(p, data)
            self._fill_table(p, data)
            self.app.set_status("Performance suite finished.")

        def failed(exc):
            self.run_button.configure(state="normal")
            self.app.set_status("Performance run failed: %s" % exc)

        self.tasks.start(work, done, lambda v: self.progress.configure(value=v),
                         failed)

    # -- rendering ---------------------------------------------------------
    def _show_placeholder(self) -> None:
        fig = self.plot.figure
        fig.clear()
        ax = fig.add_subplot(111)
        theme.empty_message(
            ax, "Press 'Run performance suite'.\n"
                "Acceleration, braking, the limit handling curve\n"
                "and a simulated g-g envelope are computed from the models.")
        self.plot.draw()

    def _plot(self, p: VehicleParameters, data: dict) -> None:
        accel = data["accel"]
        brake = data["brake"]
        ramp = data["ramp"]
        gg = data["gg"]

        fig = self.plot.figure
        fig.clear()
        gs = fig.add_gridspec(2, 3, hspace=0.5, wspace=0.3, left=0.065,
                              right=0.975, top=0.90, bottom=0.09)

        # --- speed vs time ---
        ax = fig.add_subplot(gs[0, 0])
        ax.plot(accel.time, accel.speed, color=theme.SERIES[0], linewidth=2.0,
                label="acceleration")
        ax.plot(brake.time, brake.speed, color=theme.SERIES[1], linewidth=2.0,
                label="braking")
        theme.reference_line(ax, p.speed_max, label="speed_max")
        theme.style_axes(ax, "Speed vs time", "t [s]", "v [m/s]")
        ax.legend(loc="center right")

        # --- distance ---
        ax = fig.add_subplot(gs[0, 1])
        ax.plot(accel.time, accel.distance, color=theme.SERIES[0],
                linewidth=2.0, label="acceleration")
        ax.plot(brake.time, brake.distance, color=theme.SERIES[1],
                linewidth=2.0, label="braking")
        ax.annotate("stop in %.1f m" % brake.metrics["s_stop"],
                    xy=(brake.time[-1], brake.distance[-1]), xytext=(-6, 8),
                    textcoords="offset points", ha="right", fontsize=7.5,
                    color=theme.SERIES[1])
        theme.style_axes(ax, "Distance vs time", "t [s]", "s [m]")
        ax.legend(loc="upper left")

        # --- achieved ax ---
        ax = fig.add_subplot(gs[0, 2])
        ax.plot(accel.speed, accel.accel, color=theme.SERIES[0], linewidth=2.0,
                label="acceleration")
        ax.plot(brake.speed, brake.accel, color=theme.SERIES[1], linewidth=2.0,
                label="braking")
        theme.reference_line(ax, p.accel_max, label="accel_max")
        theme.reference_line(ax, p.accel_min, label="accel_min")
        theme.style_axes(ax, "Achieved a_x (drag included)", "v [m/s]",
                         "a_x [m/s2]")
        ax.legend(loc="center right")

        # --- handling diagram ---
        ax = fig.add_subplot(gs[1, 0])
        ay_g = ramp.lateral_accel / GRAVITY
        ax.plot(ay_g, rad2deg(ramp.understeer_angle), color=theme.SERIES[0],
                linewidth=2.0, label="simulated")
        k = analysis.understeer_gradient(p)
        ref_ay = np.linspace(0.0, max(float(np.nanmax(ay_g)), 0.1), 50)
        ax.plot(ref_ay, rad2deg(k * ref_ay * GRAVITY), color=theme.REFERENCE,
                linestyle="--", linewidth=1.2, label="linear K = %+.2f deg/g"
                % analysis.understeer_gradient_deg_per_g(p))
        peak = ramp.metrics["ay_max_g"]
        ax.plot([peak], [rad2deg(ramp.understeer_angle[
            int(np.argmax(np.abs(ramp.lateral_accel)))])], "o",
            color=theme.CRITICAL, markersize=6)
        ax.annotate("limit %.2f g" % peak,
                    xy=(peak, rad2deg(ramp.understeer_angle[
                        int(np.argmax(np.abs(ramp.lateral_accel)))])),
                    xytext=(-8, 10), textcoords="offset points", ha="right",
                    fontsize=8, color=theme.CRITICAL)
        theme.reference_line(ax, p.friction, axis="x", label="mu",
                             color=theme.CRITICAL)
        theme.style_axes(ax, "Handling diagram (ramp steer at %.0f m/s)"
                         % data["v_limit"], "a_y [g]",
                         "delta - L/R [deg]")
        ax.legend(loc="upper left")

        # --- g-g ---
        ax = fig.add_subplot(gs[1, 1])
        ax.scatter(np.abs(gg.ay) / GRAVITY, gg.ax / GRAVITY, s=14,
                   color=theme.SERIES[0], alpha=0.55, linewidths=0,
                   label="simulated operating points")
        if gg.envelope.size:
            ax.plot(gg.envelope[:, 1] / GRAVITY, gg.envelope[:, 0] / GRAVITY,
                    color=theme.SERIES[1], linewidth=2.0, label="envelope")
        circle = np.linspace(-0.5 * math.pi, 0.5 * math.pi, 100)
        mu = p.friction
        ax.plot(mu * np.cos(circle), mu * np.sin(circle), color=theme.REFERENCE,
                linestyle="--", linewidth=1.2, label="friction circle mu = %.2f"
                % mu)
        theme.reference_line(ax, p.accel_max / GRAVITY, label="accel_max")
        theme.reference_line(ax, p.accel_min / GRAVITY, label="accel_min")
        theme.style_axes(ax, "g-g envelope at %.0f m/s" % data["gg_speed"],
                         "|a_y| [g]", "a_x [g]")
        ax.legend(loc="lower left", fontsize=7)

        # --- cornering speed vs radius ---
        ax = fig.add_subplot(gs[1, 2])
        radii = np.linspace(5.0, 200.0, 200)
        v_sim = max_cornering_speed(ramp.metrics["ay_max"], radii)
        v_mu = max_cornering_speed(p.friction * GRAVITY, radii)
        ax.plot(radii, v_sim, color=theme.SERIES[0], linewidth=2.0,
                label="from simulated a_y max")
        ax.plot(radii, v_mu, color=theme.REFERENCE, linestyle="--",
                linewidth=1.2, label="from mu*g")
        theme.reference_line(ax, p.speed_max, label="speed_max")
        g = AckermannGeometry.from_params(p)
        theme.reference_line(ax, minimum_turn_radius(g, p.steer_max), axis="x",
                             label="min turn R")
        theme.style_axes(ax, "Maximum cornering speed", "radius [m]",
                         "v [m/s]")
        ax.legend(loc="lower right")

        fig.suptitle("%s, %s tire, mu = %.2f"
                     % (self.fields.get_str("model"), data["tire"], p.friction),
                     x=0.065, ha="left", fontsize=10, color=theme.INK_2, y=0.97)
        self.plot.draw()

    def _fill_table(self, p: VehicleParameters, data: dict) -> None:
        accel = data["accel"]
        brake = data["brake"]
        ramp = data["ramp"]
        ideal_stop = data["v_brake"] ** 2 / (2.0 * abs(p.accel_min))
        k_lin = analysis.understeer_gradient(p)
        k_meas = ramp.metrics.get("k_measured")

        rows = [
            ["Top speed reached", "%.2f" % accel.metrics["v_reached"], "m/s",
             "%.1f km/h, envelope limit is %.1f km/h"
             % (3.6 * accel.metrics["v_reached"], 3.6 * p.speed_max)],
            ["Time to speed_max", "%.2f" % accel.metrics["t_total"], "s",
             "over %.1f m" % accel.metrics["s_total"]],
        ]
        for mark in (30, 50, 100):
            key = "t_%d_kmh" % mark
            if key in accel.metrics:
                rows.append(["0 - %d km/h" % mark, "%.2f" % accel.metrics[key],
                             "s", "commanded a_x = %.2f m/s2" % p.accel_max])
        rows += [
            ["Braking distance from %.0f m/s" % data["v_brake"],
             "%.2f" % brake.metrics["s_stop"], "m",
             "ideal point mass would need %.2f m" % ideal_stop],
            ["Braking time", "%.2f" % brake.metrics["t_stop"], "s",
             "requires mu >= %.2f, available %.2f"
             % (brake.metrics["mu_required"], p.friction)],
            ["Limit lateral acceleration", "%.3f" % ramp.metrics["ay_max_g"],
             "g", "mu*g bound is %.2f g - the models reach %.0f %% of it"
             % (p.friction, 100.0 * ramp.metrics["ay_max_g"] / p.friction)],
            ["Steer at the limit",
             "%.2f" % rad2deg(ramp.metrics["steer_at_ay_max"]), "deg",
             "body slip there %.2f deg"
             % rad2deg(ramp.metrics["beta_at_ay_max"])],
            ["Understeer gradient (linear)",
             "%+.3f" % analysis.understeer_gradient_deg_per_g(p), "deg/g",
             "closed form from the parameter set"],
        ]
        if k_meas is not None:
            rows.append([
                "Understeer gradient (measured)",
                "%+.3f" % rad2deg(k_meas * GRAVITY), "deg/g",
                "fitted below 30 %% of the limit; %.0f %% of the closed form"
                % (100.0 * k_meas / k_lin if abs(k_lin) > 1e-9 else 0.0)])
        rows.append([
            "Cornering speed at min radius",
            "%.2f" % math.sqrt(ramp.metrics["ay_max"]
                               * minimum_turn_radius(
                                   AckermannGeometry.from_params(p),
                                   p.steer_max)), "m/s",
            "at the tightest geometric radius the steering allows"])
        self.table.set_rows(rows)
