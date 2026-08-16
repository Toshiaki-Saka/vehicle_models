# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Manoeuvre tab: run one test through several models and compare them."""

from __future__ import annotations

import tkinter as tk
from tkinter import filedialog, ttk
from typing import List, Optional

import numpy as np

from .. import linear_analysis as analysis
from ..integrator import IntegratorType
from ..maneuvers import (BRAKE_IN_TURN, CONSTANT_RADIUS,
                         LATERAL_PATH_MANEUVERS, MANEUVER_KINDS,
                         PATH_MANEUVERS, RAMP_STEER, ROUTE, SINE_DWELL,
                         SINE_STEER, STEP_STEER, STRAIGHT_LINE, ManeuverConfig,
                         reference_path)
from ..parameters import VehicleParameters
from ..runner import MODEL_CATALOG, RunResult, run_maneuver, to_csv
from ..tires import TIRE_TYPES
from ..types import GRAVITY, deg2rad, rad2deg
from . import theme
from .widgets import FieldGrid, PlotPanel, ScrollFrame, Table, TaskRunner

DEFAULT_MODELS = ("kin_cog", "dynamic", "double_track")

# Which extra fields matter for which manoeuvre, so the panel can grey out the
# rest instead of implying that every number is in play.
RELEVANT = {
    "steer_amplitude": {STEP_STEER, SINE_STEER, SINE_DWELL, BRAKE_IN_TURN},
    "frequency": {SINE_STEER, SINE_DWELL},
    "ramp_rate": {RAMP_STEER},
    "radius": {CONSTANT_RADIUS},
    "brake_accel": {BRAKE_IN_TURN, STRAIGHT_LINE},
    "brake_start": {BRAKE_IN_TURN, STRAIGHT_LINE},
    "lane_offset": set(LATERAL_PATH_MANEUVERS),
    "section_length": set(LATERAL_PATH_MANEUVERS),
    # The route driver owns the longitudinal channel: it drives its own speed
    # profile, so the speed hold of the runner never gets a say.
    "hold_speed": set(MANEUVER_KINDS) - {ROUTE},
}


class ManeuverTab(ttk.Frame):
    def __init__(self, master, app, **kwargs):
        super().__init__(master, **kwargs)
        self.app = app
        self.results: List[RunResult] = []
        self.cfg: Optional[ManeuverConfig] = None
        self.tasks = TaskRunner(self)

        controls = ttk.Frame(self)
        controls.pack(side="left", fill="y", padx=(6, 0), pady=6)
        self._build_controls(controls)

        right = ttk.PanedWindow(self, orient="vertical")
        right.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        self.plot = PlotPanel(right, figsize=(9.5, 7.0))
        right.add(self.plot, weight=4)
        table_frame = ttk.Frame(right)
        right.add(table_frame, weight=1)
        ttk.Label(table_frame, text="Response metrics",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w")
        self.table = Table(table_frame, [
            ("model", "Model", 210),
            ("r_peak", "peak r [deg/s]", 95),
            ("r_final", "steady r [deg/s]", 105),
            ("overshoot", "overshoot [%]", 95),
            ("t90", "T90 [s]", 70),
            ("ay_peak", "peak a_y [g]", 90),
            ("beta_peak", "peak beta [deg]", 100),
            ("v_final", "final v [m/s]", 90),
        ], height=8)
        self.table.pack(fill="both", expand=True)

        self._show_placeholder()

    # -- controls ---------------------------------------------------------
    def _build_controls(self, parent: ttk.Frame) -> None:
        scroller = ScrollFrame(parent, width=208)
        scroller.pack(fill="both", expand=True)
        body = scroller.body

        box = ttk.LabelFrame(body, text="Manoeuvre")
        box.pack(fill="x", padx=(0, 8), pady=(0, 6))
        self.fields = FieldGrid(box)
        self.fields.pack(fill="x", padx=6, pady=4)
        f = self.fields
        self.kind_var = f.add_combo("kind", "Type", MANEUVER_KINDS, STEP_STEER,
                                    width=16)
        f.add_entry("duration", "Duration", 8.0, "s")
        f.add_entry("dt", "Time step", 0.002, "s")
        f.add_entry("initial_speed", "Speed", 20.0, "m/s")
        f.add_check("hold_speed", "Hold speed (PI)", True)
        f.add_entry("steer_amplitude", "Steer amplitude", 3.0, "deg")
        f.add_entry("t_start", "Input start", 1.0, "s")
        f.add_entry("frequency", "Sine frequency", 0.5, "Hz")
        f.add_entry("ramp_rate", "Ramp rate", 1.5, "deg/s")
        f.add_entry("radius", "Radius", 50.0, "m")
        f.add_entry("brake_accel", "Brake a_x", -4.0, "m/s2")
        f.add_entry("brake_start", "Brake start", 3.0, "s")
        f.add_entry("lane_offset", "Lane offset", 3.5, "m")
        f.add_entry("section_length", "Section length", 30.0, "m")
        self.kind_var.trace_add("write", lambda *_: self._update_enabled())

        box = ttk.LabelFrame(body, text="Models")
        box.pack(fill="x", padx=(0, 8), pady=(0, 6))
        self.model_vars = {}
        for option in MODEL_CATALOG:
            var = tk.BooleanVar(value=option.key in DEFAULT_MODELS)
            self.model_vars[option.key] = var
            row = ttk.Frame(box)
            row.pack(fill="x", padx=6, pady=1)
            swatch = tk.Canvas(row, width=10, height=10, highlightthickness=0)
            swatch.create_rectangle(0, 0, 10, 10, fill=theme.model_color(option.key),
                                    outline="")
            swatch.pack(side="left", padx=(0, 5))
            ttk.Checkbutton(row, text=option.label, variable=var).pack(
                side="left", anchor="w")

        box = ttk.LabelFrame(body, text="Solver")
        box.pack(fill="x", padx=(0, 8), pady=(0, 6))
        self.solver = FieldGrid(box)
        self.solver.pack(fill="x", padx=6, pady=4)
        self.solver.add_combo("tire", "Tire model", list(TIRE_TYPES.keys()),
                              "Fiala", width=16)
        self.solver.add_combo("integrator", "Integrator",
                              [t.value for t in IntegratorType], "RK4", width=16)

        actions = ttk.Frame(body)
        actions.pack(fill="x", padx=(0, 8), pady=(0, 6))
        self.run_button = ttk.Button(actions, text="Run", command=self.run)
        self.run_button.pack(fill="x")
        ttk.Button(actions, text="Export CSV", command=self.export_csv).pack(
            fill="x", pady=(4, 0))
        self.progress = ttk.Progressbar(actions, mode="determinate", maximum=1.0)
        self.progress.pack(fill="x", pady=(6, 0))
        self.note = ttk.Label(body, text="", foreground=theme.MUTED,
                              wraplength=210, justify="left")
        self.note.pack(fill="x", padx=(0, 8))

        self._update_enabled()

    def _update_enabled(self) -> None:
        """Grey out the fields the current manoeuvre does not read."""
        kind = self.kind_var.get()
        for key, kinds in RELEVANT.items():
            widget = self.fields.widgets.get(key)
            if widget is not None:
                widget.configure(state="normal" if kind in kinds else "disabled")
        if kind == ROUTE:
            self._suggest_route_duration()

    def _suggest_route_duration(self) -> None:
        """Offer a duration that actually gets to the end of the route.

        A route is a kilometre of driving, not the 8 s a step steer needs, and
        a run that stops a third of the way along looks like a broken tracker
        rather than a truncated run. The value stays editable.
        """
        try:
            from ..route import load_route, speed_profile, travel_time
            route = load_route()
            profile = speed_profile(route, self.app.params)
        except (OSError, ValueError) as exc:
            self.app.set_status("Could not read the reference route: %s" % exc)
            return
        self.fields.set_value("duration", round(travel_time(route, profile)
                                                * 1.2 + 3.0, 1))
        self.note.configure(text="Reference route: %.0f m, tightest radius "
                                 "%.0f m. The driver plans its own speed "
                                 "profile, so Speed only sets where it starts."
                                 % (route.length,
                                    1.0 / max(abs(route.curvature).max(),
                                              1e-9)))

    # -- running ----------------------------------------------------------
    def _config(self) -> ManeuverConfig:
        f = self.fields
        return ManeuverConfig(
            kind=self.kind_var.get(),
            duration=max(f.get_float("duration", 8.0), 0.1),
            dt=max(f.get_float("dt", 0.002), 1e-4),
            initial_speed=f.get_float("initial_speed", 20.0),
            hold_speed=f.get_bool("hold_speed"),
            steer_amplitude=deg2rad(f.get_float("steer_amplitude", 3.0)),
            t_start=f.get_float("t_start", 1.0),
            frequency=max(f.get_float("frequency", 0.5), 1e-3),
            ramp_rate=deg2rad(f.get_float("ramp_rate", 1.5)),
            radius=f.get_float("radius", 50.0),
            brake_accel=f.get_float("brake_accel", -4.0),
            brake_start=f.get_float("brake_start", 3.0),
            lane_offset=f.get_float("lane_offset", 3.5),
            section_length=max(f.get_float("section_length", 30.0), 1.0),
        )

    def run(self) -> None:
        if self.tasks.busy:
            return
        params = self.app.params
        cfg = self._config()
        keys = [o.key for o in MODEL_CATALOG if self.model_vars[o.key].get()]
        if not keys:
            self.app.set_status("Select at least one model.")
            return

        tire = self.solver.get_str("tire")
        method = IntegratorType(self.solver.get_str("integrator"))
        n = cfg.n_steps() * len(keys)
        self.app.set_status("Running %s: %d model-steps..." % (cfg.kind, n))
        self.run_button.configure(state="disabled")
        self.progress.configure(value=0.0)

        warnings = []
        if cfg.initial_speed > params.speed_max + 1e-9:
            warnings.append("initial speed above speed_max: the kinematic "
                            "models clamp their speed.")
        if cfg.dt > 0.02:
            warnings.append("time step above 20 ms: the stiff dynamic models "
                            "may show integration error.")
        self.note.configure(text=" ".join(warnings))

        def work(progress):
            return run_maneuver(params, cfg, keys, tire, method, progress)

        def done(results):
            self.run_button.configure(state="normal")
            self.progress.configure(value=1.0)
            self.results = results  # type: ignore[assignment]
            self.cfg = cfg
            self._plot(results, cfg, params)
            self._fill_table(results, cfg, params)
            wall = results[0].summary.get("wall_time", 0.0) if results else 0.0
            self.app.set_status("%s finished in %.2f s (%d models, dt = %g s)."
                                % (cfg.kind, wall, len(results), cfg.dt))
            self.app.on_run_complete(results, cfg, tire)

        def failed(exc):
            self.run_button.configure(state="normal")
            self.app.set_status("Run failed: %s" % exc)

        self.tasks.start(work, done, lambda v: self.progress.configure(value=v),
                         failed)

    def export_csv(self) -> None:
        if not self.results:
            self.app.set_status("Nothing to export - run a manoeuvre first.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile="maneuver.csv")
        if not path:
            return
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(to_csv(self.results))
        self.app.set_status("Wrote %s" % path)

    # -- plotting ---------------------------------------------------------
    def _show_placeholder(self) -> None:
        fig = self.plot.figure
        fig.clear()
        ax = fig.add_subplot(111)
        theme.empty_message(
            ax, "Pick a manoeuvre and press Run.\n"
                "The same input is pushed through every selected model.")
        self.plot.draw()

    def _plot(self, results: List[RunResult], cfg: ManeuverConfig,
              params: VehicleParameters) -> None:
        fig = self.plot.figure
        fig.clear()
        gs = fig.add_gridspec(3, 2, hspace=0.6, wspace=0.24, left=0.07,
                              right=0.955, top=0.885, bottom=0.06)
        ax_xy = fig.add_subplot(gs[0, 0])
        ax_steer = fig.add_subplot(gs[0, 1])
        ax_r = fig.add_subplot(gs[1, 0])
        ax_ay = fig.add_subplot(gs[1, 1])
        ax_beta = fig.add_subplot(gs[2, 0])
        ax_v = fig.add_subplot(gs[2, 1])

        # --- trajectory ---
        path = reference_path(cfg)
        if path.size:
            ax_xy.plot(path[:, 0], path[:, 1], color=theme.REFERENCE,
                       linestyle="--", linewidth=1.2, label="reference path",
                       zorder=1)
        x_span = y_span = 0.0
        for res in results:
            ax_xy.plot(res["x"], res["y"], color=theme.model_color(res.key),
                       linewidth=2.0, label=res.label)
            x_span = max(x_span, float(np.ptp(res["x"])))
            y_span = max(y_span, float(np.ptp(res["y"])))
        # A lane change is 100 m long and 3 m wide: true scale would hide it,
        # so the aspect is released and the title says so.
        stretched = x_span > 6.0 * max(y_span, 1e-6)
        theme.style_axes(ax_xy, "Path (y exaggerated)" if stretched else "Path",
                         "x [m]", "y [m]")
        ax_xy.set_aspect("auto" if stretched else "equal",
                         adjustable="datalim" if not stretched else "box")

        # --- steer ---
        for res in results:
            ax_steer.plot(res.time, rad2deg(res["steer"]),
                          color=theme.model_color(res.key), linewidth=2.0)
        steer_title = "Road wheel angle"
        if cfg.kind in PATH_MANEUVERS:
            steer_title += " (driver output)"
        else:
            ax_steer.plot(results[0].time, rad2deg(results[0]["steer_cmd"]),
                          color=theme.REFERENCE, linestyle="--", linewidth=1.2,
                          label="command")
            ax_steer.legend(loc="upper right")
        theme.style_axes(ax_steer, steer_title, "t [s]", "delta [deg]")

        # --- yaw rate ---
        for res in results:
            ax_r.plot(res.time, rad2deg(res["r"]),
                      color=theme.model_color(res.key), linewidth=2.0)
        ss = self._closed_form(cfg, params)
        if ss is not None:
            theme.reference_line(ax_r, rad2deg(ss.yaw_rate),
                                 label="linear steady state", align="left")
        theme.style_axes(ax_r, "Yaw rate", "t [s]", "r [deg/s]")
        self._direct_labels(ax_r, results, "r", rad2deg)

        # --- lateral acceleration ---
        for res in results:
            ax_ay.plot(res.time, res["ay"] / GRAVITY,
                       color=theme.model_color(res.key), linewidth=2.0)
        theme.reference_line(ax_ay, params.friction, label="mu (tire limit)")
        theme.reference_line(ax_ay, -params.friction)
        theme.reference_line(ax_ay, 0.4, label="kinematic validity",
                             color=theme.WARNING, style=":", align="left")
        theme.style_axes(ax_ay, "Lateral acceleration", "t [s]", "a_y [g]")

        # --- side slip ---
        for res in results:
            ax_beta.plot(res.time, rad2deg(res["beta"]),
                         color=theme.model_color(res.key), linewidth=2.0)
        theme.style_axes(ax_beta, "Body slip angle", "t [s]", "beta [deg]")
        self._direct_labels(ax_beta, results, "beta", rad2deg)

        # --- speed ---
        for res in results:
            ax_v.plot(res.time, res["v"], color=theme.model_color(res.key),
                      linewidth=2.0)
        theme.reference_line(ax_v, cfg.initial_speed, label="target")
        theme.style_axes(ax_v, "Speed", "t [s]", "v [m/s]")

        handles, labels = ax_xy.get_legend_handles_labels()
        fig.legend(handles, labels, loc="upper center", ncol=min(4, len(labels)),
                   bbox_to_anchor=(0.5, 0.999), frameon=False)
        fig.suptitle("%s  -  v0 = %.1f m/s,  dt = %g s"
                     % (cfg.kind, cfg.initial_speed, cfg.dt),
                     x=0.07, ha="left", fontsize=10, color=theme.INK_2, y=0.945)
        self.plot.draw()

    @staticmethod
    def _direct_labels(ax, results, channel: str, scale) -> None:
        """Label the traces at their right-hand end when there are few enough.

        Models that converge to the same value would print their labels on top
        of each other, so the labels are pushed apart vertically first.
        """
        if len(results) > 4:
            return
        entries = []
        for res in results:
            values = res.channels[channel]
            if values.size == 0 or not np.isfinite(values[-1]):
                continue
            entries.append([scale(float(values[-1])), res])
        if not entries:
            return

        ax.figure.canvas.draw_idle()
        low, high = ax.get_ylim()
        gap = 0.075 * (high - low)  # keeps two labels one line apart
        entries.sort(key=lambda e: e[0])
        for i in range(1, len(entries)):
            if entries[i][0] - entries[i - 1][0] < gap:
                entries[i][0] = entries[i - 1][0] + gap

        for y, res in entries:
            ax.annotate(res.label.split(" (")[0],
                        xy=(res.time[-1], y), xytext=(4, 0),
                        textcoords="offset points",
                        color=theme.model_color(res.key), fontsize=7.5,
                        va="center", ha="left", clip_on=False)

    @staticmethod
    def _closed_form(cfg: ManeuverConfig, params: VehicleParameters):
        """The linear steady state this manoeuvre should converge to, if any."""
        if cfg.kind == STEP_STEER:
            return analysis.steady_state_cornering(params, cfg.initial_speed,
                                                   cfg.steer_amplitude)
        if cfg.kind == CONSTANT_RADIUS:
            delta = analysis.required_steer_angle(params, cfg.radius,
                                                  cfg.initial_speed)
            return analysis.steady_state_cornering(params, cfg.initial_speed,
                                                   delta)
        return None

    def _fill_table(self, results: List[RunResult], cfg: ManeuverConfig,
                    params: VehicleParameters) -> None:
        rows, tags = [], []
        for res in results:
            s = res.summary
            rows.append([
                res.label,
                "%.2f" % rad2deg(s.get("r_peak", float("nan"))),
                "%.2f" % rad2deg(s.get("r_final", float("nan"))),
                ("%.1f" % (100.0 * s["overshoot"])) if "overshoot" in s else "-",
                ("%.2f" % s["t_response"]) if "t_response" in s else "-",
                "%.3f" % (s.get("ay_peak", float("nan")) / GRAVITY),
                "%.2f" % rad2deg(s.get("beta_peak", float("nan"))),
                "%.2f" % s.get("v_final", float("nan")),
            ])
            tags.append("")

        ss = self._closed_form(cfg, params)
        if ss is not None:
            rows.append([
                "Linear closed form (reference)", "-",
                "%.2f" % rad2deg(ss.yaw_rate), "-", "-",
                "%.3f" % (ss.lateral_accel / GRAVITY),
                "%.2f" % rad2deg(ss.side_slip),
                "%.2f" % cfg.initial_speed,
            ])
            tags.append("reference")
        self.table.set_rows(rows, tags)
