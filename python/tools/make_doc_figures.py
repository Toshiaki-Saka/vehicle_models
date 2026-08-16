#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Regenerate the figures used by docs_en/python-gui.md.

    python tools/make_doc_figures.py [output_dir]

Runs the same code paths the GUI runs, so the documentation always shows what
the application actually produces. Needs a display for Tk; the figures
themselves are rendered by matplotlib.
"""

from __future__ import annotations

import os
import sys
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle_models_py.gui.app import VehicleModelsApp
from vehicle_models_py.maneuvers import LANE_CHANGE, ManeuverConfig
from vehicle_models_py.performance import (acceleration_run, braking_run,
                                           gg_diagram, ramp_steer_run)
from vehicle_models_py.runner import run_maneuver
from vehicle_models_py.types import deg2rad

DPI = 100


def main() -> int:
    out = (sys.argv[1] if len(sys.argv) > 1
           else os.path.join(os.path.dirname(os.path.dirname(
               os.path.dirname(os.path.abspath(__file__)))), "docs_en",
               "images"))
    os.makedirs(out, exist_ok=True)

    root = tk.Tk()
    root.geometry("1220x700")
    app = VehicleModelsApp(root)
    root.update()
    params = app.params

    def save(panel, name):
        path = os.path.join(out, name)
        panel.figure.savefig(path, dpi=DPI)
        print("wrote", path)

    save(app.handling_tab.plot, "handling.png")
    save(app.tire_tab.plot, "tire.png")
    save(app.ackermann_tab.plot, "ackermann.png")

    # Step steer through four models.
    cfg = app.maneuver_tab._config()
    results = run_maneuver(params, cfg,
                           ["kin_cog", "linear2dof", "dynamic", "double_track"],
                           "Fiala")
    app.maneuver_tab._plot(results, cfg, params)
    app.maneuver_tab._fill_table(results, cfg, params)
    save(app.maneuver_tab.plot, "maneuver.png")

    # Closed-loop double lane change.
    cfg_lc = ManeuverConfig(kind=LANE_CHANGE, duration=8.0, dt=0.005,
                            initial_speed=15.0, lane_offset=3.5,
                            section_length=30.0)
    res_lc = run_maneuver(params, cfg_lc,
                          ["kin_cog", "dynamic", "double_track"], "Fiala")
    app.maneuver_tab._plot(res_lc, cfg_lc, params)
    save(app.maneuver_tab.plot, "lane_change.png")

    # Animation, frozen mid-manoeuvre.
    app.animation_tab.on_run_complete(results, cfg, params)
    app.animation_tab._draw_frame(int(0.6 * results[0].time.size))
    save(app.animation_tab.plot, "animation.png")

    # Performance suite.
    data = {
        "accel": acceleration_run(params, "Fiala", True, dt=0.01),
        "brake": braking_run(params, 20.0, "Fiala", True, dt=0.005),
        "ramp": ramp_steer_run(params, 20.0, "Fiala", True,
                               ramp_rate=deg2rad(1.5), duration=25.0, dt=0.005),
        "gg": gg_diagram(params, 20.0, "Fiala", True, n_ax=7, n_steer=14),
        "v_brake": 20.0, "v_limit": 20.0, "gg_speed": 20.0, "tire": "Fiala",
        "four_wheel": True,
    }
    app.performance_tab._plot(params, data)
    app.performance_tab._fill_table(params, data)
    save(app.performance_tab.plot, "performance.png")

    root.destroy()
    return 0


if __name__ == "__main__":
    sys.exit(main())
