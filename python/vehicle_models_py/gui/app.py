# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""The main window.

One vehicle definition on the left, one analysis per tab on the right. Every
tab reads the same :class:`VehicleParameters`, so switching tabs never changes
the vehicle under discussion.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk
from typing import List

from ..maneuvers import ManeuverConfig
from ..parameters import VehicleParameters
from ..runner import MODEL_CATALOG, RunResult
from . import theme
from .ackermann_tab import AckermannTab
from .animation_tab import AnimationTab
from .handling_tab import HandlingTab
from .maneuver_tab import ManeuverTab
from .param_panel import ParameterPanel
from .performance_tab import PerformanceTab
from .tire_tab import TireTab

TITLE = "vehicle_models - simulation and performance explorer"

ABOUT = """vehicle_models simulation GUI

A Python port of the header-only C++ library, driven from one shared vehicle
parameter set. The port is validated against the same closed-form results as
the C++ unit tests (python/tests/test_port.py).

Models
""" + "\n".join("  - %s: %s" % (m.label, m.note) for m in MODEL_CATALOG) + """

Units are SI throughout (m, s, rad, N, kg); angles are shown in degrees in the
user interface only. Positive steer angle = left turn = positive yaw rate.
"""


class VehicleModelsApp(ttk.Frame):
    def __init__(self, master: tk.Tk):
        super().__init__(master)
        self.master.title(TITLE)
        self.params = VehicleParameters()
        self.pack(fill="both", expand=True)

        self._init_style()
        self._build_menu()

        self.panes = ttk.PanedWindow(self, orient="horizontal")
        self.panes.pack(fill="both", expand=True)

        self.param_panel = ParameterPanel(self.panes, self.on_params_changed)
        self.panes.add(self.param_panel, weight=0)

        self.notebook = ttk.Notebook(self.panes)
        self.panes.add(self.notebook, weight=1)

        self.maneuver_tab = ManeuverTab(self.notebook, self)
        self.handling_tab = HandlingTab(self.notebook, self)
        self.tire_tab = TireTab(self.notebook, self)
        self.ackermann_tab = AckermannTab(self.notebook, self)
        self.performance_tab = PerformanceTab(self.notebook, self)
        self.animation_tab = AnimationTab(self.notebook, self)

        self.notebook.add(self.maneuver_tab, text="  Manoeuvre  ")
        self.notebook.add(self.animation_tab, text="  Animation  ")
        self.notebook.add(self.handling_tab, text="  Handling analysis  ")
        self.notebook.add(self.performance_tab, text="  Performance  ")
        self.notebook.add(self.tire_tab, text="  Tire models  ")
        self.notebook.add(self.ackermann_tab, text="  Ackermann  ")

        self.status = ttk.Label(self, text="Ready.", anchor="w",
                                foreground=theme.INK_2, padding=(8, 3))
        self.status.pack(fill="x", side="bottom")

        # Render the analytic tabs once with the default vehicle.
        self.on_params_changed(self.param_panel.params)

    # -- infrastructure ----------------------------------------------------
    def _init_style(self) -> None:
        theme.apply_matplotlib_style()
        style = ttk.Style()
        if "vista" in style.theme_names():
            style.theme_use("vista")
        elif "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure(".", background=theme.PLANE, foreground=theme.INK)
        style.configure("TFrame", background=theme.PLANE)
        style.configure("TLabel", background=theme.PLANE)
        style.configure("TLabelframe", background=theme.PLANE)
        style.configure("TLabelframe.Label", background=theme.PLANE,
                        foreground=theme.INK_2)
        style.configure("TCheckbutton", background=theme.PLANE)
        style.configure("TNotebook", background=theme.PLANE)
        style.configure("Treeview", rowheight=20, background=theme.SURFACE,
                        fieldbackground=theme.SURFACE)
        style.configure("Treeview.Heading", font=("Segoe UI", 8, "bold"))
        self.master.configure(background=theme.PLANE)

    def _build_menu(self) -> None:
        menubar = tk.Menu(self.master)
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Export manoeuvre CSV...",
                              command=lambda: self.maneuver_tab.export_csv())
        file_menu.add_separator()
        file_menu.add_command(label="Quit", command=self.master.destroy)
        menubar.add_cascade(label="File", menu=file_menu)

        view_menu = tk.Menu(menubar, tearoff=0)
        self.show_params = tk.BooleanVar(value=True)
        view_menu.add_checkbutton(label="Vehicle panel",
                                  variable=self.show_params,
                                  command=self._toggle_param_panel)
        menubar.add_cascade(label="View", menu=view_menu)

        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="About",
                              command=lambda: messagebox.showinfo(
                                  "About", ABOUT))
        menubar.add_cascade(label="Help", menu=help_menu)
        self.master.config(menu=menubar)

    def _toggle_param_panel(self) -> None:
        """Give the plots the whole window on a small screen."""
        if self.show_params.get():
            self.panes.insert(0, self.param_panel, weight=0)
        else:
            self.panes.forget(self.param_panel)

    # -- shared state ------------------------------------------------------
    def set_status(self, text: str) -> None:
        self.status.configure(text=text)
        self.status.update_idletasks()

    def on_params_changed(self, params: VehicleParameters) -> None:
        self.params = params
        self.handling_tab.refresh()
        self.tire_tab.refresh()
        self.ackermann_tab.refresh()
        self.set_status("Vehicle updated: wheelbase %.2f m, mass %.0f kg, "
                        "mu %.2f." % (params.wheel_base(), params.mass,
                                      params.friction))

    def on_run_complete(self, results: List[RunResult], cfg: ManeuverConfig,
                        tire_kind: str) -> None:
        self.animation_tab.on_run_complete(results, cfg, self.params)


def main() -> None:
    root = tk.Tk()
    # The layout is designed for 1560x980 reference pixels. Scale that to the
    # display dpi, then clamp to the screen: with Windows display scaling an
    # unscaled window is either tiny or larger than the panel it opens on.
    scale = max(1.0, min(3.0, float(root.winfo_fpixels("1i")) / 96.0))
    width = min(int(1560 * scale), root.winfo_screenwidth() - 60)
    height = min(int(980 * scale), root.winfo_screenheight() - 100)
    root.geometry("%dx%d+%d+%d" % (width, height, 20, 20))
    root.minsize(int(880 * scale), int(560 * scale))
    VehicleModelsApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
