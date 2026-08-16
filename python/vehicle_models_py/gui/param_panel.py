# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""The vehicle definition panel, shared by every tab.

One parameter set drives the whole window: change it here and every analysis
re-renders against the same vehicle, which is what makes the cross-model
comparison meaningful.
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from typing import Callable, List, Tuple

from .. import linear_analysis as analysis
from ..ackermann import AckermannGeometry, minimum_turn_radius
from ..parameters import PRESETS, VehicleParameters
from ..types import GRAVITY, deg2rad, rad2deg
from . import theme
from .widgets import FieldGrid, ScrollFrame

# (attribute, label, unit, is_angle)
FIELDS: List[Tuple[str, str, str, bool]] = [
    ("__geometry__", "Geometry", "", False),
    ("l_f", "CoG to front axle", "m", False),
    ("l_r", "CoG to rear axle", "m", False),
    ("track_front", "Front track", "m", False),
    ("track_rear", "Rear track", "m", False),
    ("cg_height", "CoG height", "m", False),
    ("wheel_radius", "Wheel radius", "m", False),
    ("__inertia__", "Inertia", "", False),
    ("mass", "Mass", "kg", False),
    ("inertia_z", "Yaw inertia", "kg m2", False),
    ("__tires__", "Tires (per axle)", "", False),
    ("cornering_stiffness_front", "C_front", "N/rad", False),
    ("cornering_stiffness_rear", "C_rear", "N/rad", False),
    ("friction", "Road friction mu", "-", False),
    ("__resist__", "Resistances", "", False),
    ("drag_area", "0.5 rho Cd A", "N/(m/s)2", False),
    ("rolling_resistance", "Rolling resistance", "-", False),
    ("__steering__", "Steering", "", False),
    ("steering_ratio", "Steering ratio", "-", False),
    ("ackermann_ratio", "Ackermann ratio", "-", False),
    ("steer_max", "Steer limit", "deg", True),
    ("steer_rate_max", "Steer rate limit", "deg/s", True),
    ("steer_time_constant", "Steer time constant", "s", False),
    ("__limits__", "Actuation limits", "", False),
    ("accel_max", "Max acceleration", "m/s2", False),
    ("accel_min", "Max deceleration", "m/s2", False),
    ("speed_max", "Max speed", "m/s", False),
    ("__numerics__", "Numerics", "", False),
    ("low_speed_guard", "Low speed guard", "m/s", False),
]


class ParameterPanel(ttk.Frame):
    def __init__(self, master, on_apply: Callable[[VehicleParameters], None],
                 **kwargs):
        super().__init__(master, **kwargs)
        self.on_apply = on_apply
        self.params = VehicleParameters()

        header = ttk.Frame(self)
        header.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Label(header, text="Vehicle", font=("Segoe UI", 11, "bold")).pack(
            anchor="w")
        self.preset_var = tk.StringVar(value="Passenger car")
        preset = ttk.Combobox(header, textvariable=self.preset_var,
                              values=list(PRESETS.keys()), state="readonly")
        preset.pack(fill="x", pady=(4, 0))
        preset.bind("<<ComboboxSelected>>", self._on_preset)

        scroller = ScrollFrame(self, width=224)
        scroller.pack(fill="both", expand=True, padx=(8, 0), pady=4)
        self.grid_fields = FieldGrid(scroller.body)
        self.grid_fields.pack(fill="x", padx=(0, 8))

        for key, label, unit, is_angle in FIELDS:
            if key.startswith("__"):
                self.grid_fields.add_separator(label)
                continue
            value = getattr(self.params, key)
            self.grid_fields.add_entry(key, label,
                                       rad2deg(value) if is_angle else value,
                                       unit)

        buttons = ttk.Frame(self)
        buttons.pack(fill="x", padx=8, pady=(4, 2))
        ttk.Button(buttons, text="Apply", command=self.apply).pack(
            side="left", fill="x", expand=True)
        ttk.Button(buttons, text="Reset", command=self._on_preset).pack(
            side="left", fill="x", expand=True, padx=(6, 0))

        self.status = ttk.Label(self, text="", foreground=theme.MUTED,
                                wraplength=240, justify="left")
        self.status.pack(fill="x", padx=8, pady=(0, 4))

        derived = ttk.LabelFrame(self, text="Derived")
        derived.pack(fill="x", padx=8, pady=(0, 8))
        self.derived_text = tk.Text(derived, height=11, width=30, relief="flat",
                                    background=theme.SURFACE, foreground=theme.INK_2,
                                    font=("Consolas", 8), highlightthickness=0)
        self.derived_text.pack(fill="both", expand=True, padx=4, pady=4)
        self.derived_text.configure(state="disabled")

        self._refresh_derived()

    # -- events -----------------------------------------------------------
    def _on_preset(self, _event=None) -> None:
        factory = PRESETS.get(self.preset_var.get())
        if factory is None:
            return
        self.params = factory()
        for key, _label, _unit, is_angle in FIELDS:
            if key.startswith("__"):
                continue
            value = getattr(self.params, key)
            self.grid_fields.set_value(key, rad2deg(value) if is_angle else value)
        self.apply()

    def apply(self) -> None:
        p = VehicleParameters()
        for key, _label, _unit, is_angle in FIELDS:
            if key.startswith("__"):
                continue
            raw = self.grid_fields.get_float(key, getattr(p, key))
            setattr(p, key, deg2rad(raw) if is_angle else raw)

        errors = p.validate()
        if errors:
            self.status.configure(text="Invalid: " + "; ".join(errors),
                                  foreground=theme.CRITICAL)
            return
        self.status.configure(text="Parameter set is valid.",
                              foreground=theme.MUTED)
        self.params = p
        self._refresh_derived()
        self.on_apply(p)

    # -- derived quantities ------------------------------------------------
    def _refresh_derived(self) -> None:
        p = self.params
        g = AckermannGeometry.from_params(p)
        k_deg_g = analysis.understeer_gradient_deg_per_g(p)
        v_ch = analysis.characteristic_speed(p)
        v_cr = analysis.critical_speed(p)
        front_share = 100.0 * p.l_r / p.wheel_base()

        lines = [
            "wheelbase      %7.3f m" % p.wheel_base(),
            "weight distr.  %5.1f / %.1f %%" % (front_share, 100.0 - front_share),
            "static Fz f/r  %5.0f / %.0f N" % (p.static_load_front(),
                                               p.static_load_rear()),
            "understeer K   %+7.3f deg/g" % k_deg_g,
            "static margin  %+7.4f" % analysis.static_margin(p),
            "NSP behind fr. %7.3f m" % analysis.neutral_steer_point(p),
        ]
        if math.isfinite(v_ch):
            lines.append("V_char         %7.2f m/s (%.0f km/h)" % (v_ch, v_ch * 3.6))
        if math.isfinite(v_cr):
            lines.append("V_crit         %7.2f m/s (%.0f km/h)" % (v_cr, v_cr * 3.6))
        if not math.isfinite(v_ch) and not math.isfinite(v_cr):
            lines.append("neutral steer  (K = 0)")
        lines += [
            "a_y max (mu g) %7.2f m/s2 (%.2f g)" % (p.friction * GRAVITY,
                                                    p.friction),
            "min turn R     %7.2f m" % minimum_turn_radius(g, p.steer_max),
            "handwheel lock %7.1f deg" % rad2deg(p.steer_max * p.steering_ratio),
        ]

        self.derived_text.configure(state="normal")
        self.derived_text.delete("1.0", "end")
        self.derived_text.insert("1.0", "\n".join(lines))
        self.derived_text.configure(state="disabled")
