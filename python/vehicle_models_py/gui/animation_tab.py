# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Animation tab: play back a manoeuvre as a moving vehicle.

The top view shows what the numbers mean -- the wheels sit at their real
Ackermann angles, the discs under them scale with the vertical load, and the
arrow is the velocity vector, so load transfer and body slip are visible
rather than merely plotted.

The scene itself lives in ``vehicle_models_py.animation`` and is shared with
the command-line demo (``tools/make_animation_demo.py``); this tab only owns
the transport controls around it.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import List, Optional

from ..animation import VehicleScene
from ..maneuvers import ManeuverConfig
from ..parameters import VehicleParameters
from ..runner import RunResult
from . import theme
from .widgets import FieldGrid, PlotPanel

FRAME_MS = 33


class AnimationTab(ttk.Frame):
    def __init__(self, master, app, **kwargs):
        super().__init__(master, **kwargs)
        self.app = app
        self.results: List[RunResult] = []
        self.cfg: Optional[ManeuverConfig] = None
        self.params: Optional[VehicleParameters] = None
        self.scene: Optional[VehicleScene] = None
        self.index = 0
        self.playing = False
        self._job: Optional[str] = None

        controls = ttk.Frame(self)
        controls.pack(side="left", fill="y", padx=(6, 0), pady=6)

        box = ttk.LabelFrame(controls, text="Playback")
        box.pack(fill="x", pady=(0, 6))
        self.fields = FieldGrid(box)
        self.fields.pack(fill="x", padx=6, pady=4)
        self.model_var = self.fields.add_combo("model", "Model", ("-",), "-",
                                               width=16)
        self.fields.add_entry("rate", "Playback rate", 1.0, "x")
        self.fields.add_entry("span", "View span", 25.0, "m")
        self.model_var.trace_add("write", lambda *_: self._on_model_change())

        buttons = ttk.Frame(controls)
        buttons.pack(fill="x", pady=(0, 6))
        self.play_button = ttk.Button(buttons, text="Play", command=self.toggle)
        self.play_button.pack(side="left", fill="x", expand=True)
        ttk.Button(buttons, text="Reset", command=self.reset).pack(
            side="left", fill="x", expand=True, padx=(6, 0))

        self.time_var = tk.DoubleVar(value=0.0)
        self.slider = ttk.Scale(controls, from_=0.0, to=1.0, orient="horizontal",
                                variable=self.time_var,
                                command=self._on_slider)
        self.slider.pack(fill="x")
        self.readout = tk.Text(controls, height=10, width=28, relief="flat",
                               background=theme.SURFACE, foreground=theme.INK_2,
                               font=("Consolas", 8), highlightthickness=0)
        self.readout.pack(fill="x", pady=(8, 0))
        self.readout.configure(state="disabled")

        ttk.Label(controls, text=(
            "Disc area under each wheel\n"
            "is proportional to its vertical\n"
            "load. Only the double-track\n"
            "model resolves the four wheels\n"
            "individually; the single-track\n"
            "models split each axle evenly."),
            foreground=theme.MUTED, justify="left").pack(anchor="w",
                                                         pady=(10, 0))

        self.plot = PlotPanel(self, figsize=(9.5, 6.8))
        self.plot.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        self._show_placeholder()

    # -- data ---------------------------------------------------------------
    def on_run_complete(self, results: List[RunResult], cfg: ManeuverConfig,
                        params: VehicleParameters) -> None:
        self.stop()
        self.results = results
        self.cfg = cfg
        self.params = params
        labels = [r.label for r in results]
        combo = self.fields.widgets.get("model")
        if combo is not None:
            combo.configure(values=labels)  # type: ignore[call-arg]
        if labels and self.model_var.get() not in labels:
            self.model_var.set(labels[-1])
        else:
            self._on_model_change()

    def _current(self) -> Optional[RunResult]:
        for res in self.results:
            if res.label == self.model_var.get():
                return res
        return self.results[-1] if self.results else None

    def _on_model_change(self) -> None:
        res = self._current()
        if res is None or self.cfg is None or self.params is None:
            return
        self.index = 0
        self.slider.configure(to=float(res.time[-1]))
        self.time_var.set(0.0)
        self.scene = VehicleScene(self.plot.figure, self.params, res, self.cfg,
                                  span=self.fields.get_float("span", 25.0))
        self.plot.draw()
        self._draw_frame(0)

    # -- playback -----------------------------------------------------------
    def toggle(self) -> None:
        if not self.results:
            self.app.set_status("Run a manoeuvre first, then play it back here.")
            return
        self.playing = not self.playing
        self.play_button.configure(text="Pause" if self.playing else "Play")
        if self.playing:
            self._tick()

    def stop(self) -> None:
        self.playing = False
        self.play_button.configure(text="Play")
        if self._job is not None:
            self.after_cancel(self._job)
            self._job = None

    def reset(self) -> None:
        self.stop()
        self.index = 0
        self.time_var.set(0.0)
        self._draw_frame(0)

    def _tick(self) -> None:
        if not self.playing or self.cfg is None:
            return
        res = self._current()
        if res is None:
            return
        rate = max(self.fields.get_float("rate", 1.0), 0.05)
        stride = max(1, int(round(rate * (FRAME_MS / 1000.0) / self.cfg.dt)))
        self.index += stride
        if self.index >= res.time.size:
            self.index = res.time.size - 1
            self._draw_frame(self.index)
            self.stop()
            return
        self.time_var.set(float(res.time[self.index]))
        self._draw_frame(self.index)
        self._job = self.after(FRAME_MS, self._tick)

    def _on_slider(self, _value: str) -> None:
        if self.playing or self.cfg is None:
            return
        res = self._current()
        if res is None:
            return
        target = float(self.time_var.get())
        self.index = int(min(max(target / self.cfg.dt, 0), res.time.size - 1))
        self._draw_frame(self.index)

    # -- scene --------------------------------------------------------------
    def _show_placeholder(self) -> None:
        fig = self.plot.figure
        fig.clear()
        ax = fig.add_subplot(111)
        theme.empty_message(
            ax, "Run a manoeuvre on the Manoeuvre tab,\n"
                "then press Play here to watch it.")
        self.plot.draw()

    def _draw_frame(self, i: int) -> None:
        scene = self.scene
        if scene is None:
            return
        scene.span = max(self.fields.get_float("span", 25.0), 8.0)
        i = int(min(max(i, 0), scene.result.time.size - 1))
        scene.draw(i)
        self._update_readout(scene, i)
        self.plot.canvas.draw_idle()

    def _update_readout(self, scene: VehicleScene, i: int) -> None:
        self.readout.configure(state="normal")
        self.readout.delete("1.0", "end")
        self.readout.insert("1.0", "\n".join(scene.readout_lines(i)))
        self.readout.configure(state="disabled")
