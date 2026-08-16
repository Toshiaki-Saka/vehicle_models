# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Small Tk building blocks shared by the tabs."""

from __future__ import annotations

import queue
import threading
import tkinter as tk
from tkinter import ttk
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
from matplotlib.figure import Figure

from . import theme


def scale_factor(widget: tk.Misc) -> float:
    """Physical pixels per 96-dpi reference pixel.

    Tk scales fonts with the display DPI but not explicit pixel sizes, so on a
    150 % or 200 % display an unscaled panel width would be half the size the
    text inside it needs. Every hard-coded pixel dimension goes through here.
    """
    try:
        return max(1.0, min(3.0, float(widget.winfo_fpixels("1i")) / 96.0))
    except tk.TclError:
        return 1.0


class ScrollFrame(ttk.Frame):
    """A vertically scrollable container. Put widgets into ``.body``."""

    def __init__(self, master, width: int = 260, **kwargs):
        super().__init__(master, **kwargs)
        width = int(width * scale_factor(self))
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0,
                                width=width, background=theme.PLANE)
        self.scroll = ttk.Scrollbar(self, orient="vertical",
                                    command=self.canvas.yview)
        self.body = ttk.Frame(self.canvas)
        self._window = self.canvas.create_window((0, 0), window=self.body,
                                                 anchor="nw")

        self.body.bind("<Configure>", self._on_body_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.canvas.configure(yscrollcommand=self.scroll.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scroll.pack(side="right", fill="y")
        self.bind_all("<MouseWheel>", self._on_mousewheel, add="+")

    def _on_body_configure(self, _event) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event) -> None:
        self.canvas.itemconfigure(self._window, width=event.width)

    def _on_mousewheel(self, event) -> None:
        widget = self.winfo_containing(event.x_root, event.y_root)
        while widget is not None:
            if widget is self.canvas:
                self.canvas.yview_scroll(int(-event.delta / 120), "units")
                return
            widget = getattr(widget, "master", None)


class FieldGrid(ttk.Frame):
    """Two-column ``label / entry`` grid with typed access to the values."""

    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.vars: Dict[str, tk.StringVar] = {}
        self.widgets: Dict[str, tk.Widget] = {}
        self._row = 0
        self.columnconfigure(1, weight=1)

    def add_entry(self, key: str, label: str, value, unit: str = "",
                  width: int = 9) -> tk.StringVar:
        var = tk.StringVar(value=self._format(value))
        ttk.Label(self, text=label).grid(row=self._row, column=0, sticky="w",
                                         padx=(0, 6), pady=1)
        entry = ttk.Entry(self, textvariable=var, width=width, justify="right")
        entry.grid(row=self._row, column=1, sticky="ew", pady=1)
        if unit:
            ttk.Label(self, text=unit, foreground=theme.MUTED).grid(
                row=self._row, column=2, sticky="w", padx=(4, 0))
        self.vars[key] = var
        self.widgets[key] = entry
        self._row += 1
        return var

    def add_combo(self, key: str, label: str, values: Sequence[str],
                  value: str, width: int = 18) -> tk.StringVar:
        var = tk.StringVar(value=value)
        ttk.Label(self, text=label).grid(row=self._row, column=0, sticky="w",
                                         padx=(0, 6), pady=1)
        combo = ttk.Combobox(self, textvariable=var, values=list(values),
                             state="readonly", width=width)
        combo.grid(row=self._row, column=1, columnspan=2, sticky="ew", pady=1)
        self.vars[key] = var
        self.widgets[key] = combo
        self._row += 1
        return var

    def add_check(self, key: str, label: str, value: bool) -> tk.BooleanVar:
        var = tk.BooleanVar(value=value)
        check = ttk.Checkbutton(self, text=label, variable=var)
        check.grid(row=self._row, column=0, columnspan=3, sticky="w", pady=1)
        self.vars[key] = var  # type: ignore[assignment]
        self.widgets[key] = check
        self._row += 1
        return var

    def add_separator(self, text: str = "") -> None:
        if text:
            ttk.Label(self, text=text, font=("Segoe UI", 8, "bold"),
                      foreground=theme.INK_2).grid(
                row=self._row, column=0, columnspan=3, sticky="w", pady=(8, 2))
            self._row += 1
        ttk.Separator(self, orient="horizontal").grid(
            row=self._row, column=0, columnspan=3, sticky="ew", pady=(0, 4))
        self._row += 1

    @staticmethod
    def _format(value) -> str:
        if isinstance(value, float):
            return ("%.6g" % value)
        return str(value)

    def get_float(self, key: str, fallback: float = 0.0) -> float:
        try:
            return float(self.vars[key].get())
        except (ValueError, KeyError):
            return fallback

    def get_bool(self, key: str) -> bool:
        return bool(self.vars[key].get())

    def get_str(self, key: str) -> str:
        return str(self.vars[key].get())

    def set_value(self, key: str, value) -> None:
        if key in self.vars:
            self.vars[key].set(self._format(value))


class PlotPanel(ttk.Frame):
    """A matplotlib figure with the navigation toolbar underneath."""

    def __init__(self, master, figsize: Tuple[float, float] = (9.0, 6.0),
                 dpi: int = 100, **kwargs):
        super().__init__(master, **kwargs)
        # Match the figure dpi to the display so plot text stays the same
        # visual size as the Tk text next to it.
        dpi = int(dpi * scale_factor(self))
        self.figure = Figure(figsize=figsize, dpi=dpi)
        self.figure.patch.set_facecolor(theme.SURFACE)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        widget = self.canvas.get_tk_widget()
        widget.configure(background=theme.SURFACE, highlightthickness=0)

        self.toolbar = NavigationToolbar2Tk(self.canvas, self,
                                            pack_toolbar=False)
        self.toolbar.config(background=theme.PLANE)
        for child in self.toolbar.winfo_children():
            try:
                child.configure(background=theme.PLANE)
            except tk.TclError:
                pass
        self.toolbar.update()

        widget.pack(side="top", fill="both", expand=True)
        self.toolbar.pack(side="bottom", fill="x")

    def draw(self) -> None:
        self.canvas.draw_idle()

    def clear(self) -> None:
        self.figure.clear()


class Table(ttk.Frame):
    """Read-only Treeview used for the metric tables."""

    def __init__(self, master, columns: Sequence[Tuple[str, str, int]],
                 height: int = 6, **kwargs):
        super().__init__(master, **kwargs)
        keys = [c[0] for c in columns]
        scale = scale_factor(self)
        self.tree = ttk.Treeview(self, columns=keys, show="headings",
                                 height=height, selectmode="none")
        for key, title, width in columns:
            self.tree.heading(key, text=title)
            anchor = "w" if width >= 150 else "e"
            self.tree.column(key, width=int(width * scale), anchor=anchor,
                             stretch=True)
        scroll = ttk.Scrollbar(self, orient="vertical",
                               command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.tree.tag_configure("reference", foreground=theme.MUTED)

    def set_rows(self, rows: Sequence[Sequence[str]],
                 tags: Optional[Sequence[str]] = None) -> None:
        self.tree.delete(*self.tree.get_children())
        for i, row in enumerate(rows):
            tag = (tags[i],) if tags and i < len(tags) else ()
            self.tree.insert("", "end", values=list(row), tags=tag)


class TaskRunner:
    """Run a long simulation off the Tk thread and report progress back.

    Tk is not thread safe, so the worker only ever pushes messages into a
    queue; everything that touches a widget happens in the polling callback.
    """

    def __init__(self, root: tk.Misc, poll_ms: int = 60):
        self.root = root
        self.poll_ms = poll_ms
        self.queue: "queue.Queue[Tuple[str, object]]" = queue.Queue()
        self.busy = False
        self._on_done: Optional[Callable[[object], None]] = None
        self._on_progress: Optional[Callable[[float], None]] = None
        self._on_error: Optional[Callable[[BaseException], None]] = None

    def start(self, work: Callable[[Callable[[float], None]], object],
              on_done: Callable[[object], None],
              on_progress: Optional[Callable[[float], None]] = None,
              on_error: Optional[Callable[[BaseException], None]] = None) -> bool:
        if self.busy:
            return False
        self.busy = True
        self._on_done = on_done
        self._on_progress = on_progress
        self._on_error = on_error

        def progress(value: float) -> None:
            self.queue.put(("progress", value))

        def target() -> None:
            try:
                result = work(progress)
            except BaseException as exc:  # reported in the UI, not swallowed
                self.queue.put(("error", exc))
            else:
                self.queue.put(("done", result))

        threading.Thread(target=target, daemon=True).start()
        self.root.after(self.poll_ms, self._poll)
        return True

    def _poll(self) -> None:
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "progress":
                    if self._on_progress:
                        self._on_progress(float(payload))  # type: ignore[arg-type]
                elif kind == "done":
                    self.busy = False
                    if self._on_done:
                        self._on_done(payload)
                    return
                elif kind == "error":
                    self.busy = False
                    if self._on_error:
                        self._on_error(payload)  # type: ignore[arg-type]
                    return
        except queue.Empty:
            pass
        if self.busy:
            self.root.after(self.poll_ms, self._poll)


def button_row(master, buttons: Sequence[Tuple[str, Callable[[], None]]],
               ) -> Tuple[ttk.Frame, List[ttk.Button]]:
    frame = ttk.Frame(master)
    widgets = []
    for i, (text, command) in enumerate(buttons):
        b = ttk.Button(frame, text=text, command=command)
        b.grid(row=0, column=i, padx=(0 if i == 0 else 6, 0), sticky="w")
        widgets.append(b)
    return frame, widgets
