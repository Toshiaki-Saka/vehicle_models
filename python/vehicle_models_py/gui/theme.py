# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""One visual language for every plot and every widget in the app.

The categorical slots are assigned per model *key*, never per plotted position,
so a model keeps its colour when other models are switched on and off.
"""

from __future__ import annotations

from typing import Dict, Sequence

import matplotlib as mpl

# --- surfaces and ink ------------------------------------------------------
SURFACE = "#fcfcfb"  # chart surface
PLANE = "#f9f9f7"  # window background
INK = "#0b0b0b"  # primary text
INK_2 = "#52514e"  # secondary text
MUTED = "#898781"  # axis labels, ticks
GRID = "#e1e0d9"  # hairline grid
AXIS = "#c3c2b7"  # baseline / axis
BORDER = "#e1e0d9"

# --- categorical slots, in fixed order -------------------------------------
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100",
          "#e87ba4", "#008300", "#4a3aa7", "#e34948")

# --- sequential ramp (blue), light -> dark ---------------------------------
SEQUENTIAL = ("#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#2a78d6",
              "#256abf", "#184f95", "#0d366b")

# --- status ----------------------------------------------------------------
GOOD = "#0ca30c"
WARNING = "#fab219"
SERIOUS = "#ec835a"
CRITICAL = "#d03b3b"

# Colour per model key: fixed, so the identity survives any filtering.
MODEL_COLORS: Dict[str, str] = {
    "kin_rear": SERIES[0],
    "kin_cog": SERIES[1],
    "kin_steer": SERIES[2],
    "linear2dof": SERIES[3],
    "dynamic": SERIES[4],
    "blended": SERIES[5],
    "double_track": SERIES[6],
}

REFERENCE = MUTED  # closed-form / limit reference lines


def model_color(key: str) -> str:
    return MODEL_COLORS.get(key, SERIES[7])


def apply_matplotlib_style() -> None:
    """Recessive chrome, 2 px marks, no boxes around anything."""
    mpl.rcParams.update({
        "figure.facecolor": SURFACE,
        "figure.edgecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": AXIS,
        "axes.labelcolor": INK_2,
        "axes.titlecolor": INK,
        "axes.titlesize": 10,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.titlepad": 8,
        "axes.labelsize": 9,
        "axes.linewidth": 0.8,
        "axes.grid": True,
        "axes.axisbelow": True,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "lines.linewidth": 2.0,
        "lines.markersize": 4.0,
        "legend.frameon": False,
        "legend.fontsize": 8,
        "legend.labelcolor": INK_2,
        "font.size": 9,
        "font.family": "sans-serif",
        "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
        "figure.autolayout": False,
    })


def style_axes(ax, title: str = "", xlabel: str = "", ylabel: str = "") -> None:
    """Drop the top/right spines and label the axes consistently."""
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    if title:
        ax.set_title(title)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.tick_params(length=3, width=0.8)


def reference_line(ax, value: float, axis: str = "y", label: str = "",
                   color: str = REFERENCE, style: str = "--",
                   align: str = "right") -> None:
    """A closed-form or limit reference, drawn so it never competes with data.

    ``align`` moves the label to the other end of the line, which is how a
    reference label avoids the direct labels at the right-hand end of a trace.
    """
    if axis == "y":
        ax.axhline(value, color=color, linestyle=style, linewidth=1.2,
                   zorder=1)
    else:
        ax.axvline(value, color=color, linestyle=style, linewidth=1.2,
                   zorder=1)
    if label:
        if axis == "y":
            x = 0.995 if align == "right" else 0.005
            ax.annotate(label, xy=(x, value), xycoords=("axes fraction", "data"),
                        ha=align, va="bottom", fontsize=7.5, color=color)
        else:
            y = 0.98 if align == "right" else 0.02
            ax.annotate(label, xy=(value, y), xycoords=("data", "axes fraction"),
                        ha="left", va="top" if align == "right" else "bottom",
                        fontsize=7.5, color=color, rotation=90)


def empty_message(ax, text: str) -> None:
    ax.clear()
    ax.set_axis_off()
    ax.text(0.5, 0.5, text, ha="center", va="center", color=MUTED,
            fontsize=9, transform=ax.transAxes)


def sequential_colors(n: int) -> Sequence[str]:
    """``n`` steps of the blue ramp, light to dark, never lighter than step 250."""
    if n <= 1:
        return [SEQUENTIAL[4]]
    usable = SEQUENTIAL[1:]
    idx = [round(i * (len(usable) - 1) / (n - 1)) for i in range(n)]
    return [usable[i] for i in idx]
