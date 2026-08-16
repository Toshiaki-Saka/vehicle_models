#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Draw the figures that go with the derivations in docs_*/models.md.

    python tools/make_model_figures.py [output_dir]

Two kinds of figure live here:

* schematics (frames, Ackermann geometry, free body diagrams, the brush
  contact patch) drawn from the same symbols the derivations use, and
* plots computed by the library itself (tire curves, yaw rate gain,
  integrator convergence), so the picture cannot drift away from the code.

The algebra stays in the document; a figure carries symbols and at most the
one relation it exists to show. Labels are symbols only, so a single set of
figures serves both the Japanese and the English text. No Tk needed.
"""

from __future__ import annotations

import math
import os
import sys

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, FancyArrowPatch, Polygon, Rectangle

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle_models_py import linear_analysis as analysis
from vehicle_models_py.integrator import IntegratorType, step
from vehicle_models_py.parameters import make_passenger_car_parameters
from vehicle_models_py.tires import make_tire
from vehicle_models_py.types import deg2rad
from vehicle_models_py.unicycle import (UnicycleModel, unicycle_input,
                                        unicycle_state)

# --- palette, matching vehicle_models_py/gui/theme.py -----------------------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
GREEN = "#1baf7a"
AMBER = "#eda100"
PINK = "#e87ba4"
PURPLE = "#4a3aa7"
RED = "#d03b3b"

BODY_FILL = "#e8eef7"
BODY_EDGE = "#9fb4d0"
DPI = 160

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "font.size": 9,
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Segoe UI", "Arial"],
    "mathtext.fontset": "dejavusans",
})


# --- drawing helpers -------------------------------------------------------
def schematic(ax, xlim, ylim):
    ax.set_aspect("equal")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.axis("off")


def arrow(ax, p0, p1, color=INK, lw=1.6, style="-|>", ls="-", mut=11, z=3):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=mut,
                                 color=color, lw=lw, linestyle=ls,
                                 shrinkA=0, shrinkB=0, zorder=z))


def label(ax, xy, text, color=INK, size=10, ha="center", va="center", z=6):
    ax.text(xy[0], xy[1], text, color=color, fontsize=size, ha=ha, va=va,
            zorder=z)


def angle_arc(ax, centre, r, a0, a1, color=MUTED, lw=1.1, text=None,
              text_r=None, size=10):
    ax.add_patch(Arc(centre, 2 * r, 2 * r, angle=0, theta1=math.degrees(a0),
                     theta2=math.degrees(a1), color=color, lw=lw, zorder=2))
    if text:
        am = 0.5 * (a0 + a1)
        rr = text_r if text_r is not None else r * 1.45
        label(ax, (centre[0] + rr * math.cos(am), centre[1] + rr * math.sin(am)),
              text, color=color, size=size)


def dashed(ax, p0, p1, color=AXIS, lw=1.0, ls=(0, (4, 3)), z=1):
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=color, lw=lw, linestyle=ls,
            zorder=z)


def dim(ax, p0, p1, text, gap=0.22, color=MUTED, size=9):
    """Dimension line between two points, label offset along the normal."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    n = math.hypot(dx, dy)
    if n == 0:
        return
    nx, ny = -dy / n, dx / n
    arrow(ax, p0, p1, color=color, lw=0.9, style="<|-|>", mut=7, z=2)
    mx, my = 0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])
    label(ax, (mx + nx * gap, my + ny * gap), text, color=color, size=size)


def wheel(ax, centre, angle=0.0, length=0.42, width=0.16, color=INK,
          fill="#d8d7d1", z=4):
    c, s = math.cos(angle), math.sin(angle)
    hx, hy = length / 2, width / 2
    pts = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    rot = [(centre[0] + c * px - s * py, centre[1] + s * px + c * py)
           for px, py in pts]
    ax.add_patch(Polygon(rot, closed=True, facecolor=fill, edgecolor=color,
                         lw=1.2, zorder=z))


def body(ax, xy, w, h, angle=0.0, alpha=1.0):
    r = Rectangle(xy, w, h, facecolor=BODY_FILL, edgecolor=BODY_EDGE, lw=1.2,
                  zorder=1, alpha=alpha)
    if angle:
        t = matplotlib.transforms.Affine2D().rotate_deg_around(
            xy[0], xy[1], math.degrees(angle)) + ax.transData
        r.set_transform(t)
    ax.add_patch(r)
    return r


def caption(ax, text, x=0.5, y=-0.02, size=9, color=INK_2):
    ax.text(x, y, text, transform=ax.transAxes, ha="center", va="top",
            fontsize=size, color=color)


def save(fig, out, name):
    path = os.path.join(out, name)
    fig.savefig(path, dpi=DPI, bbox_inches="tight", pad_inches=0.12)
    plt.close(fig)
    print("  " + name)


# --- 1. frames and sign convention -----------------------------------------
def fig_frames(out):
    fig, ax = plt.subplots(figsize=(5.4, 3.3))
    schematic(ax, (-0.9, 4.9), (-1.1, 3.0))

    arrow(ax, (-0.55, -0.7), (0.75, -0.7), color=MUTED, lw=1.2)
    arrow(ax, (-0.55, -0.7), (-0.55, 0.6), color=MUTED, lw=1.2)
    label(ax, (0.88, -0.7), "$X$", color=MUTED)
    label(ax, (-0.55, 0.74), "$Y$", color=MUTED)

    psi = math.radians(28.0)
    cx, cy = 2.15, 0.95
    c, s = math.cos(psi), math.sin(psi)

    def B(px, py):
        return (cx + c * px - s * py, cy + s * px + c * py)

    pts = [(-1.2, -0.4), (1.2, -0.4), (1.2, 0.4), (-1.2, 0.4)]
    ax.add_patch(Polygon([B(*p) for p in pts], closed=True,
                         facecolor=BODY_FILL, edgecolor=BODY_EDGE, lw=1.2))
    for px, py in ((-0.8, -0.4), (-0.8, 0.4), (0.9, -0.4), (0.9, 0.4)):
        wheel(ax, B(px, py), psi, length=0.34, width=0.13)

    arrow(ax, B(0, 0), B(1.85, 0), color=BLUE, lw=1.6)
    arrow(ax, B(0, 0), B(0, 1.15), color=BLUE, lw=1.6)
    label(ax, B(2.02, -0.02), "$x$", color=BLUE)
    label(ax, B(-0.06, 1.32), "$y$", color=BLUE)

    # heading reference and yaw angle, kept below the body x axis
    dashed(ax, (cx, cy), (cx + 2.5, cy))
    angle_arc(ax, (cx, cy), 1.5, 0.0, psi, color=INK_2)
    label(ax, (cx + 1.72 * math.cos(psi * 0.42),
               cy + 1.72 * math.sin(psi * 0.42) - 0.1),
          r"$\psi$", color=INK_2)

    # velocity, side slip positive (v rotated further CCW than the body axis)
    beta = math.radians(15.0)
    vlen = 1.5
    arrow(ax, (cx, cy), (cx + vlen * math.cos(psi + beta),
                         cy + vlen * math.sin(psi + beta)),
          color=ORANGE, lw=2.0)
    label(ax, (cx + (vlen + 0.24) * math.cos(psi + beta),
               cy + (vlen + 0.24) * math.sin(psi + beta)),
          "$v$", color=ORANGE)
    angle_arc(ax, (cx, cy), 1.02, psi, psi + beta, color=ORANGE)
    label(ax, (cx + 1.2 * math.cos(psi + beta * 0.5) + 0.12,
               cy + 1.2 * math.sin(psi + beta * 0.5) + 0.1),
          r"$\beta$", color=ORANGE)

    # yaw rate, drawn behind the vehicle
    ax.add_patch(Arc((cx, cy), 1.05, 1.05, theta1=140, theta2=205, color=GREEN,
                     lw=1.6, zorder=2))
    a_end = math.radians(140)
    arrow(ax, (cx + 0.525 * math.cos(a_end + 0.12),
               cy + 0.525 * math.sin(a_end + 0.12)),
          (cx + 0.525 * math.cos(a_end), cy + 0.525 * math.sin(a_end)),
          color=GREEN, lw=1.6, mut=9)
    label(ax, (cx - 0.95, cy + 0.62), r"$r=\dot\psi$", color=GREEN, size=9)

    ax.plot([cx], [cy], "o", color=INK, ms=4.5, zorder=7)
    label(ax, (cx + 0.16, cy - 0.26), "CoG", color=INK_2, size=8, ha="left")
    caption(ax, r"right-handed frame:  $x$ forward, $y$ left,"
                r"  $\psi$, $r$, $\delta$, $\beta > 0$ in a left turn")
    save(fig, out, "derivation_frames.png")


# --- 2. differential drive --------------------------------------------------
def fig_unicycle(out):
    fig, ax = plt.subplots(figsize=(4.6, 3.4))
    b, R = 1.6, 2.6
    schematic(ax, (-1.5, 3.1), (-1.7, R + 0.75))

    icr = (0.0, R)
    ax.plot([icr[0]], [icr[1]], "o", color=RED, ms=5, zorder=7)
    label(ax, (icr[0], icr[1] + 0.32), "ICR", color=RED, size=9)
    dashed(ax, icr, (0.0, -b / 2 - 0.45), color=MUTED)

    body(ax, (-0.5, -b / 2 - 0.04), 1.0, b + 0.08)
    wheel(ax, (0.0, b / 2), 0.0, length=0.6, width=0.2)
    wheel(ax, (0.0, -b / 2), 0.0, length=0.6, width=0.2)
    ax.plot([0, 0], [-b / 2, b / 2], color=INK_2, lw=1.0, zorder=2)
    ax.plot([0], [0], "o", color=INK, ms=4, zorder=6)

    label(ax, (-0.18, R * 0.6), "$R$", color=MUTED, size=10, ha="right")
    dim(ax, (-0.85, -b / 2), (-0.85, b / 2), "$b$", gap=0.24, color=INK_2)

    # inner (left) wheel is slower than the outer (right) one
    arrow(ax, (0.0, b / 2), (0.9, b / 2), color=BLUE, lw=1.8)
    arrow(ax, (0.0, -b / 2), (1.55, -b / 2), color=BLUE, lw=1.8)
    label(ax, (0.98, b / 2 + 0.26), r"$R_w\omega_l$", color=BLUE, size=9,
          ha="left")
    label(ax, (1.63, -b / 2 + 0.26), r"$R_w\omega_r$", color=BLUE, size=9,
          ha="left")
    arrow(ax, (0.0, 0.0), (1.25, 0.0), color=ORANGE, lw=2.0)
    label(ax, (1.34, 0.0), "$v$", color=ORANGE, ha="left")

    caption(ax, r"every point turns about the ICR:  $v=\omega R$")
    save(fig, out, "derivation_unicycle.png")


# --- 3. Ackermann geometry --------------------------------------------------
def fig_ackermann(out):
    fig, ax = plt.subplots(figsize=(5.2, 4.0))
    L, T, R = 2.7, 1.6, 4.0
    icr = (0.0, R)
    d_l = math.atan2(L, R - T / 2)
    d_r = math.atan2(L, R + T / 2)
    schematic(ax, (-1.9, 4.6), (-2.1, R + 0.8))

    body(ax, (-0.45, -T / 2 - 0.05), L + 0.9, T + 0.1)
    ax.plot([0, 0], [-T / 2, T / 2], color=INK_2, lw=1.2, zorder=2)
    ax.plot([L, L], [-T / 2, T / 2], color=INK_2, lw=1.2, zorder=2)
    wheel(ax, (0.0, T / 2), 0.0, length=0.5, width=0.18)
    wheel(ax, (0.0, -T / 2), 0.0, length=0.5, width=0.18)
    wheel(ax, (L, T / 2), d_l, length=0.5, width=0.18)
    wheel(ax, (L, -T / 2), d_r, length=0.5, width=0.18)

    ax.plot([icr[0]], [icr[1]], "o", color=RED, ms=5, zorder=7)
    label(ax, (icr[0] - 0.12, icr[1] + 0.3), "ICR", color=RED, size=9,
          ha="right")
    dashed(ax, icr, (0.0, -T / 2 - 0.5), color=AXIS)

    for p, col in (((L, T / 2), GREEN), ((L, -T / 2), AMBER)):
        ax.plot([icr[0], p[0]], [icr[1], p[1]], color=col, lw=1.1,
                linestyle=(0, (5, 3)), zorder=2)
        d = d_l if col is GREEN else d_r
        ax.plot([p[0], p[0] + 1.0 * math.cos(d)],
                [p[1], p[1] + 1.0 * math.sin(d)], color=col, lw=1.5, zorder=5)
        dashed(ax, p, (p[0] + 1.05, p[1]), color=AXIS)
        angle_arc(ax, p, 0.62, 0.0, d, color=col, lw=1.0)
    label(ax, (L + 0.95, T / 2 + 0.62), r"$\delta_l$", color=GREEN, size=10)
    label(ax, (L + 1.12, -T / 2 + 0.3), r"$\delta_r$", color=AMBER, size=10)

    # the ICR sits on the rear axle line: that is the whole constraint
    label(ax, (-0.28, R * 0.55), "$R$", color=MUTED, size=10, ha="right")
    dim(ax, (0.0, -T / 2 - 1.0), (L, -T / 2 - 1.0), "$L$", gap=-0.3,
        color=INK_2)
    dim(ax, (L + 0.75, -T / 2), (L + 0.75, T / 2), "$T$", gap=0.28,
        color=INK_2)

    caption(ax, r"both front wheels must be normal to their own radius"
                r" from the ICR")
    save(fig, out, "derivation_ackermann.png")


# --- 4. kinematic bicycle ---------------------------------------------------
def fig_kinematic_bicycle(out):
    fig, ax = plt.subplots(figsize=(5.0, 3.9))
    L, lr = 2.7, 1.5
    delta = math.radians(36.0)
    R = L / math.tan(delta)
    icr = (0.0, R)
    schematic(ax, (-1.6, 4.6), (-1.6, R + 0.9))

    ax.plot([0, L], [0, 0], color=INK_2, lw=1.4, zorder=2)
    wheel(ax, (0.0, 0.0), 0.0, length=0.6, width=0.2)
    wheel(ax, (L, 0.0), delta, length=0.6, width=0.2)
    ax.plot([icr[0]], [icr[1]], "o", color=RED, ms=5, zorder=7)
    label(ax, (icr[0] - 0.14, icr[1] + 0.28), "ICR", color=RED, size=9,
          ha="right")

    dashed(ax, (0.0, 0.0), icr, color=BLUE)
    dashed(ax, (L, 0.0), icr, color=GREEN)
    dashed(ax, (L, 0.0), (L + 1.15, 0.0), color=AXIS)
    ax.plot([L, L + 1.1 * math.cos(delta)], [0, 1.1 * math.sin(delta)],
            color=GREEN, lw=1.5, zorder=5)
    angle_arc(ax, (L, 0.0), 0.68, 0.0, delta, color=GREEN)
    label(ax, (L + 0.92, 0.36), r"$\delta$", color=GREEN)

    cog = (lr, 0.0)
    ax.plot([cog[0]], [cog[1]], "o", color=INK, ms=4.5, zorder=7)
    beta = math.atan2(lr, R)
    vlen = 1.35
    arrow(ax, cog, (cog[0] + vlen * math.cos(beta),
                    cog[1] + vlen * math.sin(beta)), color=ORANGE, lw=2.0)
    label(ax, (cog[0] + vlen * math.cos(beta) + 0.12,
               cog[1] + vlen * math.sin(beta) + 0.18), "$v$", color=ORANGE)
    angle_arc(ax, cog, 0.62, 0.0, beta, color=ORANGE)
    label(ax, (cog[0] + 0.8 * math.cos(beta * 0.5),
               0.8 * math.sin(beta * 0.5) - 0.02), r"$\beta$", color=ORANGE,
          size=9)
    dashed(ax, cog, (cog[0] + 1.5, 0.0), color=AXIS)
    dashed(ax, cog, icr, color=ORANGE)

    dim(ax, (0.0, -0.62), (lr, -0.62), "$l_r$", gap=-0.24, color=INK_2)
    dim(ax, (lr, -0.62), (L, -0.62), "$l_f$", gap=-0.24, color=INK_2)
    label(ax, (-0.85, R * 0.5), "$R$", color=BLUE, size=10)

    caption(ax, r"no side slip at the tires: each wheel rolls along its"
                r" own plane")
    save(fig, out, "derivation_kinematic_bicycle.png")


# --- 5. slip angle ----------------------------------------------------------
def fig_slip_angle(out):
    fig, axes = plt.subplots(1, 2, figsize=(8.2, 3.2))

    ax = axes[0]
    schematic(ax, (-1.5, 3.0), (-1.6, 1.7))
    wheel(ax, (0, 0), 0.0, length=1.5, width=0.45)
    dashed(ax, (-1.3, 0), (2.1, 0), color=AXIS)
    label(ax, (2.15, 0.0), "wheel plane", color=MUTED, size=8, ha="left")
    al = math.radians(26.0)
    arrow(ax, (0, 0), (1.65 * math.cos(-al), 1.65 * math.sin(-al)),
          color=ORANGE, lw=2.0)
    label(ax, (1.72 * math.cos(-al) + 0.14, 1.72 * math.sin(-al) - 0.14),
          "$v_w$", color=ORANGE, ha="left")
    angle_arc(ax, (0, 0), 1.0, -al, 0.0, color=ORANGE)
    label(ax, (1.22 * math.cos(-al * 0.5), 1.22 * math.sin(-al * 0.5) - 0.04),
          r"$\alpha$", color=ORANGE)
    arrow(ax, (0, 0), (0, 1.15), color=GREEN, lw=1.8)
    label(ax, (0.0, 1.34), "$F_y$", color=GREEN)
    ax.set_title(r"(a)  $\alpha$ is measured from the wheel plane to $v_w$",
                 color=INK, fontsize=9, loc="left")

    ax = axes[1]
    L, lr = 2.7, 1.4
    schematic(ax, (-1.2, 3.9), (-1.7, 1.7))
    ax.plot([0, L], [0, 0], color=INK_2, lw=1.4, zorder=2)
    delta = math.radians(18.0)
    wheel(ax, (0, 0), 0.0, length=0.55, width=0.18)
    wheel(ax, (L, 0), delta, length=0.55, width=0.18)
    cog = (lr, 0.0)
    ax.plot([cog[0]], [cog[1]], "o", color=INK, ms=4.5, zorder=7)

    arrow(ax, cog, (cog[0] + 0.8, 0.0), color=BLUE, lw=1.7)
    label(ax, (cog[0] + 0.42, -0.24), "$v_x$", color=BLUE)
    arrow(ax, cog, (cog[0], 0.8), color=PURPLE, lw=1.7)
    label(ax, (cog[0] - 0.24, 0.9), "$v_y$", color=PURPLE)

    arrow(ax, (L, 0.0), (L, 0.72), color=GREEN, lw=1.6)
    label(ax, (L + 0.08, 0.88), r"$+l_f r$", color=GREEN, size=9, ha="left")
    arrow(ax, (0.0, 0.0), (0.0, -0.72), color=AMBER, lw=1.6)
    label(ax, (-0.08, -0.9), r"$-l_r r$", color=AMBER, size=9, ha="right")

    dim(ax, (0.0, -1.25), (lr, -1.25), "$l_r$", gap=-0.22, color=INK_2)
    dim(ax, (lr, -1.25), (L, -1.25), "$l_f$", gap=-0.22, color=INK_2)
    ax.set_title("(b)  rigid body: the yaw rate adds $\\pm l\\,r$",
                 color=INK, fontsize=9, loc="left")
    save(fig, out, "derivation_slip_angle.png")


# --- 6. single track free body diagram --------------------------------------
def fig_single_track_fbd(out):
    fig, ax = plt.subplots(figsize=(5.6, 3.0))
    L, lr = 3.0, 1.6
    delta = math.radians(22.0)
    schematic(ax, (-1.1, 4.9), (-1.5, 1.9))

    body(ax, (-0.5, -0.4), L + 1.0, 0.8, alpha=0.7)
    ax.plot([0, L], [0, 0], color=INK_2, lw=1.3, zorder=2)
    wheel(ax, (0, 0), 0.0, length=0.6, width=0.2)
    wheel(ax, (L, 0), delta, length=0.6, width=0.2)
    cog = (lr, 0.0)
    ax.plot([cog[0]], [cog[1]], "o", color=INK, ms=5, zorder=7)
    label(ax, (cog[0] + 0.02, -0.3), "CoG", color=INK_2, size=8)

    n = delta + math.pi / 2
    arrow(ax, (L, 0), (L + 1.0 * math.cos(n), 1.0 * math.sin(n)), color=GREEN,
          lw=2.0)
    label(ax, (L + 1.14 * math.cos(n) - 0.06, 1.18 * math.sin(n)), "$F_{yf}$",
          color=GREEN)
    dashed(ax, (L, 0), (L + 0.95 * math.cos(delta), 0.95 * math.sin(delta)),
           color=AXIS)
    dashed(ax, (L, 0), (L + 1.0, 0.0), color=AXIS)
    angle_arc(ax, (L, 0), 0.62, 0.0, delta, color=MUTED, lw=1.0)
    label(ax, (L + 0.86, 0.16), r"$\delta$", color=MUTED, size=9)

    arrow(ax, (0, 0), (0, 1.0), color=AMBER, lw=2.0)
    label(ax, (-0.02, 1.18), "$F_{yr}$", color=AMBER)

    arrow(ax, (0, -0.62), (1.0, -0.62), color=BLUE, lw=1.8, z=7)
    label(ax, (1.08, -0.62), "$F_x$", color=BLUE, size=9, ha="left")
    arrow(ax, (cog[0] + 0.55, 0.34), (cog[0] - 0.45, 0.34), color=RED, lw=1.6,
          z=7)
    label(ax, (cog[0] + 0.62, 0.34), r"$F_{\mathrm{res}}$", color=RED, size=9,
          ha="left")

    dim(ax, (0.0, -1.1), (lr, -1.1), "$l_r$", gap=-0.22, color=INK_2)
    dim(ax, (lr, -1.1), (L, -1.1), "$l_f$", gap=-0.22, color=INK_2)
    caption(ax, r"forces resolved in the body frame; moments taken about"
                r" the CoG")
    save(fig, out, "derivation_single_track_fbd.png")


# --- 7. load transfer -------------------------------------------------------
def fig_load_transfer(out):
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.0))

    ax = axes[0]
    L, lr, h = 3.0, 1.6, 1.0
    schematic(ax, (-1.0, 4.3), (-1.3, 2.1))
    ax.plot([-0.6, L + 0.6], [0, 0], color=AXIS, lw=1.2)
    body(ax, (-0.35, 0.28), L + 0.7, 0.85, alpha=0.75)
    for cx in (0.0, L):
        ax.add_patch(plt.Circle((cx, 0.28), 0.28, facecolor="#d8d7d1",
                                edgecolor=INK, lw=1.1, zorder=4))
    cog = (lr, h)
    ax.plot([cog[0]], [cog[1]], "o", color=INK, ms=5, zorder=7)
    dashed(ax, (lr, 0), (lr, h), color=AXIS)
    dim(ax, (lr + 0.42, 0.0), (lr + 0.42, h), "$h$", gap=0.2, color=INK_2)
    dim(ax, (0.0, -0.55), (lr, -0.55), "$l_r$", gap=-0.2, color=INK_2)
    dim(ax, (lr, -0.55), (L, -0.55), "$l_f$", gap=-0.2, color=INK_2)

    arrow(ax, cog, (cog[0] - 1.0, cog[1]), color=RED, lw=1.9)
    label(ax, (cog[0] - 1.08, cog[1] + 0.24), "$m a_x$", color=RED, ha="right")
    arrow(ax, cog, (cog[0], cog[1] - 0.7), color=INK_2, lw=1.6)
    label(ax, (cog[0] - 0.26, cog[1] - 0.48), "$mg$", color=INK_2, size=9)
    arrow(ax, (0.0, 0.0), (0.0, 0.9), color=AMBER, lw=1.8, z=7)
    label(ax, (-0.2, 0.75), "$F_{zr}$", color=AMBER, size=9, ha="right")
    arrow(ax, (L, 0.0), (L, 1.2), color=GREEN, lw=1.8, z=7)
    label(ax, (L + 0.2, 1.05), "$F_{zf}$", color=GREEN, size=9, ha="left")
    ax.set_title(r"(a)  about the rear contact:  $F_{zf}L=mg\,l_r-ma_xh$",
                 color=INK, fontsize=9, loc="left")

    ax = axes[1]
    T, h = 2.4, 1.0
    schematic(ax, (-2.1, 2.5), (-1.3, 2.1))
    ax.plot([-T / 2 - 0.7, T / 2 + 0.7], [0, 0], color=AXIS, lw=1.2)
    body(ax, (-T / 2 - 0.25, 0.28), T + 0.5, 0.85, alpha=0.75)
    for cx in (-T / 2, T / 2):
        ax.add_patch(plt.Circle((cx, 0.28), 0.28, facecolor="#d8d7d1",
                                edgecolor=INK, lw=1.1, zorder=4))
    cog = (0.0, h)
    ax.plot([cog[0]], [cog[1]], "o", color=INK, ms=5, zorder=7)
    dashed(ax, (0, 0), (0, h), color=AXIS)
    dim(ax, (0.4, 0.0), (0.4, h), "$h$", gap=0.2, color=INK_2)
    dim(ax, (-T / 2, -0.55), (T / 2, -0.55), "$T$", gap=-0.2, color=INK_2)
    # the inertial force pushes toward the outer wheel, which gains the load
    arrow(ax, cog, (cog[0] + 1.0, cog[1]), color=RED, lw=1.9)
    label(ax, (cog[0] + 0.5, cog[1] + 0.3), "$m a_y$", color=RED)
    arrow(ax, (-T / 2, 0.0), (-T / 2, 0.7), color=BLUE, lw=1.8, z=7)
    label(ax, (-T / 2 - 0.2, 0.58), "$F_{z,\\mathrm{in}}$", color=BLUE,
          size=9, ha="right")
    arrow(ax, (T / 2, 0.0), (T / 2, 1.3), color=PINK, lw=1.8, z=7)
    label(ax, (T / 2 + 0.2, 1.12), "$F_{z,\\mathrm{out}}$", color=PINK,
          size=9, ha="left")
    ax.set_title(r"(b)  about the inner contact:  $\Delta F_z=ma_yh/T$",
                 color=INK, fontsize=9, loc="left")
    save(fig, out, "derivation_load_transfer.png")


# --- 8. brush model contact patch -------------------------------------------
def fig_brush(out):
    fig, ax = plt.subplots(figsize=(6.0, 3.2))
    a = 1.0
    peak = 1.15          # mu * qz peak
    k = 0.95             # c_p * tan(alpha)
    xb = k * a * a / peak - a          # interior crossing, see the document

    ax.set_xlim(-1.22, 1.35)
    ax.set_ylim(-0.06, 1.55)
    ax.set_facecolor(SURFACE)
    for sp in ("top", "right", "left"):
        ax.spines[sp].set_visible(False)
    ax.spines["bottom"].set_color(AXIS)
    ax.set_xticks([-1, xb, 0, 1])
    ax.set_xticklabels(["$-a$\n(trailing)", "$x_b$", "0", "$+a$\n(leading)"],
                       color=MUTED, fontsize=8)
    ax.set_yticks([])

    x = np.linspace(-a, a, 600)
    bound = peak * (1 - (x / a) ** 2)
    shear = k * (a - x)

    ax.fill_between(x, 0, np.minimum(shear, bound), color=BLUE, alpha=0.10,
                    zorder=0)
    ax.plot(x, bound, color=RED, lw=2.0, zorder=4)
    ax.plot(x, shear, color=BLUE, lw=2.0, zorder=4)
    ax.axvline(xb, color=AXIS, lw=1.0, ls=(0, (4, 3)), zorder=1)
    ax.plot([xb], [k * (a - xb)], "o", color=INK, ms=5, zorder=6)

    label(ax, (-0.86, 0.42), r"$\mu\,q_z(x)$", color=RED, size=10)
    label(ax, (0.70, 0.95), r"$c_p\tan\alpha\,(a-x)$", color=BLUE, size=10)
    label(ax, (-0.62, 0.12), "sliding", color=RED, size=9)
    label(ax, (0.42, 0.12), "adhesion", color=BLUE, size=9)
    ax.set_title("bristle shear against the friction bound; the shaded area"
                 " is $F_y$", color=INK, fontsize=9, loc="left")
    ax.set_xlabel("position in the contact patch", color=INK_2, fontsize=9)
    save(fig, out, "derivation_brush.png")


# --- 9. tire curves (computed) ----------------------------------------------
def fig_tire_curves(out):
    p = make_passenger_car_parameters()
    fz = p.static_load_front() / 2.0
    ca = p.cornering_stiffness_front / 2.0
    mu = p.friction
    alpha = np.linspace(0.0, deg2rad(18.0), 400)

    fig, ax = plt.subplots(figsize=(5.6, 3.2))
    for kind, col in (("Linear", BLUE), ("Fiala", ORANGE), ("Pacejka", GREEN)):
        tire = make_tire(kind, ca, fz, mu)
        fy = [tire.lateral_force(al, fz) for al in alpha]
        ax.plot(np.degrees(alpha), np.array(fy) / 1000.0, color=col, lw=2.0,
                label=kind)
    ax.plot(np.degrees(alpha), ca * alpha / 1000.0, color=MUTED, lw=1.2,
            ls=(0, (5, 3)), label=r"$C_\alpha\,\alpha$")
    ymax = mu * fz / 1000.0
    ax.axhline(ymax, color=RED, lw=1.2, ls=(0, (2, 3)))
    ax.text(0.35, ymax * 1.035, r"$\mu F_z$", color=RED, fontsize=9)
    a_sl = math.degrees(math.atan(3.0 * mu * fz / ca))
    ax.axvline(a_sl, color=ORANGE, lw=1.0, ls=(0, (2, 3)))
    ax.text(a_sl - 0.3, ymax * 0.55, r"$\alpha_{sl}$", color=ORANGE,
            fontsize=9, ha="right")
    ax.set_xlabel(r"slip angle $\alpha$  [deg]", color=INK_2, fontsize=9)
    ax.set_ylabel(r"$F_y$  [kN]", color=INK_2, fontsize=9)
    ax.set_xlim(0, 18)
    ax.set_ylim(0, ymax * 1.22)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.spines["left"].set_color(AXIS)
    ax.spines["bottom"].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    ax.set_title("matched slope at the origin and matched peak", color=INK,
                 fontsize=9, loc="left")
    save(fig, out, "derivation_tire_curves.png")


# --- 10. yaw rate gain (computed) -------------------------------------------
def fig_handling(out):
    car = make_passenger_car_parameters()
    # A rear-soft variant, chosen so the critical speed is well clear of the
    # characteristic speed and both marks stay readable. make_oversteer_car_
    # parameters() is the library preset; its V_cr happens to sit on top of
    # the passenger car's V_ch.
    over = make_passenger_car_parameters()
    over.cornering_stiffness_rear = 61500.0
    neutral = make_passenger_car_parameters()
    neutral.cornering_stiffness_rear = (
        neutral.cornering_stiffness_front * neutral.l_f / neutral.l_r)

    v = np.linspace(0.2, 45.0, 900)
    L = car.wheel_base()
    top = 9.0
    fig, ax = plt.subplots(figsize=(5.6, 3.2))

    for p, col, name in ((car, BLUE, r"understeer  $K>0$"),
                         (neutral, MUTED, r"neutral  $K=0$"),
                         (over, ORANGE, r"oversteer  $K<0$")):
        K = analysis.understeer_gradient(p)
        den = L + K * v ** 2
        gain = np.where(den > 1e-3, v / np.where(den > 1e-3, den, 1.0), np.nan)
        gain = np.where(gain <= top * 1.05, gain, np.nan)
        ax.plot(v, gain, color=col, lw=2.0, label=name)

    vch = analysis.characteristic_speed(car)
    Kc = analysis.understeer_gradient(car)
    ax.plot([vch], [vch / (L + Kc * vch ** 2)], "o", color=BLUE, ms=5, zorder=6)
    ax.axvline(vch, color=BLUE, lw=1.0, ls=(0, (2, 3)))
    ax.text(vch - 0.7, 0.35, r"$V_{ch}$", color=BLUE, fontsize=9, ha="right")
    vcr = analysis.critical_speed(over)
    if vcr and math.isfinite(vcr):
        ax.axvline(vcr, color=ORANGE, lw=1.0, ls=(0, (2, 3)))
        ax.text(vcr - 0.7, 7.9, r"$V_{cr}$", color=ORANGE, fontsize=9,
                ha="right")

    ax.set_xlabel("$v$  [m/s]", color=INK_2, fontsize=9)
    ax.set_ylabel(r"$r/\delta$  [1/s]", color=INK_2, fontsize=9)
    ax.set_xlim(0, 45)
    ax.set_ylim(0, top)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.spines["left"].set_color(AXIS)
    ax.spines["bottom"].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    ax.set_title(r"$r/\delta=v/(L+Kv^2)$, evaluated by the library",
                 color=INK, fontsize=9, loc="left")
    save(fig, out, "derivation_handling.png")


# --- 11. integrator convergence (computed) ----------------------------------
def fig_integrator(out):
    model = UnicycleModel()
    v, omega = 10.0, 0.5
    t_end = (math.pi / 2) / omega
    R = v / omega
    ex = R * math.sin(omega * t_end)
    ey = R * (1 - math.cos(omega * t_end))

    steps = np.array([0.2, 0.1, 0.05, 0.025, 0.0125, 0.00625])
    fig, ax = plt.subplots(figsize=(5.6, 3.2))
    for kind, col, name, order in ((IntegratorType.EULER, BLUE, "Euler", 1),
                                   (IntegratorType.HEUN, ORANGE, "Heun", 2),
                                   (IntegratorType.RK4, GREEN, "RK4", 4)):
        errs = []
        for h in steps:
            n = int(round(t_end / h))
            x = unicycle_state(0.0, 0.0, 0.0)
            u = unicycle_input(v, omega)
            for _ in range(n):
                x = step(model, x, u, t_end / n, kind)
            errs.append(max(math.hypot(x[0] - ex, x[1] - ey), 1e-16))
        ax.loglog(steps, errs, "o-", color=col, lw=1.8, ms=4,
                  label="%s  ($p=%d$)" % (name, order))
    ax.set_xlabel("step $h$  [s]", color=INK_2, fontsize=9)
    ax.set_ylabel("position error  [m]", color=INK_2, fontsize=9)
    ax.grid(color=GRID, lw=0.8, which="both")
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.spines["left"].set_color(AXIS)
    ax.spines["bottom"].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    ax.set_title(r"global error $\propto h^p$ on a quarter circle",
                 color=INK, fontsize=9, loc="left")
    save(fig, out, "derivation_integrator.png")


# --- 12. double track -------------------------------------------------------
def fig_double_track(out):
    fig, ax = plt.subplots(figsize=(5.8, 3.2))
    L, lr, Tf, Tr = 3.0, 1.6, 1.7, 1.7
    dl, dr = math.radians(24.0), math.radians(18.0)
    schematic(ax, (-1.5, 5.3), (-2.0, 2.2))

    body(ax, (-0.5, -Tr / 2 - 0.06), L + 1.0, Tr + 0.12, alpha=0.7)
    ax.plot([0, 0], [-Tr / 2, Tr / 2], color=INK_2, lw=1.2, zorder=2)
    ax.plot([L, L], [-Tf / 2, Tf / 2], color=INK_2, lw=1.2, zorder=2)
    ax.plot([0, L], [0, 0], color=AXIS, lw=1.0, ls=(0, (4, 3)), zorder=1)

    wheel(ax, (L, Tf / 2), dl, length=0.55, width=0.2)
    wheel(ax, (L, -Tf / 2), dr, length=0.55, width=0.2)
    wheel(ax, (0, Tr / 2), 0.0, length=0.55, width=0.2)
    wheel(ax, (0, -Tr / 2), 0.0, length=0.55, width=0.2)
    for xy, name, dy in (((L, Tf / 2), "fl", 0.34), ((L, -Tf / 2), "fr", -0.34),
                         ((0, Tr / 2), "rl", 0.34), ((0, -Tr / 2), "rr", -0.34)):
        label(ax, (xy[0] - 0.5, xy[1] + dy), name, color=MUTED, size=8)

    ax.plot([lr], [0.0], "o", color=INK, ms=5, zorder=7)

    arrow(ax, (0, Tr / 2), (0.5, Tr / 2), color=BLUE, lw=1.8, z=7)
    arrow(ax, (0, -Tr / 2), (1.0, -Tr / 2), color=BLUE, lw=1.8, z=7)
    label(ax, (0.58, Tr / 2 - 0.02), "$F_{x,rl}$", color=BLUE, size=9,
          ha="left")
    label(ax, (1.08, -Tr / 2 - 0.02), "$F_{x,rr}$", color=BLUE, size=9,
          ha="left")

    n_l, n_r = dl + math.pi / 2, dr + math.pi / 2
    arrow(ax, (L, Tf / 2), (L + 0.8 * math.cos(n_l),
                            Tf / 2 + 0.8 * math.sin(n_l)), color=GREEN, lw=1.8)
    arrow(ax, (L, -Tf / 2), (L + 0.8 * math.cos(n_r),
                             -Tf / 2 + 0.8 * math.sin(n_r)), color=GREEN,
          lw=1.8)
    label(ax, (L - 0.62, Tf / 2 + 0.95), "$F_{y,fl}$", color=GREEN, size=9)
    label(ax, (L - 0.85, -Tf / 2 + 0.45), "$F_{y,fr}$", color=GREEN, size=9)

    dim(ax, (0.0, -Tr / 2 - 0.6), (lr, -Tr / 2 - 0.6), "$l_r$", gap=-0.22,
        color=INK_2)
    dim(ax, (lr, -Tr / 2 - 0.6), (L, -Tr / 2 - 0.6), "$l_f$", gap=-0.22,
        color=INK_2)
    dim(ax, (L + 0.85, -Tf / 2), (L + 0.85, Tf / 2), "$T_f$", gap=0.26,
        color=INK_2)

    caption(ax, r"a left/right force difference has a moment arm $T/2$")
    save(fig, out, "derivation_double_track.png")


def main() -> int:
    out = (sys.argv[1] if len(sys.argv) > 1
           else os.path.join(os.path.dirname(os.path.dirname(
               os.path.dirname(os.path.abspath(__file__)))), "docs_en",
               "images"))
    os.makedirs(out, exist_ok=True)
    print("writing to", out)
    for fn in (fig_frames, fig_unicycle, fig_ackermann, fig_kinematic_bicycle,
               fig_slip_angle, fig_single_track_fbd, fig_load_transfer,
               fig_brush, fig_tire_curves, fig_handling, fig_integrator,
               fig_double_track):
        fn(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
