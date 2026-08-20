#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Compare the Python port against the C++ ``step_steer`` example.

    python tools/compare_with_cpp.py ../build/step_steer

Runs the compiled example, reproduces exactly the same experiment through the
Python bindings, and reports the largest absolute difference per channel.

Both sides execute the same C++ code, and examples/step_steer.cpp now prints
with std::setprecision(17) -- enough significant digits for every double to
round-trip exactly -- so the CSV no longer quantises what it reports. What
remains is only the agreement of the two call paths themselves, which the
tolerance below is set to measure.

Exit code 0 if every channel agrees within its tolerance.
"""

from __future__ import annotations

import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from vehicle_models_py import (DoubleTrackModel, DynamicBicycleModel,
                               FialaTire, KinematicBicycleModel, LinearTire,
                               ReferencePoint, deg2rad, dynamic_input,
                               dynamic_state, kinematic_input, kinematic_state,
                               make_passenger_car_parameters, rad2deg,
                               side_slip_of, step)

# Same contract as math.isclose: a channel agrees where every sample satisfies
# |a - b| <= max(REL_TOL * max(|a|, |b|), ABS_TOL). The relative term carries
# columns that grow with the manoeuvre; ABS_TOL keeps the bound meaningful
# where a channel passes through zero.
REL_TOL = 1e-9
ABS_TOL = 1e-12


def tolerance_for(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Per-sample tolerance for two columns being compared."""
    return np.maximum(REL_TOL * np.maximum(np.abs(a), np.abs(b)), ABS_TOL)


# examples/step_steer.cpp
VX = 20.0
DELTA = deg2rad(3.0)
DT = 0.002
T_STEP = 0.5
T_END = 5.0


def python_reference() -> np.ndarray:
    """Same experiment as examples/step_steer.cpp, same column order."""
    p = make_passenger_car_parameters()
    kinematic = KinematicBicycleModel(p, ReferencePoint.CENTER_OF_GRAVITY)
    linear_tire = DynamicBicycleModel(p, LinearTire(), LinearTire())
    double_track = DoubleTrackModel(p, tire_front=FialaTire(),
                                    tire_rear=FialaTire())

    x_kin = kinematic_state(0, 0, 0, VX)
    x_dyn = dynamic_state(0, 0, 0, VX, 0, 0)
    x_dtr = dynamic_state(0, 0, 0, VX, 0, 0)

    rows = []
    t = 0.0
    while t <= T_END + 1e-9:
        steer = DELTA if t >= T_STEP else 0.0
        u_kin = kinematic_input(0.0, steer)
        u_dyn = dynamic_input(0.0, steer)

        r_kin = x_kin[3] * math.tan(steer) / p.wheel_base()
        ay_dyn = linear_tire.measured_lateral_acceleration(x_dyn, u_dyn)
        ay_dtr = double_track.compute_forces(x_dtr, u_dyn).ay

        rows.append([t, rad2deg(steer), r_kin, x_dyn[5], x_dtr[5],
                     rad2deg(side_slip_of(x_dyn)), ay_dyn, ay_dtr])

        x_kin = step(kinematic, x_kin, u_kin, DT)
        x_dyn = step(linear_tire, x_dyn, u_dyn, DT)
        x_dtr = step(double_track, x_dtr, u_dyn, DT)
        t += DT
    return np.asarray(rows)


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    binary = sys.argv[1]
    if not os.path.exists(binary):
        print("not found: %s\nBuild it first:\n"
              "  cmake -S . -B build -DVEHICLE_MODELS_BUILD_EXAMPLES=ON\n"
              "  cmake --build build -j" % binary, file=sys.stderr)
        return 2

    out = subprocess.run([binary], capture_output=True, text=True, check=True)
    lines = [line for line in out.stdout.splitlines() if line.strip()]
    header = lines[0].split(",")
    cpp = np.array([[float(v) for v in line.split(",")] for line in lines[1:]])
    py = python_reference()

    n = min(len(cpp), len(py))
    if len(cpp) != len(py):
        print("row count differs: C++ %d, Python %d - comparing the first %d"
              % (len(cpp), len(py), n))
    cpp, py = cpp[:n], py[:n]

    # The tolerance varies per sample, so the sample that decides the verdict
    # is the one closest to its own bound -- not, in general, the one with the
    # largest absolute difference. Both reported numbers come from that sample.
    print("(at the sample closest to its own bound)")
    print("%-14s %14s %14s %10s" % ("channel", "|diff|", "tolerance",
                                    "verdict"))
    worst_ok = True
    for i, name in enumerate(header):
        diff = np.abs(cpp[:, i] - py[:, i])
        tol = tolerance_for(cpp[:, i], py[:, i])
        ok = bool(np.all(diff <= tol))
        worst_ok = worst_ok and ok
        binding = int(np.argmax(diff / tol))
        print("%-14s %14.3e %14.3e %10s"
              % (name, diff[binding], tol[binding], "ok" if ok else "MISMATCH"))

    print("\n%s over %d samples (rel_tol=%g, abs_tol=%g)"
          % ("all channels agree" if worst_ok else "DIFFERENCES FOUND", n,
             REL_TOL, ABS_TOL))
    return 0 if worst_ok else 1


if __name__ == "__main__":
    sys.exit(main())
