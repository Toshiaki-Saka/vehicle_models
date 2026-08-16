#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Animated demo of the vehicle models -- no GUI, no C++ build, no tkinter.

    python demo_animation.py                       # play it on screen
    python demo_animation.py --save demo.gif       # write an animated GIF
    python demo_animation.py --maneuver slalom --model dynamic --compare kin_cog

Runs one manoeuvre through the model in focus (plus any number of comparison
models drawn as dashed ghosts) and animates the top view: body outline, the
four wheels at their real Ackermann angles, discs scaling with wheel load, the
velocity vector, and the yaw rate / lateral acceleration / body slip channels
with a cursor at the current time.

The scene is the one the GUI animation tab shows -- both call
``vehicle_models_py.animation.VehicleScene``.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vehicle_models_py import PRESETS, TIRE_TYPES, deg2rad, rad2deg  # noqa: E402
from vehicle_models_py.animation import (build_animation, gif_fps,  # noqa: E402
                                         save_animation)
from vehicle_models_py.gui import theme  # noqa: E402
from vehicle_models_py.maneuvers import (BRAKE_IN_TURN, CONSTANT_RADIUS,  # noqa: E402
                                         LANE_CHANGE, SINE_DWELL, SLALOM,
                                         STEP_STEER, ManeuverConfig)
from vehicle_models_py.runner import MODEL_BY_KEY, run_maneuver  # noqa: E402
from vehicle_models_py.types import GRAVITY  # noqa: E402

# Each entry is a manoeuvre kind plus the settings that make it worth watching:
# enough lateral acceleration for the models to part company, short enough to
# fit in a few seconds of animation.
MANEUVERS = {
    "lane_change": (LANE_CHANGE, dict(
        duration=8.0, initial_speed=15.0, lane_offset=3.5,
        section_length=30.0), 28.0),
    "slalom": (SLALOM, dict(
        duration=12.0, initial_speed=13.0, lane_offset=2.5,
        section_length=25.0), 30.0),
    "step": (STEP_STEER, dict(
        duration=6.0, initial_speed=20.0, steer_amplitude=deg2rad(4.0),
        t_start=1.0), 30.0),
    "sine_dwell": (SINE_DWELL, dict(
        duration=8.0, initial_speed=18.0, steer_amplitude=deg2rad(6.0),
        frequency=0.7, t_start=1.0), 30.0),
    "constant_radius": (CONSTANT_RADIUS, dict(
        duration=12.0, initial_speed=15.0, radius=40.0), 46.0),
    "brake_in_turn": (BRAKE_IN_TURN, dict(
        duration=8.0, initial_speed=20.0, steer_amplitude=deg2rad(4.0),
        brake_accel=-6.0, brake_start=3.0), 32.0),
}

PRESET_KEYS = {
    "passenger": "Passenger car",
    "shuttle": "Low-speed shuttle",
    "buggy": "Off-road buggy",
    "oversteer": "Oversteering car",
}


def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--maneuver", choices=sorted(MANEUVERS),
                   default="lane_change", help="test manoeuvre to run")
    p.add_argument("--model", choices=sorted(MODEL_BY_KEY),
                   default="double_track", help="model in focus")
    p.add_argument("--compare", default="kin_cog",
                   help="comma separated models drawn as ghosts "
                        "(empty string for none)")
    p.add_argument("--preset", choices=sorted(PRESET_KEYS), default="passenger",
                   help="vehicle preset")
    p.add_argument("--tire", choices=list(TIRE_TYPES), default="Fiala",
                   help="tire model for the dynamic models")
    p.add_argument("--speed", type=float, default=None,
                   help="initial speed [m/s], overrides the manoeuvre default")
    p.add_argument("--duration", type=float, default=None,
                   help="run length [s], overrides the manoeuvre default")
    p.add_argument("--dt", type=float, default=0.005, help="time step [s]")
    p.add_argument("--span", type=float, default=None,
                   help="width of the camera window [m]")
    p.add_argument("--fps", type=int, default=20, help="animation frame rate")
    p.add_argument("--rate", type=float, default=1.0,
                   help="playback speed as a multiple of real time")
    p.add_argument("--dpi", type=int, default=100, help="figure resolution")
    p.add_argument("--no-channels", action="store_true",
                   help="top view only, without the three channel plots")
    p.add_argument("--save", metavar="PATH", default=None,
                   help="write to PATH (.gif needs Pillow, .mp4 needs ffmpeg) "
                        "instead of playing on screen")
    p.add_argument("--colors", type=int, default=128,
                   help="GIF palette size; 0 keeps the writer's own palette")
    return p.parse_args(argv)


def build_config(args) -> ManeuverConfig:
    kind, defaults, _span = MANEUVERS[args.maneuver]
    cfg = ManeuverConfig(kind=kind, dt=args.dt, **defaults)
    if args.speed is not None:
        cfg.initial_speed = args.speed
    if args.duration is not None:
        cfg.duration = args.duration
    return cfg


def report(results, cfg) -> None:
    """Say in numbers what the animation says in pictures."""
    print("%-28s %10s %10s %10s" % ("model", "r_peak", "ay_peak", "beta_peak"))
    for res in results:
        s = res.summary
        print("%-28s %8.2f d/s %8.3f g %8.2f deg"
              % (res.label, rad2deg(s.get("r_peak", 0.0)),
                 s.get("ay_peak", 0.0) / GRAVITY,
                 rad2deg(s.get("beta_peak", 0.0))))
    print("\n%s, %.0f s at %.1f m/s, dt = %g s"
          % (cfg.kind, cfg.duration, cfg.initial_speed, cfg.dt))


def main(argv=None) -> int:
    args = parse_args(argv or sys.argv[1:])
    theme.apply_matplotlib_style()

    params = PRESETS[PRESET_KEYS[args.preset]]()
    problems = params.validate()
    if problems:
        for message in problems:
            print("parameter error: %s" % message, file=sys.stderr)
        return 1

    cfg = build_config(args)
    ghost_keys = [k for k in args.compare.split(",") if k.strip()]
    unknown = [k for k in ghost_keys if k not in MODEL_BY_KEY]
    if unknown:
        print("unknown model(s): %s" % ", ".join(unknown), file=sys.stderr)
        return 1
    ghost_keys = [k for k in ghost_keys if k != args.model]

    print("simulating %s ..." % cfg.kind)
    results = run_maneuver(params, cfg, [args.model] + ghost_keys, args.tire)
    focus = results[0]
    ghosts = results[1:]
    report(results, cfg)

    _kind, _defaults, default_span = MANEUVERS[args.maneuver]
    span = args.span if args.span is not None else default_span

    fps = float(args.fps)
    if args.save is not None and args.save.lower().endswith(".gif"):
        fps = gif_fps(fps)
        if abs(fps - args.fps) > 0.05:
            print("GIF frame delays are quantised to 10 ms: writing %.2f fps "
                  "instead of %d, so playback stays real time"
                  % (fps, args.fps))

    figure = None
    if args.save is None:
        # On-screen playback needs a figure the interactive backend manages.
        import matplotlib.pyplot as plt
        figure = plt.figure(figsize=(9.5, 5.4) if not args.no_channels
                            else (6.4, 5.4), dpi=args.dpi)

    figure, anim = build_animation(
        params, focus, cfg, ghosts=ghosts, fps=fps, rate=args.rate,
        span=span, show_channels=not args.no_channels, dpi=args.dpi,
        figure=figure)

    if args.save is None:
        import matplotlib.pyplot as plt
        print("close the window to quit")
        plt.show()
        return 0

    directory = os.path.dirname(os.path.abspath(args.save))
    os.makedirs(directory, exist_ok=True)
    print("writing %s ..." % args.save)
    save_animation(anim, args.save, fps=fps, colors=args.colors)
    print("wrote %s (%.1f MB)"
          % (args.save, os.path.getsize(args.save) / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
