#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Drive the reference route in ``data/`` with every vehicle model at once.

    python demo_route.py                              # play it on screen
    python demo_route.py --save route.mp4             # write a movie
    python demo_route.py --save route.gif --rate 8    # write an animated GIF
    python demo_route.py --models all --overview route.png
    python demo_route.py --route my_course.csv --preset shuttle

One driver model -- pure pursuit on the route plus a curvature-limited speed
profile -- is given to every selected vehicle model, so what the animation
shows is the difference between the models and nothing else. The camera
follows the pack, the minimap says where on the route it is, and the three
channels are cross-track error, speed against the profile, and lateral
acceleration against the friction limit.

``--models linear2dof`` is worth watching once: the linear 2-DOF model holds
its longitudinal speed by construction, so it cannot brake for a corner and
leaves the road at the first tight one. That is the model being used outside
its assumptions, not a bug.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vehicle_models_py import TIRE_TYPES, rad2deg  # noqa: E402
from vehicle_models_py.animation import gif_fps, save_animation  # noqa: E402
from vehicle_models_py.gui import theme  # noqa: E402
from vehicle_models_py.maneuvers import ROUTE, ManeuverConfig  # noqa: E402
from vehicle_models_py.parameters import PRESETS  # noqa: E402
from vehicle_models_py.route import (DEFAULT_ROUTE_PATH,  # noqa: E402
                                     analyse_tracking, load_route,
                                     speed_profile, travel_time)
from vehicle_models_py.route_animation import (DEFAULT_SPAN,  # noqa: E402
                                               build_route_animation,
                                               finish_index, route_overview)
from vehicle_models_py.runner import (MODEL_BY_KEY, MODEL_CATALOG,  # noqa: E402
                                      rear_axle_track, run_maneuver)
from vehicle_models_py.types import GRAVITY  # noqa: E402

PRESET_KEYS = {
    "passenger": "Passenger car",
    "shuttle": "Low-speed shuttle",
    "buggy": "Off-road buggy",
    "oversteer": "Oversteering car",
}

DEFAULT_MODELS = "kin_cog,dynamic,double_track"


def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--route", default=DEFAULT_ROUTE_PATH,
                   help="route CSV (default: data/reference_route.csv)")
    p.add_argument("--models", default=DEFAULT_MODELS,
                   help="comma separated model keys, or 'all'. Available: %s"
                        % ", ".join(MODEL_BY_KEY))
    p.add_argument("--preset", choices=sorted(PRESET_KEYS), default="passenger",
                   help="vehicle preset")
    p.add_argument("--tire", choices=list(TIRE_TYPES), default="Fiala",
                   help="tire model for the dynamic models")
    p.add_argument("--ay-limit", type=float, default=0.35, metavar="RATIO",
                   help="lateral acceleration the speed profile plans for, "
                        "as a fraction of mu*g (default 0.35)")
    p.add_argument("--speed-limit", type=float, default=None, metavar="MPS",
                   help="cap on the profile [m/s] (default: params.speed_max)")
    p.add_argument("--dt", type=float, default=0.005, help="time step [s]")
    p.add_argument("--duration", type=float, default=None,
                   help="run length [s] (default: from the speed profile)")
    p.add_argument("--lookahead-base", type=float, default=4.0,
                   help="pure pursuit lookahead at standstill [m]")
    p.add_argument("--lookahead-gain", type=float, default=0.5,
                   help="pure pursuit lookahead per m/s [s]")
    p.add_argument("--span", type=float, default=DEFAULT_SPAN,
                   help="width of the camera window [m]")
    p.add_argument("--fps", type=int, default=20, help="animation frame rate")
    p.add_argument("--rate", type=float, default=4.0,
                   help="playback speed as a multiple of real time")
    p.add_argument("--dpi", type=int, default=100, help="figure resolution")
    p.add_argument("--save", metavar="PATH", default=None,
                   help="write the animation to PATH (.gif needs Pillow, "
                        ".mp4 needs ffmpeg) instead of playing on screen")
    p.add_argument("--overview", metavar="PATH", default=None,
                   help="also write a static overview figure to PATH")
    p.add_argument("--colors", type=int, default=128,
                   help="GIF palette size; 0 keeps the writer's own palette")
    p.add_argument("--full-tail", action="store_true",
                   help="keep animating after the last model reached the goal")
    return p.parse_args(argv)


def resolve_models(spec: str):
    if spec.strip().lower() == "all":
        return [option.key for option in MODEL_CATALOG]
    keys = [k.strip() for k in spec.split(",") if k.strip()]
    unknown = [k for k in keys if k not in MODEL_BY_KEY]
    if unknown:
        raise SystemExit("unknown model(s): %s\navailable: %s"
                         % (", ".join(unknown), ", ".join(MODEL_BY_KEY)))
    return keys


def report(results, reports, route, profile) -> None:
    """Say in numbers what the animation says in pictures."""
    print("\n%-30s %9s %9s %9s %9s %9s"
          % ("model", "max |e|", "rms e", "peak a_y", "max |b|", "finish"))
    for res, rep in zip(results, reports):
        s = rep.summary
        finish = ("%7.1f s" % s["finish_time"] if "finish_time" in s
                  else "%6.0f %%" % (100.0 * s["progress"]))
        print("%-30s %7.2f m %7.2f m %7.3f g %7.2f d %s"
              % (res.label, s["lateral_max"], s["lateral_rms"],
                 res.summary.get("ay_peak", 0.0) / GRAVITY,
                 rad2deg(res.summary.get("beta_peak", 0.0)), finish))
    print("\nroute %s: %.0f m, %d points, tightest radius %.0f m"
          % (route.name, route.length, len(route),
             1.0 / max(abs(route.curvature).max(), 1e-9)))
    print("speed profile %.1f - %.1f m/s, ideal drive time %.1f s"
          % (profile.min(), profile.max(), travel_time(route, profile)))


def main(argv=None) -> int:
    args = parse_args(argv or sys.argv[1:])
    theme.apply_matplotlib_style()

    params = PRESETS[PRESET_KEYS[args.preset]]()
    problems = params.validate()
    if problems:
        for message in problems:
            print("parameter error: %s" % message, file=sys.stderr)
        return 1

    keys = resolve_models(args.models)
    route = load_route(args.route)
    profile = speed_profile(route, params, ay_ratio=args.ay_limit,
                            speed_max=args.speed_limit)
    duration = (args.duration if args.duration is not None
                else travel_time(route, profile) * 1.2 + 3.0)

    cfg = ManeuverConfig(kind=ROUTE, duration=duration, dt=args.dt,
                         initial_speed=float(profile[0]), route=route,
                         route_speed=profile, route_ay_ratio=args.ay_limit,
                         lookahead_base=args.lookahead_base,
                         lookahead_gain=args.lookahead_gain)

    print("driving %s (%.0f m) for %.0f s with %d model(s) ..."
          % (os.path.basename(args.route), route.length, duration, len(keys)))
    results = run_maneuver(params, cfg, keys, args.tire)
    reports = []
    for res in results:
        x, y, yaw = rear_axle_track(res, params)
        reports.append(analyse_tracking(route, x, y, yaw, time=res.time,
                                        profile=profile))
    report(results, reports, route, profile)

    title = "%s on %s" % (PRESET_KEYS[args.preset], route.name)
    if args.overview:
        from matplotlib.figure import Figure
        figure = Figure(figsize=(11.5, 6.6), dpi=max(args.dpi, 120))
        route_overview(figure, params, route, results, reports, profile, title)
        _write_figure(figure, args.overview)

    fps = float(args.fps)
    if args.save is not None and args.save.lower().endswith(".gif"):
        fps = gif_fps(fps)
        if abs(fps - args.fps) > 0.05:
            print("GIF frame delays are quantised to 10 ms: writing %.2f fps "
                  "instead of %d, so playback stays real time"
                  % (fps, args.fps))

    stop = None if args.full_tail else finish_index(reports, route)
    if stop is not None:  # a second of everyone parked on the goal
        stop = min(stop + int(round(1.0 / cfg.dt)), results[0].time.size - 1)

    figure = None
    if args.save is None:
        import matplotlib.pyplot as plt
        figure = plt.figure(figsize=(11.0, 6.2), dpi=args.dpi)

    figure, anim = build_route_animation(
        params, route, results, reports, profile=profile, dt=cfg.dt, fps=fps,
        rate=args.rate, span=args.span, stop_index=stop, title=title,
        dpi=args.dpi, figure=figure)

    if args.save is None:
        import matplotlib.pyplot as plt
        print("\nclose the window to quit")
        plt.show()
        return 0

    _ensure_directory(args.save)
    print("\nwriting %s ..." % args.save)
    save_animation(anim, args.save, fps=fps, colors=args.colors)
    print("wrote %s (%.1f MB)"
          % (args.save, os.path.getsize(args.save) / 1e6))
    return 0


def _ensure_directory(path: str) -> None:
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)


def _write_figure(figure, path: str) -> None:
    _ensure_directory(path)
    figure.savefig(path, facecolor=figure.get_facecolor())
    print("wrote %s" % path)


if __name__ == "__main__":
    sys.exit(main())
