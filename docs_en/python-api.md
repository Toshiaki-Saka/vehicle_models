# Python package reference

日本語版: [`docs_ja/python-api.md`](../docs_ja/python-api.md)

`python/vehicle_models_py` is a set of bindings to the C++ library, plus the
simulation infrastructure the GUI needs. **The equations of motion exist only
in `include/vehicle_models/*.hpp`**; Python reaches them through the `_core`
extension module. Each `.py` below is a re-export shim that adds the numpy
calling convention (states and inputs as 1-D arrays) and a few GUI
conveniences, and contains no physics.

```
python/
  src/core_module.cpp    pybind11 bindings — the one entry point
  vehicle_models_py/
    _core*.pyd/.so       build artefact
    types.py             -> vehicle_models/types.hpp
    parameters.py        -> vehicle_models/vehicle_parameters.hpp
    tires.py             -> vehicle_models/tire/tire_models.hpp
    ackermann.py         -> vehicle_models/ackermann.hpp
    integrator.py        -> vehicle_models/integrator.hpp
    unicycle.py          -> vehicle_models/unicycle.hpp
    kinematic_bicycle.py -> vehicle_models/kinematic_bicycle.hpp
    dynamic_bicycle.py   -> vehicle_models/dynamic_bicycle.hpp
    double_track.py      -> vehicle_models/double_track.hpp
    linear_analysis.py   -> vehicle_models/linear_analysis.hpp
    maneuvers.py    ] simulation infrastructure that exists
    runner.py       ] only on the Python side
    performance.py  ]
    route.py        ]
    gui/                 tkinter + matplotlib front end
  tests/test_port.py     validation against the C++ unit tests
  tools/make_doc_figures.py
  run_gui.py
```

## Installing

The extension module has to be built, so a C++17 compiler and CMake 3.16 or
newer are required. From the repository root:

```bash
pip install .
```

For development an in-place build is handier. `_core` is written straight into
`python/vehicle_models_py/`, so `python run_gui.py` picks it up:

```bash
cmake -S . -B build -DVEHICLE_MODELS_BUILD_PYTHON=ON
cmake --build build --config Release --target _core
```

After changing a C++ header, rebuild `_core`; that is all the GUI needs to pick
the change up.

---

## Conventions

**States and inputs are 1-D NumPy arrays**, not structs. Each module exports the
index constants and a factory function:

```python
from vehicle_models_py import dynamic_state, dynamic_input
from vehicle_models_py.dynamic_bicycle import X, Y, YAW, VX, VY, R

x = dynamic_state(0.0, 0.0, 0.0, 15.0, 0.0, 0.0)   # [x, y, yaw, vx, vy, r]
u = dynamic_input(fx=0.0, steer=0.05)               # [F_x, delta]
print(x[VX], x[R])
```

**Models are dataclasses** carrying their parameters, and satisfy the same
implicit interface the C++ integrators rely on:

```python
derivative(state, input) -> ndarray
normalize_state(state)   -> ndarray     # wraps angles, clamps; modifies in place
```

Anything with those two methods works with `step()` and `simulate()`, including
a model you add yourself.

**Units are SI** (m, s, rad, N, kg) exactly as in C++; `deg2rad` / `rad2deg` are
in `types.py`. $\delta > 0$ is a left turn and gives a positive yaw rate.

---

## Quick start

```python
from vehicle_models_py import (KinematicBicycleModel, ReferencePoint,
                               DynamicBicycleModel, DoubleTrackModel,
                               FialaTire, IntegratorType, deg2rad,
                               dynamic_input, dynamic_state, kinematic_input,
                               kinematic_state, make_passenger_car_parameters,
                               simulate, step)
from vehicle_models_py import linear_analysis as analysis

p = make_passenger_car_parameters()
print(p.validate())                       # [] means the set is usable

# Kinematic bicycle, rear axle reference, 10 ms period
model = KinematicBicycleModel(p, ReferencePoint.REAR_AXLE)
x = kinematic_state(0.0, 0.0, 0.0, 4.0)
x = step(model, x, kinematic_input(0.0, deg2rad(8.0)), 0.01)

# Nonlinear single track with a Fiala tire
dyn = DynamicBicycleModel(p, FialaTire(), FialaTire())
s = dynamic_state(0, 0, 0, 15.0, 0, 0)
s = step(dyn, s, dyn.input_from_acceleration(0.0, deg2rad(3.0)), 0.01)
f = dyn.compute_forces(s, dynamic_input(0.0, deg2rad(3.0)))
print(f.slip_front, f.fy_front, f.fz_front, f.ay)

# Four wheels
dt_model = DoubleTrackModel(p, tire_front=FialaTire(), tire_rear=FialaTire())
forces = dt_model.compute_forces(s, dynamic_input(0.0, deg2rad(3.0)))
print(forces.normal_load, forces.slip_angle, forces.steer.left)

# Closed-form handling analysis
print(analysis.understeer_gradient_deg_per_g(p))
ss = analysis.steady_state_cornering(p, 15.0, deg2rad(2.0))
print(ss.yaw_rate, ss.side_slip, ss.radius)

# Integrate a constant input
x_end = simulate(dyn, s, dynamic_input(0.0, deg2rad(2.0)), 5.0, 0.001,
                 IntegratorType.RK4)
```

---

## Modules

### `types.py`
`PI`, `GRAVITY`, `deg2rad`, `rad2deg`, `normalize_angle`, `clamp_value`,
`signum`, `guard_denominator`, `Pose2D`.

### `parameters.py`
`VehicleParameters` (dataclass with the same fields and defaults as the C++
struct), `wheel_base()`, `static_load_front()`, `static_load_rear()`,
`validate()`, `copy()`. Presets: `make_passenger_car_parameters`,
`make_shuttle_parameters`, `make_buggy_parameters`,
`make_oversteer_car_parameters`, and the `PRESETS` dictionary used by the GUI.

The static axle loads are

$$
F_{zf} = \frac{m g l_r}{L},
\qquad
F_{zr} = \frac{m g l_f}{L}
$$

### `tires.py`
`LinearTire`, `FialaTire`, `PacejkaTire` — each with
`lateral_force(slip_angle, normal_force)` and
`cornering_stiffness_at(normal_force)`. Also
`PacejkaTire.from_cornering_stiffness()`, `friction_ellipse_scale()`, and
`make_tire(kind, cornering_stiffness, nominal_load, friction)`, which builds any
of the three matched to the same stiffness and peak force.

The friction ellipse scaling is

$$
F_y \leftarrow F_y \sqrt{1 - \left(\frac{F_x}{\mu F_z}\right)^2}
$$

### `ackermann.py`
`AckermannGeometry` (`from_params()`), `WheelAngles`, `WheelSpeeds`,
`turn_radius`, `steer_angle_for_radius`, `road_wheel_angles`,
`bicycle_angle_from_wheels`, `handwheel_to_road_wheel`,
`road_wheel_to_handwheel`, `ackermann_error`, `wheel_speeds`,
`wheel_angular_rates`, `minimum_turn_radius`.

$$
R = \frac{L}{\tan\delta},
\qquad
\delta = \arctan\frac{L}{R}
$$

### `integrator.py`
`IntegratorType` (`EULER`, `HEUN`, `RK4`), `step_euler`, `step_heun`,
`step_rk4`, `step`, `simulate`. RK4 is

$$
x_{k+1} = x_k + \frac{h}{6}\left(k_1 + 2k_2 + 2k_3 + k_4\right)
$$

### `unicycle.py`
`UnicycleModel`, `DifferentialDriveModel`, `DifferentialDriveParams`, and the
state/input factories `unicycle_state`, `unicycle_input`, `wheel_rate_input`.

### `kinematic_bicycle.py`
`ReferencePoint` (`REAR_AXLE`, `CENTER_OF_GRAVITY`, `FRONT_AXLE`),
`KinematicBicycleModel` (`side_slip()`, `lateral_acceleration()`),
`KinematicBicycleSteerModel`, and `kinematic_state`, `kinematic_input`,
`steer_dynamics_state`.

$$
\beta = \arctan\frac{l_r \tan\delta}{L},
\qquad
a_y = \frac{v^2 \tan\delta}{L}
$$

### `dynamic_bicycle.py`
`DynamicBicycleModel` (`compute_forces()` → `BicycleForces`,
`input_from_acceleration()`, `sync_tires_from_params()`),
`LinearLateralBicycleModel` (`state_matrix()`, `input_matrix()` return the $A$
and $B$ matrices as NumPy arrays), `BlendedBicycleModel` (`blend_factor()`), plus
`dynamic_state`, `dynamic_input`, `lateral_state`, `steer_input`, `speed_of`,
`side_slip_of`.

### `double_track.py`
`DoubleTrackModel`, `DoubleTrackParams`, `DoubleTrackForces` and the wheel
indices `FL, FR, RL, RR`. Per-wheel quantities are plain lists of four floats
indexed by those constants; `DoubleTrackForces.load_sum()` replaces the C++
`WheelQuantities::sum()`, and the identity

$$
\sum_{i \in \{fl, fr, rl, rr\}} F_{z,i} = m g
$$

is checked by the unit tests.

### `linear_analysis.py`
`understeer_gradient`, `understeer_gradient_deg_per_g`, `characteristic_speed`,
`critical_speed`, `neutral_steer_point`, `static_margin`, `yaw_rate_gain`,
`lateral_acceleration_gain`, `required_steer_angle`, `steady_state_cornering`
→ `SteadyState`, `yaw_mode` → `YawMode`, `max_lateral_acceleration`.

$$
K = \frac{m}{L}\left(\frac{l_r}{C_f} - \frac{l_f}{C_r}\right),
\qquad
\frac{r}{\delta} = \frac{v}{L + K v^2},
\qquad
V_{ch} = \sqrt{\frac{L}{K}},
\qquad
V_{cr} = \sqrt{-\frac{L}{K}}
$$

---

## Simulation infrastructure (Python only)

### `maneuvers.py`
`ManeuverConfig` holds everything about a test run. `Maneuver` wraps it and
answers `command(t, pose, v) -> (steer, ax_override)`; `ax_override = None`
hands the longitudinal channel to the runner's speed controller. Open-loop
profiles cover step, ramp, sine, sine-with-dwell, constant radius, braking in a
turn and straight-line runs; `PurePursuitDriver` follows the slalom and double
lane change reference paths, which `reference_path()` returns for plotting.
`RouteDriver` follows a `Route` (kind `ROUTE`): pure pursuit against the arc
length of the polyline, plus a PI on the route's speed profile with a preview
scan that brakes for a corner before entering it.

### `route.py`
A reference route as a polyline, so a path may turn through any angle instead of
being a `y = f(x)` curve:

```python
from vehicle_models_py.route import (analyse_tracking, load_route,
                                     speed_profile, travel_time)
from vehicle_models_py.maneuvers import ROUTE, ManeuverConfig
from vehicle_models_py.runner import rear_axle_track, run_maneuver

route = load_route()                       # data/reference_route.csv
profile = speed_profile(route, p, ay_ratio=0.35)   # [m/s] per route point
cfg = ManeuverConfig(kind=ROUTE, dt=0.005, route=route, route_speed=profile,
                     duration=travel_time(route, profile) * 1.2 + 3.0,
                     initial_speed=float(profile[0]))

results = run_maneuver(p, cfg, ["kin_cog", "dynamic", "double_track"], "Fiala")
for res in results:
    report = analyse_tracking(route, *rear_axle_track(res, p),
                              time=res.time, profile=profile)
    print(res.label, report.summary["lateral_max"], report.summary["finish_time"])
```

- `load_route(path)` locates columns by header name (`x_m`, `y_m`, `yaw`,
  `curvature` and aliases), so extra columns and a different column order are
  accepted; `yaw` and `curvature` are differentiated from the geometry when the
  file does not carry them. `Route` derives the arc length itself.
- `Route.project(x, y, hint)` returns the nearest point on the route as a
  `Projection` (`index`, `s`, signed `lateral`, `heading`, `curvature`). The
  hint keeps the search local, which is what makes it correct on a route that
  passes close to itself.
- `speed_profile(route, params, ...)` limits the speed at each route point by the
  curvature,

  $\displaystyle v_i \le \min\left(\sqrt{\frac{a_{y,\max}}{\lvert \kappa_i \rvert}},\ v_{\max}\right), \qquad a_{y,\max} = \eta\ \mu g$

  (the ratio $\eta$ is `ay_ratio`, 0.35 by default), then applies a backward pass
  at `accel_min` and a forward pass at `accel_max` so that

  $\displaystyle \left\lvert \frac{v_{i+1}^2 - v_i^2}{2\ \Delta s_i} \right\rvert \le a_{x,\max}$

  holds everywhere — braking therefore starts before the corner, and the profile
  is one the vehicle can actually produce.
- `analyse_tracking(route, x, y, yaw, ...)` returns a `TrackingReport` with the
  per-sample cross-track error, heading error and progress. Feed it the rear
  axle trajectory (`runner.rear_axle_track`), not the CoG, or the body slip
  angle is counted as a tracking error.

### `route_animation.py`
`RouteScene` / `build_route_animation` animate a route run with every model in
one figure — camera following the pack, minimap, and cross-track error, speed
and lateral acceleration channels. `route_overview` is the same run as a still.
`python demo_route.py` is the command-line front end.

### `runner.py`
Adapters that hide the different state layouts and inputs behind one interface,
so the same manoeuvre can be pushed through every model:

```python
from vehicle_models_py.maneuvers import ManeuverConfig, STEP_STEER
from vehicle_models_py.runner import run_maneuver, to_csv
from vehicle_models_py.parameters import make_passenger_car_parameters
from vehicle_models_py.types import deg2rad

p = make_passenger_car_parameters()
cfg = ManeuverConfig(kind=STEP_STEER, duration=8.0, dt=0.002,
                     initial_speed=20.0, steer_amplitude=deg2rad(3.0),
                     t_start=1.0, hold_speed=True)

results = run_maneuver(p, cfg, ["kin_cog", "dynamic", "double_track"],
                       tire_kind="Fiala")
for res in results:
    print(res.label, res.summary["r_final"], res.summary["ay_peak"])
    print(res["r"][-1], res["beta"][-1])      # channels are NumPy arrays

open("run.csv", "w").write(to_csv(results))
```

`MODEL_CATALOG` lists the available model keys (`kin_rear`, `kin_cog`,
`kin_steer`, `linear2dof`, `dynamic`, `blended`, `double_track`);
`STATE_REFERENCE` says which point of the vehicle each one integrates as its
`(x, y)`, and `rear_axle_track(result, params)` converts a run to the rear axle
so two models can be compared without a `l_r` offset polluting the result. Every run
fills the same channel set — `x, y, yaw, vx, vy, v, r, beta, ax, ay, steer_cmd,
steer, alpha_f, alpha_r, curvature, fz_fl…fz_rr, fy_f, fy_r, steer_l, steer_r` —
with `NaN` where a model cannot produce a quantity, so a plot of a missing
channel shows nothing rather than a misleading zero.

### `performance.py`
`acceleration_run`, `braking_run`, `ramp_steer_run` (the handling diagram and
the measured understeer gradient), `gg_diagram`, `max_cornering_speed`,
`steady_state_table`. Each takes a progress callback where it is worth having
one.

The handling diagram plots the relation

$$
\delta - \frac{L}{R} = K a_y + \mathcal{O}(a_y^2)
$$

from a ramp steer, so the slope near the origin is the understeer gradient $K$.

---

## Differences from the C++ API

None of them touch the mathematics. The same functions are called, so the same
inputs give the same double-precision results. Only the calling convention
differs.

| C++ | Python | Why |
|---|---|---|
| `StateVector<Derived, N>` structs with named accessors | 1-D `numpy.ndarray` plus index constants | Makes logging and plotting natural; the boundary copies into the fixed-size struct |
| `DynamicBicycleModel<Tire>` template | `DynamicBicycleModel(params, tire_front, tire_rear)` | Python has no templates. The three instantiations are held in a `std::variant` and picked by tire type at construction |
| `tire.corneringStiffness(fz)` | `tire.cornering_stiffness_at(fz)` | `cornering_stiffness` is already the name of the data field |
| `WheelQuantities` | length-4 `numpy` array, indexed with `FL/FR/RL/RR` | Per-wheel quantities plot directly |
| `enum class IntegratorType` | a Python `Enum` whose values are strings (`"RK4"`, …) | The GUI enumerates it and looks members up by value; converted to the C++ enum at the boundary |
| `syncTiresFromParams()` copies stiffness only | `sync_tires_from_params(sync_friction=False)` | Same default as C++, with an opt-in — see below |
| camelCase | snake_case | PEP 8 |
| three presets | plus `make_oversteer_car_parameters()` | A finite critical speed is the most instructive case in the handling view |

### Two constraints

**Both axles must use the same tire class.** The C++ model is a template over a
single tire type, so front and rear cannot mix tire models; doing so raises
`TypeError`.

**Compare `ReferencePoint` and `IntegratorType` with `==`, not `is`.** pybind11
enums hand back a fresh object on every access, so unlike a Python `Enum` they
do not compare equal by identity.

### The friction propagation gap

`DynamicBicycleModel::syncTiresFromParams()` in C++ copies
`cornering_stiffness_front/rear` into the tires but not `params.friction`, so a
single-track model built from a preset with $\mu = 0.7$ (the buggy) still runs on
tires whose own $\mu$ is $1.0$ — so the saturation bound

$$
\lvert F_y \rvert \le \mu F_z
$$

is evaluated with the wrong $\mu$. `DoubleTrackModel` does propagate it.
The bindings expose the C++ behaviour unchanged by default and add
`sync_tires_from_params(sync_friction=True)`; the GUI and `performance.py` use
the opt-in, so every model on screen shares one road surface. Using the library
directly from C++, set `tire_front.friction` yourself.

---

## Adding your own model

Provide the two methods and it works with the existing integrators, and with the
runner if you also add an adapter. In that case `step()` and `simulate()` fall
back to the Python implementation in `integrator.py`, because the C++ integrator
cannot call into a Python model. To add a model to the library proper, add it to
the C++ headers and expose it through `_core` instead.

```python
import numpy as np
from dataclasses import dataclass
from vehicle_models_py.types import normalize_angle
from vehicle_models_py import simulate

@dataclass
class MyModel:
    gain: float = 1.0
    n_states = 3

    def derivative(self, s, u):
        return np.array([u[0] * np.cos(s[2]), u[0] * np.sin(s[2]),
                         self.gain * u[1]])

    def normalize_state(self, s):
        s[2] = normalize_angle(s[2])
        return s

x = simulate(MyModel(), np.zeros(3), np.array([2.0, 0.3]), 5.0, 0.01)
```

To make it appear in the GUI, subclass `runner.ModelAdapter` (implement `reset`,
`sample`, `advance`, `speed`) and append a `ModelOption` to
`runner.MODEL_CATALOG`, then give it a colour in `gui/theme.py:MODEL_COLORS`.
