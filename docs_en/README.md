# vehicle_models (English documentation)

日本語版: [`docs_ja/README.md`](../docs_ja/README.md)

English edition of the project documentation. Every document in this folder has a
Japanese counterpart with the same file name under
[`docs_ja/`](../docs_ja).

| Document | Contents |
|---|---|
| [models.md](models.md) | Equations of motion of every model, and the closed-form handling results |
| [python-gui.md](python-gui.md) | The Python simulation GUI: install, tabs, worked examples |
| [python-api.md](python-api.md) | Python package reference and the differences from the C++ API |
| [validation.md](validation.md) | How the Python port is verified against the C++ library |

Formulas are written in LaTeX and render on GitHub, in VS Code and in any
Markdown viewer with math support.

---

## What this is

A header-only C++17 library of vehicle motion models: bicycle models (kinematic
and dynamic), Ackermann geometry, a four-wheel double-track model, tire models
and linear handling analysis — with no external dependencies.

- **No dependencies** (standard library only). Eigen is not required
- **Header only**. Usable through `add_subdirectory`, `FetchContent` or
  `find_package`
- Builds on Windows (MSVC), Linux (GCC, Clang) and macOS, in C++17 and C++20
- Warning-free under `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`
- Unit tests check the models against analytic solutions (steady state
  cornering, convergence order, load transfer sums)

Alongside the C++ library there is a **Python port with a simulation GUI** under
[`python/`](../python), which drives every model from one shared parameter set
and shows the differences between them; see [python-gui.md](python-gui.md).

---

## Models

| Model | States | State vector | Purpose and validity |
|---|---|---|---|
| `UnicycleModel` | 3 | $x, y, \psi$ | Skid steer, planner-side abstraction. Can rotate in place (wrong for an Ackermann vehicle) |
| `DifferentialDriveModel` | 3 | $x, y, \psi$ | Wheel angular rates as input; `toBodyVelocity` / `toWheelRates` convert both ways |
| `KinematicBicycleModel` | 4 | $x, y, \psi, v$ | Reference point selectable: rear axle, CoG or front axle. Valid up to roughly $0.4\,g$ |
| `KinematicBicycleSteerModel` | 5 | $+\ \delta$ | Adds a first order steering actuator with a rate limit. For validating controllers against real hardware |
| `LinearLateralBicycleModel` | 5 | $x, y, \psi, v_y, r$ | Linear 2-DOF at constant $v_x$. $A$ and $B$ available directly — the design plant for LQR/MPC |
| `DynamicBicycleModel<Tire>` | 6 | $x, y, \psi, v_x, v_y, r$ | Nonlinear single track; the tire model is a template parameter |
| `BlendedBicycleModel<Tire>` | 6 | same | Blends to kinematic at low speed. Removes the low-speed singularity for shuttle / valet ranges |
| `DoubleTrackModel<Tire>` | 6 | same | Four wheels: longitudinal and lateral load transfer, per-wheel Ackermann angles, combined slip |

Tire models: `LinearTire` (linear + saturation), `FialaTire` (brush, cubic
build-up), `PacejkaTire` (Magic Formula, pure lateral force), plus the friction
ellipse `frictionEllipseScale`.

Geometry and analysis utilities:

- `ackermann.hpp` — inner/outer wheel angles, turn radius, per-wheel speeds,
  Ackermann error, handwheel conversion, minimum turn radius
- `linear_analysis.hpp` — understeer gradient $K$, characteristic speed, critical
  speed, neutral steer point, static margin, yaw rate gain, steady state
  cornering, eigenvalues and damping ratio of the yaw mode

---

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Windows (Visual Studio):

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Options: `VEHICLE_MODELS_BUILD_TESTS`, `VEHICLE_MODELS_BUILD_EXAMPLES`,
`VEHICLE_MODELS_INSTALL`, `VEHICLE_MODELS_WERROR`. They default to OFF when the
project is consumed as a subproject.

### Using it from another project

```cmake
# 1) as a subdirectory
add_subdirectory(third_party/vehicle_models)

# 2) as an installed package
find_package(vehicle_models 0.1 REQUIRED)

target_link_libraries(my_app PRIVATE vehicle_models::vehicle_models)
```

Because it only depends on the standard library, `include/` can also be vendored
straight into a ROS 2 package.

---

## Usage

```cpp
#include "vehicle_models/vehicle_models.hpp"
using namespace vehicle_models;

VehicleParameters p = makeShuttleParameters();   // low-speed shuttle preset
for (const auto& e : p.validate()) std::cerr << e << "\n";  // sanity check

// Kinematic bicycle (rear axle reference) at a 10 ms period
KinematicBicycleModel model(p, ReferencePoint::RearAxle);
auto x = KinematicBicycleState::make(0.0, 0.0, 0.0, 4.0);   // x, y, yaw, v
const auto u = KinematicBicycleInput::make(0.0, deg2rad(8.0));  // accel, steer
x = step(model, x, u, 0.01);                     // RK4 by default

// Nonlinear single track with a Fiala tire
DynamicBicycleModel<tire::FialaTire> dyn(p);
auto s = DynamicBicycleState::make(0, 0, 0, 15.0, 0, 0);
s = step(dyn, s, dyn.inputFromAcceleration(0.0, deg2rad(3.0)), 0.01);
const auto f = dyn.computeForces(s, DynamicBicycleInput::make(0.0, deg2rad(3.0)));
// f.slip_front, f.fy_front, f.fz_front, f.ay ... intermediates for logging and monitors

// Ackermann geometry
const auto g = AckermannGeometry::from(p);
const auto wheels = roadWheelAngles(g, deg2rad(20.0));   // inner / outer angles
const auto speeds = wheelSpeeds(g, 5.0, 0.3);            // four wheel speeds

// Linear handling analysis (closed form)
const double K = analysis::understeerGradientDegPerG(p); // [deg/g]
const auto ss = analysis::steadyStateCornering(p, 15.0, deg2rad(2.0));
```

Integrators are `stepEuler` / `stepHeun` / `stepRK4`, or
`step(..., IntegratorType::RK4)`. `simulate(model, x0, u, duration, dt)`
integrates a constant input in one call. A model only has to provide three
things — the `State` / `Input` types, `derivative()` and `normalizeState()` — so
your own model works with the same integrators.

---

## Examples

```bash
./build/handling_report   # steering geometry table and handling figures for the three presets
./build/step_steer > step_steer.csv   # step steer through kinematic, single-track and double-track at once
```

Output of `handling_report` for the passenger car preset:

```
understeer gradient : +0.0034 rad/(m/s^2)  (+1.917 deg/g)
static margin       : +0.1056
characteristic speed: 28.13 m/s (101.3 km/h)
```

`step_steer` shows that at 20 m/s with a 3 deg step, the kinematic model
over-predicts the yaw rate by about 1.5x (0.388 vs 0.248 rad/s). The boundary
where model choice starts to matter is visible directly in the numbers.

---

## Design notes

- **Sign convention**: $\delta > 0$ is a left turn (positive yaw rate). For
  tires, $\alpha > 0$ gives $F_y > 0$. Slip angles are

  $$
  \alpha_f = \delta - \arctan\frac{v_y + l_f r}{v_x},
    \qquad
    \alpha_r = -\arctan\frac{v_y - l_r r}{v_x}
  $$

- **Low-speed singularity**: the $1/v_x$ of the dynamic models is floored by
  `guardDenominator`, so the derivative stays finite even at standstill. The
  lateral behaviour below `low_speed_guard` is nevertheless not trustworthy —
  use `BlendedBicycleModel` there.
- **Load transfer**: the double-track model uses a one-pass predictor (estimate
  $a_x$, $a_y$ on the static loads → recompute the loads → recompute the
  forces). It does not iterate, so the execution time is deterministic.
- **Units**: SI (m, s, rad, N, kg). Angles are always in radians;
  `deg2rad` / `rad2deg` are provided.

The full equations are in [models.md](models.md).

---

## License

Apache License 2.0
