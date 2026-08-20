# Validating the Python bindings

日本語版: [`docs_ja/validation.md`](../docs_ja/validation.md)

The Python side does not reimplement the models: it calls
`include/vehicle_models/*.hpp` through the `_core` pybind11 module. So the
question is no longer whether two implementations agree. What has to be
verified is that **the bindings do not misplace a state element, an argument
order or a unit**, and the three checks below cover exactly that.

---

## 1. The C++ unit tests, re-run in Python

[`python/tests/test_port.py`](../python/tests/test_port.py) mirrors
`test/test_kinematics.cpp`, `test/test_dynamics.cpp` and
`test/test_integrator.cpp` one to one — the same vehicles, the same manoeuvres,
the same tolerances.

```bash
cd python
python tests/test_port.py
```

```
checks run : 141
all checks passed
```

What it covers:

| Section | Checks against |
|---|---|
| unicycle | closed circle after $2\pi/\omega$, straight-line distance, turn in place |
| differential drive | wheel-rate ↔ body-velocity round trip, pure rotation, travelled distance |
| ackermann geometry | the ideal condition $\cot\delta_o - \cot\delta_i = T/L$, mirror symmetry, radius and handwheel round trips, minimum turn radius |
| wheel speeds | rear average equals the body speed, outer wheels faster, angular rate conversion |
| kinematic bicycle | yaw rate $v\tan\delta/L$, closed circular path, equivalence of the three reference points, acceleration and limit clamping |
| steering actuator | rate limit right after the step, convergence to the command, mechanical limit |
| tire models | linear slope and saturation, Fiala slope/monotonicity/sliding value, Pacejka $BCD$ and peak, friction ellipse at 0 / 0.6 / 1.0 |
| linear handling analysis | gain at the characteristic speed is half the neutral value, $\delta = L/R + K a_y$, stability of an understeering vehicle at every speed, instability above the critical speed of an oversteering one |
| linear model vs closed form | 20 s simulation lands on `steadyStateCornering()` within $10^{-6}$ |
| dynamic bicycle | steady state within 2 %, straight running is an equilibrium, commanded acceleration, load transfer sums to $mg$, finite derivative at standstill, $\lvert F_y \rvert \le \mu F_z$ |
| blending | blend factor at the ends and in the middle, crawl speed follows the kinematic prediction, identical to the plain dynamic model above the blend speed ($10^{-12}$) |
| double track | static load split, load transfer under braking and cornering, loads always sum to $mg$, inner wheel steers more, agreement with the single-track model below the limit (5 %), bounded $a_y$, combined slip reduces the lateral force |
| parameter validation | the three presets are valid; a deliberately broken set reports exactly three violations |
| integrator order | error ratios of 2, 4 and 16 for Euler, Heun and RK4 on a quarter circle |
| runner / performance | the dynamic model reaches the closed-form steady state, the kinematic model over-predicts it, the speed controller holds the operating point, braking distance and limit lateral acceleration are physically plausible |

The convergence check is the ratio

$$
\frac{\lVert e(h) \rVert}{\lVert e(h/2) \rVert} \approx 2^{p},
\qquad p = 1,\ 2,\ 4
$$

for Euler, Heun and RK4 respectively. It doubles as evidence that the
integration itself runs in C++: the orders come out because `_core`
instantiates the `integrator.hpp` templates on a type-erased model. There is no
copy of the integrator on the Python side.

---

## 2. The documented C++ outputs, reproduced

The figures published in the C++ `README.md` come back unchanged when called
from Python:

| Quantity | C++ README | Python |
|---|---|---|
| Understeer gradient (passenger car) | `+0.0034 rad/(m/s²)` / `+1.917 deg/g` | `+0.0034` / `+1.917 deg/g` |
| Static margin | `+0.1056` | `+0.1056` |
| Characteristic speed | `28.13 m/s (101.3 km/h)` | `28.13 m/s (101.3 km/h)` |
| Step steer at 20 m/s, 3 deg — kinematic yaw rate | `0.388 rad/s` | `0.388206 rad/s` |
| Step steer at 20 m/s, 3 deg — single track yaw rate | `0.248 rad/s` | `0.247893 rad/s` |

Reproduce them with:

```bash
cd python
python -c "
from vehicle_models_py import make_passenger_car_parameters as car
from vehicle_models_py import linear_analysis as a
p = car()
print('%+.3f deg/g' % a.understeer_gradient_deg_per_g(p))
print('%+.4f' % a.static_margin(p))
print('%.2f m/s' % a.characteristic_speed(p))
"
```

---

## 3. Direct comparison with the compiled example

[`python/tools/compare_with_cpp.py`](../python/tools/compare_with_cpp.py) runs
the compiled `step_steer` example, reproduces the identical experiment through
the bindings, and checks every sample of every channel against the tolerance
defined below, reporting the sample that comes closest to it:

```bash
cmake -S . -B build -DVEHICLE_MODELS_BUILD_EXAMPLES=ON -DVEHICLE_MODELS_BUILD_PYTHON=ON
cmake --build build --config Release
cd python
python tools/compare_with_cpp.py ../build/Release/step_steer
```

```
(at the sample closest to its own bound)
channel                |diff|      tolerance    verdict
t                   0.000e+00      1.000e-12         ok
steer_deg           0.000e+00      1.000e-12         ok
r_kin               0.000e+00      1.000e-12         ok
r_dyn               0.000e+00      1.000e-12         ok
r_dtr               0.000e+00      1.000e-12         ok
beta_dyn_deg        0.000e+00      1.000e-12         ok
ay_dyn              0.000e+00      1.000e-12         ok
ay_dtr              0.000e+00      1.000e-12         ok
```

**Where the tolerance comes from.** Both sides execute the same C++ code, so
the only difference the comparison could see is one the CSV introduces.
`examples/step_steer.cpp` writes with `std::setprecision(17)`, the shortest
precision at which every `double` round-trips exactly, so the file carries the
computed value unchanged and contributes nothing. What is left is the
agreement of the two call paths, and the check bounds it the way
`math.isclose` does:

$$
\bigl\lvert y^{\text{C++}}_k - y^{\text{Py}}_k \bigr\rvert
\le \max\bigl(\varepsilon_{\text{rel}} \max(\lvert y^{\text{C++}}_k \rvert,
\lvert y^{\text{Py}}_k \rvert),\ \varepsilon_{\text{abs}}\bigr),
\qquad
\varepsilon_{\text{rel}} = 10^{-9},\quad \varepsilon_{\text{abs}} = 10^{-12}
$$

Every channel comes out bit-identical, so the reported difference is exactly
zero and the bound is never approached. The `tolerance` column shows the bound
at the sample closest to it, which for an exact match is the first row, where
all channels are zero and $`\varepsilon_{\text{abs}}`$ governs.

> An earlier revision printed the CSV with `%.4f` and `%.6f` while checking a
> flat $10^{-9}$ — a condition the rounded output could never meet no matter how
> well the two sides agreed. The tolerance was first relaxed to half the printed
> quantum to make the check honest; raising the output precision instead removed
> the quantisation altogether and let the tight bound stand.

---

## Equivalence at the migration

When the models moved behind the C++ bindings, the numerical equivalence with
the previous pure-Python implementation was measured directly: 4 presets × 7
models × 3 tire models = 84 combinations, all 23 `runner` channels, 400 steps
each. **Every channel agreed to within $10^{-9}$.** Regenerating the figures in
`docs_en/images/` reproduced the committed files byte for byte.

The migration surfaced exactly one behavioural difference, since fixed.
pybind11 enums return a fresh object on every access, so `runner.py` comparing
`ReferencePoint` with `is` always took the false branch and evaluated the
CoG-referenced kinematic model as if it were rear-axle referenced. It now uses
`==`. Keep that in mind when writing similar comparisons.

---

## Numerical notes

- States are 1-D `numpy` arrays on the Python side and are copied into the
  fixed-size C++ `StateVector` at the boundary. The conversion moves values
  only; no rounding happens.
- The manoeuvre runner samples the outputs *before* the integration step, so a
  logged sample at time $t$ is the state at $t$, not at $t + \Delta t$. The C++
  `step_steer` example does the same.
- `simulate()` shortens the final sub-step to land exactly on the requested
  duration: with $N = \lfloor T/\Delta t \rfloor$ full steps the last one is
  $T - N\Delta t$. This is the C++ implementation, called directly.
- The intermediate RK4 stages $k_1 \dots k_4$ are not passed through
  `normalizeState()`. Only the final state of a step is normalized.
