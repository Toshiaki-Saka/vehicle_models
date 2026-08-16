# Validating the Python port

日本語版: [`docs_ja/validation.md`](../docs_ja/validation.md)

The Python models are only useful if they are the *same* models. Three
independent checks back that up.

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

for Euler, Heun and RK4 respectively.

---

## 2. The documented C++ outputs, reproduced

The figures published in the C++ `README.md` are reproduced exactly by the
Python port:

| Quantity | C++ README | Python port |
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
the compiled `step_steer` example, reproduces the identical experiment in
Python, and reports the largest absolute difference per channel:

$$
\varepsilon_{\text{channel}}
= \max_{k}\ \bigl\lvert y^{\text{C++}}_k - y^{\text{Py}}_k \bigr\rvert
$$

```bash
cmake -S . -B build -DVEHICLE_MODELS_BUILD_EXAMPLES=ON
cmake --build build -j
cd python
python tools/compare_with_cpp.py ../build/step_steer
```

Both sides run the same equations, the same RK4 integrator and the same
parameters over 2501 samples, so the differences should be at round-off level.
The script exits non-zero if any channel drifts beyond $\varepsilon > 10^{-9}$,
which makes it usable as a regression check in CI once a C++ toolchain is
available.

> This third check requires a C++ compiler. It was **not** executed while the
> port was written — that environment had CMake but no compiler — so the claim
> it makes is "run this to confirm", not "this was confirmed here". Checks 1 and
> 2 were both run and pass.

---

## Known differences

There is one behavioural difference, and it is deliberate:
`DynamicBicycleModel::syncTiresFromParams()` in C++ propagates the cornering
stiffness but not `params.friction`, so a single-track model built from a
low-friction preset still uses tires with $\mu = 1.0$. The Python port
reproduces that by default and offers `sync_tires_from_params(sync_friction=True)`
as an opt-in, which the GUI and `performance.py` use so that every model on
screen shares one road surface. See
[python-api.md](python-api.md#the-friction-propagation-gap).

Anything else that differs between the two implementations is a bug in the port.

---

## Numerical notes

- Both implementations use IEEE-754 doubles and the same order of operations in
  the derivative functions, so agreement to round-off is expected rather than
  lucky.
- The manoeuvre runner samples the outputs *before* the integration step, so a
  logged sample at time $t$ is the state at $t$, not at $t + \Delta t$. The C++
  `step_steer` example does the same.
- `simulate()` shortens the final sub-step to land exactly on the requested
  duration, again as in C++: with $N = \lfloor T/\Delta t \rfloor$ full steps the
  last one is $T - N\Delta t$.
- The intermediate RK4 stages $k_1 \dots k_4$ are not passed through
  `normalize_state()`, in both implementations. Only the final state of a step is
  normalized.
