# Python simulation GUI

日本語版: [`docs_ja/python-gui.md`](../docs_ja/python-gui.md)

An interactive desktop application that runs every model of the library from one
shared vehicle definition and shows what separates them. It exists to answer the
questions the numbers alone do not: *which model do I need for this manoeuvre,
where does the simpler one stop being valid, and what can this vehicle actually
do?*

The models are the C++ library itself, reached through the bindings in
[`python/vehicle_models_py`](../python/vehicle_models_py) — see
[python-api.md](python-api.md) for the API and
[validation.md](validation.md) for how the bindings are verified.

---

## Requirements and start-up

Python 3.8 or newer, a C++17 compiler and CMake 3.16 or newer. From the
repository root:

```bash
pip install .                        # builds the _core extension
pip install matplotlib               # the GUI also needs it
cd python
python run_gui.py
```

`tkinter` ships with the standard Python installers on Windows and macOS. On
Debian/Ubuntu install it with `sudo apt install python3-tk`.

Alternatively, with `python/` on the path:

```bash
python -m vehicle_models_py.gui
```

---

## The window

```
+----------------------+-------------------------------------------------+
| Vehicle              |  Manoeuvre | Animation | Handling analysis |     |
|  preset selector     |  Performance | Tire models | Ackermann      |     |
|  ~25 parameters      |                                                 |
|  validation message  |     controls  |            plots                |
|  derived quantities  |               |            metric table         |
+----------------------+-------------------------------------------------+
| status bar                                                             |
```

**The vehicle panel on the left is global.** Every tab reads the same
`VehicleParameters`, so switching tabs never changes the vehicle under
discussion. Press **Apply** after editing a field; the values are checked by the
same `validate()` rules as the C++ struct, and an invalid set is rejected with a
message instead of being silently used. **Reset** reloads the selected preset.

Presets: *Passenger car* (the library default), *Low-speed shuttle*, *Off-road
buggy*, *Oversteering car* (a passenger car with the axle stiffnesses swapped —
the most instructive case in the handling view).

The **Derived** box under the buttons updates on every Apply and is often all
you need: wheelbase, weight distribution, static axle loads, understeer gradient
$K$, static margin, neutral steer point, characteristic or critical speed, the
lateral acceleration bound $a_{y,\max} = \mu g$, minimum turn radius and the
handwheel angle at full lock.

If the window is cramped on a small screen, **View → Vehicle panel** hides the
left column, and the divider between the panels can be dragged.

Angles are entered and displayed in **degrees** everywhere in the interface;
internally everything is SI and radians, as in the C++ library.

---

## Manoeuvre tab

![Step steer through four models](images/maneuver.png)

One input, several models, the same plot. This is the tab that shows *model
choice as a number*.

### Controls

| Control | Meaning |
|---|---|
| Type | The manoeuvre (see the list below). Fields that the selected manoeuvre does not read are greyed out |
| Duration / Time step | Simulated time and the fixed integration step |
| Speed | Initial speed, and the target of the speed controller |
| Hold speed (PI) | Closes a PI loop on the speed. Without it a lateral manoeuvre bleeds speed through cornering drag in the dynamic models but not in the kinematic ones, and the comparison would be between two different operating points |
| Steer amplitude / Input start | Step and sine amplitude, and the time the input starts |
| Sine frequency | Frequency of the sine manoeuvres |
| Ramp rate | Steer rate of the ramp manoeuvre |
| Radius | Target radius of the constant-radius manoeuvre |
| Brake a_x / Brake start | Deceleration and its onset |
| Lane offset / Section length | Geometry of the path-following manoeuvres |
| Models | Which models to run. Each has a fixed colour, used identically in every plot |
| Tire model | Linear, Fiala or Pacejka — matched to the same cornering stiffness $C_\alpha$ and the same peak $\mu F_z$, so only the saturation shape differs |
| Integrator | Euler, Heun or RK4 |

### Manoeuvres

- **Step steer (J-turn)** — the classic yaw response test. The reference line in
  the yaw-rate plot is the closed-form linear steady state.
- **Ramp steer** — slowly increasing steer; the basis of the handling diagram in
  the Performance tab.
- **Sine steer** / **Sine with dwell** — frequency response and the NHTSA-style
  test with a 500 ms dwell at the second peak.
- **Constant radius** — holds the steer angle that the linear analysis says is
  needed for the requested radius, ramped in over one second.
- **Braking in a turn** — a step steer with a braking phase; the case where
  combined slip and load transfer matter and the single-track model starts to
  disagree with the double-track one.
- **Straight-line accel / brake** — full acceleration, then braking.
- **Slalom** / **Double lane change** — closed loop. A pure-pursuit driver model
  follows a reference path, so the models are compared *under the same driver*
  rather than under the same open-loop input.
- **Reference route** — closed loop on the kilometre of road in
  [`data/reference_route.csv`](../data/README.md), with a speed profile planned
  from its curvature. See [Route following](#route-following) below.

### Reading the plots

- **Path** — the trajectories, plus the reference path for the closed-loop
  manoeuvres. If the manoeuvre is much longer than it is wide (a lane change),
  the y axis is exaggerated and the title says so.
- **Road wheel angle** — the actual angle per model against the command. Only
  the actuator model lags behind the command; for path manoeuvres this plot is
  the driver's output.
- **Yaw rate** — with the linear steady state as a reference line.
- **Lateral acceleration** — with the tire limit $\mu g$ and the $0.4\ g$ line
  above which the kinematic models are no longer valid.
- **Body slip angle** — note the sign: the kinematic model always predicts a
  positive $\beta$ in a left turn, while any dynamic model turns negative above

  $\displaystyle v > \sqrt{\frac{l_r L C_r}{m l_f}}$

  the speed at which the rear axle needs a slip angle to generate its force —
  compare the linear steady state $\beta = \left(l_r - m l_f v^2/(L C_r)\right)/R$.
  That sign flip is the clearest single symptom of the kinematic model's validity
  limit.
- **Speed** — the operating point, and how hard the speed controller had to
  work.

The table underneath gives peak and steady yaw rate, overshoot, the 90 %
response time, peak lateral acceleration, peak body slip and the final speed,
with the closed-form linear values as the last row.

**Export CSV** writes every channel of every model as one wide table
(`<model>.<channel>` columns).

### Closed-loop example

![Double lane change](images/lane_change.png)

With the same driver, the dynamic models need visibly more steering and lag the
reference path, because the tires have to build up a slip angle before they
produce force — the kinematic model has no such delay.

---

## Animation tab

![Animation](images/animation.png)

Plays back the last manoeuvre. Select a model, press **Play**, or drag the time
slider.

- The body outline, and the four wheels at their actual steer angles (the
  double-track model steers the two front wheels differently — that is the
  Ackermann geometry at work).
- **The disc under each wheel scales with its vertical load.** Load transfer
  becomes visible: in a left turn the right-hand discs grow, and under braking
  the front ones do. The single-track models split each axle evenly, so their
  discs only move front to rear.
- The red arrow is the velocity vector at the CoG — the angle between it and the
  body axis is the body slip angle.
- The HUD and the read-out on the left give the instantaneous values, including
  the axle slip angles and all four wheel loads.
- The three plots on the right show the whole run with a cursor at the current
  time.

`Playback rate` is a multiplier on real time; `View span` is the width of the
camera window in metres.

---

## Route following

![Every model on the reference route](images/route_demo.gif)

The manoeuvres above are 100 m long and shaped by one steering input. A route is
a kilometre of road with corners of different radii, and driving it asks a
different question: *given the same driver, how far off the road does each model
end up, and where?*

`Reference route` in the Manoeuvre tab drives
[`data/reference_route.csv`](../data/README.md) — 1010 m of straights, clothoid
transitions and four bends down to a 25 m radius. Selecting it fills in a
duration long enough to reach the goal; `Speed` then only sets the speed the run
starts at, because the driver plans its own.

The same run from the command line, with the animation above:

```bash
cd python
python demo_route.py                                   # play it on screen
python demo_route.py --save route.gif --rate 8         # write it out
python demo_route.py --models all --overview route.png
python demo_route.py --route my_course.csv --preset shuttle
```

### The driver

One driver serves every model, so what is left on screen is the difference
between the models:

- **Lateral** — pure pursuit against the route. The steer angle follows from the
  lookahead distance $L_d$ and the angle $\eta$ to the target point,

  $\displaystyle L_d = \mathrm{clamp}(4 + 0.5\ v,\ 3,\ 18)\ [\mathrm{m}], \qquad \delta = \arctan\frac{2 L \sin\eta}{L_d}$

  with the target taken $L_d$ metres *along the route* from the vehicle's
  projection onto it. Because the target is found by arc length rather than by
  $x$, the road may turn through any angle.
- **Longitudinal** — a speed profile computed once from the route curvature
  $\kappa$,

  $\displaystyle v = \min\left(\sqrt{\frac{0.35\ \mu g}{\lvert \kappa \rvert}},\ v_{\max}\right)$

  then a backward pass at `accel_min` so braking starts before the corner and a
  forward pass at `accel_max` so the profile is one the vehicle can produce. A PI
  loop tracks it, and a preview scan of the profile ahead adds braking early.
- Every model is steered **from its rear axle**. The kinematic models integrate
  that point already; for the others the pose is shifted back by $l_r$ first,

  $\displaystyle x_r = x - l_r \cos\psi, \qquad y_r = y - l_r \sin\psi$

  Without that, half the models would be steered from a point 1.5 m further
  forward and would cut every corner by that much.

### Reading the animation

- The camera follows the pack and widens automatically when the models drift
  apart; the minimap shows where on the route the pack is.
- **Cross-track error** is the signed distance from the route, measured at the
  rear axle. Positive is left of the direction of travel.
- **Speed** carries the reference profile as a dashed line — the gap on corner
  entry is the model failing to slow down in time, not the profile changing.
- **Lateral acceleration** carries the limit $\mu g$ and the $0.4\ g$ line below
  which the kinematic models are defensible. The profile is planned for
  $0.35\ \mu g$, so the whole run sits just under that line by design: this is the
  regime where the kinematic model is *supposed* to be adequate.

![Route overview](images/route_overview.png)

For the passenger-car preset, one kilometre at up to 0.37 g:

| Model | max &#124;e&#124; | rms e | finish |
|---|---|---|---|
| Kinematic (CoG) | 0.33 m | 0.08 m | 61.0 s |
| Dynamic bicycle (Fiala) | 0.79 m | 0.28 m | 61.6 s |
| Double track (Fiala) | 0.80 m | 0.28 m | 61.1 s |

The kinematic model tracks the route more than twice as tightly as the dynamic
ones — not because it is better, but because it cannot represent what makes the
others miss. Its tires build force with no slip angle, so it turns exactly as
much as the steer angle says; the dynamic models need a slip angle first and
therefore run wide on every corner entry, which is what the error traces show:
flat on the straights, a bump at every bend, largest at the two 25/30 m
junctions. A tracking controller tuned against the kinematic model alone is
tuned against a vehicle that does not exist.

**`--models linear2dof` is worth watching once.** The linear 2-DOF model holds
its longitudinal speed by construction, so it cannot brake for a corner and
cannot stop at the goal: it takes the 25 m bends at 20 m/s — the geometry alone
already asks for

$$
a_y = \frac{v^2}{R} = \frac{(20\ \mathrm{m/s})^2}{25\ \mathrm{m}}
      = 16\ \mathrm{m/s^2} \approx 1.6\ g
$$

— demands 1.76 g of lateral acceleration from tires that have $\mu = 1.0$, and
ends up 17 m off the route. That is not a bug — it is a constant-speed design plant being asked to
drive a course, and the animation makes the assumption visible.

---

## Handling analysis tab

![Handling analysis](images/handling.png)

Purely analytic — everything here is a closed-form result of the linear
single-track model, so it re-renders instantly when a parameter changes. Use it
to understand *what kind of vehicle* the parameter set describes before running
anything.

- **Yaw rate gain**

  $\displaystyle \frac{r}{\delta} = \frac{v}{L + K v^2}$

  against the neutral-steer reference $v/L$. For an understeering vehicle it
  peaks at the characteristic speed $V_{ch} = \sqrt{L/K}$; for an oversteering one
  it diverges at the critical speed $V_{cr} = \sqrt{-L/K}$ (marked in red).
- **Lateral acceleration gain** $a_y/\delta = v^2/(L + K v^2)$, with the limit
  $\mu g$ — above that line the linear prediction is no longer physically
  reachable.
- **Steer angle to hold a radius**, split into the Ackermann term and the slip
  term,

  $\displaystyle \delta = \underbrace{\frac{L}{R}}_{\text{geometry}} + \underbrace{K a_y}_{\text{tire slip}}$

  The curve stops where $v^2/R$ exceeds $\mu g$.
- **Yaw mode frequency and damping** against speed (from 2 m/s upward, where the
  $1/v_x$ terms stop dominating), from the eigenvalues of $A$:

  $\displaystyle \omega_n = \sqrt{\det A}, \qquad \zeta = -\frac{\mathrm{tr} A}{2\sqrt{\det A}}$

  An unstable speed range is shaded.
- **Eigenvalue locus** of the $[v_y,\ r]$ system, coloured by speed. Crossing into
  the right half plane, $\mathrm{Re}\lambda > 0$, is the critical speed.

The table gives the same quantities as numbers, each with a short reading of
what it means.

**Try this:** select the *Oversteering car* preset. $K$ becomes negative, the
characteristic speed is replaced by a finite critical speed of about 27 m/s, the
yaw rate gain diverges there and the eigenvalue locus crosses zero at the same
speed. Then run a step steer at 30 m/s in the Manoeuvre tab and watch the
divergence in the time domain.

---

## Performance tab

![Performance](images/performance.png)

What the vehicle can actually do. Press **Run performance suite** (a few
seconds; it runs in the background and reports progress).

- **Speed and distance against time** for a full acceleration run and a braking
  run.
- **Achieved $a_x$ against speed** — the commanded acceleration minus the driving
  resistance,

  $\displaystyle a_x = a_{x,\text{cmd}} - \frac{F_{\text{res}}(v)}{m}$

  which is why the achieved value falls off with speed.
- **Handling diagram** — $\delta - L/R$ against $a_y$ from a slow ramp steer. The
  initial slope *is* the understeer gradient,

  $\displaystyle K = \left.\frac{\partial}{\partial a_y}\left(\delta - \frac{L}{R}\right)\right|_{a_y \to 0}$

  (the closed-form $K$ is drawn as a reference); where the curve turns back is the
  tire limit, and the marked point is the maximum lateral acceleration the model
  reaches.
- **g-g envelope** — the accelerations actually achieved over a grid of
  longitudinal demands and steer angles, with the point-mass friction circle
  $a_x^2 + a_y^2 \le (\mu g)^2$ for comparison. The flat top and bottom are the
  actuation limits `accel_max` and `accel_min`, not the tires.
- **Maximum cornering speed against radius**, from the simulated limit and from
  the point-mass bound $v_{\max} = \sqrt{\mu g R}$.

The table lists top speed, 0–30/50/100 km/h times, braking distance against the
ideal point-mass value $s = v_0^2/(2\mu g)$, limit lateral acceleration as the
fraction $a_{y,\max}/(\mu g)$, and the understeer gradient measured from the ramp
compared with the closed form. The
measured and closed-form K agreeing is a good check that the parameter set and
the tire model are consistent.

`Model` selects single-track or double-track for the whole suite; the difference
in limit lateral acceleration between them is the effect of load transfer and
per-wheel saturation.

---

## Tire models tab

![Tire models](images/tire.png)

All three tire models matched to the same cornering stiffness at $\alpha = 0$ and
the same peak $\mu F_z$, so the only thing left to compare is the saturation
shape.

- **Lateral force against slip angle** at one load, with the linear tangent
  $C_\alpha \alpha$ and the bound $\mu F_z$.
- **Load sensitivity** — one model at five vertical loads. Note that the peak
  force scales with load while the slope does not, which is exactly why load
  transfer costs grip.
- **Normalized force** $F_y/(\mu F_z)$ — the saturation shape alone: the hard
  corner of the linear tire, the smooth cubic build-up of Fiala, the peak-then-
  decay of the Magic Formula.
- **Friction ellipse** — how much lateral force is left once a fraction of the
  friction is spent longitudinally:

  $\displaystyle \frac{F_y}{F_{y,0}} = \sqrt{1 - \left(\frac{F_x}{\mu F_z}\right)^2}$

  Spending 50 % of the friction on braking leaves
  $\sqrt{1 - 0.5^2} \approx 87\ \%$ of the lateral force; spending 80 % leaves
  $\sqrt{1 - 0.8^2} = 60\ \%$.

**Load from vehicle** fills the load and friction fields from the current
vehicle parameters and the selected axle.

---

## Ackermann tab

![Ackermann geometry](images/ackermann.png)

- **Top view** at the chosen steer angle: the two front wheels at their
  individual angles, the common turn centre, and the lines from the centre to
  each wheel. With ideal Ackermann all four lines meet at that centre.
- **Road wheel angles** against the bicycle steer angle, with the ideal
  Ackermann curves and the parallel-steer line for comparison.
- **Ackermann error of the outer wheel** for five values of `ackermann_ratio` $k$:
  the deviation $\delta_{\text{outer}} - \delta_{\text{outer,ideal}}$ from the
  ideal geometry that a real rack has.
- **Wheel speeds** against yaw rate at a chosen speed — what the wheel speed
  sensors see, and the basis of any odometry or slip estimate.

The table gives the inner and outer angles, the Ackermann error, the turn radius,
the minimum turn radius at full lock, the handwheel angle and the wheel speed
spread.

---

## Worked examples

**1. Where does the kinematic model break down?**
Manoeuvre tab, step steer, models *Kinematic (CoG)* + *Dynamic bicycle*. Run at
5 m/s: the two yaw rates are on top of each other. Run at 20 m/s: the kinematic
model over-predicts the yaw rate by about 1.5x (0.388 vs 0.248 rad/s for the
default car), and the body slip angles have opposite signs. The lateral
acceleration plot shows why — the run has crossed the $0.4\ g$ line.

**2. Why the blended model exists.**
Select the *Low-speed shuttle* preset, set the speed to 0.5 m/s and a 15 deg
step, and run *Dynamic bicycle* against *Blended kinematic/dynamic*. Below the
low-speed guard the plain dynamic model's lateral behaviour is meaningless,
while the blended model follows the kinematic prediction.

**3. Does the tire model matter?**
Step steer at 3 deg: Linear, Fiala and Pacejka give practically the same answer.
Raise the amplitude until $a_y$ approaches $\mu g$ and they separate — the linear
tire keeps its stiffness right up to the clip, while the other two soften first.
The Performance tab quantifies the difference in limit lateral acceleration.

**4. Single track versus double track.**
Braking in a turn, with a strong deceleration. The double-track model loses more
lateral acceleration, because the friction ellipse is applied per wheel on the
transferred loads, while the single-track model only sees an axle-level load
shift.

**5. Ackermann ratio.**
Set `ackermann_ratio` to 0 (parallel steering) and look at the Ackermann tab:
the error of the outer wheel grows to several degrees at full lock. At normal
driving angles the difference is negligible — which is why the single-track
model is adequate above walking pace and inadequate for a parking manoeuvre.

---

## Notes and limits

- **The speed controller** is a PI loop with anti-windup, clamped to
  `accel_min` / `accel_max`:

  $\displaystyle a_{x,\text{cmd}} = \mathrm{clamp}\left(K_p (v_{\text{ref}} - v) + K_i \int (v_{\text{ref}} - v)\ dt,\ a_{x,\min},\ a_{x,\max}\right)$

  For the kinematic models its output is the acceleration directly; for the
  dynamic models it becomes a longitudinal force $F_x = m\ a_{x,\text{cmd}}$, so
  the achieved acceleration is smaller by the driving resistance $F_{\text{res}}/m$.
- **`speed_max` clamps the kinematic models.** If the initial speed of a
  manoeuvre exceeds it, a warning appears under the Run button: the kinematic
  models will clamp their speed while the dynamic ones will not, which makes the
  comparison invalid. Raise `speed_max` for such runs.
- **The linear 2-DOF model holds its longitudinal speed constant** by
  construction. Its speed trace is therefore a flat line, and its longitudinal
  acceleration is always zero.
- **Road friction.** The GUI propagates $\mu$ (`friction`) to the tires of every
  model. The C++ `DynamicBicycleModel::syncTiresFromParams()` copies only the
  cornering stiffness, leaving the tire's own $\mu$ at its default of 1.0; the Python
  port keeps that behaviour by default but the GUI opts into propagating it, so
  all models on screen share one road surface. See
  [python-api.md](python-api.md#deliberate-differences-from-the-c-api).
- **Cost.** A step steer of 8 s at 2 ms through four models takes well under a
  second; the performance suite takes a few seconds. Both run off the UI thread,
  so the window stays responsive.
- The pure-pursuit driver of the path manoeuvres is deliberately simple. It is
  there to give every model the same driver, not to be a good controller.

---

## Regenerating the figures in this document

```bash
cd python
python tools/make_doc_figures.py       # writes docs_en/images/*.png
```

The script drives the same code paths as the GUI, so the documentation always
shows what the application actually produces.
