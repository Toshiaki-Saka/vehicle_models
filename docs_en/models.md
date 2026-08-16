# Equations of motion

日本語版: [`docs_ja/models.md`](../docs_ja/models.md)

Symbols: $L = l_f + l_r$ (wheelbase), $m$ (mass), $I_z$ (yaw inertia), $C_f, C_r$
(cornering stiffness per axle $[\mathrm{N/rad}]$), $\delta$ (front road wheel
angle), $\psi$ (yaw angle), $r = \dot\psi$ (yaw rate), $\beta$ (body slip angle).
Right-handed coordinates; $\delta > 0$ is a left turn.

---

## 1. Unicycle / differential drive

$$
\begin{aligned}
\dot x &= v \cos\psi \\
\dot y &= v \sin\psi \\
\dot\psi &= \omega
\end{aligned}
$$

For a differential drive, from the wheel angular rates $\omega_l, \omega_r$,
wheel radius $R_w$ and track $b$:

$$
v = \frac{R_w (\omega_r + \omega_l)}{2},
\qquad
\omega = \frac{R_w (\omega_r - \omega_l)}{b}
$$

There is no lower bound on the turn radius (the vehicle can rotate in place), so
this is *not* a valid behaviour model for an Ackermann-steered vehicle. Use it as
a planner-side abstraction, or for skid-steer platforms only.

---

## 2. Ackermann geometry

Turn radius at the rear axle centre: $R = L / \tan\delta$. Ideal Ackermann
satisfies

$$
\cot\delta_{\text{outer}} - \cot\delta_{\text{inner}} = \frac{T}{L}
$$

($T$ is the front track). This library derives the ideal angles from

$$
\cot\delta_{\text{left}} = \cot\delta - \frac{T}{2L},
\qquad
\cot\delta_{\text{right}} = \cot\delta + \frac{T}{2L}
$$

and then blends linearly toward parallel steering with the ratio $k$
(`ackermann_ratio`):

$$
\delta_i = \delta + k\,(\delta_{i,\text{ideal}} - \delta)
\qquad
\begin{cases}
k = 1 & \text{ideal Ackermann} \\
k = 0 & \text{parallel steering}
\end{cases}
$$

Real steering racks usually have $k < 1$ (an Ackermann percentage of 60–80 % is
typical); `ackermannError()` returns the resulting deviation of the outer wheel.

Wheel speeds (rigid body motion):

$$
\begin{aligned}
v_{rl} &= v - \frac{T_r}{2} r,
&\qquad
v_{rr} &= v + \frac{T_r}{2} r \\[4pt]
v_{fl} &= \left(v - \frac{T_f}{2} r\right)\cos\delta_l + L r \sin\delta_l,
&\qquad
v_{fr} &= \left(v + \frac{T_f}{2} r\right)\cos\delta_r + L r \sin\delta_r
\end{aligned}
$$

The front wheels are projected onto their own steered direction, which is the
quantity a wheel speed sensor observes.

---

## 3. Kinematic bicycle

Assumes zero tire slip. The equations depend on the reference point.

**Rear axle**

$$
\dot x = v\cos\psi,
\qquad
\dot y = v\sin\psi,
\qquad
\dot\psi = \frac{v\tan\delta}{L},
\qquad
\dot v = a
$$

**Centre of gravity** ($\beta = \arctan\left(l_r \tan\delta / L\right)$, $v$ is
the CoG speed)

$$
\dot x = v\cos(\psi + \beta),
\qquad
\dot y = v\sin(\psi + \beta),
\qquad
\dot\psi = \frac{v\cos\beta\,\tan\delta}{L}
$$

**Front axle** ($v$ is the front wheel speed)

$$
\dot x = v\cos(\psi + \delta),
\qquad
\dot y = v\sin(\psi + \delta),
\qquad
\dot\psi = \frac{v\sin\delta}{L}
$$

Valid range: lateral acceleration below roughly $0.4\,g$. Above that the tire
slip angles are no longer negligible and the yaw rate is badly over-predicted
(see `examples/step_steer.cpp`, and the Manoeuvre tab of the Python GUI, which
shows the same comparison interactively).

Version including the steering actuator:

$$
\dot\delta = \operatorname{clamp}\!\left(\frac{\delta_{\text{cmd}} - \delta}{\tau},\ \pm\dot\delta_{\max}\right)
$$

---

## 4. Linear 2-DOF (lateral) model

The classic handling model: constant $v_x$, small angles.

$$
\frac{d}{dt}
\begin{bmatrix} v_y \\ r \end{bmatrix}
=
\begin{bmatrix}
-\dfrac{C_f + C_r}{m v_x} & -v_x - \dfrac{l_f C_f - l_r C_r}{m v_x} \\[10pt]
-\dfrac{l_f C_f - l_r C_r}{I_z v_x} & -\dfrac{l_f^2 C_f + l_r^2 C_r}{I_z v_x}
\end{bmatrix}
\begin{bmatrix} v_y \\ r \end{bmatrix}
+
\begin{bmatrix} \dfrac{C_f}{m} \\[10pt] \dfrac{l_f C_f}{I_z} \end{bmatrix}
\delta
$$

`stateMatrix()` / `inputMatrix()` return this $A$ and $B$ directly, so the model
can be used as the design plant for an LQR or MPC, or as the prediction model of
an observer.

### Closed-form results (`linear_analysis.hpp`)

Understeer gradient:

$$
K = \frac{m}{L}\left(\frac{l_r}{C_f} - \frac{l_f}{C_r}\right)
\quad [\mathrm{rad/(m/s^2)}],
\qquad
\delta = \frac{L}{R} + K a_y
$$

- $K > 0$: understeer → characteristic speed $V_{ch} = \sqrt{L/K}$ (the speed at
  which the yaw rate gain is half the neutral-steer value)
- $K < 0$: oversteer → critical speed $V_{cr} = \sqrt{-L/K}$ (divergent above it)

Steady state cornering:

$$
\frac{r}{\delta} = \frac{v}{L + K v^2},
\qquad
\frac{a_y}{\delta} = \frac{v^2}{L + K v^2},
\qquad
\beta = \frac{1}{R}\left(l_r - \frac{m l_f v^2}{L C_r}\right)
$$

Neutral steer point (distance behind the front axle) and static margin:

$$
x_{NSP} = \frac{L C_r}{C_f + C_r},
\qquad
SM = \frac{x_{NSP} - l_f}{L}
$$

The yaw mode follows from the eigenvalues of $A$:

$$
\omega_n = \sqrt{\det A},
\qquad
\zeta = -\frac{\operatorname{tr} A}{2\sqrt{\det A}}
$$

---

## 5. Nonlinear single track (dynamic bicycle)

$$
\begin{aligned}
m\left(\dot v_x - v_y r\right) &= F_x - F_{yf}\sin\delta - F_{\text{res}} \\
m\left(\dot v_y + v_x r\right) &= F_{yf}\cos\delta + F_{yr} \\
I_z \dot r &= l_f F_{yf}\cos\delta - l_r F_{yr}
\end{aligned}
$$

Slip angles:

$$
\alpha_f = \delta - \arctan\frac{v_y + l_f r}{v_x},
\qquad
\alpha_r = -\arctan\frac{v_y - l_r r}{v_x}
$$

$v_x$ is guarded (`guardDenominator`). Longitudinal load transfer is
quasi-static:

$$
\Delta F_z = \frac{m a_x h}{L},
\qquad
F_{zf} = \frac{m g l_r}{L} - \Delta F_z,
\qquad
F_{zr} = \frac{m g l_f}{L} + \Delta F_z
$$

Driving resistance is aerodynamic drag plus rolling resistance:

$$
F_{\text{res}} = \underbrace{\tfrac{1}{2}\rho C_d A\, v_x |v_x|}_{\text{aerodynamic}}
+ \underbrace{\mu_{rr}\, m g \tanh\frac{v_x}{0.1}}_{\text{rolling}}
$$

The coefficient $\tfrac{1}{2}\rho C_d A$ of the first term is the parameter
`drag_area` $[\mathrm{N/(m/s)^2}]$ and $\mu_{rr}$ is `rolling_resistance`. The
$\tanh$ smooths the sign reversal at standstill.

---

## 6. Kinematic / dynamic blending

$\lambda(v_x)$ is 0 at low speed and 1 at high speed; the lateral derivatives are
blended:

$$
\begin{aligned}
\lambda &= \operatorname{clamp}\!\left(\frac{|v_x| - v_{lo}}{v_{hi} - v_{lo}},\ 0,\ 1\right) \\[4pt]
\dot v_y &= \lambda\, \dot v_{y,\text{dyn}} + (1 - \lambda)\frac{v_{y,\text{kin}} - v_y}{\tau} \\[4pt]
\dot r &= \lambda\, \dot r_{\text{dyn}} + (1 - \lambda)\frac{r_{\text{kin}} - r}{\tau}
\end{aligned}
$$

with

$$
v_{y,\text{kin}} = v_x \tan\beta_{\text{kin}},
\qquad
r_{\text{kin}} = \frac{v_x \cos\beta_{\text{kin}} \tan\delta}{L}
$$

At low speed the states are pulled toward the kinematic solution through a first
order lag, which avoids the $1/v_x$ singularity of the single-track model; at
high speed the model is purely dynamic again.

---

## 7. Double track (four wheels)

Per-wheel slip angles (in a left turn $\delta_l > \delta_r$):

$$
\begin{aligned}
\alpha_{fl} &= \delta_l - \arctan\frac{v_y + l_f r}{v_x - T_f r/2},
&\qquad
\alpha_{fr} &= \delta_r - \arctan\frac{v_y + l_f r}{v_x + T_f r/2} \\[6pt]
\alpha_{rl} &= -\arctan\frac{v_y - l_r r}{v_x - T_r r/2},
&\qquad
\alpha_{rr} &= -\arctan\frac{v_y - l_r r}{v_x + T_r r/2}
\end{aligned}
$$

Load transfer (longitudinal + lateral, with roll stiffness distribution $k_f$):

$$
\begin{aligned}
F_{zf,\text{axle}} &= \frac{m g l_r}{L} - \frac{m a_x h}{L},
&\qquad
F_{zr,\text{axle}} &= \frac{m g l_f}{L} + \frac{m a_x h}{L} \\[6pt]
\Delta F_{z,\text{front}} &= \frac{m a_y h k_f}{T_f},
&\qquad
\Delta F_{z,\text{rear}} &= \frac{m a_y h (1 - k_f)}{T_r}
\end{aligned}
$$

This is solved with a one-pass predictor (compute the forces on the static loads
→ estimate $a_x$, $a_y$ → recompute the loads → recompute the forces). Because it
does not iterate, the WCET can be bounded.

The yaw moment includes the contribution of the longitudinal forces:

$$
M_z = l_f\left(F_{y,fl} + F_{y,fr}\right) - l_r\left(F_{y,rl} + F_{y,rr}\right)
+ \frac{T_f}{2}\left(F_{x,fr} - F_{x,fl}\right)
+ \frac{T_r}{2}\left(F_{x,rr} - F_{x,rl}\right)
$$

($F_x$, $F_y$ already rotated into the body frame). This is the first model in
the library that can represent torque vectoring or yaw control through a
left/right brake force difference.

---

## 8. Tire models

**Linear**:

$$
F_y = C_\alpha \alpha,
\qquad \text{saturated at } |F_y| \le \mu F_z
$$

**Fiala (brush)**: cubic build-up until the sliding angle

$$
\alpha_{sl} = \arctan\frac{3\mu F_z}{C_\alpha}
$$

and constant $\mu F_z$ beyond it.

$$
F_y = C_\alpha \tan\alpha
- \frac{C_\alpha^2}{3\mu F_z}\left|\tan\alpha\right|\tan\alpha
+ \frac{C_\alpha^3}{27\mu^2 F_z^2}\tan^3\alpha
$$

**Pacejka Magic Formula (pure lateral slip)**:

$$
F_y = D \sin\Big(C \arctan\big(B\alpha - E(B\alpha - \arctan B\alpha)\big)\Big),
\qquad D = \mu F_z
$$

The slope at $\alpha = 0$ (the cornering stiffness) is

$$
\left.\frac{\partial F_y}{\partial \alpha}\right|_{\alpha=0} = BCD
$$

`PacejkaTire::fromCorneringStiffness()` solves for the $B$ that matches a given
$C_\alpha$ at a nominal load, so the Magic Formula can be swapped in for the
linear tire and compared directly.

**Combined slip**: scaling by the friction ellipse

$$
F_y \leftarrow F_y \sqrt{1 - \left(\frac{F_x}{\mu F_z}\right)^2}
$$

---

## 9. Integrators

`stepEuler` (1st order), `stepHeun` (2nd order), `stepRK4` (4th order). The local
truncation errors are

$$
e_{\text{Euler}} = \mathcal{O}(h^2),
\qquad
e_{\text{Heun}} = \mathcal{O}(h^3),
\qquad
e_{\text{RK4}} = \mathcal{O}(h^5)
$$

giving global errors of $\mathcal{O}(h)$, $\mathcal{O}(h^2)$ and
$\mathcal{O}(h^4)$. `test_integrator.cpp` verifies this on a quarter-circle
trajectory from the error ratio when the step is halved:

$$
\frac{e(h)}{e(h/2)} \approx 2,\ 4,\ 16
$$

RK4 is more than enough at a 10 ms control period; choose Euler when imitating a
fixed-point implementation on the target ECU.
