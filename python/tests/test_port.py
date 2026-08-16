# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Validation of the Python port against the C++ unit tests.

The checks below mirror ``test/test_kinematics.cpp``, ``test/test_dynamics.cpp``
and ``test/test_integrator.cpp`` one to one: same vehicles, same manoeuvres,
same tolerances. Passing this file means the Python models reproduce the same
closed-form behaviour the C++ library is verified against.

Run with ``python tests/test_port.py`` or ``pytest``.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle_models_py import (PI, AckermannGeometry, DifferentialDriveModel,
                               DifferentialDriveParams, DoubleTrackModel,
                               DynamicBicycleModel, FialaTire, FL, FR,
                               IntegratorType, KinematicBicycleModel,
                               KinematicBicycleSteerModel, LinearTire,
                               LinearLateralBicycleModel, PacejkaTire,
                               ReferencePoint, RL, RR, UnicycleModel,
                               VehicleParameters, ackermann_error,
                               bicycle_angle_from_wheels, deg2rad,
                               dynamic_input, dynamic_state,
                               friction_ellipse_scale, handwheel_to_road_wheel,
                               kinematic_input, kinematic_state, lateral_state,
                               make_buggy_parameters,
                               make_passenger_car_parameters,
                               make_shuttle_parameters, minimum_turn_radius,
                               road_wheel_angles, road_wheel_to_handwheel,
                               side_slip_of, simulate, steer_angle_for_radius,
                               steer_dynamics_state, steer_input, turn_radius,
                               unicycle_input, unicycle_state,
                               wheel_angular_rates, wheel_rate_input,
                               wheel_speeds)
from vehicle_models_py import linear_analysis as analysis
from vehicle_models_py.types import GRAVITY

_FAILURES = []
_CHECKS = 0
_SECTION = ""


def section(name):
    global _SECTION
    _SECTION = name


def check(condition, what):
    global _CHECKS
    _CHECKS += 1
    if not condition:
        _FAILURES.append("[%s] %s" % (_SECTION, what))


def check_near(value, expected, tol, what=""):
    check(abs(value - expected) <= tol,
          "%s: %.12g != %.12g (tol %.3g)" % (what or "value", value, expected, tol))


# ---------------------------------------------------------------------------
# kinematics
# ---------------------------------------------------------------------------

def test_unicycle():
    section("unicycle")
    model = UnicycleModel()
    x0 = unicycle_state(0.0, 0.0, 0.0)

    straight = simulate(model, x0, unicycle_input(2.0, 0.0), 3.0, 0.001)
    check_near(straight[0], 6.0, 1e-6, "straight x")
    check_near(straight[1], 0.0, 1e-9, "straight y")

    v, w = 3.0, 0.5
    circle = simulate(model, x0, unicycle_input(v, w), 2.0 * PI / w, 1e-4)
    check_near(circle[0], 0.0, 1e-4, "closed circle x")
    check_near(circle[1], 0.0, 1e-4, "closed circle y")

    spin = simulate(model, x0, unicycle_input(0.0, 1.0), 1.0, 1e-3)
    check_near(spin[2], 1.0, 1e-9, "spin yaw")
    check_near(spin[0], 0.0, 1e-12, "spin x")


def test_differential_drive():
    section("differential drive")
    p = DifferentialDriveParams(wheel_radius=0.20, track=0.60)
    model = DifferentialDriveModel(p)

    wheels = model.to_wheel_rates(1.5, 0.8)
    body = model.to_body_velocity(wheels)
    check_near(body[0], 1.5, 1e-12, "round trip v")
    check_near(body[1], 0.8, 1e-12, "round trip omega")

    equal = model.to_body_velocity(wheel_rate_input(5.0, 5.0))
    check_near(equal[1], 0.0, 1e-12, "equal rates omega")
    opposite = model.to_body_velocity(wheel_rate_input(-5.0, 5.0))
    check_near(opposite[0], 0.0, 1e-12, "opposite rates v")

    x = simulate(model, unicycle_state(), wheel_rate_input(5.0, 5.0), 2.0, 1e-3)
    check_near(x[0], 2.0 * p.wheel_radius * 5.0, 1e-9, "travelled distance")


def test_ackermann_geometry():
    section("ackermann geometry")
    g = AckermannGeometry(wheel_base=2.70, track_front=1.55, track_rear=1.55,
                          ackermann_ratio=1.0)
    delta = deg2rad(20.0)
    wa = road_wheel_angles(g, delta)

    check(wa.left > delta, "inner wheel steers more")
    check(wa.right < delta, "outer wheel steers less")

    cot_out = 1.0 / math.tan(wa.right)
    cot_in = 1.0 / math.tan(wa.left)
    check_near(cot_out - cot_in, g.track_front / g.wheel_base, 1e-9,
               "ideal Ackermann condition")

    check_near(ackermann_error(g, delta), 0.0, 1e-9, "ideal Ackermann error")
    parallel = AckermannGeometry(wheel_base=2.70, track_front=1.55,
                                 track_rear=1.55, ackermann_ratio=0.0)
    wp = road_wheel_angles(parallel, delta)
    check_near(wp.left, delta, 1e-12, "parallel left")
    check_near(wp.right, delta, 1e-12, "parallel right")
    check(ackermann_error(parallel, delta) > 0.0, "parallel steering has error")

    check_near(bicycle_angle_from_wheels(g, wa.left, wa.right), delta, 1e-9,
               "bicycle angle round trip")
    check_near(bicycle_angle_from_wheels(g, 0.0, 0.0), 0.0, 1e-12, "zero angle")

    zero = road_wheel_angles(g, 0.0)
    check_near(zero.left, 0.0, 1e-12, "zero left")
    right = road_wheel_angles(g, -delta)
    check_near(right.left, -wa.right, 1e-12, "mirror left")
    check_near(right.right, -wa.left, 1e-12, "mirror right")

    radius = turn_radius(g, delta)
    check_near(steer_angle_for_radius(g, radius), delta, 1e-12, "radius round trip")
    check(not math.isfinite(turn_radius(g, 0.0)), "straight radius is infinite")
    g.steering_ratio = 16.0
    check_near(handwheel_to_road_wheel(g, road_wheel_to_handwheel(g, delta)),
               delta, 1e-12, "handwheel round trip")

    check(minimum_turn_radius(g, deg2rad(35.0)) > g.wheel_base,
          "min radius exceeds wheelbase")
    check(minimum_turn_radius(g, deg2rad(35.0))
          < minimum_turn_radius(g, deg2rad(20.0)), "more lock, smaller radius")


def test_wheel_speeds():
    section("wheel speeds")
    g = AckermannGeometry(wheel_base=2.70, track_front=1.55, track_rear=1.55)

    straight = wheel_speeds(g, 10.0, 0.0)
    for name in ("rear_left", "rear_right", "front_left", "front_right"):
        check_near(getattr(straight, name), 10.0, 1e-12, "straight " + name)

    v, yaw_rate = 10.0, 0.3
    turning = wheel_speeds(g, v, yaw_rate)
    check(turning.rear_right > turning.rear_left, "outer rear faster")
    check(turning.front_right > turning.front_left, "outer front faster")
    check_near(0.5 * (turning.rear_left + turning.rear_right), v, 1e-12,
               "rear average")

    rates = wheel_angular_rates(turning, 0.32)
    check_near(rates.rear_left * 0.32, turning.rear_left, 1e-12, "angular rate")


def test_kinematic_bicycle():
    section("kinematic bicycle")
    p = make_passenger_car_parameters()
    check(not p.validate(), "default parameters are valid")

    rear = KinematicBicycleModel(p, ReferencePoint.REAR_AXLE)
    v, delta = 8.0, deg2rad(5.0)

    x1 = simulate(rear, kinematic_state(0, 0, 0, v), kinematic_input(0.0, delta),
                  1.0, 1e-4)
    check_near(x1[2], v * math.tan(delta) / p.wheel_base(), 1e-6, "yaw after 1 s")
    check_near(x1[3], v, 1e-12, "speed held")

    radius = p.wheel_base() / math.tan(delta)
    period = 2.0 * PI * radius / v
    loop = simulate(rear, kinematic_state(0, 0, 0, v),
                    kinematic_input(0.0, delta), period, 1e-4)
    check_near(loop[0], 0.0, 1e-3, "closed loop x")
    check_near(loop[1], 0.0, 1e-3, "closed loop y")

    cog = KinematicBicycleModel(p, ReferencePoint.CENTER_OF_GRAVITY)
    beta = cog.side_slip(delta)
    d_rear = rear.derivative(kinematic_state(0, 0, 0, v),
                             kinematic_input(0.0, delta))
    d_cog = cog.derivative(kinematic_state(0, 0, 0, v / math.cos(beta)),
                           kinematic_input(0.0, delta))
    check_near(d_cog[2], d_rear[2], 1e-12, "CoG yaw rate matches")
    check(beta > 0.0, "positive side slip for a left turn")

    front = KinematicBicycleModel(p, ReferencePoint.FRONT_AXLE)
    d_front = front.derivative(kinematic_state(0, 0, 0, v),
                               kinematic_input(0.0, delta))
    check_near(d_front[2], v * math.sin(delta) / p.wheel_base(), 1e-12,
               "front axle yaw rate")

    accelerated = simulate(rear, kinematic_state(0, 0, 0, 0),
                           kinematic_input(1.5, 0.0), 4.0, 1e-3)
    check_near(accelerated[3], 6.0, 1e-9, "accelerated speed")
    clamped = rear.derivative(kinematic_state(0, 0, 0, 0),
                              kinematic_input(100.0, deg2rad(90.0)))
    check_near(clamped[3], p.accel_max, 1e-12, "acceleration clamped")

    check_near(rear.lateral_acceleration(kinematic_state(0, 0, 0, v), delta),
               v * v / radius, 1e-9, "lateral acceleration helper")


def test_steering_dynamics():
    section("steering actuator dynamics")
    p = make_passenger_car_parameters()
    p.steer_time_constant = 0.10
    p.steer_rate_max = deg2rad(30.0)
    model = KinematicBicycleSteerModel(KinematicBicycleModel(p))

    cmd = deg2rad(10.0)
    x0 = steer_dynamics_state(0, 0, 0, 5.0, 0.0)

    d0 = model.derivative(x0, kinematic_input(0.0, cmd))
    check_near(d0[4], p.steer_rate_max, 1e-12, "rate limited")

    x1 = simulate(model, x0, kinematic_input(0.0, cmd), 2.0, 1e-3)
    check_near(x1[4], cmd, 1e-6, "converges to command")

    x2 = simulate(model, x0, kinematic_input(0.0, deg2rad(80.0)), 3.0, 1e-3)
    check(abs(x2[4]) <= p.steer_max + 1e-12, "mechanical limit respected")


# ---------------------------------------------------------------------------
# dynamics
# ---------------------------------------------------------------------------

def idealized_car():
    p = make_passenger_car_parameters()
    p.drag_area = 0.0
    p.rolling_resistance = 0.0
    return p


def test_tire_models():
    section("tire models")
    fz, c = 4000.0, 60000.0

    lin = LinearTire(cornering_stiffness=c, friction=1.0)
    check_near(lin.lateral_force(0.0, fz), 0.0, 1e-12, "linear at zero")
    check_near(lin.lateral_force(0.01, fz), c * 0.01, 1e-9, "linear slope")
    check_near(lin.lateral_force(1.0, fz), fz, 1e-9, "linear saturation")
    check_near(lin.lateral_force(-1.0, fz), -fz, 1e-9, "linear saturation neg")

    fiala = FialaTire(cornering_stiffness=c, friction=1.0)
    check_near(fiala.lateral_force(1e-4, fz) / 1e-4, c, c * 1e-3, "fiala slope")
    check(fiala.lateral_force(0.05, fz) > fiala.lateral_force(0.02, fz),
          "fiala monotone")
    check_near(fiala.lateral_force(0.6, fz), fz, 1e-9, "fiala sliding")
    check(abs(fiala.lateral_force(0.08, fz)) <= fz + 1e-9, "fiala bounded")
    check(fiala.lateral_force(0.04, fz) < lin.lateral_force(0.04, fz),
          "fiala below linear in the transition")

    pac = PacejkaTire.from_cornering_stiffness(c, fz, 1.0)
    check_near(pac.lateral_force(1e-5, fz) / 1e-5, c, c * 1e-3, "pacejka slope")
    check_near(pac.cornering_stiffness_at(fz), c, c * 1e-9, "pacejka BCD")
    peak = pac.lateral_force(0.15, fz)
    check(peak > 0.9 * fz, "pacejka peak")
    check(pac.lateral_force(0.6, fz) < peak, "pacejka falls back after peak")

    check_near(friction_ellipse_scale(0.0, 1.0, fz), 1.0, 1e-12, "ellipse at 0")
    check_near(friction_ellipse_scale(fz, 1.0, fz), 0.0, 1e-12, "ellipse at max")
    check_near(friction_ellipse_scale(0.6 * fz, 1.0, fz), 0.8, 1e-12, "ellipse 0.6")


def test_linear_analysis():
    section("linear handling analysis")
    p = idealized_car()

    k = analysis.understeer_gradient(p)
    check(k > 0.0, "default set understeers")
    check(analysis.static_margin(p) > 0.0, "positive static margin")
    check(math.isfinite(analysis.characteristic_speed(p)),
          "finite characteristic speed")
    check(not math.isfinite(analysis.critical_speed(p)),
          "no critical speed when understeering")

    v_ch = analysis.characteristic_speed(p)
    check_near(analysis.yaw_rate_gain(p, v_ch), 0.5 * v_ch / p.wheel_base(),
               1e-9, "gain at characteristic speed")

    radius, vx = 50.0, 15.0
    delta_req = analysis.required_steer_angle(p, radius, vx)
    check(delta_req > p.wheel_base() / radius, "more than Ackermann")
    ss = analysis.steady_state_cornering(p, vx, delta_req)
    check_near(ss.radius, radius, 1e-6, "steady radius")
    check_near(ss.lateral_accel, vx * vx / radius, 1e-6, "steady ay")
    check(abs(ss.slip_front) > abs(ss.slip_rear), "front works harder")

    check(analysis.yaw_mode(p, 5.0).stable, "stable at 5 m/s")
    check(analysis.yaw_mode(p, 40.0).stable, "stable at 40 m/s")
    check(analysis.yaw_mode(p, 20.0).natural_frequency > 0.0, "positive omega_n")

    over = idealized_car()
    over.cornering_stiffness_front = 130000.0
    over.cornering_stiffness_rear = 70000.0
    check(analysis.understeer_gradient(over) < 0.0, "oversteering K")
    v_cr = analysis.critical_speed(over)
    check(math.isfinite(v_cr), "finite critical speed")
    check(analysis.yaw_mode(over, 0.5 * v_cr).stable, "stable below V_cr")
    check(not analysis.yaw_mode(over, 1.3 * v_cr).stable, "unstable above V_cr")


def test_linear_model_against_closed_form():
    section("linear lateral model vs closed form")
    p = idealized_car()
    vx, delta = 15.0, deg2rad(2.0)

    model = LinearLateralBicycleModel(p, vx)
    x = simulate(model, lateral_state(0, 0, 0, 0, 0), steer_input(delta),
                 20.0, 1e-3)
    ss = analysis.steady_state_cornering(p, vx, delta)

    check_near(x[4], ss.yaw_rate, 1e-6, "yaw rate")
    check_near(x[3] / vx, ss.side_slip, 1e-6, "side slip")


def test_dynamic_bicycle():
    section("dynamic bicycle")
    p = idealized_car()
    model = DynamicBicycleModel(p, LinearTire(), LinearTire())
    model.longitudinal_load_transfer = False

    vx, delta = 15.0, deg2rad(2.0)
    x0 = dynamic_state(0, 0, 0, vx, 0, 0)
    x = simulate(model, x0, dynamic_input(0.0, delta), 20.0, 1e-3)

    ss = analysis.steady_state_cornering(p, float(x[3]), delta)
    check(vx > x[3] > 0.9 * vx, "cornering drag bleeds a little speed")
    check_near(x[5], ss.yaw_rate, 0.02 * abs(ss.yaw_rate), "yaw rate")
    check_near(side_slip_of(x), ss.side_slip, 1e-3, "side slip")

    d = model.derivative(dynamic_state(0, 0, 0, vx, 0, 0),
                         dynamic_input(0.0, 0.0))
    check_near(d[3], 0.0, 1e-9, "straight running vx_dot")
    check_near(d[4], 0.0, 1e-9, "straight running vy_dot")
    check_near(d[5], 0.0, 1e-9, "straight running r_dot")

    accel = model.derivative(dynamic_state(0, 0, 0, vx, 0, 0),
                             model.input_from_acceleration(1.0, 0.0))
    check_near(accel[3], 1.0, 1e-9, "commanded acceleration")

    loaded = DynamicBicycleModel(p, LinearTire(), LinearTire())
    f = loaded.compute_forces(dynamic_state(0, 0, 0, vx, 0, 0),
                              loaded.input_from_acceleration(-4.0, 0.0))
    check(f.fz_front > p.static_load_front(), "braking loads the front")
    check(f.fz_rear < p.static_load_rear(), "braking unloads the rear")
    check_near(f.fz_front + f.fz_rear, p.mass * GRAVITY, 1e-6, "load sum")

    d_stop = model.derivative(dynamic_state(0, 0, 0, 0, 0, 0),
                              dynamic_input(0.0, deg2rad(20.0)))
    check(all(math.isfinite(value) for value in d_stop),
          "finite derivative at standstill")

    nonlinear = DynamicBicycleModel(p, FialaTire(), FialaTire())
    hard = nonlinear.compute_forces(dynamic_state(0, 0, 0, 20.0, 0, 0),
                                    dynamic_input(0.0, deg2rad(30.0)))
    check(abs(hard.fy_front) <= p.friction * hard.fz_front + 1e-6,
          "tire cannot exceed mu*Fz")
    check(abs(hard.ay) <= analysis.max_lateral_acceleration(p) + 1e-6,
          "ay bounded by mu*g")


def test_blended_model():
    section("kinematic/dynamic blending")
    from vehicle_models_py import BlendedBicycleModel
    p = idealized_car()
    p.low_speed_guard = 1.0
    core = DynamicBicycleModel(p, LinearTire(), LinearTire())
    model = BlendedBicycleModel(core, blend_speed_low=1.0, blend_speed_high=4.0)

    check_near(model.blend_factor(0.5), 0.0, 1e-12, "blend low")
    check_near(model.blend_factor(5.0), 1.0, 1e-12, "blend high")
    check(0.0 < model.blend_factor(2.5) < 1.0, "blend mid")

    vx, delta = 0.5, deg2rad(15.0)
    x = simulate(model, dynamic_state(0, 0, 0, vx, 0, 0),
                 dynamic_input(0.0, delta), 3.0, 1e-3)
    r_kin = x[3] * math.tan(delta) / p.wheel_base()
    check_near(x[5], r_kin, 0.05 * abs(r_kin), "crawl follows kinematics")

    plain = DynamicBicycleModel(p, LinearTire(), LinearTire())
    s = dynamic_state(0, 0, 0, 20.0, 0.2, 0.1)
    u = dynamic_input(0.0, deg2rad(3.0))
    db_d = plain.derivative(s, u)
    bb_d = model.derivative(s, u)
    for i in range(6):
        check_near(bb_d[i], db_d[i], 1e-12, "identical above blend speed")


def test_double_track():
    section("double track")
    p = idealized_car()
    model = DoubleTrackModel(p, tire_front=FialaTire(), tire_rear=FialaTire())
    vx = 15.0

    stat = model.compute_forces(dynamic_state(0, 0, 0, vx, 0, 0),
                                dynamic_input(0.0, 0.0))
    check_near(stat.load_sum(), p.mass * GRAVITY, 1e-6, "static load sum")
    check_near(stat.normal_load[FL], stat.normal_load[FR], 1e-9, "even split")
    check_near(stat.normal_load[FL], 0.5 * p.static_load_front(), 1e-6,
               "front wheel load")

    braking = model.compute_forces(dynamic_state(0, 0, 0, vx, 0, 0),
                                   model.input_from_acceleration(-4.0, 0.0))
    check(braking.normal_load[FL] > stat.normal_load[FL], "front loads up")
    check(braking.normal_load[RL] < stat.normal_load[RL], "rear unloads")
    check_near(braking.load_sum(), p.mass * GRAVITY, 1e-6, "braking load sum")

    delta = deg2rad(4.0)
    turning = model.compute_forces(dynamic_state(0, 0, 0, vx, 0, 0),
                                   dynamic_input(0.0, delta))
    check(turning.ay > 0.0, "left turn gives positive ay")
    check(turning.normal_load[FR] > turning.normal_load[FL], "outer front loads")
    check(turning.normal_load[RR] > turning.normal_load[RL], "outer rear loads")
    check_near(turning.load_sum(), p.mass * GRAVITY, 1e-6, "turning load sum")
    check(turning.steer.left > turning.steer.right, "inner wheel steers more")

    single = DynamicBicycleModel(p, FialaTire(), FialaTire())
    x_dt = simulate(model, dynamic_state(0, 0, 0, vx, 0, 0),
                    dynamic_input(0.0, deg2rad(2.0)), 15.0, 1e-3)
    x_st = simulate(single, dynamic_state(0, 0, 0, vx, 0, 0),
                    dynamic_input(0.0, deg2rad(2.0)), 15.0, 1e-3)
    check_near(x_dt[5], x_st[5], 0.05 * abs(x_st[5]),
               "double and single track agree below the limit")

    limit = model.compute_forces(dynamic_state(0, 0, 0, 30.0, 0, 0),
                                 dynamic_input(0.0, deg2rad(30.0)))
    check(abs(limit.ay) <= p.friction * GRAVITY + 1e-6, "ay stays bounded")

    combined = DoubleTrackModel(p, tire_front=FialaTire(), tire_rear=FialaTire())
    combined.dt_params.combined_slip = True
    brake_turn = combined.compute_forces(
        dynamic_state(0, 0, 0, vx, 0, 0),
        combined.input_from_acceleration(-6.0, delta))
    combined.dt_params.combined_slip = False
    pure_turn = combined.compute_forces(
        dynamic_state(0, 0, 0, vx, 0, 0),
        combined.input_from_acceleration(-6.0, delta))
    check(abs(brake_turn.ay) < abs(pure_turn.ay),
          "combined slip reduces lateral force")


def test_parameter_validation():
    section("parameter validation")
    check(not make_shuttle_parameters().validate(), "shuttle preset valid")
    check(not make_buggy_parameters().validate(), "buggy preset valid")

    bad = VehicleParameters()
    bad.mass = -1.0
    bad.ackermann_ratio = 2.0
    bad.steer_max = deg2rad(120.0)
    check(len(bad.validate()) == 3, "three violations detected")


def test_integrator_order():
    section("integrator convergence order")
    model = UnicycleModel()
    v, w = 4.0, 0.8
    duration = 0.5 * PI / w  # quarter circle
    radius = v / w
    exact = np.array([radius * math.sin(w * duration),
                      radius * (1.0 - math.cos(w * duration))])

    def error(method, dt):
        x = simulate(model, unicycle_state(), unicycle_input(v, w), duration,
                     dt, method)
        return float(np.hypot(x[0] - exact[0], x[1] - exact[1]))

    for method, factor in ((IntegratorType.EULER, 2.0),
                           (IntegratorType.HEUN, 4.0),
                           (IntegratorType.RK4, 16.0)):
        coarse = error(method, 0.02)
        fine = error(method, 0.01)
        if fine < 1e-13:
            continue  # already at machine precision
        ratio = coarse / fine
        check(0.7 * factor < ratio < 1.4 * factor,
              "%s ratio %.2f (expected ~%.0f)" % (method.value, ratio, factor))

    # Euler is the least accurate, RK4 the most.
    e_euler = error(IntegratorType.EULER, 0.01)
    e_heun = error(IntegratorType.HEUN, 0.01)
    e_rk4 = error(IntegratorType.RK4, 0.01)
    check(e_euler > e_heun > e_rk4, "accuracy ordering")


# ---------------------------------------------------------------------------
# simulation infrastructure
# ---------------------------------------------------------------------------

def test_runner_and_maneuvers():
    section("runner")
    from vehicle_models_py.maneuvers import ManeuverConfig, STEP_STEER
    from vehicle_models_py.runner import run_maneuver, to_csv

    p = make_passenger_car_parameters()
    cfg = ManeuverConfig(kind=STEP_STEER, duration=4.0, dt=0.005,
                         initial_speed=20.0, steer_amplitude=deg2rad(3.0),
                         t_start=0.5, hold_speed=True)
    results = run_maneuver(p, cfg, ["kin_cog", "dynamic", "double_track"],
                           tire_kind="Linear")
    check(len(results) == 3, "three models ran")

    by_key = {r.key: r for r in results}
    ss = analysis.steady_state_cornering(p, cfg.initial_speed,
                                         cfg.steer_amplitude)
    # The dynamic model must land on the closed-form steady state.
    check_near(by_key["dynamic"].summary["r_final"], ss.yaw_rate,
               0.05 * abs(ss.yaw_rate), "dynamic reaches closed form")
    # The kinematic model over-predicts the yaw rate at this speed.
    check(by_key["kin_cog"].summary["r_final"]
          > 1.3 * by_key["dynamic"].summary["r_final"],
          "kinematic over-predicts yaw rate at 20 m/s")
    # Speed hold keeps the run at the requested operating point.
    check(abs(by_key["dynamic"].summary["v_final"] - 20.0) < 0.2,
          "speed controller holds 20 m/s")

    csv = to_csv(results)
    check(csv.count("\n") == len(results[0].time) + 1, "csv row count")


def test_performance_module():
    section("performance")
    from vehicle_models_py.performance import (acceleration_run, braking_run,
                                               ramp_steer_run)
    p = make_passenger_car_parameters()

    acc = acceleration_run(p, "Fiala", four_wheel=False, dt=0.02)
    check(acc.metrics["v_reached"] > 0.9 * p.speed_max, "reaches near v_max")
    check(acc.metrics["t_total"] > 0.0, "acceleration takes time")
    # Resistance means the achieved ax is below the commanded one.
    check(acc.accel[-1] < p.accel_max, "drag reduces the achieved ax")

    brake = braking_run(p, 20.0, "Fiala", four_wheel=False, dt=0.005)
    ideal = 20.0 ** 2 / (2.0 * abs(p.accel_min))
    check(brake.metrics["s_stop"] < ideal,
          "resistance shortens the stop vs the ideal %.1f m" % ideal)
    check(brake.metrics["s_stop"] > 0.7 * ideal, "stopping distance plausible")

    ramp = ramp_steer_run(p, 20.0, "Fiala", four_wheel=True, ramp_rate=0.02,
                          duration=12.0, dt=0.01)
    check(ramp.metrics["ay_max"] <= p.friction * GRAVITY + 1e-6,
          "limit ay bounded by mu*g")
    check(ramp.metrics["ay_max"] > 0.6 * p.friction * GRAVITY,
          "limit ay reaches a decent fraction of mu*g")
    if "k_measured" in ramp.metrics:
        k_lin = ramp.metrics["k_linear"]
        check(abs(ramp.metrics["k_measured"] - k_lin) < 0.35 * abs(k_lin),
              "measured understeer gradient matches the closed form")


def main():
    tests = [
        test_unicycle, test_differential_drive, test_ackermann_geometry,
        test_wheel_speeds, test_kinematic_bicycle, test_steering_dynamics,
        test_tire_models, test_linear_analysis,
        test_linear_model_against_closed_form, test_dynamic_bicycle,
        test_blended_model, test_double_track, test_parameter_validation,
        test_integrator_order, test_runner_and_maneuvers,
        test_performance_module,
    ]
    for fn in tests:
        fn()

    print("checks run : %d" % _CHECKS)
    if _FAILURES:
        print("FAILED     : %d" % len(_FAILURES))
        for line in _FAILURES:
            print("  " + line)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
