// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#include "test_support.hpp"
#include "vehicle_models/vehicle_models.hpp"

using namespace vehicle_models;

namespace {

VehicleParameters idealizedCar() {
  VehicleParameters p = makePassengerCarParameters();
  p.drag_area = 0.0;
  p.rolling_resistance = 0.0;
  return p;
}

void testTireModels() {
  vmtest::section("tire models");
  const double fz = 4000.0;
  const double c = 60000.0;

  tire::LinearTire lin;
  lin.cornering_stiffness = c;
  lin.friction = 1.0;
  VM_CHECK_NEAR(lin.lateralForce(0.0, fz), 0.0, 1e-12);
  VM_CHECK_NEAR(lin.lateralForce(0.01, fz), c * 0.01, 1e-9);
  VM_CHECK_NEAR(lin.lateralForce(1.0, fz), fz, 1e-9);          // saturated
  VM_CHECK_NEAR(lin.lateralForce(-1.0, fz), -fz, 1e-9);

  tire::FialaTire fiala;
  fiala.cornering_stiffness = c;
  fiala.friction = 1.0;
  // Small slip -> same slope as the linear tire.
  VM_CHECK_NEAR(fiala.lateralForce(1e-4, fz) / 1e-4, c, c * 1e-3);
  // Monotone up to the sliding angle, then constant at mu*Fz.
  VM_CHECK(fiala.lateralForce(0.05, fz) > fiala.lateralForce(0.02, fz));
  VM_CHECK_NEAR(fiala.lateralForce(0.6, fz), fz, 1e-9);
  VM_CHECK(std::fabs(fiala.lateralForce(0.08, fz)) <= fz + 1e-9);
  // Below the linear tire in the transition region (progressive saturation).
  VM_CHECK(fiala.lateralForce(0.04, fz) < lin.lateralForce(0.04, fz));

  auto pac = tire::PacejkaTire::fromCorneringStiffness(c, fz, 1.0);
  VM_CHECK_NEAR(pac.lateralForce(1e-5, fz) / 1e-5, c, c * 1e-3);
  VM_CHECK_NEAR(pac.corneringStiffness(fz), c, c * 1e-9);
  // Magic formula peaks above mu*Fz*0.9 and falls back after the peak.
  const double peak = pac.lateralForce(0.15, fz);
  VM_CHECK(peak > 0.9 * fz);
  VM_CHECK(pac.lateralForce(0.6, fz) < peak);

  // Friction ellipse.
  VM_CHECK_NEAR(tire::frictionEllipseScale(0.0, 1.0, fz), 1.0, 1e-12);
  VM_CHECK_NEAR(tire::frictionEllipseScale(fz, 1.0, fz), 0.0, 1e-12);
  VM_CHECK_NEAR(tire::frictionEllipseScale(0.6 * fz, 1.0, fz), 0.8, 1e-12);
}

void testLinearAnalysis() {
  vmtest::section("linear handling analysis");
  const VehicleParameters p = idealizedCar();

  // Default set is understeering (rear stiffer than front).
  const double k = analysis::understeerGradient(p);
  VM_CHECK(k > 0.0);
  VM_CHECK(analysis::staticMargin(p) > 0.0);
  VM_CHECK(std::isfinite(analysis::characteristicSpeed(p)));
  VM_CHECK(!std::isfinite(analysis::criticalSpeed(p)));

  // At the characteristic speed the yaw rate gain is half the neutral value.
  const double v_ch = analysis::characteristicSpeed(p);
  VM_CHECK_NEAR(analysis::yawRateGain(p, v_ch), 0.5 * v_ch / p.wheelBase(), 1e-9);

  // Steer angle needed for a radius is Ackermann + K * a_y.
  const double radius = 50.0;
  const double vx = 15.0;
  const double delta_req = analysis::requiredSteerAngle(p, radius, vx);
  VM_CHECK(delta_req > p.wheelBase() / radius);
  const auto ss = analysis::steadyStateCornering(p, vx, delta_req);
  VM_CHECK_NEAR(ss.radius, radius, 1e-6);
  VM_CHECK_NEAR(ss.lateral_accel, vx * vx / radius, 1e-6);

  // Understeer means the front axle works at a larger slip angle.
  VM_CHECK(std::fabs(ss.slip_front) > std::fabs(ss.slip_rear));

  // Stable at every speed when understeering.
  VM_CHECK(analysis::yawMode(p, 5.0).stable);
  VM_CHECK(analysis::yawMode(p, 40.0).stable);
  VM_CHECK(analysis::yawMode(p, 20.0).natural_frequency > 0.0);

  // Oversteering variant: finite critical speed, unstable above it.
  VehicleParameters over = p;
  over.cornering_stiffness_front = 130000.0;
  over.cornering_stiffness_rear = 70000.0;
  VM_CHECK(analysis::understeerGradient(over) < 0.0);
  const double v_cr = analysis::criticalSpeed(over);
  VM_CHECK(std::isfinite(v_cr));
  VM_CHECK(analysis::yawMode(over, 0.5 * v_cr).stable);
  VM_CHECK(!analysis::yawMode(over, 1.3 * v_cr).stable);
}

void testLinearModelAgainstClosedForm() {
  vmtest::section("linear lateral model vs closed form");
  const VehicleParameters p = idealizedCar();
  const double vx = 15.0;
  const double delta = deg2rad(2.0);

  LinearLateralBicycleModel model(p, vx);
  const auto x = simulate(model, LateralBicycleState::make(0, 0, 0, 0, 0),
                          SteerInput::make(delta), 20.0, 1e-3);
  const auto ss = analysis::steadyStateCornering(p, vx, delta);

  VM_CHECK_NEAR(x.yawRate(), ss.yaw_rate, 1e-6);
  VM_CHECK_NEAR(x.vy() / vx, ss.side_slip, 1e-6);
}

void testDynamicBicycle() {
  vmtest::section("dynamic bicycle");
  const VehicleParameters p = idealizedCar();
  DynamicBicycleModel<tire::LinearTire> model(p);
  model.longitudinal_load_transfer = false;

  const double vx = 15.0;
  const double delta = deg2rad(2.0);
  const auto x0 = DynamicBicycleState::make(0, 0, 0, vx, 0, 0);
  const auto x = simulate(model, x0, DynamicBicycleInput::make(0.0, delta), 20.0, 1e-3);

  // With Fx = 0 the cornering drag slowly bleeds off speed, so the closed form
  // is evaluated at the speed the model actually reached.
  const auto ss = analysis::steadyStateCornering(p, x.vx(), delta);
  VM_CHECK(x.vx() < vx && x.vx() > 0.9 * vx);
  VM_CHECK_NEAR(x.yawRate(), ss.yaw_rate, 0.02 * std::fabs(ss.yaw_rate));
  VM_CHECK_NEAR(x.sideSlip(), ss.side_slip, 1e-3);

  // Straight running is an equilibrium.
  const auto d = model.derivative(DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
                                  DynamicBicycleInput::make(0.0, 0.0));
  VM_CHECK_NEAR(d[3], 0.0, 1e-9);
  VM_CHECK_NEAR(d[4], 0.0, 1e-9);
  VM_CHECK_NEAR(d[5], 0.0, 1e-9);

  // Longitudinal force produces the expected acceleration.
  const auto accel = model.derivative(DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
                                      model.inputFromAcceleration(1.0, 0.0));
  VM_CHECK_NEAR(accel[3], 1.0, 1e-9);

  // Load transfer under braking moves load to the front axle.
  DynamicBicycleModel<tire::LinearTire> loaded(p);
  const auto f = loaded.computeForces(DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
                                      loaded.inputFromAcceleration(-4.0, 0.0));
  VM_CHECK(f.fz_front > p.staticLoadFront());
  VM_CHECK(f.fz_rear < p.staticLoadRear());
  VM_CHECK_NEAR(f.fz_front + f.fz_rear, p.mass * kGravity, 1e-6);

  // Guarded slip angle keeps the model finite at standstill.
  const auto d_stop = model.derivative(DynamicBicycleState::make(0, 0, 0, 0, 0, 0),
                                       DynamicBicycleInput::make(0.0, deg2rad(20.0)));
  for (std::size_t i = 0; i < DynamicBicycleState::kDim; ++i) {
    VM_CHECK(std::isfinite(d_stop[i]));
  }

  // A saturating tire cannot exceed mu * Fz.
  DynamicBicycleModel<tire::FialaTire> nonlinear(p);
  const auto hard = nonlinear.computeForces(
      DynamicBicycleState::make(0, 0, 0, 20.0, 0, 0),
      DynamicBicycleInput::make(0.0, deg2rad(30.0)));
  VM_CHECK(std::fabs(hard.fy_front) <= p.friction * hard.fz_front + 1e-6);
  VM_CHECK(std::fabs(hard.ay) <= analysis::maxLateralAcceleration(p) + 1e-6);
}

void testBlendedModel() {
  vmtest::section("kinematic/dynamic blending");
  VehicleParameters p = idealizedCar();
  p.low_speed_guard = 1.0;
  BlendedBicycleModel<tire::LinearTire> model(p);
  model.blend_speed_low = 1.0;
  model.blend_speed_high = 4.0;

  VM_CHECK_NEAR(model.blendFactor(0.5), 0.0, 1e-12);
  VM_CHECK_NEAR(model.blendFactor(5.0), 1.0, 1e-12);
  VM_CHECK(model.blendFactor(2.5) > 0.0 && model.blendFactor(2.5) < 1.0);

  // At crawl speed the yaw rate follows the kinematic prediction.
  const double vx = 0.5;
  const double delta = deg2rad(15.0);
  const auto x = simulate(model, DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
                          DynamicBicycleInput::make(0.0, delta), 3.0, 1e-3);
  const double r_kin = x.vx() * std::tan(delta) / p.wheelBase();
  VM_CHECK_NEAR(x.yawRate(), r_kin, 0.05 * std::fabs(r_kin));

  // At high speed it is the plain dynamic model again.
  DynamicBicycleModel<tire::LinearTire> plain(p);
  const auto s = DynamicBicycleState::make(0, 0, 0, 20.0, 0.2, 0.1);
  const auto u = DynamicBicycleInput::make(0.0, deg2rad(3.0));
  const auto db = plain.derivative(s, u);
  const auto bb = model.derivative(s, u);
  for (std::size_t i = 0; i < DynamicBicycleState::kDim; ++i) {
    VM_CHECK_NEAR(bb[i], db[i], 1e-12);
  }
}

void testDoubleTrack() {
  vmtest::section("double track");
  VehicleParameters p = idealizedCar();
  DoubleTrackModel<tire::FialaTire> model(p);

  const double vx = 15.0;

  // Static: axle loads split evenly, total equals the vehicle weight.
  const auto stat = model.computeForces(DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
                                        DynamicBicycleInput::make(0.0, 0.0));
  VM_CHECK_NEAR(stat.normal_load.sum(), p.mass * kGravity, 1e-6);
  VM_CHECK_NEAR(stat.normal_load[WheelIndex::FrontLeft],
                stat.normal_load[WheelIndex::FrontRight], 1e-9);
  VM_CHECK_NEAR(stat.normal_load[WheelIndex::FrontLeft],
                0.5 * p.staticLoadFront(), 1e-6);

  // Braking: load moves forward.
  const auto braking = model.computeForces(
      DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
      model.inputFromAcceleration(-4.0, 0.0));
  VM_CHECK(braking.normal_load[WheelIndex::FrontLeft] >
           stat.normal_load[WheelIndex::FrontLeft]);
  VM_CHECK(braking.normal_load[WheelIndex::RearLeft] <
           stat.normal_load[WheelIndex::RearLeft]);
  VM_CHECK_NEAR(braking.normal_load.sum(), p.mass * kGravity, 1e-6);

  // Left turn: load moves to the right (outer) wheels, inner wheels unload.
  const double delta = deg2rad(4.0);
  const auto turning = model.computeForces(
      DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
      DynamicBicycleInput::make(0.0, delta));
  VM_CHECK(turning.ay > 0.0);
  VM_CHECK(turning.normal_load[WheelIndex::FrontRight] >
           turning.normal_load[WheelIndex::FrontLeft]);
  VM_CHECK(turning.normal_load[WheelIndex::RearRight] >
           turning.normal_load[WheelIndex::RearLeft]);
  VM_CHECK_NEAR(turning.normal_load.sum(), p.mass * kGravity, 1e-6);

  // Ackermann: the inner front wheel runs at the larger steer angle.
  VM_CHECK(turning.steer.left > turning.steer.right);

  // Below the limit the double track and the single track agree closely.
  DynamicBicycleModel<tire::FialaTire> single(p);
  const auto x_dt = simulate(model, DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
                             DynamicBicycleInput::make(0.0, deg2rad(2.0)), 15.0, 1e-3);
  const auto x_st = simulate(single, DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
                             DynamicBicycleInput::make(0.0, deg2rad(2.0)), 15.0, 1e-3);
  VM_CHECK_NEAR(x_dt.yawRate(), x_st.yawRate(), 0.05 * std::fabs(x_st.yawRate()));

  // Beyond the friction limit the lateral acceleration stays bounded.
  const auto limit = model.computeForces(
      DynamicBicycleState::make(0, 0, 0, 30.0, 0, 0),
      DynamicBicycleInput::make(0.0, deg2rad(30.0)));
  VM_CHECK(std::fabs(limit.ay) <= p.friction * kGravity + 1e-6);

  // Combined slip: hard braking reduces the available lateral force.
  DoubleTrackModel<tire::FialaTire> combined(p);
  combined.dt_params.combined_slip = true;
  const auto brake_turn = combined.computeForces(
      DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
      combined.inputFromAcceleration(-6.0, delta));
  combined.dt_params.combined_slip = false;
  const auto pure_turn = combined.computeForces(
      DynamicBicycleState::make(0, 0, 0, vx, 0, 0),
      combined.inputFromAcceleration(-6.0, delta));
  VM_CHECK(std::fabs(brake_turn.ay) < std::fabs(pure_turn.ay));
}

void testParameterValidation() {
  vmtest::section("parameter validation");
  VM_CHECK(makeShuttleParameters().validate().empty());
  VM_CHECK(makeBuggyParameters().validate().empty());

  VehicleParameters bad;
  bad.mass = -1.0;
  bad.ackermann_ratio = 2.0;
  bad.steer_max = deg2rad(120.0);
  VM_CHECK(bad.validate().size() == 3u);
}

}  // namespace

int main() {
  testTireModels();
  testLinearAnalysis();
  testLinearModelAgainstClosedForm();
  testDynamicBicycle();
  testBlendedModel();
  testDoubleTrack();
  testParameterValidation();
  return vmtest::summary("test_dynamics");
}
