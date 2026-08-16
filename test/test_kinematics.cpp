// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#include "test_support.hpp"
#include "vehicle_models/vehicle_models.hpp"

using namespace vehicle_models;

namespace {

void testUnicycle() {
  vmtest::section("unicycle");
  UnicycleModel model;
  const auto x0 = UnicycleState::make(0.0, 0.0, 0.0);

  const auto straight = simulate(model, x0, UnicycleInput::make(2.0, 0.0), 3.0, 0.001);
  VM_CHECK_NEAR(straight.x(), 6.0, 1e-6);
  VM_CHECK_NEAR(straight.y(), 0.0, 1e-9);

  // Constant v and omega -> circle of radius v/omega, closed after 2*pi/omega.
  const double v = 3.0, w = 0.5;
  const auto circle = simulate(model, x0, UnicycleInput::make(v, w), 2.0 * kPi / w, 1e-4);
  VM_CHECK_NEAR(circle.x(), 0.0, 1e-4);
  VM_CHECK_NEAR(circle.y(), 0.0, 1e-4);

  // Turn in place.
  const auto spin = simulate(model, x0, UnicycleInput::make(0.0, 1.0), 1.0, 1e-3);
  VM_CHECK_NEAR(spin.yaw(), 1.0, 1e-9);
  VM_CHECK_NEAR(spin.x(), 0.0, 1e-12);
}

void testDifferentialDrive() {
  vmtest::section("differential drive");
  DifferentialDriveParams p;
  p.wheel_radius = 0.20;
  p.track = 0.60;
  DifferentialDriveModel model(p);

  const auto wheels = model.toWheelRates(1.5, 0.8);
  const auto body = model.toBodyVelocity(wheels);
  VM_CHECK_NEAR(body.v(), 1.5, 1e-12);
  VM_CHECK_NEAR(body.omega(), 0.8, 1e-12);

  // Equal wheel rates -> straight, opposite rates -> pure rotation.
  const auto equal = model.toBodyVelocity(WheelRateInput::make(5.0, 5.0));
  VM_CHECK_NEAR(equal.omega(), 0.0, 1e-12);
  const auto opposite = model.toBodyVelocity(WheelRateInput::make(-5.0, 5.0));
  VM_CHECK_NEAR(opposite.v(), 0.0, 1e-12);

  const auto x = simulate(model, UnicycleState::make(0, 0, 0),
                          WheelRateInput::make(5.0, 5.0), 2.0, 1e-3);
  VM_CHECK_NEAR(x.x(), 2.0 * p.wheel_radius * 5.0, 1e-9);
}

void testAckermannGeometry() {
  vmtest::section("ackermann geometry");
  AckermannGeometry g;
  g.wheel_base = 2.70;
  g.track_front = 1.55;
  g.track_rear = 1.55;
  g.ackermann_ratio = 1.0;

  const double delta = deg2rad(20.0);  // left turn
  const auto wa = roadWheelAngles(g, delta);

  // Inner (left) wheel is steered more than the outer one.
  VM_CHECK(wa.left > delta);
  VM_CHECK(wa.right < delta);

  // Ideal Ackermann condition: cot(outer) - cot(inner) = T / L
  const double cot_out = 1.0 / std::tan(wa.right);
  const double cot_in = 1.0 / std::tan(wa.left);
  VM_CHECK_NEAR(cot_out - cot_in, g.track_front / g.wheel_base, 1e-9);

  // Ideal geometry has zero Ackermann error, parallel steering does not.
  VM_CHECK_NEAR(ackermannError(g, delta), 0.0, 1e-9);
  AckermannGeometry parallel = g;
  parallel.ackermann_ratio = 0.0;
  const auto wp = roadWheelAngles(parallel, delta);
  VM_CHECK_NEAR(wp.left, delta, 1e-12);
  VM_CHECK_NEAR(wp.right, delta, 1e-12);
  VM_CHECK(ackermannError(parallel, delta) > 0.0);

  // Round trip through the equivalent bicycle angle.
  VM_CHECK_NEAR(bicycleAngleFromWheels(g, wa.left, wa.right), delta, 1e-9);
  VM_CHECK_NEAR(bicycleAngleFromWheels(g, 0.0, 0.0), 0.0, 1e-12);

  // Straight ahead and right turn.
  const auto zero = roadWheelAngles(g, 0.0);
  VM_CHECK_NEAR(zero.left, 0.0, 1e-12);
  VM_CHECK_NEAR(zero.right, 0.0, 1e-12);
  const auto right = roadWheelAngles(g, -delta);
  VM_CHECK_NEAR(right.left, -wa.right, 1e-12);
  VM_CHECK_NEAR(right.right, -wa.left, 1e-12);

  // Radius round trip and handwheel conversion.
  const double radius = turnRadius(g, delta);
  VM_CHECK_NEAR(steerAngleForRadius(g, radius), delta, 1e-12);
  VM_CHECK(!std::isfinite(turnRadius(g, 0.0)));
  g.steering_ratio = 16.0;
  VM_CHECK_NEAR(handwheelToRoadWheel(g, roadWheelToHandwheel(g, delta)), delta, 1e-12);

  // Minimum turn radius is larger than the wheelbase and shrinks with lock.
  VM_CHECK(minimumTurnRadius(g, deg2rad(35.0)) > g.wheel_base);
  VM_CHECK(minimumTurnRadius(g, deg2rad(35.0)) < minimumTurnRadius(g, deg2rad(20.0)));
}

void testWheelSpeeds() {
  vmtest::section("wheel speeds");
  AckermannGeometry g;
  g.wheel_base = 2.70;
  g.track_front = 1.55;
  g.track_rear = 1.55;

  const auto straight = wheelSpeeds(g, 10.0, 0.0);
  VM_CHECK_NEAR(straight.rear_left, 10.0, 1e-12);
  VM_CHECK_NEAR(straight.rear_right, 10.0, 1e-12);
  VM_CHECK_NEAR(straight.front_left, 10.0, 1e-12);
  VM_CHECK_NEAR(straight.front_right, 10.0, 1e-12);

  // Left turn: the right (outer) wheels run faster, rear average stays v.
  const double v = 10.0;
  const double yaw_rate = 0.3;
  const auto turning = wheelSpeeds(g, v, yaw_rate);
  VM_CHECK(turning.rear_right > turning.rear_left);
  VM_CHECK(turning.front_right > turning.front_left);
  VM_CHECK_NEAR(0.5 * (turning.rear_left + turning.rear_right), v, 1e-12);

  const auto rates = wheelAngularRates(turning, 0.32);
  VM_CHECK_NEAR(rates.rear_left * 0.32, turning.rear_left, 1e-12);
}

void testKinematicBicycle() {
  vmtest::section("kinematic bicycle");
  VehicleParameters p = makePassengerCarParameters();
  VM_CHECK(p.validate().empty());

  KinematicBicycleModel rear(p, ReferencePoint::RearAxle);
  const double v = 8.0;
  const double delta = deg2rad(5.0);

  // Yaw rate must match v * tan(delta) / L.
  const auto x1 = simulate(rear, KinematicBicycleState::make(0, 0, 0, v),
                           KinematicBicycleInput::make(0.0, delta), 1.0, 1e-4);
  VM_CHECK_NEAR(x1.yaw(), v * std::tan(delta) / p.wheelBase(), 1e-6);
  VM_CHECK_NEAR(x1.v(), v, 1e-12);

  // Driving a full circle returns to the start.
  const double radius = p.wheelBase() / std::tan(delta);
  const double period = 2.0 * kPi * radius / v;
  const auto loop = simulate(rear, KinematicBicycleState::make(0, 0, 0, v),
                             KinematicBicycleInput::make(0.0, delta), period, 1e-4);
  VM_CHECK_NEAR(loop.x(), 0.0, 1e-3);
  VM_CHECK_NEAR(loop.y(), 0.0, 1e-3);

  // CoG reference: with v_cg = v_rear / cos(beta) both give the same yaw rate.
  KinematicBicycleModel cog(p, ReferencePoint::CenterOfGravity);
  const double beta = cog.sideSlip(delta);
  const auto d_rear = rear.derivative(KinematicBicycleState::make(0, 0, 0, v),
                                      KinematicBicycleInput::make(0.0, delta));
  const auto d_cog = cog.derivative(KinematicBicycleState::make(0, 0, 0, v / std::cos(beta)),
                                    KinematicBicycleInput::make(0.0, delta));
  VM_CHECK_NEAR(d_cog[2], d_rear[2], 1e-12);
  VM_CHECK(beta > 0.0);

  // Front axle reference: yaw rate v * sin(delta) / L.
  KinematicBicycleModel front(p, ReferencePoint::FrontAxle);
  const auto d_front = front.derivative(KinematicBicycleState::make(0, 0, 0, v),
                                        KinematicBicycleInput::make(0.0, delta));
  VM_CHECK_NEAR(d_front[2], v * std::sin(delta) / p.wheelBase(), 1e-12);

  // Acceleration and limits.
  const auto accelerated = simulate(rear, KinematicBicycleState::make(0, 0, 0, 0),
                                    KinematicBicycleInput::make(1.5, 0.0), 4.0, 1e-3);
  VM_CHECK_NEAR(accelerated.v(), 6.0, 1e-9);
  const auto clamped = rear.derivative(KinematicBicycleState::make(0, 0, 0, 0),
                                       KinematicBicycleInput::make(100.0, deg2rad(90.0)));
  VM_CHECK_NEAR(clamped[3], p.accel_max, 1e-12);

  // Lateral acceleration helper.
  VM_CHECK_NEAR(rear.lateralAcceleration(KinematicBicycleState::make(0, 0, 0, v), delta),
                v * v / radius, 1e-9);
}

void testSteeringDynamics() {
  vmtest::section("steering actuator dynamics");
  VehicleParameters p = makePassengerCarParameters();
  p.steer_time_constant = 0.10;
  p.steer_rate_max = deg2rad(30.0);
  KinematicBicycleSteerModel model(p);

  const double cmd = deg2rad(10.0);
  const auto x0 = SteerDynamicsState::make(0, 0, 0, 5.0, 0.0);

  // Rate limited right after the step.
  const auto d0 = model.derivative(x0, KinematicBicycleInput::make(0.0, cmd));
  VM_CHECK_NEAR(d0[4], p.steer_rate_max, 1e-12);

  // Converges to the command.
  const auto x1 = simulate(model, x0, KinematicBicycleInput::make(0.0, cmd), 2.0, 1e-3);
  VM_CHECK_NEAR(x1.steer(), cmd, 1e-6);

  // Never exceeds the mechanical limit.
  const auto x2 = simulate(model, x0,
                           KinematicBicycleInput::make(0.0, deg2rad(80.0)), 3.0, 1e-3);
  VM_CHECK(std::fabs(x2.steer()) <= p.steer_max + 1e-12);
}

}  // namespace

int main() {
  testUnicycle();
  testDifferentialDrive();
  testAckermannGeometry();
  testWheelSpeeds();
  testKinematicBicycle();
  testSteeringDynamics();
  return vmtest::summary("test_kinematics");
}
