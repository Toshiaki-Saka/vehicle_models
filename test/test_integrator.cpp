// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#include "test_support.hpp"
#include "vehicle_models/vehicle_models.hpp"

using namespace vehicle_models;

namespace {

/// Position error after a quarter circle, for a given step size and method.
double circleError(double dt, IntegratorType method) {
  UnicycleModel model;
  const double v = 4.0, w = 0.8;
  const double radius = v / w;
  const double duration = 0.5 * kPi / w;  // quarter turn
  const auto x = simulate(model, UnicycleState::make(0, 0, 0),
                          UnicycleInput::make(v, w), duration, dt, method);
  // Exact solution: centre at (0, R), quarter turn ends at (R, R).
  return std::hypot(x.x() - radius, x.y() - radius);
}

void testConvergenceOrder() {
  vmtest::section("integrator convergence order");

  const double e_euler_1 = circleError(0.02, IntegratorType::Euler);
  const double e_euler_2 = circleError(0.01, IntegratorType::Euler);
  const double ratio_euler = e_euler_1 / e_euler_2;
  VM_CHECK_NEAR(ratio_euler, 2.0, 0.3);  // 1st order

  const double e_heun_1 = circleError(0.02, IntegratorType::Heun);
  const double e_heun_2 = circleError(0.01, IntegratorType::Heun);
  VM_CHECK_NEAR(e_heun_1 / e_heun_2, 4.0, 0.6);  // 2nd order

  const double e_rk4_1 = circleError(0.05, IntegratorType::RK4);
  const double e_rk4_2 = circleError(0.025, IntegratorType::RK4);
  VM_CHECK_NEAR(e_rk4_1 / e_rk4_2, 16.0, 3.0);  // 4th order

  // RK4 is far more accurate than Euler at the same step size.
  VM_CHECK(circleError(0.02, IntegratorType::RK4) <
           1e-4 * circleError(0.02, IntegratorType::Euler));
}

void testStateVectorArithmetic() {
  vmtest::section("state vector arithmetic");
  auto a = KinematicBicycleState::make(1.0, 2.0, 3.0, 4.0);
  const auto b = KinematicBicycleState::make(0.5, 0.5, 0.5, 0.5);

  const auto sum = a + b;
  VM_CHECK_NEAR(sum.x(), 1.5, 1e-12);
  const auto diff = a - b;
  VM_CHECK_NEAR(diff.v(), 3.5, 1e-12);
  const auto scaled = 2.0 * a;
  VM_CHECK_NEAR(scaled.yaw(), 6.0, 1e-12);
  VM_CHECK_NEAR((a * 0.5).y(), 1.0, 1e-12);
  a += b;
  VM_CHECK_NEAR(a.x(), 1.5, 1e-12);
  VM_CHECK(KinematicBicycleState::kDim == 4u);
}

void testAngleHelpers() {
  vmtest::section("angle helpers");
  VM_CHECK_NEAR(normalizeAngle(3.0 * kPi), -kPi, 1e-12);   // wrapped to [-pi, pi)
  VM_CHECK_NEAR(normalizeAngle(-3.0 * kPi), -kPi, 1e-12);
  VM_CHECK_NEAR(normalizeAngle(2.0 * kPi + 0.25), 0.25, 1e-12);
  VM_CHECK_NEAR(normalizeAngle(0.5), 0.5, 1e-12);
  VM_CHECK_NEAR(rad2deg(deg2rad(37.0)), 37.0, 1e-12);
  VM_CHECK_NEAR(guardDenominator(0.0, 0.5), 0.5, 1e-12);
  VM_CHECK_NEAR(guardDenominator(-0.1, 0.5), -0.5, 1e-12);
  VM_CHECK_NEAR(guardDenominator(3.0, 0.5), 3.0, 1e-12);
  VM_CHECK_NEAR(clampValue(5.0, -1.0, 1.0), 1.0, 1e-12);

  // Yaw stays wrapped over a long simulation.
  UnicycleModel model;
  const auto x = simulate(model, UnicycleState::make(0, 0, 0),
                          UnicycleInput::make(1.0, 2.0), 30.0, 1e-3);
  VM_CHECK(std::fabs(x.yaw()) <= kPi + 1e-12);
}

}  // namespace

int main() {
  testConvergenceOrder();
  testStateVectorArithmetic();
  testAngleHelpers();
  return vmtest::summary("test_integrator");
}
