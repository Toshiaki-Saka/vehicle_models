// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_UNICYCLE_HPP
#define VEHICLE_MODELS_UNICYCLE_HPP

#include "vehicle_models/types.hpp"

namespace vehicle_models {

/// [x, y, yaw]
struct UnicycleState : StateVector<UnicycleState, 3> {
  double& x() { return data[0]; }
  double& y() { return data[1]; }
  double& yaw() { return data[2]; }
  double x() const { return data[0]; }
  double y() const { return data[1]; }
  double yaw() const { return data[2]; }

  static UnicycleState make(double x, double y, double yaw) {
    UnicycleState s;
    s.data = {x, y, yaw};
    return s;
  }
  Pose2D pose() const { return Pose2D{x(), y(), yaw()}; }
};

/// [v, omega]
struct UnicycleInput : StateVector<UnicycleInput, 2> {
  double& v() { return data[0]; }
  double& omega() { return data[1]; }
  double v() const { return data[0]; }
  double omega() const { return data[1]; }

  static UnicycleInput make(double v, double omega) {
    UnicycleInput u;
    u.data = {v, omega};
    return u;
  }
};

/**
 * @brief Unicycle (differential-drive body) model.
 *
 *   x_dot   = v * cos(yaw)
 *   y_dot   = v * sin(yaw)
 *   yaw_dot = omega
 *
 * No non-holonomic steering constraint on the turn radius, so it can rotate
 * in place. Correct for skid-steer platforms, and the usual planner-side
 * abstraction; it is NOT a valid model for an Ackermann-steered vehicle.
 */
struct UnicycleModel {
  using State = UnicycleState;
  using Input = UnicycleInput;

  State derivative(const State& s, const Input& u) const {
    State d;
    d.data[0] = u.v() * std::cos(s.yaw());
    d.data[1] = u.v() * std::sin(s.yaw());
    d.data[2] = u.omega();
    return d;
  }
  void normalizeState(State& s) const { s.yaw() = normalizeAngle(s.yaw()); }
};

// ---------------------------------------------------------------------------

struct DifferentialDriveParams {
  double wheel_radius = 0.15;  ///< [m]
  double track = 0.50;         ///< distance between the two driven wheels [m]
};

/// [omega_left, omega_right] wheel angular velocities [rad/s]
struct WheelRateInput : StateVector<WheelRateInput, 2> {
  double& left() { return data[0]; }
  double& right() { return data[1]; }
  double left() const { return data[0]; }
  double right() const { return data[1]; }

  static WheelRateInput make(double left, double right) {
    WheelRateInput u;
    u.data = {left, right};
    return u;
  }
};

/**
 * @brief Differential drive with wheel angular rates as the input.
 *
 *   v     = r * (omega_r + omega_l) / 2
 *   omega = r * (omega_r - omega_l) / track
 */
struct DifferentialDriveModel {
  using State = UnicycleState;
  using Input = WheelRateInput;

  DifferentialDriveParams params;

  DifferentialDriveModel() = default;
  explicit DifferentialDriveModel(const DifferentialDriveParams& p) : params(p) {}

  UnicycleInput toBodyVelocity(const Input& u) const {
    const double v = params.wheel_radius * (u.right() + u.left()) * 0.5;
    const double w = params.wheel_radius * (u.right() - u.left()) / params.track;
    return UnicycleInput::make(v, w);
  }

  Input toWheelRates(double v, double omega) const {
    const double half = 0.5 * omega * params.track;
    return Input::make((v - half) / params.wheel_radius,
                       (v + half) / params.wheel_radius);
  }

  State derivative(const State& s, const Input& u) const {
    return UnicycleModel().derivative(s, toBodyVelocity(u));
  }
  void normalizeState(State& s) const { s.yaw() = normalizeAngle(s.yaw()); }
};

}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_UNICYCLE_HPP
