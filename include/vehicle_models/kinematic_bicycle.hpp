// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_KINEMATIC_BICYCLE_HPP
#define VEHICLE_MODELS_KINEMATIC_BICYCLE_HPP

#include "vehicle_models/ackermann.hpp"
#include "vehicle_models/types.hpp"
#include "vehicle_models/vehicle_parameters.hpp"

namespace vehicle_models {

/// Point of the vehicle the state (x, y) refers to.
enum class ReferencePoint { RearAxle, CenterOfGravity, FrontAxle };

/// [x, y, yaw, v]
struct KinematicBicycleState : StateVector<KinematicBicycleState, 4> {
  double& x() { return data[0]; }
  double& y() { return data[1]; }
  double& yaw() { return data[2]; }
  double& v() { return data[3]; }
  double x() const { return data[0]; }
  double y() const { return data[1]; }
  double yaw() const { return data[2]; }
  double v() const { return data[3]; }

  static KinematicBicycleState make(double x, double y, double yaw, double v) {
    KinematicBicycleState s;
    s.data = {x, y, yaw, v};
    return s;
  }
  Pose2D pose() const { return Pose2D{x(), y(), yaw()}; }
};

/// [acceleration, steer angle]
struct KinematicBicycleInput : StateVector<KinematicBicycleInput, 2> {
  double& accel() { return data[0]; }
  double& steer() { return data[1]; }
  double accel() const { return data[0]; }
  double steer() const { return data[1]; }

  static KinematicBicycleInput make(double accel, double steer) {
    KinematicBicycleInput u;
    u.data = {accel, steer};
    return u;
  }
};

/**
 * @brief Kinematic bicycle model (no tire slip).
 *
 * RearAxle reference:
 *   x_dot = v cos(yaw), y_dot = v sin(yaw)
 *   yaw_dot = v tan(delta) / L,  v_dot = a
 *
 * CenterOfGravity reference (beta = atan(l_r tan(delta) / L)):
 *   x_dot = v cos(yaw + beta), y_dot = v sin(yaw + beta)
 *   yaw_dot = v cos(beta) tan(delta) / L,  v_dot = a
 *
 * FrontAxle reference (v is the front wheel speed):
 *   x_dot = v cos(yaw + delta), y_dot = v sin(yaw + delta)
 *   yaw_dot = v sin(delta) / L,  v_dot = a
 *
 * Valid while the lateral acceleration stays well below mu*g (rule of thumb:
 * below ~0.4 g). Above that, use DynamicBicycleModel.
 */
struct KinematicBicycleModel {
  using State = KinematicBicycleState;
  using Input = KinematicBicycleInput;

  VehicleParameters params;
  ReferencePoint reference = ReferencePoint::RearAxle;
  bool apply_limits = true;  ///< clamp steer / accel / speed to the params

  KinematicBicycleModel() = default;
  explicit KinematicBicycleModel(const VehicleParameters& p,
                                 ReferencePoint ref = ReferencePoint::RearAxle)
      : params(p), reference(ref) {}

  /// Body slip angle at the CoG for the current steer angle.
  double sideSlip(double steer) const {
    return std::atan(params.l_r * std::tan(steer) / params.wheelBase());
  }

  State derivative(const State& s, const Input& u) const {
    const double delta =
        apply_limits ? clampValue(u.steer(), -params.steer_max, params.steer_max)
                     : u.steer();
    double accel = apply_limits
                       ? clampValue(u.accel(), params.accel_min, params.accel_max)
                       : u.accel();
    if (apply_limits) {
      // Stop integrating speed once the envelope is reached.
      if (s.v() >= params.speed_max && accel > 0.0) accel = 0.0;
      if (s.v() <= -params.speed_max && accel < 0.0) accel = 0.0;
    }

    const double L = params.wheelBase();
    State d;
    switch (reference) {
      case ReferencePoint::CenterOfGravity: {
        const double beta = sideSlip(delta);
        d.data[0] = s.v() * std::cos(s.yaw() + beta);
        d.data[1] = s.v() * std::sin(s.yaw() + beta);
        d.data[2] = s.v() * std::cos(beta) * std::tan(delta) / L;
        break;
      }
      case ReferencePoint::FrontAxle: {
        d.data[0] = s.v() * std::cos(s.yaw() + delta);
        d.data[1] = s.v() * std::sin(s.yaw() + delta);
        d.data[2] = s.v() * std::sin(delta) / L;
        break;
      }
      case ReferencePoint::RearAxle:
      default: {
        d.data[0] = s.v() * std::cos(s.yaw());
        d.data[1] = s.v() * std::sin(s.yaw());
        d.data[2] = s.v() * std::tan(delta) / L;
        break;
      }
    }
    d.data[3] = accel;
    return d;
  }

  void normalizeState(State& s) const {
    s.yaw() = normalizeAngle(s.yaw());
    if (apply_limits) s.v() = clampValue(s.v(), -params.speed_max, params.speed_max);
  }

  /// Lateral acceleration implied by the kinematics, v^2 / R.
  double lateralAcceleration(const State& s, double steer) const {
    return s.v() * s.v() * std::tan(steer) / params.wheelBase();
  }
};

// ---------------------------------------------------------------------------

/// [x, y, yaw, v, delta]
struct SteerDynamicsState : StateVector<SteerDynamicsState, 5> {
  double& x() { return data[0]; }
  double& y() { return data[1]; }
  double& yaw() { return data[2]; }
  double& v() { return data[3]; }
  double& steer() { return data[4]; }
  double x() const { return data[0]; }
  double y() const { return data[1]; }
  double yaw() const { return data[2]; }
  double v() const { return data[3]; }
  double steer() const { return data[4]; }

  static SteerDynamicsState make(double x, double y, double yaw, double v,
                                 double steer) {
    SteerDynamicsState s;
    s.data = {x, y, yaw, v, steer};
    return s;
  }
  Pose2D pose() const { return Pose2D{x(), y(), yaw()}; }
  KinematicBicycleState toKinematic() const {
    return KinematicBicycleState::make(x(), y(), yaw(), v());
  }
};

/**
 * @brief Kinematic bicycle whose steering actuator has a first order lag and
 *        a rate limit — the difference that matters when validating a
 *        path-tracking controller against a real EPS.
 *
 *   delta_dot = clamp((delta_cmd - delta) / tau, +-rate_max)
 */
struct KinematicBicycleSteerModel {
  using State = SteerDynamicsState;
  using Input = KinematicBicycleInput;  ///< [accel, steer command]

  KinematicBicycleModel base;

  KinematicBicycleSteerModel() = default;
  explicit KinematicBicycleSteerModel(const VehicleParameters& p,
                                      ReferencePoint ref = ReferencePoint::RearAxle)
      : base(p, ref) {}

  const VehicleParameters& params() const { return base.params; }

  State derivative(const State& s, const Input& u) const {
    const auto core = base.derivative(
        s.toKinematic(), KinematicBicycleInput::make(u.accel(), s.steer()));
    const auto& p = base.params;
    const double cmd = clampValue(u.steer(), -p.steer_max, p.steer_max);
    const double rate = clampValue((cmd - s.steer()) / p.steer_time_constant,
                                   -p.steer_rate_max, p.steer_rate_max);
    State d;
    d.data[0] = core[0];
    d.data[1] = core[1];
    d.data[2] = core[2];
    d.data[3] = core[3];
    d.data[4] = rate;
    return d;
  }

  void normalizeState(State& s) const {
    const auto& p = base.params;
    s.yaw() = normalizeAngle(s.yaw());
    if (base.apply_limits) {
      s.v() = clampValue(s.v(), -p.speed_max, p.speed_max);
      s.steer() = clampValue(s.steer(), -p.steer_max, p.steer_max);
    }
  }
};

}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_KINEMATIC_BICYCLE_HPP
