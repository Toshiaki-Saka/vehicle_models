// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_ACKERMANN_HPP
#define VEHICLE_MODELS_ACKERMANN_HPP

#include <limits>

#include "vehicle_models/types.hpp"
#include "vehicle_models/vehicle_parameters.hpp"

namespace vehicle_models {

/**
 * @brief Steering geometry of a two-axle vehicle.
 *
 * Sign convention: delta > 0 means a left turn (counter-clockwise, positive
 * yaw rate). The "bicycle" steer angle delta is the angle of the virtual
 * single front wheel on the vehicle centre line.
 */
struct AckermannGeometry {
  double wheel_base = 2.70;    ///< [m]
  double track_front = 1.55;   ///< [m]
  double track_rear = 1.55;    ///< [m]
  double steering_ratio = 16.0;   ///< handwheel angle / road wheel angle [-]
  double ackermann_ratio = 1.0;   ///< 1 = ideal Ackermann, 0 = parallel steer

  static AckermannGeometry from(const VehicleParameters& p) {
    AckermannGeometry g;
    g.wheel_base = p.wheelBase();
    g.track_front = p.track_front;
    g.track_rear = p.track_rear;
    g.steering_ratio = p.steering_ratio;
    g.ackermann_ratio = p.ackermann_ratio;
    return g;
  }
};

struct WheelAngles {
  double left = 0.0;   ///< [rad]
  double right = 0.0;  ///< [rad]
};

struct WheelSpeeds {
  double front_left = 0.0;   ///< [m/s]
  double front_right = 0.0;
  double rear_left = 0.0;
  double rear_right = 0.0;
};

/// Signed turn radius at the rear axle centre. +inf when driving straight.
inline double turnRadius(const AckermannGeometry& g, double delta) {
  const double t = std::tan(delta);
  if (std::fabs(t) < 1e-12) return std::numeric_limits<double>::infinity();
  return g.wheel_base / t;
}

/// Steer angle of the virtual bicycle wheel that produces a given radius.
inline double steerAngleForRadius(const AckermannGeometry& g, double radius) {
  if (!std::isfinite(radius) || std::fabs(radius) < 1e-12) return 0.0;
  return std::atan(g.wheel_base / radius);
}

/**
 * @brief Split a bicycle steer angle into the two front road wheel angles.
 *
 * Ideal Ackermann satisfies  cot(delta_outer) - cot(delta_inner) = T / L.
 * With ackermann_ratio k the result is linearly blended toward parallel
 * steering (both wheels at delta), which is how real racks with finite
 * Ackermann percentage are usually approximated.
 */
inline WheelAngles roadWheelAngles(const AckermannGeometry& g, double delta) {
  WheelAngles w;
  if (std::fabs(delta) < 1e-9) return w;

  const double cot = 1.0 / std::tan(delta);
  const double half = 0.5 * g.track_front / g.wheel_base;
  const double ideal_left = std::atan(1.0 / (cot - half));
  const double ideal_right = std::atan(1.0 / (cot + half));

  const double k = clampValue(g.ackermann_ratio, 0.0, 1.0);
  w.left = delta + k * (ideal_left - delta);
  w.right = delta + k * (ideal_right - delta);
  return w;
}

/// Inverse of roadWheelAngles(): equivalent bicycle angle from measured
/// road wheel angles (averaged in cotangent space, which is exact for
/// ideal Ackermann).
inline double bicycleAngleFromWheels(const AckermannGeometry& /*g*/,
                                     double left, double right) {
  if (std::fabs(left) < 1e-9 && std::fabs(right) < 1e-9) return 0.0;
  const double cot_l = 1.0 / std::tan(guardDenominator(left, 1e-9));
  const double cot_r = 1.0 / std::tan(guardDenominator(right, 1e-9));
  return std::atan(2.0 / (cot_l + cot_r));
}

inline double handwheelToRoadWheel(const AckermannGeometry& g, double handwheel) {
  return handwheel / g.steering_ratio;
}
inline double roadWheelToHandwheel(const AckermannGeometry& g, double road_wheel) {
  return road_wheel * g.steering_ratio;
}

/**
 * @brief Deviation of the outer wheel from ideal Ackermann, given the inner
 *        wheel angle. Positive = the outer wheel is steered more than ideal
 *        (toward parallel / anti-Ackermann).
 */
inline double ackermannError(const AckermannGeometry& g, double delta) {
  if (std::fabs(delta) < 1e-9) return 0.0;
  const WheelAngles actual = roadWheelAngles(g, delta);
  const double inner = (delta > 0.0) ? actual.left : actual.right;
  const double outer = (delta > 0.0) ? actual.right : actual.left;
  const double ratio = g.track_front / g.wheel_base;
  const double cot_inner = 1.0 / std::tan(guardDenominator(inner, 1e-9));
  const double cot_ideal_outer =
      cot_inner + ((delta > 0.0) ? ratio : -ratio);
  const double ideal_outer = std::atan(1.0 / cot_ideal_outer);
  return std::fabs(outer) - std::fabs(ideal_outer);
}

/**
 * @brief Longitudinal speed of each wheel centre for a rigid-body motion.
 *
 * v is the speed at the rear axle centre, yaw_rate the body yaw rate.
 * The front wheel speeds are projected onto their own steered direction,
 * which is the quantity a wheel-speed sensor sees.
 */
inline WheelSpeeds wheelSpeeds(const AckermannGeometry& g, double v,
                               double yaw_rate) {
  WheelSpeeds s;
  s.rear_left = v - 0.5 * g.track_rear * yaw_rate;
  s.rear_right = v + 0.5 * g.track_rear * yaw_rate;

  // Front wheel centre velocity in body frame: (v -+ T/2 * r, L * r)
  const double vy_front = g.wheel_base * yaw_rate;
  const double vxl = v - 0.5 * g.track_front * yaw_rate;
  const double vxr = v + 0.5 * g.track_front * yaw_rate;
  const double delta = steerAngleForRadius(
      g, (std::fabs(yaw_rate) < 1e-9) ? std::numeric_limits<double>::infinity()
                                      : v / yaw_rate);
  const WheelAngles wa = roadWheelAngles(g, delta);
  s.front_left = vxl * std::cos(wa.left) + vy_front * std::sin(wa.left);
  s.front_right = vxr * std::cos(wa.right) + vy_front * std::sin(wa.right);
  return s;
}

/// Wheel angular rates [rad/s] from the wheel centre speeds.
inline WheelSpeeds wheelAngularRates(const WheelSpeeds& speeds,
                                     double wheel_radius) {
  WheelSpeeds w = speeds;
  const double r = (wheel_radius > 1e-9) ? wheel_radius : 1e-9;
  w.front_left /= r;
  w.front_right /= r;
  w.rear_left /= r;
  w.rear_right /= r;
  return w;
}

/// Minimum turn radius (outer front wheel) at full lock — a common spec value.
inline double minimumTurnRadius(const AckermannGeometry& g, double steer_max) {
  const WheelAngles wa = roadWheelAngles(g, steer_max);
  const double outer = (steer_max > 0.0) ? wa.right : wa.left;
  return std::fabs(g.wheel_base / std::sin(guardDenominator(outer, 1e-9)));
}

}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_ACKERMANN_HPP
