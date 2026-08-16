// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_LINEAR_ANALYSIS_HPP
#define VEHICLE_MODELS_LINEAR_ANALYSIS_HPP

#include <limits>

#include "vehicle_models/dynamic_bicycle.hpp"
#include "vehicle_models/vehicle_parameters.hpp"

namespace vehicle_models {
namespace analysis {

/**
 * Closed-form results of the linear single-track model. They are the
 * reference the simulation is checked against in the unit tests, and the
 * kind of quantity a plausibility monitor compares measured yaw rate with.
 */

/// Understeer gradient K [rad/(m/s^2)]:  delta = L/R + K * a_y
///   K > 0 understeer, K = 0 neutral steer, K < 0 oversteer.
inline double understeerGradient(const VehicleParameters& p) {
  return p.mass / p.wheelBase() *
         (p.l_r / p.cornering_stiffness_front - p.l_f / p.cornering_stiffness_rear);
}

/// Same quantity expressed in deg/g, the unit used in most test reports.
inline double understeerGradientDegPerG(const VehicleParameters& p) {
  return rad2deg(understeerGradient(p) * kGravity);
}

/// Characteristic speed [m/s] of an understeering vehicle (yaw rate gain
/// peaks here at half the neutral-steer value). Infinite if not understeering.
inline double characteristicSpeed(const VehicleParameters& p) {
  const double k = understeerGradient(p);
  if (k <= 1e-12) return std::numeric_limits<double>::infinity();
  return std::sqrt(p.wheelBase() / k);
}

/// Critical speed [m/s] of an oversteering vehicle (divergent above it).
/// Infinite if the vehicle is neutral or understeering.
inline double criticalSpeed(const VehicleParameters& p) {
  const double k = understeerGradient(p);
  if (k >= -1e-12) return std::numeric_limits<double>::infinity();
  return std::sqrt(-p.wheelBase() / k);
}

/// Distance of the neutral steer point behind the front axle [m].
inline double neutralSteerPoint(const VehicleParameters& p) {
  const double cf = p.cornering_stiffness_front;
  const double cr = p.cornering_stiffness_rear;
  return p.wheelBase() * cr / (cf + cr);
}

/// Static margin [-] = (x_NSP - l_f) / L. Positive means understeer.
inline double staticMargin(const VehicleParameters& p) {
  return (neutralSteerPoint(p) - p.l_f) / p.wheelBase();
}

/// Steady state yaw rate per steer angle [1/s].
inline double yawRateGain(const VehicleParameters& p, double vx) {
  const double L = p.wheelBase();
  const double k = understeerGradient(p);
  const double den = L + k * vx * vx;
  if (std::fabs(den) < 1e-12) return std::numeric_limits<double>::infinity();
  return vx / den;
}

/// Steady state lateral acceleration per steer angle [m/s^2/rad].
inline double lateralAccelerationGain(const VehicleParameters& p, double vx) {
  return vx * yawRateGain(p, vx);
}

/// Steer angle needed to hold a radius at a speed (Ackermann + slip term).
inline double requiredSteerAngle(const VehicleParameters& p, double radius,
                                 double vx) {
  if (!std::isfinite(radius) || std::fabs(radius) < 1e-9) return 0.0;
  const double ay = vx * vx / radius;
  return p.wheelBase() / radius + understeerGradient(p) * ay;
}

struct SteadyState {
  double yaw_rate = 0.0;         ///< [rad/s]
  double lateral_accel = 0.0;    ///< [m/s^2]
  double side_slip = 0.0;        ///< body slip angle beta [rad]
  double radius = 0.0;           ///< [m]
  double slip_front = 0.0;       ///< front axle slip angle [rad]
  double slip_rear = 0.0;        ///< rear axle slip angle [rad]
};

/// Closed-form steady state cornering response of the linear model.
inline SteadyState steadyStateCornering(const VehicleParameters& p, double vx,
                                        double delta) {
  SteadyState s;
  s.yaw_rate = yawRateGain(p, vx) * delta;
  s.lateral_accel = vx * s.yaw_rate;
  s.radius = (std::fabs(s.yaw_rate) < 1e-12)
                 ? std::numeric_limits<double>::infinity()
                 : vx / s.yaw_rate;
  // beta = l_r/R - m*l_f*v^2 / (L*C_r*R)
  const double inv_r = (std::isfinite(s.radius) && std::fabs(s.radius) > 1e-12)
                           ? 1.0 / s.radius
                           : 0.0;
  s.side_slip =
      (p.l_r - p.mass * p.l_f * vx * vx / (p.wheelBase() * p.cornering_stiffness_rear)) *
      inv_r;
  s.slip_rear = s.side_slip - p.l_r * s.yaw_rate / guardDenominator(vx, 1e-6);
  s.slip_front = delta - (s.side_slip + p.l_f * s.yaw_rate / guardDenominator(vx, 1e-6));
  return s;
}

struct YawMode {
  double natural_frequency = 0.0;  ///< [rad/s]
  double damping_ratio = 0.0;      ///< [-]
  double real_1 = 0.0;             ///< eigenvalue 1, real part
  double imag_1 = 0.0;             ///< eigenvalue 1, imaginary part
  double real_2 = 0.0;
  double imag_2 = 0.0;
  bool stable = false;
};

/// Eigenvalues of the [v_y, r] system — the yaw/sideslip mode.
inline YawMode yawMode(const VehicleParameters& p, double vx) {
  const LinearLateralBicycleModel model(p, vx);
  const auto a = model.stateMatrix();
  const double tr = a[0][0] + a[1][1];
  const double det = a[0][0] * a[1][1] - a[0][1] * a[1][0];

  YawMode m;
  m.natural_frequency = (det > 0.0) ? std::sqrt(det) : 0.0;
  m.damping_ratio = (det > 0.0) ? -tr / (2.0 * std::sqrt(det)) : 0.0;

  const double disc = tr * tr - 4.0 * det;
  if (disc >= 0.0) {
    const double root = std::sqrt(disc);
    m.real_1 = 0.5 * (tr + root);
    m.real_2 = 0.5 * (tr - root);
  } else {
    const double root = std::sqrt(-disc);
    m.real_1 = m.real_2 = 0.5 * tr;
    m.imag_1 = 0.5 * root;
    m.imag_2 = -0.5 * root;
  }
  m.stable = (m.real_1 < 0.0) && (m.real_2 < 0.0);
  return m;
}

/// Largest lateral acceleration the axles can support (simple mu*g bound).
inline double maxLateralAcceleration(const VehicleParameters& p) {
  return p.friction * kGravity;
}

}  // namespace analysis
}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_LINEAR_ANALYSIS_HPP
