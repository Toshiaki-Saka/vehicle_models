// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_TIRE_TIRE_MODELS_HPP
#define VEHICLE_MODELS_TIRE_TIRE_MODELS_HPP

#include <algorithm>
#include <cmath>

#include "vehicle_models/types.hpp"

namespace vehicle_models {
namespace tire {

/**
 * Sign convention used by every tire model here:
 *   positive slip angle alpha  ->  positive lateral force Fy
 *   alpha_front = delta - atan((v_y + l_f*r) / v_x)
 *   alpha_rear  =       - atan((v_y - l_r*r) / v_x)
 *
 * Implicit concept:
 *   double lateralForce(double slip_angle, double normal_force) const;
 *   double corneringStiffness(double normal_force) const;
 */

/// Linear tire with a hard mu*Fz saturation. The reference model for the
/// linear analysis in linear_analysis.hpp.
struct LinearTire {
  double cornering_stiffness = 90000.0;  ///< [N/rad]
  double friction = 1.0;                 ///< [-]

  double lateralForce(double slip_angle, double normal_force) const {
    const double f_max = friction * std::max(normal_force, 0.0);
    return clampValue(cornering_stiffness * slip_angle, -f_max, f_max);
  }
  double corneringStiffness(double /*normal_force*/ = 0.0) const {
    return cornering_stiffness;
  }
};

/// Fiala brush model: cubic build-up to full sliding at
/// alpha_sl = atan(3*mu*Fz / C). Physically shaped and only two parameters.
struct FialaTire {
  double cornering_stiffness = 90000.0;  ///< [N/rad]
  double friction = 1.0;                 ///< [-]

  double lateralForce(double slip_angle, double normal_force) const {
    const double fz = std::max(normal_force, 0.0);
    const double mu_fz = friction * fz;
    if (mu_fz <= 0.0 || cornering_stiffness <= 0.0) return 0.0;

    const double alpha_sl = std::atan(3.0 * mu_fz / cornering_stiffness);
    if (std::fabs(slip_angle) >= alpha_sl) {
      return (slip_angle >= 0.0) ? mu_fz : -mu_fz;
    }
    const double t = std::tan(slip_angle);
    const double c = cornering_stiffness;
    const double f = c * t - (c * c) / (3.0 * mu_fz) * std::fabs(t) * t +
                     (c * c * c) / (27.0 * mu_fz * mu_fz) * t * t * t;
    return clampValue(f, -mu_fz, mu_fz);
  }
  double corneringStiffness(double /*normal_force*/ = 0.0) const {
    return cornering_stiffness;
  }
};

/// Pacejka Magic Formula, pure lateral slip.
///   Fy = D * sin(C * atan(B*a - E*(B*a - atan(B*a)))),  D = mu * Fz
struct PacejkaTire {
  double B = 10.0;         ///< stiffness factor
  double C = 1.9;          ///< shape factor
  double E = 0.97;         ///< curvature factor
  double friction = 1.0;   ///< peak factor D = friction * Fz

  double lateralForce(double slip_angle, double normal_force) const {
    const double d = friction * std::max(normal_force, 0.0);
    const double ba = B * slip_angle;
    return d * std::sin(C * std::atan(ba - E * (ba - std::atan(ba))));
  }
  /// dFy/dalpha at alpha = 0, i.e. B*C*D.
  double corneringStiffness(double normal_force) const {
    return B * C * friction * std::max(normal_force, 0.0);
  }

  /// Build a Pacejka set that matches a desired cornering stiffness at a
  /// given nominal load, so it can be swapped in for LinearTire.
  static PacejkaTire fromCorneringStiffness(double cornering_stiffness,
                                            double nominal_load,
                                            double friction_coeff = 1.0) {
    PacejkaTire t;
    t.friction = friction_coeff;
    t.C = 1.9;
    t.E = 0.97;
    const double d = friction_coeff * std::max(nominal_load, 1.0);
    t.B = cornering_stiffness / (t.C * d);
    return t;
  }
};

/// Friction-ellipse scale factor for the lateral force once a longitudinal
/// force Fx is already being used at the same contact patch.
inline double frictionEllipseScale(double fx, double friction,
                                   double normal_force) {
  const double f_max = friction * std::max(normal_force, 0.0);
  if (f_max <= 0.0) return 0.0;
  const double ratio = clampValue(std::fabs(fx) / f_max, 0.0, 1.0);
  return std::sqrt(std::max(0.0, 1.0 - ratio * ratio));
}

}  // namespace tire
}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_TIRE_TIRE_MODELS_HPP
