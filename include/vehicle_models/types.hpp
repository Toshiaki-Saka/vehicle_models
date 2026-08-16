// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_TYPES_HPP
#define VEHICLE_MODELS_TYPES_HPP

#include <array>
#include <cmath>
#include <cstddef>

namespace vehicle_models {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kGravity = 9.80665;  ///< [m/s^2]

inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }

/// Wrap an angle into [-pi, pi).
inline double normalizeAngle(double angle) {
  double x = std::fmod(angle + kPi, 2.0 * kPi);
  if (x < 0.0) x += 2.0 * kPi;
  return x - kPi;
}

inline double clampValue(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline int signum(double v) { return (v > 0.0) - (v < 0.0); }

/// Keep |v| >= eps while preserving sign. Used to avoid the 1/v_x singularity
/// of the dynamic models at standstill (ISO 26262 style defensive coding:
/// the model must stay well defined over the whole operating range).
inline double guardDenominator(double v, double eps) {
  if (std::fabs(v) >= eps) return v;
  return (v < 0.0) ? -eps : eps;
}

/**
 * @brief CRTP base giving a plain struct fixed-size vector arithmetic.
 *
 * Derived classes only add named accessors, so they stay aggregates and add
 * no runtime cost. The arithmetic operators are what the generic integrators
 * in integrator.hpp rely on.
 */
template <typename Derived, std::size_t N>
struct StateVector {
  static constexpr std::size_t kDim = N;
  std::array<double, N> data{};

  static constexpr std::size_t size() { return N; }
  double& operator[](std::size_t i) { return data[i]; }
  double operator[](std::size_t i) const { return data[i]; }

  friend Derived operator+(const Derived& a, const Derived& b) {
    Derived r = a;
    for (std::size_t i = 0; i < N; ++i) r.data[i] += b.data[i];
    return r;
  }
  friend Derived operator-(const Derived& a, const Derived& b) {
    Derived r = a;
    for (std::size_t i = 0; i < N; ++i) r.data[i] -= b.data[i];
    return r;
  }
  friend Derived operator*(const Derived& a, double s) {
    Derived r = a;
    for (std::size_t i = 0; i < N; ++i) r.data[i] *= s;
    return r;
  }
  friend Derived operator*(double s, const Derived& a) { return a * s; }
  friend Derived& operator+=(Derived& a, const Derived& b) {
    for (std::size_t i = 0; i < N; ++i) a.data[i] += b.data[i];
    return a;
  }
};

/// Planar pose, used as a common exchange type between models.
struct Pose2D {
  double x = 0.0;    ///< [m]
  double y = 0.0;    ///< [m]
  double yaw = 0.0;  ///< [rad]
};

}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_TYPES_HPP
