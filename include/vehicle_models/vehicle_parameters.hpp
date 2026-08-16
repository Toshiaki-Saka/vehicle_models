// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_VEHICLE_PARAMETERS_HPP
#define VEHICLE_MODELS_VEHICLE_PARAMETERS_HPP

#include <string>
#include <vector>

#include "vehicle_models/types.hpp"

namespace vehicle_models {

/**
 * @brief Full parameter set shared by every model in the library.
 *
 * A single struct keeps one vehicle definition consistent across the
 * kinematic, dynamic and double-track models, which is what makes
 * cross-model plausibility checks (a common SOTIF technique) meaningful.
 */
struct VehicleParameters {
  // --- geometry -----------------------------------------------------------
  double l_f = 1.20;          ///< CoG to front axle [m]
  double l_r = 1.50;          ///< CoG to rear axle [m]
  double track_front = 1.55;  ///< front track width [m]
  double track_rear = 1.55;   ///< rear track width [m]
  double cg_height = 0.55;    ///< CoG height above ground [m]
  double wheel_radius = 0.32; ///< effective rolling radius [m]

  // --- inertia ------------------------------------------------------------
  double mass = 1600.0;       ///< total mass [kg]
  double inertia_z = 2600.0;  ///< yaw moment of inertia [kg m^2]

  // --- tires (per axle, i.e. both wheels combined) ------------------------
  double cornering_stiffness_front = 90000.0;  ///< [N/rad]
  double cornering_stiffness_rear = 110000.0;  ///< [N/rad]
  double friction = 1.0;                       ///< peak road friction [-]

  // --- resistances --------------------------------------------------------
  double drag_area = 0.40;            ///< 0.5*rho*Cd*A [N/(m/s)^2]
  double rolling_resistance = 0.012;  ///< [-]

  // --- steering -----------------------------------------------------------
  double steering_ratio = 16.0;               ///< handwheel / road wheel [-]
  double ackermann_ratio = 1.0;               ///< 1 = ideal, 0 = parallel
  double steer_max = deg2rad(35.0);           ///< road wheel limit [rad]
  double steer_rate_max = deg2rad(90.0);      ///< road wheel rate limit [rad/s]
  double steer_time_constant = 0.06;          ///< 1st order actuator lag [s]

  // --- actuation limits ---------------------------------------------------
  double accel_max = 2.0;    ///< [m/s^2]
  double accel_min = -5.0;   ///< [m/s^2]
  double speed_max = 20.0;   ///< [m/s]

  // --- numerics -----------------------------------------------------------
  double low_speed_guard = 1.0;  ///< |v_x| floor of the dynamic models [m/s]

  double wheelBase() const { return l_f + l_r; }
  double staticLoadFront() const { return mass * kGravity * l_r / wheelBase(); }
  double staticLoadRear() const { return mass * kGravity * l_f / wheelBase(); }

  /// @return list of violated constraints; empty means the set is usable.
  std::vector<std::string> validate() const {
    std::vector<std::string> errors;
    auto require = [&errors](bool ok, const char* msg) {
      if (!ok) errors.emplace_back(msg);
    };
    require(l_f > 0.0, "l_f must be positive");
    require(l_r > 0.0, "l_r must be positive");
    require(track_front > 0.0, "track_front must be positive");
    require(track_rear > 0.0, "track_rear must be positive");
    require(cg_height >= 0.0, "cg_height must be non-negative");
    require(wheel_radius > 0.0, "wheel_radius must be positive");
    require(mass > 0.0, "mass must be positive");
    require(inertia_z > 0.0, "inertia_z must be positive");
    require(cornering_stiffness_front > 0.0, "front cornering stiffness must be positive");
    require(cornering_stiffness_rear > 0.0, "rear cornering stiffness must be positive");
    require(friction > 0.0, "friction must be positive");
    require(drag_area >= 0.0, "drag_area must be non-negative");
    require(rolling_resistance >= 0.0, "rolling_resistance must be non-negative");
    require(steering_ratio > 0.0, "steering_ratio must be positive");
    require(ackermann_ratio >= 0.0 && ackermann_ratio <= 1.0,
            "ackermann_ratio must be within [0, 1]");
    require(steer_max > 0.0 && steer_max < 0.5 * kPi, "steer_max must be within (0, pi/2)");
    require(steer_rate_max > 0.0, "steer_rate_max must be positive");
    require(steer_time_constant > 0.0, "steer_time_constant must be positive");
    require(accel_max > 0.0, "accel_max must be positive");
    require(accel_min < 0.0, "accel_min must be negative");
    require(speed_max > 0.0, "speed_max must be positive");
    require(low_speed_guard > 0.0, "low_speed_guard must be positive");
    return errors;
  }
};

/// Compact low-speed automated shuttle (~6 m class).
inline VehicleParameters makeShuttleParameters() {
  VehicleParameters p;
  p.l_f = 1.35;
  p.l_r = 1.65;
  p.track_front = 1.48;
  p.track_rear = 1.48;
  p.cg_height = 0.85;
  p.wheel_radius = 0.33;
  p.mass = 3000.0;
  p.inertia_z = 6500.0;
  p.cornering_stiffness_front = 130000.0;
  p.cornering_stiffness_rear = 160000.0;
  p.steering_ratio = 18.0;
  p.steer_max = deg2rad(30.0);
  p.accel_max = 1.0;
  p.accel_min = -2.5;
  p.speed_max = 5.6;  // 20 km/h
  return p;
}

/// Small off-road buggy / test mule.
inline VehicleParameters makeBuggyParameters() {
  VehicleParameters p;
  p.l_f = 0.80;
  p.l_r = 0.90;
  p.track_front = 1.20;
  p.track_rear = 1.20;
  p.cg_height = 0.45;
  p.wheel_radius = 0.28;
  p.mass = 450.0;
  p.inertia_z = 220.0;
  p.cornering_stiffness_front = 25000.0;
  p.cornering_stiffness_rear = 28000.0;
  p.friction = 0.7;
  p.drag_area = 0.25;
  p.steering_ratio = 10.0;
  p.steer_max = deg2rad(40.0);
  p.speed_max = 11.0;
  return p;
}

/// Mid-size passenger car (default of VehicleParameters).
inline VehicleParameters makePassengerCarParameters() { return VehicleParameters(); }

}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_VEHICLE_PARAMETERS_HPP
