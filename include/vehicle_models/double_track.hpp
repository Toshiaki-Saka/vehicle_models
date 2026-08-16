// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_DOUBLE_TRACK_HPP
#define VEHICLE_MODELS_DOUBLE_TRACK_HPP

#include "vehicle_models/ackermann.hpp"
#include "vehicle_models/dynamic_bicycle.hpp"
#include "vehicle_models/tire/tire_models.hpp"
#include "vehicle_models/types.hpp"
#include "vehicle_models/vehicle_parameters.hpp"

namespace vehicle_models {

enum class WheelIndex { FrontLeft = 0, FrontRight = 1, RearLeft = 2, RearRight = 3 };

struct WheelQuantities {
  std::array<double, 4> value{};  ///< indexed by WheelIndex
  double& operator[](WheelIndex i) { return value[static_cast<std::size_t>(i)]; }
  double operator[](WheelIndex i) const {
    return value[static_cast<std::size_t>(i)];
  }
  double sum() const { return value[0] + value[1] + value[2] + value[3]; }
};

struct DoubleTrackForces {
  WheelQuantities slip_angle;   ///< [rad]
  WheelQuantities normal_load;  ///< [N]
  WheelQuantities lateral;      ///< [N], wheel frame
  WheelQuantities longitudinal; ///< [N], wheel frame
  WheelAngles steer;            ///< front road wheel angles [rad]
  double ax = 0.0;              ///< [m/s^2]
  double ay = 0.0;              ///< [m/s^2]
};

/// Additional parameters that only the double-track model needs.
struct DoubleTrackParams {
  double front_roll_stiffness_ratio = 0.55;  ///< share of lateral transfer on the front axle
  double front_drive_ratio = 0.0;            ///< 0 = RWD, 1 = FWD, 0.5 = AWD
  double front_brake_ratio = 0.65;           ///< brake force share on the front axle
  bool combined_slip = true;                 ///< apply the friction ellipse
};

/**
 * @brief 3-DOF double-track (four-wheel) model.
 *
 * Adds to the single-track model: individual Ackermann wheel angles,
 * longitudinal and lateral load transfer, per-wheel tire saturation and
 * combined slip. This is where understeer at the limit, inner-wheel lift and
 * the difference between ideal and real Ackermann actually show up — the
 * single-track model cannot reproduce any of them.
 *
 * Load transfer is evaluated with a one-pass predictor: forces are first
 * computed on the static loads to estimate a_x / a_y, then recomputed on the
 * transferred loads. No iteration, deterministic execution time.
 */
template <typename Tire = tire::FialaTire>
struct DoubleTrackModel {
  using State = DynamicBicycleState;   ///< [x, y, yaw, vx, vy, r]
  using Input = DynamicBicycleInput;   ///< [Fx total, steer]

  VehicleParameters params;
  DoubleTrackParams dt_params;
  Tire tire_front;  ///< per-wheel tire, i.e. half the axle cornering stiffness
  Tire tire_rear;

  DoubleTrackModel() { syncTiresFromParams(); }
  explicit DoubleTrackModel(const VehicleParameters& p) : params(p) {
    syncTiresFromParams();
  }

  /// Split the axle cornering stiffness of `params` over the two wheels, so
  /// the double-track and single-track models describe the same vehicle.
  void syncTiresFromParams() {
    setTireFriction(tire_front, params.friction);
    setTireFriction(tire_rear, params.friction);
    setTireStiffness(tire_front, 0.5 * params.cornering_stiffness_front,
                     0.5 * params.staticLoadFront());
    setTireStiffness(tire_rear, 0.5 * params.cornering_stiffness_rear,
                     0.5 * params.staticLoadRear());
  }

  AckermannGeometry geometry() const { return AckermannGeometry::from(params); }

  DoubleTrackForces computeForces(const State& s, const Input& u) const {
    const double delta =
        clampValue(u.steer(), -params.steer_max, params.steer_max);
    const double tf = params.track_front;
    const double tr = params.track_rear;

    DoubleTrackForces f;
    f.steer = roadWheelAngles(geometry(), delta);

    // --- slip angles, per wheel -------------------------------------------
    const double vy = s.vy();
    const double r = s.yawRate();
    const double eps = params.low_speed_guard;
    const double vx_fl = guardDenominator(s.vx() - 0.5 * tf * r, eps);
    const double vx_fr = guardDenominator(s.vx() + 0.5 * tf * r, eps);
    const double vx_rl = guardDenominator(s.vx() - 0.5 * tr * r, eps);
    const double vx_rr = guardDenominator(s.vx() + 0.5 * tr * r, eps);

    f.slip_angle[WheelIndex::FrontLeft] =
        f.steer.left - std::atan((vy + params.l_f * r) / vx_fl);
    f.slip_angle[WheelIndex::FrontRight] =
        f.steer.right - std::atan((vy + params.l_f * r) / vx_fr);
    f.slip_angle[WheelIndex::RearLeft] =
        -std::atan((vy - params.l_r * r) / vx_rl);
    f.slip_angle[WheelIndex::RearRight] =
        -std::atan((vy - params.l_r * r) / vx_rr);

    // --- longitudinal force distribution ----------------------------------
    const double front_share = (u.fx() >= 0.0) ? dt_params.front_drive_ratio
                                               : dt_params.front_brake_ratio;
    const double fx_front = 0.5 * front_share * u.fx();
    const double fx_rear = 0.5 * (1.0 - front_share) * u.fx();
    f.longitudinal[WheelIndex::FrontLeft] = fx_front;
    f.longitudinal[WheelIndex::FrontRight] = fx_front;
    f.longitudinal[WheelIndex::RearLeft] = fx_rear;
    f.longitudinal[WheelIndex::RearRight] = fx_rear;

    // --- pass 1: static loads, to estimate a_x / a_y -----------------------
    const double ax_est = u.fx() / params.mass;
    double ay_est = 0.0;
    {
      DoubleTrackForces tmp = f;
      distributeLoads(tmp, ax_est, 0.0);
      evaluateTires(tmp);
      ay_est = lateralAcceleration(tmp, f.steer, delta);
    }

    // --- pass 2: transferred loads ----------------------------------------
    distributeLoads(f, ax_est, ay_est);
    evaluateTires(f);
    f.ax = longitudinalAcceleration(f, f.steer);
    f.ay = lateralAcceleration(f, f.steer, delta);
    return f;
  }

  State derivative(const State& s, const Input& u) const {
    const DoubleTrackForces f = computeForces(s, u);
    const double tf = params.track_front;
    const double tr = params.track_rear;

    // Yaw moment from all four contact patches.
    auto fx_body = [&](WheelIndex i, double steer_angle) {
      return f.longitudinal[i] * std::cos(steer_angle) -
             f.lateral[i] * std::sin(steer_angle);
    };
    auto fy_body = [&](WheelIndex i, double steer_angle) {
      return f.longitudinal[i] * std::sin(steer_angle) +
             f.lateral[i] * std::cos(steer_angle);
    };

    const double mz =
        params.l_f * (fy_body(WheelIndex::FrontLeft, f.steer.left) +
                      fy_body(WheelIndex::FrontRight, f.steer.right)) -
        params.l_r * (fy_body(WheelIndex::RearLeft, 0.0) +
                      fy_body(WheelIndex::RearRight, 0.0)) +
        0.5 * tf * (fx_body(WheelIndex::FrontRight, f.steer.right) -
                    fx_body(WheelIndex::FrontLeft, f.steer.left)) +
        0.5 * tr * (fx_body(WheelIndex::RearRight, 0.0) -
                    fx_body(WheelIndex::RearLeft, 0.0));

    State d;
    d.data[0] = s.vx() * std::cos(s.yaw()) - s.vy() * std::sin(s.yaw());
    d.data[1] = s.vx() * std::sin(s.yaw()) + s.vy() * std::cos(s.yaw());
    d.data[2] = s.yawRate();
    d.data[3] = f.ax + s.vy() * s.yawRate();
    d.data[4] = f.ay - s.vx() * s.yawRate();
    d.data[5] = mz / params.inertia_z;
    return d;
  }

  void normalizeState(State& s) const { s.yaw() = normalizeAngle(s.yaw()); }

  Input inputFromAcceleration(double ax, double steer) const {
    return Input::make(params.mass * ax, steer);
  }

 private:
  void distributeLoads(DoubleTrackForces& f, double ax, double ay) const {
    const double L = params.wheelBase();
    const double h = params.cg_height;
    const double m = params.mass;

    const double fz_front_axle =
        std::max(0.0, params.staticLoadFront() - m * ax * h / L);
    const double fz_rear_axle =
        std::max(0.0, params.staticLoadRear() + m * ax * h / L);

    const double kf = clampValue(dt_params.front_roll_stiffness_ratio, 0.0, 1.0);
    const double d_front = m * ay * h * kf / params.track_front;
    const double d_rear = m * ay * h * (1.0 - kf) / params.track_rear;

    // Positive a_y (left turn) unloads the left (inner) wheels.
    f.normal_load[WheelIndex::FrontLeft] =
        std::max(0.0, 0.5 * fz_front_axle - d_front);
    f.normal_load[WheelIndex::FrontRight] =
        std::max(0.0, 0.5 * fz_front_axle + d_front);
    f.normal_load[WheelIndex::RearLeft] =
        std::max(0.0, 0.5 * fz_rear_axle - d_rear);
    f.normal_load[WheelIndex::RearRight] =
        std::max(0.0, 0.5 * fz_rear_axle + d_rear);
  }

  void evaluateTires(DoubleTrackForces& f) const {
    for (std::size_t i = 0; i < 4; ++i) {
      const auto w = static_cast<WheelIndex>(i);
      const Tire& t = (i < 2) ? tire_front : tire_rear;
      double fy = t.lateralForce(f.slip_angle[w], f.normal_load[w]);
      if (dt_params.combined_slip) {
        fy *= tire::frictionEllipseScale(f.longitudinal[w], params.friction,
                                         f.normal_load[w]);
      }
      f.lateral[w] = fy;
    }
  }

  double longitudinalAcceleration(const DoubleTrackForces& f,
                                  const WheelAngles& steer) const {
    double fx = 0.0;
    fx += f.longitudinal[WheelIndex::FrontLeft] * std::cos(steer.left) -
          f.lateral[WheelIndex::FrontLeft] * std::sin(steer.left);
    fx += f.longitudinal[WheelIndex::FrontRight] * std::cos(steer.right) -
          f.lateral[WheelIndex::FrontRight] * std::sin(steer.right);
    fx += f.longitudinal[WheelIndex::RearLeft];
    fx += f.longitudinal[WheelIndex::RearRight];
    return fx / params.mass;
  }

  double lateralAcceleration(const DoubleTrackForces& f,
                             const WheelAngles& steer, double /*delta*/) const {
    double fy = 0.0;
    fy += f.longitudinal[WheelIndex::FrontLeft] * std::sin(steer.left) +
          f.lateral[WheelIndex::FrontLeft] * std::cos(steer.left);
    fy += f.longitudinal[WheelIndex::FrontRight] * std::sin(steer.right) +
          f.lateral[WheelIndex::FrontRight] * std::cos(steer.right);
    fy += f.lateral[WheelIndex::RearLeft];
    fy += f.lateral[WheelIndex::RearRight];
    return fy / params.mass;
  }

  static void setTireStiffness(tire::LinearTire& t, double c, double) {
    t.cornering_stiffness = c;
  }
  static void setTireStiffness(tire::FialaTire& t, double c, double) {
    t.cornering_stiffness = c;
  }
  static void setTireStiffness(tire::PacejkaTire& t, double c, double fz) {
    t.B = c / (t.C * t.friction * std::max(fz, 1.0));
  }
  template <typename T>
  static void setTireStiffness(T&, double, double) {}

  static void setTireFriction(tire::LinearTire& t, double mu) { t.friction = mu; }
  static void setTireFriction(tire::FialaTire& t, double mu) { t.friction = mu; }
  static void setTireFriction(tire::PacejkaTire& t, double mu) { t.friction = mu; }
  template <typename T>
  static void setTireFriction(T&, double) {}
};

}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_DOUBLE_TRACK_HPP
