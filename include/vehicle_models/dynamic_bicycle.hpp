// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_DYNAMIC_BICYCLE_HPP
#define VEHICLE_MODELS_DYNAMIC_BICYCLE_HPP

#include "vehicle_models/kinematic_bicycle.hpp"
#include "vehicle_models/tire/tire_models.hpp"
#include "vehicle_models/types.hpp"
#include "vehicle_models/vehicle_parameters.hpp"

namespace vehicle_models {

/// [x, y, yaw, v_x, v_y, yaw_rate] — velocities in the body frame.
struct DynamicBicycleState : StateVector<DynamicBicycleState, 6> {
  double& x() { return data[0]; }
  double& y() { return data[1]; }
  double& yaw() { return data[2]; }
  double& vx() { return data[3]; }
  double& vy() { return data[4]; }
  double& yawRate() { return data[5]; }
  double x() const { return data[0]; }
  double y() const { return data[1]; }
  double yaw() const { return data[2]; }
  double vx() const { return data[3]; }
  double vy() const { return data[4]; }
  double yawRate() const { return data[5]; }

  static DynamicBicycleState make(double x, double y, double yaw, double vx,
                                  double vy, double yaw_rate) {
    DynamicBicycleState s;
    s.data = {x, y, yaw, vx, vy, yaw_rate};
    return s;
  }
  Pose2D pose() const { return Pose2D{x(), y(), yaw()}; }
  double speed() const { return std::hypot(vx(), vy()); }
  /// Body slip angle beta [rad].
  double sideSlip() const { return std::atan2(vy(), guardDenominator(vx(), 1e-6)); }
};

/// [F_x, steer] — F_x is the total longitudinal tire force [N].
struct DynamicBicycleInput : StateVector<DynamicBicycleInput, 2> {
  double& fx() { return data[0]; }
  double& steer() { return data[1]; }
  double fx() const { return data[0]; }
  double steer() const { return data[1]; }

  static DynamicBicycleInput make(double fx, double steer) {
    DynamicBicycleInput u;
    u.data = {fx, steer};
    return u;
  }
};

/// Intermediate quantities of one derivative evaluation. Useful for logging,
/// for plausibility monitors and for comparing models against each other.
struct BicycleForces {
  double slip_front = 0.0;   ///< [rad]
  double slip_rear = 0.0;    ///< [rad]
  double fz_front = 0.0;     ///< [N]
  double fz_rear = 0.0;      ///< [N]
  double fy_front = 0.0;     ///< [N]
  double fy_rear = 0.0;      ///< [N]
  double f_resist = 0.0;     ///< aero + rolling resistance [N]
  double ax = 0.0;           ///< body longitudinal acceleration [m/s^2]
  double ay = 0.0;           ///< body lateral acceleration [m/s^2]
};

/**
 * @brief Nonlinear 3-DOF single-track (bicycle) model.
 *
 *   m (vx_dot - vy*r) = Fx - Fyf sin(d) - Fres
 *   m (vy_dot + vx*r) = Fyf cos(d) + Fyr
 *   Iz r_dot          = l_f Fyf cos(d) - l_r Fyr
 *
 * Slip angles use a guarded 1/v_x, so the model stays finite at standstill,
 * but below `low_speed_guard` its lateral behaviour is not trustworthy —
 * use BlendedBicycleModel there.
 *
 * @tparam Tire tire model (LinearTire, FialaTire, PacejkaTire, ...).
 */
template <typename Tire = tire::LinearTire>
struct DynamicBicycleModel {
  using State = DynamicBicycleState;
  using Input = DynamicBicycleInput;
  using TireModel = Tire;

  VehicleParameters params;
  Tire tire_front;
  Tire tire_rear;
  bool longitudinal_load_transfer = true;

  DynamicBicycleModel() { syncTiresFromParams(); }
  explicit DynamicBicycleModel(const VehicleParameters& p) : params(p) {
    syncTiresFromParams();
  }

  /// Copy the axle cornering stiffness / friction of `params` into the tires.
  void syncTiresFromParams() {
    setTireStiffness(tire_front, params.cornering_stiffness_front,
                     params.staticLoadFront());
    setTireStiffness(tire_rear, params.cornering_stiffness_rear,
                     params.staticLoadRear());
  }

  BicycleForces computeForces(const State& s, const Input& u) const {
    BicycleForces f;
    const double delta = clampValue(u.steer(), -params.steer_max, params.steer_max);
    const double L = params.wheelBase();
    const double vx_g = guardDenominator(s.vx(), params.low_speed_guard);

    f.slip_front = delta - std::atan((s.vy() + params.l_f * s.yawRate()) / vx_g);
    f.slip_rear = -std::atan((s.vy() - params.l_r * s.yawRate()) / vx_g);

    // Quasi-static longitudinal load transfer, using the commanded Fx.
    const double ax_cmd = u.fx() / params.mass;
    const double dfz = longitudinal_load_transfer
                           ? params.mass * ax_cmd * params.cg_height / L
                           : 0.0;
    f.fz_front = std::max(0.0, params.staticLoadFront() - dfz);
    f.fz_rear = std::max(0.0, params.staticLoadRear() + dfz);

    f.fy_front = tire_front.lateralForce(f.slip_front, f.fz_front);
    f.fy_rear = tire_rear.lateralForce(f.slip_rear, f.fz_rear);

    f.f_resist = params.drag_area * s.vx() * std::fabs(s.vx()) +
                 params.rolling_resistance * params.mass * kGravity *
                     std::tanh(s.vx() / 0.1);

    f.ax = (u.fx() - f.fy_front * std::sin(delta) - f.f_resist) / params.mass;
    f.ay = (f.fy_front * std::cos(delta) + f.fy_rear) / params.mass;
    return f;
  }

  State derivative(const State& s, const Input& u) const {
    const double delta = clampValue(u.steer(), -params.steer_max, params.steer_max);
    const BicycleForces f = computeForces(s, u);

    State d;
    d.data[0] = s.vx() * std::cos(s.yaw()) - s.vy() * std::sin(s.yaw());
    d.data[1] = s.vx() * std::sin(s.yaw()) + s.vy() * std::cos(s.yaw());
    d.data[2] = s.yawRate();
    d.data[3] = f.ax + s.vy() * s.yawRate();
    d.data[4] = f.ay - s.vx() * s.yawRate();
    d.data[5] = (params.l_f * f.fy_front * std::cos(delta) -
                 params.l_r * f.fy_rear) /
                params.inertia_z;
    return d;
  }

  void normalizeState(State& s) const { s.yaw() = normalizeAngle(s.yaw()); }

  /// Lateral acceleration as an IMU at the CoG would measure it.
  double measuredLateralAcceleration(const State& s, const Input& u) const {
    return computeForces(s, u).ay;
  }

  /// Convenience: build the input from a desired longitudinal acceleration.
  Input inputFromAcceleration(double ax, double steer) const {
    return Input::make(params.mass * ax, steer);
  }

 private:
  static void setTireStiffness(tire::LinearTire& t, double c, double /*fz*/) {
    t.cornering_stiffness = c;
  }
  static void setTireStiffness(tire::FialaTire& t, double c, double /*fz*/) {
    t.cornering_stiffness = c;
  }
  static void setTireStiffness(tire::PacejkaTire& t, double c, double fz) {
    t.B = c / (t.C * t.friction * std::max(fz, 1.0));
  }
  template <typename T>
  static void setTireStiffness(T&, double, double) {}
};

// ---------------------------------------------------------------------------

/// [x, y, yaw, v_y, yaw_rate] at a constant longitudinal speed.
struct LateralBicycleState : StateVector<LateralBicycleState, 5> {
  double& x() { return data[0]; }
  double& y() { return data[1]; }
  double& yaw() { return data[2]; }
  double& vy() { return data[3]; }
  double& yawRate() { return data[4]; }
  double x() const { return data[0]; }
  double y() const { return data[1]; }
  double yaw() const { return data[2]; }
  double vy() const { return data[3]; }
  double yawRate() const { return data[4]; }

  static LateralBicycleState make(double x, double y, double yaw, double vy,
                                  double yaw_rate) {
    LateralBicycleState s;
    s.data = {x, y, yaw, vy, yaw_rate};
    return s;
  }
  Pose2D pose() const { return Pose2D{x(), y(), yaw()}; }
};

/// [steer]
struct SteerInput : StateVector<SteerInput, 1> {
  double& steer() { return data[0]; }
  double steer() const { return data[0]; }
  static SteerInput make(double steer) {
    SteerInput u;
    u.data = {steer};
    return u;
  }
};

/**
 * @brief Linear 2-DOF lateral model at constant v_x — the classic
 *        handling model, and the plant most lateral controllers are
 *        designed against.
 *
 *   [vy_dot]   [a11 a12][vy]   [b1]
 *   [ r_dot] = [a21 a22][ r] + [b2] delta
 */
struct LinearLateralBicycleModel {
  using State = LateralBicycleState;
  using Input = SteerInput;

  VehicleParameters params;
  double longitudinal_speed = 10.0;  ///< [m/s]

  LinearLateralBicycleModel() = default;
  LinearLateralBicycleModel(const VehicleParameters& p, double vx)
      : params(p), longitudinal_speed(vx) {}

  /// Continuous-time A matrix of [v_y, r].
  std::array<std::array<double, 2>, 2> stateMatrix() const {
    const double m = params.mass;
    const double iz = params.inertia_z;
    const double cf = params.cornering_stiffness_front;
    const double cr = params.cornering_stiffness_rear;
    const double lf = params.l_f;
    const double lr = params.l_r;
    const double vx = guardDenominator(longitudinal_speed, params.low_speed_guard);
    return {{{-(cf + cr) / (m * vx), -vx - (lf * cf - lr * cr) / (m * vx)},
             {-(lf * cf - lr * cr) / (iz * vx),
              -(lf * lf * cf + lr * lr * cr) / (iz * vx)}}};
  }

  /// Continuous-time B matrix of [v_y, r] for the steer input.
  std::array<double, 2> inputMatrix() const {
    const double cf = params.cornering_stiffness_front;
    return {cf / params.mass, params.l_f * cf / params.inertia_z};
  }

  State derivative(const State& s, const Input& u) const {
    const double delta = clampValue(u.steer(), -params.steer_max, params.steer_max);
    const auto a = stateMatrix();
    const auto b = inputMatrix();
    const double vx = longitudinal_speed;

    State d;
    d.data[0] = vx * std::cos(s.yaw()) - s.vy() * std::sin(s.yaw());
    d.data[1] = vx * std::sin(s.yaw()) + s.vy() * std::cos(s.yaw());
    d.data[2] = s.yawRate();
    d.data[3] = a[0][0] * s.vy() + a[0][1] * s.yawRate() + b[0] * delta;
    d.data[4] = a[1][0] * s.vy() + a[1][1] * s.yawRate() + b[1] * delta;
    return d;
  }

  void normalizeState(State& s) const { s.yaw() = normalizeAngle(s.yaw()); }
};

// ---------------------------------------------------------------------------

/**
 * @brief Kinematic / dynamic blended model.
 *
 * Below `blend_speed_low` the lateral states are driven toward the values the
 * kinematic bicycle would produce (first order, time constant
 * `blend_time_constant`); above `blend_speed_high` the model is purely
 * dynamic. This removes the low-speed singularity of the single-track model
 * while keeping the dynamic behaviour where it matters — the standard
 * arrangement for shuttle / valet speed ranges.
 */
template <typename Tire = tire::LinearTire>
struct BlendedBicycleModel {
  using State = DynamicBicycleState;
  using Input = DynamicBicycleInput;

  DynamicBicycleModel<Tire> dynamic;
  double blend_speed_low = 1.0;         ///< [m/s]
  double blend_speed_high = 4.0;        ///< [m/s]
  double blend_time_constant = 0.10;    ///< [s]

  BlendedBicycleModel() = default;
  explicit BlendedBicycleModel(const VehicleParameters& p) : dynamic(p) {}

  const VehicleParameters& params() const { return dynamic.params; }

  double blendFactor(double vx) const {
    const double a = std::fabs(vx);
    if (blend_speed_high <= blend_speed_low) return a >= blend_speed_high ? 1.0 : 0.0;
    return clampValue((a - blend_speed_low) / (blend_speed_high - blend_speed_low),
                      0.0, 1.0);
  }

  State derivative(const State& s, const Input& u) const {
    const auto& p = dynamic.params;
    const double delta = clampValue(u.steer(), -p.steer_max, p.steer_max);
    const double lambda = blendFactor(s.vx());
    State d = dynamic.derivative(s, u);
    if (lambda >= 1.0) return d;

    // Kinematic reference for the lateral states.
    const double L = p.wheelBase();
    const double beta_kin = std::atan(p.l_r * std::tan(delta) / L);
    const double r_kin = s.vx() * std::cos(beta_kin) * std::tan(delta) / L;
    const double vy_kin = s.vx() * std::tan(beta_kin);
    const double tau = std::max(blend_time_constant, 1e-3);

    const double vy_dot_kin = (vy_kin - s.vy()) / tau;
    const double r_dot_kin = (r_kin - s.yawRate()) / tau;
    const double f_resist = p.drag_area * s.vx() * std::fabs(s.vx()) +
                            p.rolling_resistance * p.mass * kGravity *
                                std::tanh(s.vx() / 0.1);
    const double vx_dot_kin = (u.fx() - f_resist) / p.mass;

    d.data[3] = lambda * d.data[3] + (1.0 - lambda) * vx_dot_kin;
    d.data[4] = lambda * d.data[4] + (1.0 - lambda) * vy_dot_kin;
    d.data[5] = lambda * d.data[5] + (1.0 - lambda) * r_dot_kin;
    return d;
  }

  void normalizeState(State& s) const { s.yaw() = normalizeAngle(s.yaw()); }
};

}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_DYNAMIC_BICYCLE_HPP
