// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
//
// Step steer (J-turn) at constant speed, run through three models at once.
// Writes CSV to stdout:
//   step_steer > step_steer.csv
//
#include <cstdio>

#include "vehicle_models/vehicle_models.hpp"

using namespace vehicle_models;

int main() {
  const VehicleParameters p = makePassengerCarParameters();

  const double vx = 20.0;            // [m/s]
  const double delta = deg2rad(3.0); // step steer amplitude
  const double dt = 0.002;           // [s]
  const double t_step = 0.5;         // [s]
  const double t_end = 5.0;          // [s]

  KinematicBicycleModel kinematic(p, ReferencePoint::CenterOfGravity);
  DynamicBicycleModel<tire::LinearTire> linear_tire(p);
  DoubleTrackModel<tire::FialaTire> double_track(p);

  auto x_kin = KinematicBicycleState::make(0, 0, 0, vx);
  auto x_dyn = DynamicBicycleState::make(0, 0, 0, vx, 0, 0);
  auto x_dtr = DynamicBicycleState::make(0, 0, 0, vx, 0, 0);

  std::printf("t,steer_deg,r_kin,r_dyn,r_dtr,beta_dyn_deg,ay_dyn,ay_dtr\n");
  for (double t = 0.0; t <= t_end + 1e-9; t += dt) {
    const double steer = (t >= t_step) ? delta : 0.0;

    const auto u_kin = KinematicBicycleInput::make(0.0, steer);
    const auto u_dyn = DynamicBicycleInput::make(0.0, steer);

    const double r_kin = x_kin.v() * std::tan(steer) / p.wheelBase();
    const double ay_dyn = linear_tire.measuredLateralAcceleration(x_dyn, u_dyn);
    const double ay_dtr = double_track.computeForces(x_dtr, u_dyn).ay;

    std::printf("%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", t, rad2deg(steer),
                r_kin, x_dyn.yawRate(), x_dtr.yawRate(),
                rad2deg(x_dyn.sideSlip()), ay_dyn, ay_dtr);

    x_kin = step(kinematic, x_kin, u_kin, dt);
    x_dyn = step(linear_tire, x_dyn, u_dyn, dt);
    x_dtr = step(double_track, x_dtr, u_dyn, dt);
  }

  const auto ss = analysis::steadyStateCornering(p, vx, delta);
  std::fprintf(stderr, "closed-form steady state: r=%.4f rad/s, beta=%.3f deg, ay=%.3f m/s^2\n",
               ss.yaw_rate, rad2deg(ss.side_slip), ss.lateral_accel);
  return 0;
}
