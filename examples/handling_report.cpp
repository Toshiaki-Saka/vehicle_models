// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
//
// Prints the steering geometry table and the linear handling metrics of the
// three built-in vehicle presets.
//
#include <cstdio>
#include <string>
#include <vector>

#include "vehicle_models/vehicle_models.hpp"

using namespace vehicle_models;

namespace {

void printGeometryTable(const std::string& name, const VehicleParameters& p) {
  const AckermannGeometry g = AckermannGeometry::from(p);
  std::printf("\n--- %s : steering geometry (L=%.2f m, T=%.2f m) ---\n",
              name.c_str(), g.wheel_base, g.track_front);
  std::printf("%10s %10s %10s %10s %12s\n", "delta[deg]", "inner[deg]",
              "outer[deg]", "R_rear[m]", "ack_err[deg]");
  for (double d = 5.0; d <= rad2deg(p.steer_max) + 1e-9; d += 5.0) {
    const double delta = deg2rad(d);
    const auto wa = roadWheelAngles(g, delta);
    std::printf("%10.1f %10.2f %10.2f %10.2f %12.3f\n", d, rad2deg(wa.left),
                rad2deg(wa.right), turnRadius(g, delta),
                rad2deg(ackermannError(g, delta)));
  }
  std::printf("minimum turn radius at full lock: %.2f m\n",
              minimumTurnRadius(g, p.steer_max));
}

void printHandlingMetrics(const std::string& name, const VehicleParameters& p) {
  std::printf("\n--- %s : linear handling metrics ---\n", name.c_str());
  const auto errors = p.validate();
  if (!errors.empty()) {
    for (const auto& e : errors) std::printf("  parameter error: %s\n", e.c_str());
    return;
  }
  std::printf("understeer gradient : %+.4f rad/(m/s^2)  (%+.3f deg/g)\n",
              analysis::understeerGradient(p), analysis::understeerGradientDegPerG(p));
  std::printf("static margin       : %+.4f\n", analysis::staticMargin(p));
  const double v_ch = analysis::characteristicSpeed(p);
  const double v_cr = analysis::criticalSpeed(p);
  if (std::isfinite(v_ch))
    std::printf("characteristic speed: %.2f m/s (%.1f km/h)\n", v_ch, v_ch * 3.6);
  if (std::isfinite(v_cr))
    std::printf("critical speed      : %.2f m/s (%.1f km/h)  <-- oversteer\n",
                v_cr, v_cr * 3.6);

  std::printf("%8s %14s %14s %12s %10s\n", "v[m/s]", "r/delta[1/s]", "ay/delta",
              "wn[rad/s]", "zeta");
  for (double v = 2.0; v <= p.speed_max + 1e-9; v += std::max(2.0, p.speed_max / 6.0)) {
    const auto mode = analysis::yawMode(p, v);
    std::printf("%8.1f %14.4f %14.4f %12.3f %10.3f%s\n", v,
                analysis::yawRateGain(p, v), analysis::lateralAccelerationGain(p, v),
                mode.natural_frequency, mode.damping_ratio,
                mode.stable ? "" : "  UNSTABLE");
  }
}

}  // namespace

int main() {
  const std::vector<std::pair<std::string, VehicleParameters>> presets = {
      {"passenger car", makePassengerCarParameters()},
      {"automated shuttle", makeShuttleParameters()},
      {"buggy", makeBuggyParameters()},
  };

  for (const auto& kv : presets) {
    printGeometryTable(kv.first, kv.second);
    printHandlingMetrics(kv.first, kv.second);
  }
  return 0;
}
