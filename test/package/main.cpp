// SPDX-License-Identifier: Apache-2.0
// Verifies that the installed package exposes usable headers and targets.
#include <cstdio>

#include "vehicle_models/vehicle_models.hpp"

int main() {
  using namespace vehicle_models;
  const VehicleParameters p = makeShuttleParameters();
  if (!p.validate().empty()) return 1;

  KinematicBicycleModel model(p, ReferencePoint::RearAxle);
  auto x = KinematicBicycleState::make(0.0, 0.0, 0.0, 4.0);
  for (int i = 0; i < 100; ++i) {
    x = step(model, x, KinematicBicycleInput::make(0.0, deg2rad(8.0)), 0.01);
  }
  std::printf("pose = (%.3f, %.3f, %.3f deg), K = %.3f deg/g\n", x.x(), x.y(),
              rad2deg(x.yaw()), analysis::understeerGradientDegPerG(p));
  return (x.x() > 0.0 && x.y() > 0.0) ? 0 : 1;
}
