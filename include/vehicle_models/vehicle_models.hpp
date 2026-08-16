// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
//
// vehicle_models — header-only C++17 vehicle dynamics model library.
//
//   UnicycleModel                 3 states, skid-steer / planner abstraction
//   DifferentialDriveModel        wheel rates in, unicycle body out
//   AckermannGeometry (free fn)   steering geometry, wheel angles and speeds
//   KinematicBicycleModel         4 states, rear axle / CoG / front axle
//   KinematicBicycleSteerModel    5 states, + steering lag and rate limit
//   LinearLateralBicycleModel     5 states, linear 2-DOF plant at constant v_x
//   DynamicBicycleModel<Tire>     6 states, nonlinear single track
//   BlendedBicycleModel<Tire>     6 states, kinematic <-> dynamic blending
//   DoubleTrackModel<Tire>        6 states, four wheels with load transfer
//
#ifndef VEHICLE_MODELS_VEHICLE_MODELS_HPP
#define VEHICLE_MODELS_VEHICLE_MODELS_HPP

#include "vehicle_models/ackermann.hpp"
#include "vehicle_models/double_track.hpp"
#include "vehicle_models/dynamic_bicycle.hpp"
#include "vehicle_models/integrator.hpp"
#include "vehicle_models/kinematic_bicycle.hpp"
#include "vehicle_models/linear_analysis.hpp"
#include "vehicle_models/tire/tire_models.hpp"
#include "vehicle_models/types.hpp"
#include "vehicle_models/unicycle.hpp"
#include "vehicle_models/vehicle_parameters.hpp"

#define VEHICLE_MODELS_VERSION_MAJOR 0
#define VEHICLE_MODELS_VERSION_MINOR 1
#define VEHICLE_MODELS_VERSION_PATCH 0

#endif  // VEHICLE_MODELS_VEHICLE_MODELS_HPP
