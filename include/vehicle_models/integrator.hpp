// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_INTEGRATOR_HPP
#define VEHICLE_MODELS_INTEGRATOR_HPP

#include "vehicle_models/types.hpp"

namespace vehicle_models {

/**
 * Every model in this library satisfies the following implicit concept:
 *
 *   using State = ...;                        // StateVector derived
 *   using Input = ...;                        // StateVector derived
 *   State derivative(const State&, const Input&) const;
 *   void  normalizeState(State&) const;       // wrap angles, clamp, ...
 *
 * which is all the integrators below need.
 */
enum class IntegratorType { Euler, Heun, RK4 };

template <typename Model>
typename Model::State stepEuler(const Model& model,
                                const typename Model::State& x,
                                const typename Model::Input& u, double dt) {
  typename Model::State next = x + model.derivative(x, u) * dt;
  model.normalizeState(next);
  return next;
}

/// Explicit trapezoidal (Heun) method, 2nd order.
template <typename Model>
typename Model::State stepHeun(const Model& model,
                               const typename Model::State& x,
                               const typename Model::Input& u, double dt) {
  const auto k1 = model.derivative(x, u);
  const auto k2 = model.derivative(x + k1 * dt, u);
  typename Model::State next = x + (k1 + k2) * (0.5 * dt);
  model.normalizeState(next);
  return next;
}

/// Classical Runge-Kutta, 4th order. Default for all simulations here.
template <typename Model>
typename Model::State stepRK4(const Model& model,
                              const typename Model::State& x,
                              const typename Model::Input& u, double dt) {
  const auto k1 = model.derivative(x, u);
  const auto k2 = model.derivative(x + k1 * (0.5 * dt), u);
  const auto k3 = model.derivative(x + k2 * (0.5 * dt), u);
  const auto k4 = model.derivative(x + k3 * dt, u);
  typename Model::State next =
      x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (dt / 6.0);
  model.normalizeState(next);
  return next;
}

/// Zero-order-hold step with a selectable method.
template <typename Model>
typename Model::State step(const Model& model, const typename Model::State& x,
                           const typename Model::Input& u, double dt,
                           IntegratorType method = IntegratorType::RK4) {
  switch (method) {
    case IntegratorType::Euler:
      return stepEuler(model, x, u, dt);
    case IntegratorType::Heun:
      return stepHeun(model, x, u, dt);
    case IntegratorType::RK4:
    default:
      return stepRK4(model, x, u, dt);
  }
}

/// Integrate over `duration` with fixed sub-steps of `dt`.
template <typename Model>
typename Model::State simulate(const Model& model,
                               const typename Model::State& x0,
                               const typename Model::Input& u, double duration,
                               double dt,
                               IntegratorType method = IntegratorType::RK4) {
  typename Model::State x = x0;
  double t = 0.0;
  while (t < duration - 1e-12) {
    const double h = (duration - t < dt) ? (duration - t) : dt;
    x = step(model, x, u, h, method);
    t += h;
  }
  return x;
}

}  // namespace vehicle_models

#endif  // VEHICLE_MODELS_INTEGRATOR_HPP
