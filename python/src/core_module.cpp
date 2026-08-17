// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
//
// pybind11 bindings for the header-only vehicle_models C++ library.
//
// This module is the ONLY implementation of the equations of motion reachable
// from Python. The modules in vehicle_models_py/ are thin re-export shims over
// it, so a change to include/vehicle_models/*.hpp is picked up by the GUI as
// soon as the extension is rebuilt.
//
// Two things need a wrapper rather than a direct binding:
//
//   1. The integrators in integrator.hpp are templates over a model with a
//      fixed-size State. Python needs one `step()` that takes any model, so the
//      models are type-erased behind ModelBase, whose State is the dynamically
//      sized DynVec. The integrator templates are then instantiated once, on
//      ModelBase, and the actual RK4 code still comes from the library header.
//
//   2. DynamicBicycleModel / BlendedBicycleModel / DoubleTrackModel are
//      templates over the tire type. Python picks the tire at runtime, so each
//      wrapper holds a std::variant over the three instantiations and dispatches
//      with std::visit. include/ stays untouched.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "vehicle_models/vehicle_models.hpp"

namespace py = pybind11;
using namespace vehicle_models;

namespace {

// ---------------------------------------------------------------------------
// Dynamically sized state vector: the arithmetic integrator.hpp requires.
// ---------------------------------------------------------------------------

struct DynVec {
  std::vector<double> data;

  DynVec() = default;
  explicit DynVec(std::size_t n) : data(n, 0.0) {}
};

inline DynVec operator+(const DynVec& a, const DynVec& b) {
  DynVec r = a;
  for (std::size_t i = 0; i < r.data.size(); ++i) r.data[i] += b.data[i];
  return r;
}
inline DynVec operator*(const DynVec& a, double s) {
  DynVec r = a;
  for (auto& v : r.data) v *= s;
  return r;
}
inline DynVec operator*(double s, const DynVec& a) { return a * s; }

// ---------------------------------------------------------------------------
// numpy <-> fixed-size StateVector conversion
// ---------------------------------------------------------------------------

using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;

template <typename S>
S toFixed(const DynVec& v, const char* what) {
  if (v.data.size() != S::kDim) {
    throw py::value_error(std::string(what) + ": expected " +
                          std::to_string(S::kDim) + " elements, got " +
                          std::to_string(v.data.size()));
  }
  S s;
  for (std::size_t i = 0; i < S::kDim; ++i) s.data[i] = v.data[i];
  return s;
}

template <typename S>
DynVec fromFixed(const S& s) {
  DynVec v(S::kDim);
  for (std::size_t i = 0; i < S::kDim; ++i) v.data[i] = s.data[i];
  return v;
}

DynVec toDyn(const Array& a, const char* what) {
  if (a.ndim() != 1) {
    throw py::value_error(std::string(what) + ": expected a 1-D array");
  }
  DynVec v(static_cast<std::size_t>(a.shape(0)));
  const auto r = a.template unchecked<1>();
  for (std::size_t i = 0; i < v.data.size(); ++i)
    v.data[i] = r(static_cast<py::ssize_t>(i));
  return v;
}

py::array_t<double> toNumpy(const DynVec& v) {
  py::array_t<double> out(static_cast<py::ssize_t>(v.data.size()));
  auto w = out.mutable_unchecked<1>();
  for (std::size_t i = 0; i < v.data.size(); ++i)
    w(static_cast<py::ssize_t>(i)) = v.data[i];
  return out;
}

// ---------------------------------------------------------------------------
// Type-erased model. State = DynVec, which is all integrator.hpp needs.
// ---------------------------------------------------------------------------

class ModelBase {
 public:
  using State = DynVec;
  using Input = DynVec;

  virtual ~ModelBase() = default;
  virtual DynVec derivative(const DynVec& x, const DynVec& u) const = 0;
  virtual void normalizeState(DynVec& x) const = 0;
  virtual std::size_t nStates() const = 0;
};

// ---------------------------------------------------------------------------
// Tire dispatch
// ---------------------------------------------------------------------------

enum class TireKind { Linear = 0, Fiala = 1, Pacejka = 2 };

TireKind tireKindOf(const py::object& o, const char* role) {
  if (py::isinstance<tire::LinearTire>(o)) return TireKind::Linear;
  if (py::isinstance<tire::FialaTire>(o)) return TireKind::Fiala;
  if (py::isinstance<tire::PacejkaTire>(o)) return TireKind::Pacejka;
  throw py::type_error(std::string(role) +
                       " must be a LinearTire, FialaTire or PacejkaTire");
}

/// The C++ templates carry one tire type for both axles, so the two tires have
/// to agree. The Python port used to allow mixing them; that was a capability
/// the C++ library never had.
TireKind tirePairKind(const py::object& front, const py::object& rear) {
  const TireKind kf = tireKindOf(front, "tire_front");
  const TireKind kr = tireKindOf(rear, "tire_rear");
  if (kf != kr) {
    throw py::type_error(
        "tire_front and tire_rear must be the same tire model: the C++ "
        "templates are parameterised on a single tire type");
  }
  return kf;
}

/// Deduced from the member rather than a `TireModel` typedef: only
/// DynamicBicycleModel declares one, DoubleTrackModel does not.
template <typename Model>
void assignTires(Model& m, const py::object& front, const py::object& rear) {
  using T = std::decay_t<decltype(m.tire_front)>;
  m.tire_front = front.cast<T>();
  m.tire_rear = rear.cast<T>();
}

template <typename Variant>
py::object getTireOf(const Variant& v, bool front) {
  return std::visit(
      [&](const auto& mm) -> py::object {
        return py::cast(front ? mm.tire_front : mm.tire_rear);
      },
      v);
}

template <typename Variant>
void setTireOf(Variant& v, TireKind kind, bool front, const py::object& t) {
  if (tireKindOf(t, front ? "tire_front" : "tire_rear") != kind) {
    throw py::type_error(
        "the replacement tire must be the same model as the one the object "
        "was built with");
  }
  std::visit(
      [&](auto& mm) {
        using T = std::decay_t<decltype(mm.tire_front)>;
        (front ? mm.tire_front : mm.tire_rear) = t.cast<T>();
      },
      v);
}

// ---------------------------------------------------------------------------
// Wrappers
// ---------------------------------------------------------------------------

class PyUnicycle : public ModelBase {
 public:
  UnicycleModel model;

  DynVec derivative(const DynVec& x, const DynVec& u) const override {
    return fromFixed(model.derivative(toFixed<UnicycleState>(x, "state"),
                                      toFixed<UnicycleInput>(u, "input")));
  }
  void normalizeState(DynVec& x) const override {
    auto s = toFixed<UnicycleState>(x, "state");
    model.normalizeState(s);
    x = fromFixed(s);
  }
  std::size_t nStates() const override { return UnicycleState::kDim; }
};

class PyDifferentialDrive : public ModelBase {
 public:
  DifferentialDriveModel model;

  PyDifferentialDrive() = default;
  explicit PyDifferentialDrive(const DifferentialDriveParams& p) : model(p) {}

  DynVec derivative(const DynVec& x, const DynVec& u) const override {
    return fromFixed(model.derivative(toFixed<UnicycleState>(x, "state"),
                                      toFixed<WheelRateInput>(u, "input")));
  }
  void normalizeState(DynVec& x) const override {
    auto s = toFixed<UnicycleState>(x, "state");
    model.normalizeState(s);
    x = fromFixed(s);
  }
  std::size_t nStates() const override { return UnicycleState::kDim; }
};

class PyKinematicBicycle : public ModelBase {
 public:
  KinematicBicycleModel model;

  PyKinematicBicycle() = default;
  PyKinematicBicycle(const VehicleParameters& p, ReferencePoint ref)
      : model(p, ref) {}

  DynVec derivative(const DynVec& x, const DynVec& u) const override {
    return fromFixed(
        model.derivative(toFixed<KinematicBicycleState>(x, "state"),
                         toFixed<KinematicBicycleInput>(u, "input")));
  }
  void normalizeState(DynVec& x) const override {
    auto s = toFixed<KinematicBicycleState>(x, "state");
    model.normalizeState(s);
    x = fromFixed(s);
  }
  std::size_t nStates() const override { return KinematicBicycleState::kDim; }
};

class PyKinematicBicycleSteer : public ModelBase {
 public:
  KinematicBicycleSteerModel model;

  PyKinematicBicycleSteer() = default;
  explicit PyKinematicBicycleSteer(const PyKinematicBicycle& base) {
    model.base = base.model;
  }

  DynVec derivative(const DynVec& x, const DynVec& u) const override {
    return fromFixed(
        model.derivative(toFixed<SteerDynamicsState>(x, "state"),
                         toFixed<KinematicBicycleInput>(u, "input")));
  }
  void normalizeState(DynVec& x) const override {
    auto s = toFixed<SteerDynamicsState>(x, "state");
    model.normalizeState(s);
    x = fromFixed(s);
  }
  std::size_t nStates() const override { return SteerDynamicsState::kDim; }
};

class PyLinearLateral : public ModelBase {
 public:
  LinearLateralBicycleModel model;

  PyLinearLateral() = default;
  PyLinearLateral(const VehicleParameters& p, double vx) : model(p, vx) {}

  DynVec derivative(const DynVec& x, const DynVec& u) const override {
    return fromFixed(model.derivative(toFixed<LateralBicycleState>(x, "state"),
                                      toFixed<SteerInput>(u, "input")));
  }
  void normalizeState(DynVec& x) const override {
    auto s = toFixed<LateralBicycleState>(x, "state");
    model.normalizeState(s);
    x = fromFixed(s);
  }
  std::size_t nStates() const override { return LateralBicycleState::kDim; }

  py::array_t<double> stateMatrix() const {
    const auto a = model.stateMatrix();
    py::array_t<double> out({py::ssize_t(2), py::ssize_t(2)});
    auto w = out.mutable_unchecked<2>();
    for (py::ssize_t i = 0; i < 2; ++i)
      for (py::ssize_t j = 0; j < 2; ++j)
        w(i, j) = a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
    return out;
  }
  py::array_t<double> inputMatrix() const {
    const auto b = model.inputMatrix();
    py::array_t<double> out(py::ssize_t(2));
    auto w = out.mutable_unchecked<1>();
    w(0) = b[0];
    w(1) = b[1];
    return out;
  }
};

/// Common plumbing for the three tire-templated models.
template <typename Variant>
class TireVariantModel : public ModelBase {
 public:
  Variant m;
  TireKind kind = TireKind::Linear;

  DynVec derivative(const DynVec& x, const DynVec& u) const override {
    return std::visit(
        [&](const auto& model) {
          return fromFixed(
              model.derivative(toFixed<DynamicBicycleState>(x, "state"),
                               toFixed<DynamicBicycleInput>(u, "input")));
        },
        m);
  }
  void normalizeState(DynVec& x) const override {
    auto s = toFixed<DynamicBicycleState>(x, "state");
    std::visit([&](const auto& model) { model.normalizeState(s); }, m);
    x = fromFixed(s);
  }
  std::size_t nStates() const override { return DynamicBicycleState::kDim; }
};

using DynBicycleVariant = std::variant<DynamicBicycleModel<tire::LinearTire>,
                                       DynamicBicycleModel<tire::FialaTire>,
                                       DynamicBicycleModel<tire::PacejkaTire>>;

class PyDynamicBicycle : public TireVariantModel<DynBicycleVariant> {
 public:
  PyDynamicBicycle() { m.emplace<0>(); }

  PyDynamicBicycle(const VehicleParameters& p, const py::object& front,
                   const py::object& rear) {
    kind = tirePairKind(front, rear);
    switch (kind) {
      case TireKind::Linear: {
        DynamicBicycleModel<tire::LinearTire> mm(p);
        assignTires(mm, front, rear);
        mm.syncTiresFromParams();
        m = mm;
        break;
      }
      case TireKind::Fiala: {
        DynamicBicycleModel<tire::FialaTire> mm(p);
        assignTires(mm, front, rear);
        mm.syncTiresFromParams();
        m = mm;
        break;
      }
      case TireKind::Pacejka: {
        DynamicBicycleModel<tire::PacejkaTire> mm(p);
        assignTires(mm, front, rear);
        mm.syncTiresFromParams();
        m = mm;
        break;
      }
    }
  }

  VehicleParameters params() const {
    return std::visit([](const auto& mm) { return mm.params; }, m);
  }
  void setParams(const VehicleParameters& p) {
    std::visit([&](auto& mm) { mm.params = p; }, m);
  }
  bool loadTransfer() const {
    return std::visit([](const auto& mm) { return mm.longitudinal_load_transfer; },
                      m);
  }
  void setLoadTransfer(bool v) {
    std::visit([&](auto& mm) { mm.longitudinal_load_transfer = v; }, m);
  }

  /// `sync_friction` is the one addition over the C++ API: the C++
  /// syncTiresFromParams() propagates the stiffness only, which leaves a
  /// single-track model built from a low-mu preset running on mu = 1.
  void syncTiresFromParams(bool sync_friction) {
    std::visit(
        [&](auto& mm) {
          if (sync_friction) {
            mm.tire_front.friction = mm.params.friction;
            mm.tire_rear.friction = mm.params.friction;
          }
          mm.syncTiresFromParams();
        },
        m);
  }

  BicycleForces computeForces(const Array& s, const Array& u) const {
    const auto x = toFixed<DynamicBicycleState>(toDyn(s, "state"), "state");
    const auto i = toFixed<DynamicBicycleInput>(toDyn(u, "input"), "input");
    return std::visit([&](const auto& mm) { return mm.computeForces(x, i); }, m);
  }

  double measuredLateralAcceleration(const Array& s, const Array& u) const {
    return computeForces(s, u).ay;
  }

  py::array_t<double> inputFromAcceleration(double ax, double steer) const {
    const auto u = std::visit(
        [&](const auto& mm) { return mm.inputFromAcceleration(ax, steer); }, m);
    return toNumpy(fromFixed(u));
  }

  py::object tire(bool front) const { return getTireOf(m, front); }
  void setTire(bool front, const py::object& t) { setTireOf(m, kind, front, t); }
};

using BlendedVariant = std::variant<BlendedBicycleModel<tire::LinearTire>,
                                    BlendedBicycleModel<tire::FialaTire>,
                                    BlendedBicycleModel<tire::PacejkaTire>>;

class PyBlendedBicycle : public TireVariantModel<BlendedVariant> {
 public:
  PyBlendedBicycle() { m.emplace<0>(); }

  PyBlendedBicycle(const PyDynamicBicycle& core, double lo, double hi,
                   double tau) {
    kind = core.kind;
    std::visit(
        [&](const auto& dyn) {
          using Tire = typename std::decay_t<decltype(dyn)>::TireModel;
          BlendedBicycleModel<Tire> mm;
          mm.dynamic = dyn;
          mm.blend_speed_low = lo;
          mm.blend_speed_high = hi;
          mm.blend_time_constant = tau;
          m = mm;
        },
        core.m);
  }

  VehicleParameters params() const {
    return std::visit([](const auto& mm) { return mm.dynamic.params; }, m);
  }
  double blendFactor(double vx) const {
    return std::visit([&](const auto& mm) { return mm.blendFactor(vx); }, m);
  }
  BicycleForces computeForces(const Array& s, const Array& u) const {
    const auto x = toFixed<DynamicBicycleState>(toDyn(s, "state"), "state");
    const auto i = toFixed<DynamicBicycleInput>(toDyn(u, "input"), "input");
    return std::visit(
        [&](const auto& mm) { return mm.dynamic.computeForces(x, i); }, m);
  }
  py::array_t<double> inputFromAcceleration(double ax, double steer) const {
    const auto u = std::visit(
        [&](const auto& mm) { return mm.dynamic.inputFromAcceleration(ax, steer); },
        m);
    return toNumpy(fromFixed(u));
  }

#define VM_BLEND_PROP(name, member)                                          \
  double name() const {                                                      \
    return std::visit([](const auto& mm) { return mm.member; }, m);          \
  }                                                                          \
  void set_##name(double v) {                                                \
    std::visit([&](auto& mm) { mm.member = v; }, m);                         \
  }
  VM_BLEND_PROP(blendSpeedLow, blend_speed_low)
  VM_BLEND_PROP(blendSpeedHigh, blend_speed_high)
  VM_BLEND_PROP(blendTimeConstant, blend_time_constant)
#undef VM_BLEND_PROP
};

using DoubleTrackVariant = std::variant<DoubleTrackModel<tire::LinearTire>,
                                        DoubleTrackModel<tire::FialaTire>,
                                        DoubleTrackModel<tire::PacejkaTire>>;

class PyDoubleTrack : public TireVariantModel<DoubleTrackVariant> {
 public:
  PyDoubleTrack() { m.emplace<1>(); kind = TireKind::Fiala; }

  PyDoubleTrack(const VehicleParameters& p, const DoubleTrackParams& dtp,
                const py::object& front, const py::object& rear) {
    kind = tirePairKind(front, rear);
    switch (kind) {
      case TireKind::Linear: {
        DoubleTrackModel<tire::LinearTire> mm(p);
        mm.dt_params = dtp;
        assignTires(mm, front, rear);
        mm.syncTiresFromParams();
        m = mm;
        break;
      }
      case TireKind::Fiala: {
        DoubleTrackModel<tire::FialaTire> mm(p);
        mm.dt_params = dtp;
        assignTires(mm, front, rear);
        mm.syncTiresFromParams();
        m = mm;
        break;
      }
      case TireKind::Pacejka: {
        DoubleTrackModel<tire::PacejkaTire> mm(p);
        mm.dt_params = dtp;
        assignTires(mm, front, rear);
        mm.syncTiresFromParams();
        m = mm;
        break;
      }
    }
  }

  VehicleParameters params() const {
    return std::visit([](const auto& mm) { return mm.params; }, m);
  }
  void setParams(const VehicleParameters& p) {
    std::visit([&](auto& mm) { mm.params = p; }, m);
  }
  /// Returned by reference so `model.dt_params.combined_slip = True` sticks.
  DoubleTrackParams& dtParams() {
    return std::visit([](auto& mm) -> DoubleTrackParams& { return mm.dt_params; },
                      m);
  }
  void setDtParams(const DoubleTrackParams& p) {
    std::visit([&](auto& mm) { mm.dt_params = p; }, m);
  }
  void syncTiresFromParams() {
    std::visit([](auto& mm) { mm.syncTiresFromParams(); }, m);
  }
  AckermannGeometry geometry() const {
    return std::visit([](const auto& mm) { return mm.geometry(); }, m);
  }
  DoubleTrackForces computeForces(const Array& s, const Array& u) const {
    const auto x = toFixed<DynamicBicycleState>(toDyn(s, "state"), "state");
    const auto i = toFixed<DynamicBicycleInput>(toDyn(u, "input"), "input");
    return std::visit([&](const auto& mm) { return mm.computeForces(x, i); }, m);
  }
  py::array_t<double> inputFromAcceleration(double ax, double steer) const {
    const auto u = std::visit(
        [&](const auto& mm) { return mm.inputFromAcceleration(ax, steer); }, m);
    return toNumpy(fromFixed(u));
  }

  py::object tire(bool front) const { return getTireOf(m, front); }
  void setTire(bool front, const py::object& t) { setTireOf(m, kind, front, t); }
};

// ---------------------------------------------------------------------------
// Python-facing integrator entry points
// ---------------------------------------------------------------------------

py::array_t<double> pyDerivative(const ModelBase& model, const Array& x,
                                 const Array& u) {
  return toNumpy(model.derivative(toDyn(x, "state"), toDyn(u, "input")));
}

/// Mutates `x` in place and returns it, matching the previous pure-Python
/// behaviour that the integrators and the GUI rely on.
Array pyNormalizeState(const ModelBase& model, Array x) {
  DynVec v = toDyn(x, "state");
  model.normalizeState(v);
  auto w = x.mutable_unchecked<1>();
  for (std::size_t i = 0; i < v.data.size(); ++i)
    w(static_cast<py::ssize_t>(i)) = v.data[i];
  return x;
}

py::array_t<double> pyStep(const ModelBase& model, const Array& x,
                           const Array& u, double dt, IntegratorType method) {
  return toNumpy(
      step(model, toDyn(x, "state"), toDyn(u, "input"), dt, method));
}

py::array_t<double> pySimulate(const ModelBase& model, const Array& x0,
                               const Array& u, double duration, double dt,
                               IntegratorType method) {
  return toNumpy(simulate(model, toDyn(x0, "state"), toDyn(u, "input"),
                          duration, dt, method));
}

template <typename Fn>
py::array_t<double> pyOneStep(Fn fn, const ModelBase& model, const Array& x,
                              const Array& u, double dt) {
  return toNumpy(fn(model, toDyn(x, "state"), toDyn(u, "input"), dt));
}

py::array_t<double> wheelQuantityToNumpy(const WheelQuantities& q) {
  py::array_t<double> out(py::ssize_t(4));
  auto w = out.mutable_unchecked<1>();
  for (py::ssize_t i = 0; i < 4; ++i) w(i) = q.value[static_cast<std::size_t>(i)];
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------

PYBIND11_MODULE(_core, mod) {
  mod.doc() =
      "pybind11 bindings for the vehicle_models C++ library. The equations of "
      "motion live in include/vehicle_models/*.hpp and nowhere else.";

  // --- types.hpp -----------------------------------------------------------
  mod.attr("PI") = kPi;
  mod.attr("GRAVITY") = kGravity;
  // py::vectorize keeps the C++ function as the implementation while letting
  // the plotting code pass whole numpy arrays, which the previous pure-Python
  // helpers supported implicitly. Scalar in, scalar out is preserved.
  mod.def("deg2rad", py::vectorize(deg2rad), py::arg("deg"));
  mod.def("rad2deg", py::vectorize(rad2deg), py::arg("rad"));
  mod.def("normalize_angle", py::vectorize(normalizeAngle), py::arg("angle"),
          "Wrap an angle into [-pi, pi).");
  mod.def("clamp_value", py::vectorize(clampValue), py::arg("v"), py::arg("lo"),
          py::arg("hi"));
  mod.def("signum", py::vectorize(signum), py::arg("v"));
  mod.def("guard_denominator", py::vectorize(guardDenominator), py::arg("v"),
          py::arg("eps"), "Keep |v| >= eps while preserving the sign.");

  py::class_<Pose2D>(mod, "Pose2D")
      .def(py::init<>())
      .def(py::init([](double x, double y, double yaw) {
             return Pose2D{x, y, yaw};
           }),
           py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("yaw") = 0.0)
      .def_readwrite("x", &Pose2D::x)
      .def_readwrite("y", &Pose2D::y)
      .def_readwrite("yaw", &Pose2D::yaw)
      .def("__repr__", [](const Pose2D& p) {
        return "Pose2D(x=" + std::to_string(p.x) + ", y=" + std::to_string(p.y) +
               ", yaw=" + std::to_string(p.yaw) + ")";
      });

  // --- vehicle_parameters.hpp ---------------------------------------------
  py::class_<VehicleParameters>(mod, "VehicleParameters")
      .def(py::init<>())
      .def_readwrite("l_f", &VehicleParameters::l_f)
      .def_readwrite("l_r", &VehicleParameters::l_r)
      .def_readwrite("track_front", &VehicleParameters::track_front)
      .def_readwrite("track_rear", &VehicleParameters::track_rear)
      .def_readwrite("cg_height", &VehicleParameters::cg_height)
      .def_readwrite("wheel_radius", &VehicleParameters::wheel_radius)
      .def_readwrite("mass", &VehicleParameters::mass)
      .def_readwrite("inertia_z", &VehicleParameters::inertia_z)
      .def_readwrite("cornering_stiffness_front",
                     &VehicleParameters::cornering_stiffness_front)
      .def_readwrite("cornering_stiffness_rear",
                     &VehicleParameters::cornering_stiffness_rear)
      .def_readwrite("friction", &VehicleParameters::friction)
      .def_readwrite("drag_area", &VehicleParameters::drag_area)
      .def_readwrite("rolling_resistance", &VehicleParameters::rolling_resistance)
      .def_readwrite("steering_ratio", &VehicleParameters::steering_ratio)
      .def_readwrite("ackermann_ratio", &VehicleParameters::ackermann_ratio)
      .def_readwrite("steer_max", &VehicleParameters::steer_max)
      .def_readwrite("steer_rate_max", &VehicleParameters::steer_rate_max)
      .def_readwrite("steer_time_constant",
                     &VehicleParameters::steer_time_constant)
      .def_readwrite("accel_max", &VehicleParameters::accel_max)
      .def_readwrite("accel_min", &VehicleParameters::accel_min)
      .def_readwrite("speed_max", &VehicleParameters::speed_max)
      .def_readwrite("low_speed_guard", &VehicleParameters::low_speed_guard)
      .def("wheel_base", &VehicleParameters::wheelBase)
      .def("static_load_front", &VehicleParameters::staticLoadFront)
      .def("static_load_rear", &VehicleParameters::staticLoadRear)
      .def("validate", &VehicleParameters::validate,
           "Return the list of violated constraints; empty means usable.")
      .def("copy", [](const VehicleParameters& p) { return p; });

  mod.def("make_passenger_car_parameters", &makePassengerCarParameters);
  mod.def("make_shuttle_parameters", &makeShuttleParameters);
  mod.def("make_buggy_parameters", &makeBuggyParameters);

  // --- tire/tire_models.hpp -----------------------------------------------
  py::class_<tire::LinearTire>(mod, "LinearTire")
      .def(py::init([](double c, double mu) {
             tire::LinearTire t;
             t.cornering_stiffness = c;
             t.friction = mu;
             return t;
           }),
           py::arg("cornering_stiffness") = 90000.0, py::arg("friction") = 1.0)
      .def_readwrite("cornering_stiffness", &tire::LinearTire::cornering_stiffness)
      .def_readwrite("friction", &tire::LinearTire::friction)
      .def("lateral_force", &tire::LinearTire::lateralForce,
           py::arg("slip_angle"), py::arg("normal_force"))
      .def("cornering_stiffness_at", &tire::LinearTire::corneringStiffness,
           py::arg("normal_force") = 0.0);

  py::class_<tire::FialaTire>(mod, "FialaTire")
      .def(py::init([](double c, double mu) {
             tire::FialaTire t;
             t.cornering_stiffness = c;
             t.friction = mu;
             return t;
           }),
           py::arg("cornering_stiffness") = 90000.0, py::arg("friction") = 1.0)
      .def_readwrite("cornering_stiffness", &tire::FialaTire::cornering_stiffness)
      .def_readwrite("friction", &tire::FialaTire::friction)
      .def("lateral_force", &tire::FialaTire::lateralForce,
           py::arg("slip_angle"), py::arg("normal_force"))
      .def("cornering_stiffness_at", &tire::FialaTire::corneringStiffness,
           py::arg("normal_force") = 0.0);

  py::class_<tire::PacejkaTire>(mod, "PacejkaTire")
      .def(py::init([](double B, double C, double E, double mu) {
             tire::PacejkaTire t;
             t.B = B;
             t.C = C;
             t.E = E;
             t.friction = mu;
             return t;
           }),
           py::arg("B") = 10.0, py::arg("C") = 1.9, py::arg("E") = 0.97,
           py::arg("friction") = 1.0)
      .def_readwrite("B", &tire::PacejkaTire::B)
      .def_readwrite("C", &tire::PacejkaTire::C)
      .def_readwrite("E", &tire::PacejkaTire::E)
      .def_readwrite("friction", &tire::PacejkaTire::friction)
      .def("lateral_force", &tire::PacejkaTire::lateralForce,
           py::arg("slip_angle"), py::arg("normal_force"))
      .def("cornering_stiffness_at", &tire::PacejkaTire::corneringStiffness,
           py::arg("normal_force"))
      .def_static("from_cornering_stiffness",
                  &tire::PacejkaTire::fromCorneringStiffness,
                  py::arg("cornering_stiffness"), py::arg("nominal_load"),
                  py::arg("friction_coeff") = 1.0);

  mod.def("friction_ellipse_scale", &tire::frictionEllipseScale, py::arg("fx"),
          py::arg("friction"), py::arg("normal_force"));

  // --- ackermann.hpp -------------------------------------------------------
  py::class_<AckermannGeometry>(mod, "AckermannGeometry")
      .def(py::init([](double wb, double tf, double tr, double sr, double ar) {
             AckermannGeometry g;
             g.wheel_base = wb;
             g.track_front = tf;
             g.track_rear = tr;
             g.steering_ratio = sr;
             g.ackermann_ratio = ar;
             return g;
           }),
           py::arg("wheel_base") = 2.70, py::arg("track_front") = 1.55,
           py::arg("track_rear") = 1.55, py::arg("steering_ratio") = 16.0,
           py::arg("ackermann_ratio") = 1.0)
      .def_readwrite("wheel_base", &AckermannGeometry::wheel_base)
      .def_readwrite("track_front", &AckermannGeometry::track_front)
      .def_readwrite("track_rear", &AckermannGeometry::track_rear)
      .def_readwrite("steering_ratio", &AckermannGeometry::steering_ratio)
      .def_readwrite("ackermann_ratio", &AckermannGeometry::ackermann_ratio)
      .def_static("from_params", &AckermannGeometry::from, py::arg("p"));

  py::class_<WheelAngles>(mod, "WheelAngles")
      .def(py::init<>())
      .def_readwrite("left", &WheelAngles::left)
      .def_readwrite("right", &WheelAngles::right);

  py::class_<WheelSpeeds>(mod, "WheelSpeeds")
      .def(py::init<>())
      .def_readwrite("front_left", &WheelSpeeds::front_left)
      .def_readwrite("front_right", &WheelSpeeds::front_right)
      .def_readwrite("rear_left", &WheelSpeeds::rear_left)
      .def_readwrite("rear_right", &WheelSpeeds::rear_right);

  mod.def("turn_radius", &turnRadius, py::arg("g"), py::arg("delta"));
  mod.def("steer_angle_for_radius", &steerAngleForRadius, py::arg("g"),
          py::arg("radius"));
  mod.def("road_wheel_angles", &roadWheelAngles, py::arg("g"), py::arg("delta"));
  mod.def("bicycle_angle_from_wheels", &bicycleAngleFromWheels, py::arg("g"),
          py::arg("left"), py::arg("right"));
  mod.def("handwheel_to_road_wheel", &handwheelToRoadWheel, py::arg("g"),
          py::arg("handwheel"));
  mod.def("road_wheel_to_handwheel", &roadWheelToHandwheel, py::arg("g"),
          py::arg("road_wheel"));
  mod.def("ackermann_error", &ackermannError, py::arg("g"), py::arg("delta"));
  mod.def("wheel_speeds", &wheelSpeeds, py::arg("g"), py::arg("v"),
          py::arg("yaw_rate"));
  mod.def("wheel_angular_rates", &wheelAngularRates, py::arg("speeds"),
          py::arg("wheel_radius"));
  mod.def("minimum_turn_radius", &minimumTurnRadius, py::arg("g"),
          py::arg("steer_max"));

  // --- integrator.hpp ------------------------------------------------------
  py::enum_<IntegratorType>(mod, "IntegratorType")
      .value("EULER", IntegratorType::Euler)
      .value("HEUN", IntegratorType::Heun)
      .value("RK4", IntegratorType::RK4);

  // --- the type-erased model base -----------------------------------------
  py::class_<ModelBase>(mod, "ModelBase")
      .def("derivative", &pyDerivative, py::arg("state"), py::arg("input"))
      .def("normalize_state", &pyNormalizeState, py::arg("state"))
      .def_property_readonly("n_states", [](const ModelBase& m) {
        return m.nStates();
      });

  mod.def("step", &pyStep, py::arg("model"), py::arg("x"), py::arg("u"),
          py::arg("dt"), py::arg("method") = IntegratorType::RK4);
  mod.def("simulate", &pySimulate, py::arg("model"), py::arg("x0"), py::arg("u"),
          py::arg("duration"), py::arg("dt"),
          py::arg("method") = IntegratorType::RK4);
  mod.def(
      "step_euler",
      [](const ModelBase& m, const Array& x, const Array& u, double dt) {
        return pyOneStep(&stepEuler<ModelBase>, m, x, u, dt);
      },
      py::arg("model"), py::arg("x"), py::arg("u"), py::arg("dt"));
  mod.def(
      "step_heun",
      [](const ModelBase& m, const Array& x, const Array& u, double dt) {
        return pyOneStep(&stepHeun<ModelBase>, m, x, u, dt);
      },
      py::arg("model"), py::arg("x"), py::arg("u"), py::arg("dt"));
  mod.def(
      "step_rk4",
      [](const ModelBase& m, const Array& x, const Array& u, double dt) {
        return pyOneStep(&stepRK4<ModelBase>, m, x, u, dt);
      },
      py::arg("model"), py::arg("x"), py::arg("u"), py::arg("dt"));

  // --- unicycle.hpp --------------------------------------------------------
  py::class_<PyUnicycle, ModelBase>(mod, "UnicycleModel").def(py::init<>());

  py::class_<DifferentialDriveParams>(mod, "DifferentialDriveParams")
      .def(py::init([](double r, double track) {
             DifferentialDriveParams p;
             p.wheel_radius = r;
             p.track = track;
             return p;
           }),
           py::arg("wheel_radius") = 0.15, py::arg("track") = 0.50)
      .def_readwrite("wheel_radius", &DifferentialDriveParams::wheel_radius)
      .def_readwrite("track", &DifferentialDriveParams::track);

  py::class_<PyDifferentialDrive, ModelBase>(mod, "DifferentialDriveModel")
      .def(py::init<>())
      .def(py::init<const DifferentialDriveParams&>(), py::arg("params"))
      .def_property(
          "params",
          [](const PyDifferentialDrive& d) { return d.model.params; },
          [](PyDifferentialDrive& d, const DifferentialDriveParams& p) {
            d.model.params = p;
          })
      .def("to_body_velocity",
           [](const PyDifferentialDrive& d, const Array& u) {
             return toNumpy(fromFixed(d.model.toBodyVelocity(
                 toFixed<WheelRateInput>(toDyn(u, "input"), "input"))));
           },
           py::arg("u"))
      .def("to_wheel_rates",
           [](const PyDifferentialDrive& d, double v, double omega) {
             return toNumpy(fromFixed(d.model.toWheelRates(v, omega)));
           },
           py::arg("v"), py::arg("omega"));

  // --- kinematic_bicycle.hpp ----------------------------------------------
  py::enum_<ReferencePoint>(mod, "ReferencePoint")
      .value("REAR_AXLE", ReferencePoint::RearAxle)
      .value("CENTER_OF_GRAVITY", ReferencePoint::CenterOfGravity)
      .value("FRONT_AXLE", ReferencePoint::FrontAxle);

  py::class_<PyKinematicBicycle, ModelBase>(mod, "KinematicBicycleModel")
      .def(py::init<>())
      .def(py::init<const VehicleParameters&, ReferencePoint>(),
           py::arg("params"), py::arg("reference") = ReferencePoint::RearAxle)
      .def_property(
          "params",
          [](const PyKinematicBicycle& m) { return m.model.params; },
          [](PyKinematicBicycle& m, const VehicleParameters& p) {
            m.model.params = p;
          })
      .def_property(
          "reference",
          [](const PyKinematicBicycle& m) { return m.model.reference; },
          [](PyKinematicBicycle& m, ReferencePoint r) { m.model.reference = r; })
      .def_property(
          "apply_limits",
          [](const PyKinematicBicycle& m) { return m.model.apply_limits; },
          [](PyKinematicBicycle& m, bool v) { m.model.apply_limits = v; })
      .def("side_slip",
           [](const PyKinematicBicycle& m, double steer) {
             return m.model.sideSlip(steer);
           },
           py::arg("steer"))
      .def("lateral_acceleration",
           [](const PyKinematicBicycle& m, const Array& s, double steer) {
             return m.model.lateralAcceleration(
                 toFixed<KinematicBicycleState>(toDyn(s, "state"), "state"),
                 steer);
           },
           py::arg("s"), py::arg("steer"));

  py::class_<PyKinematicBicycleSteer, ModelBase>(mod,
                                                 "KinematicBicycleSteerModel")
      .def(py::init<>())
      .def(py::init<const PyKinematicBicycle&>(), py::arg("base"))
      .def_property_readonly(
          "params",
          [](const PyKinematicBicycleSteer& m) { return m.model.params(); })
      .def_property_readonly("base", [](const PyKinematicBicycleSteer& m) {
        PyKinematicBicycle b;
        b.model = m.model.base;
        return b;
      });

  // --- dynamic_bicycle.hpp -------------------------------------------------
  py::class_<BicycleForces>(mod, "BicycleForces")
      .def(py::init<>())
      .def_readwrite("slip_front", &BicycleForces::slip_front)
      .def_readwrite("slip_rear", &BicycleForces::slip_rear)
      .def_readwrite("fz_front", &BicycleForces::fz_front)
      .def_readwrite("fz_rear", &BicycleForces::fz_rear)
      .def_readwrite("fy_front", &BicycleForces::fy_front)
      .def_readwrite("fy_rear", &BicycleForces::fy_rear)
      .def_readwrite("f_resist", &BicycleForces::f_resist)
      .def_readwrite("ax", &BicycleForces::ax)
      .def_readwrite("ay", &BicycleForces::ay);

  mod.def(
      "dynamic_speed",
      [](const Array& s) {
        return toFixed<DynamicBicycleState>(toDyn(s, "state"), "state").speed();
      },
      py::arg("s"));
  mod.def(
      "dynamic_side_slip",
      [](const Array& s) {
        return toFixed<DynamicBicycleState>(toDyn(s, "state"), "state")
            .sideSlip();
      },
      py::arg("s"), "Body slip angle beta of a 6-state dynamic state vector.");

  py::class_<PyDynamicBicycle, ModelBase>(mod, "DynamicBicycleModel")
      .def(py::init<>())
      .def(py::init([](const VehicleParameters& p, const py::object& front,
                       const py::object& rear) {
             return PyDynamicBicycle(
                 p, front.is_none() ? py::cast(tire::LinearTire()) : front,
                 rear.is_none() ? py::cast(tire::LinearTire()) : rear);
           }),
           py::arg("params"), py::arg("tire_front") = py::none(),
           py::arg("tire_rear") = py::none())
      .def_property("params", &PyDynamicBicycle::params,
                    &PyDynamicBicycle::setParams)
      .def_property("longitudinal_load_transfer",
                    &PyDynamicBicycle::loadTransfer,
                    &PyDynamicBicycle::setLoadTransfer)
      .def_property(
          "tire_front", [](const PyDynamicBicycle& m) { return m.tire(true); },
          [](PyDynamicBicycle& m, const py::object& t) { m.setTire(true, t); })
      .def_property(
          "tire_rear", [](const PyDynamicBicycle& m) { return m.tire(false); },
          [](PyDynamicBicycle& m, const py::object& t) { m.setTire(false, t); })
      .def("sync_tires_from_params", &PyDynamicBicycle::syncTiresFromParams,
           py::arg("sync_friction") = false)
      .def("compute_forces", &PyDynamicBicycle::computeForces, py::arg("s"),
           py::arg("u"))
      .def("measured_lateral_acceleration",
           &PyDynamicBicycle::measuredLateralAcceleration, py::arg("s"),
           py::arg("u"))
      .def("input_from_acceleration", &PyDynamicBicycle::inputFromAcceleration,
           py::arg("ax"), py::arg("steer"));

  py::class_<PyLinearLateral, ModelBase>(mod, "LinearLateralBicycleModel")
      .def(py::init<>())
      .def(py::init<const VehicleParameters&, double>(), py::arg("params"),
           py::arg("longitudinal_speed") = 10.0)
      .def_property(
          "params", [](const PyLinearLateral& m) { return m.model.params; },
          [](PyLinearLateral& m, const VehicleParameters& p) {
            m.model.params = p;
          })
      .def_property(
          "longitudinal_speed",
          [](const PyLinearLateral& m) { return m.model.longitudinal_speed; },
          [](PyLinearLateral& m, double v) { m.model.longitudinal_speed = v; })
      .def("state_matrix", &PyLinearLateral::stateMatrix)
      .def("input_matrix", &PyLinearLateral::inputMatrix);

  py::class_<PyBlendedBicycle, ModelBase>(mod, "BlendedBicycleModel")
      .def(py::init<>())
      .def(py::init<const PyDynamicBicycle&, double, double, double>(),
           py::arg("dynamic"), py::arg("blend_speed_low") = 1.0,
           py::arg("blend_speed_high") = 4.0,
           py::arg("blend_time_constant") = 0.10)
      .def_property_readonly("params", &PyBlendedBicycle::params)
      .def_property("blend_speed_low", &PyBlendedBicycle::blendSpeedLow,
                    &PyBlendedBicycle::set_blendSpeedLow)
      .def_property("blend_speed_high", &PyBlendedBicycle::blendSpeedHigh,
                    &PyBlendedBicycle::set_blendSpeedHigh)
      .def_property("blend_time_constant", &PyBlendedBicycle::blendTimeConstant,
                    &PyBlendedBicycle::set_blendTimeConstant)
      .def("blend_factor", &PyBlendedBicycle::blendFactor, py::arg("vx"))
      .def("compute_forces", &PyBlendedBicycle::computeForces, py::arg("s"),
           py::arg("u"))
      .def("input_from_acceleration", &PyBlendedBicycle::inputFromAcceleration,
           py::arg("ax"), py::arg("steer"));

  // --- double_track.hpp ----------------------------------------------------
  py::class_<DoubleTrackParams>(mod, "DoubleTrackParams")
      .def(py::init([](double kf, double drive, double brake, bool combined) {
             DoubleTrackParams p;
             p.front_roll_stiffness_ratio = kf;
             p.front_drive_ratio = drive;
             p.front_brake_ratio = brake;
             p.combined_slip = combined;
             return p;
           }),
           py::arg("front_roll_stiffness_ratio") = 0.55,
           py::arg("front_drive_ratio") = 0.0,
           py::arg("front_brake_ratio") = 0.65,
           py::arg("combined_slip") = true)
      .def_readwrite("front_roll_stiffness_ratio",
                     &DoubleTrackParams::front_roll_stiffness_ratio)
      .def_readwrite("front_drive_ratio", &DoubleTrackParams::front_drive_ratio)
      .def_readwrite("front_brake_ratio", &DoubleTrackParams::front_brake_ratio)
      .def_readwrite("combined_slip", &DoubleTrackParams::combined_slip);

  py::class_<DoubleTrackForces>(mod, "DoubleTrackForces")
      .def(py::init<>())
      .def_property_readonly(
          "slip_angle",
          [](const DoubleTrackForces& f) {
            return wheelQuantityToNumpy(f.slip_angle);
          })
      .def_property_readonly(
          "normal_load",
          [](const DoubleTrackForces& f) {
            return wheelQuantityToNumpy(f.normal_load);
          })
      .def_property_readonly("lateral",
                             [](const DoubleTrackForces& f) {
                               return wheelQuantityToNumpy(f.lateral);
                             })
      .def_property_readonly("longitudinal",
                             [](const DoubleTrackForces& f) {
                               return wheelQuantityToNumpy(f.longitudinal);
                             })
      .def_readwrite("steer", &DoubleTrackForces::steer)
      .def_readwrite("ax", &DoubleTrackForces::ax)
      .def_readwrite("ay", &DoubleTrackForces::ay)
      .def("load_sum",
           [](const DoubleTrackForces& f) { return f.normal_load.sum(); });

  py::class_<PyDoubleTrack, ModelBase>(mod, "DoubleTrackModel")
      .def(py::init<>())
      .def(py::init([](const VehicleParameters& p,
                       const std::optional<DoubleTrackParams>& dtp,
                       const py::object& front, const py::object& rear) {
             return PyDoubleTrack(
                 p, dtp.value_or(DoubleTrackParams()),
                 front.is_none() ? py::cast(tire::FialaTire()) : front,
                 rear.is_none() ? py::cast(tire::FialaTire()) : rear);
           }),
           py::arg("params"), py::arg("dt_params") = py::none(),
           py::arg("tire_front") = py::none(),
           py::arg("tire_rear") = py::none())
      .def_property("params", &PyDoubleTrack::params, &PyDoubleTrack::setParams)
      .def_property("dt_params", &PyDoubleTrack::dtParams,
                    &PyDoubleTrack::setDtParams,
                    py::return_value_policy::reference_internal)
      .def_property(
          "tire_front", [](const PyDoubleTrack& m) { return m.tire(true); },
          [](PyDoubleTrack& m, const py::object& t) { m.setTire(true, t); })
      .def_property(
          "tire_rear", [](const PyDoubleTrack& m) { return m.tire(false); },
          [](PyDoubleTrack& m, const py::object& t) { m.setTire(false, t); })
      .def("sync_tires_from_params", &PyDoubleTrack::syncTiresFromParams)
      .def("geometry", &PyDoubleTrack::geometry)
      .def("compute_forces", &PyDoubleTrack::computeForces, py::arg("s"),
           py::arg("u"))
      .def("input_from_acceleration", &PyDoubleTrack::inputFromAcceleration,
           py::arg("ax"), py::arg("steer"));

  // --- linear_analysis.hpp -------------------------------------------------
  py::class_<analysis::SteadyState>(mod, "SteadyState")
      .def(py::init<>())
      .def_readwrite("yaw_rate", &analysis::SteadyState::yaw_rate)
      .def_readwrite("lateral_accel", &analysis::SteadyState::lateral_accel)
      .def_readwrite("side_slip", &analysis::SteadyState::side_slip)
      .def_readwrite("radius", &analysis::SteadyState::radius)
      .def_readwrite("slip_front", &analysis::SteadyState::slip_front)
      .def_readwrite("slip_rear", &analysis::SteadyState::slip_rear);

  py::class_<analysis::YawMode>(mod, "YawMode")
      .def(py::init<>())
      .def_readwrite("natural_frequency", &analysis::YawMode::natural_frequency)
      .def_readwrite("damping_ratio", &analysis::YawMode::damping_ratio)
      .def_readwrite("real_1", &analysis::YawMode::real_1)
      .def_readwrite("imag_1", &analysis::YawMode::imag_1)
      .def_readwrite("real_2", &analysis::YawMode::real_2)
      .def_readwrite("imag_2", &analysis::YawMode::imag_2)
      .def_readwrite("stable", &analysis::YawMode::stable);

  mod.def("understeer_gradient", &analysis::understeerGradient, py::arg("p"));
  mod.def("understeer_gradient_deg_per_g", &analysis::understeerGradientDegPerG,
          py::arg("p"));
  mod.def("characteristic_speed", &analysis::characteristicSpeed, py::arg("p"));
  mod.def("critical_speed", &analysis::criticalSpeed, py::arg("p"));
  mod.def("neutral_steer_point", &analysis::neutralSteerPoint, py::arg("p"));
  mod.def("static_margin", &analysis::staticMargin, py::arg("p"));
  mod.def("yaw_rate_gain", &analysis::yawRateGain, py::arg("p"), py::arg("vx"));
  mod.def("lateral_acceleration_gain", &analysis::lateralAccelerationGain,
          py::arg("p"), py::arg("vx"));
  mod.def("required_steer_angle", &analysis::requiredSteerAngle, py::arg("p"),
          py::arg("radius"), py::arg("vx"));
  mod.def("steady_state_cornering", &analysis::steadyStateCornering,
          py::arg("p"), py::arg("vx"), py::arg("delta"));
  mod.def("yaw_mode", &analysis::yawMode, py::arg("p"), py::arg("vx"));
  mod.def("max_lateral_acceleration", &analysis::maxLateralAcceleration,
          py::arg("p"));
}
