// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Archlink Systems Lab
#ifndef VEHICLE_MODELS_TEST_SUPPORT_HPP
#define VEHICLE_MODELS_TEST_SUPPORT_HPP

#include <cmath>
#include <cstdio>

namespace vmtest {

inline int& failureCount() {
  static int count = 0;
  return count;
}
inline int& checkCount() {
  static int count = 0;
  return count;
}

inline void record(bool ok, const char* expr, const char* file, int line) {
  ++checkCount();
  if (!ok) {
    ++failureCount();
    std::printf("  FAIL %s:%d  %s\n", file, line, expr);
  }
}

inline void recordNear(double actual, double expected, double tol,
                       const char* expr, const char* file, int line) {
  ++checkCount();
  const bool ok = std::isfinite(actual) && std::fabs(actual - expected) <= tol;
  if (!ok) {
    ++failureCount();
    std::printf("  FAIL %s:%d  %s\n        actual=%.9g expected=%.9g tol=%.3g\n",
                file, line, expr, actual, expected, tol);
  }
}

inline void section(const char* name) { std::printf("[ %s ]\n", name); }

inline int summary(const char* suite) {
  std::printf("%s: %d checks, %d failures\n", suite, checkCount(), failureCount());
  return failureCount() == 0 ? 0 : 1;
}

}  // namespace vmtest

#define VM_CHECK(cond) ::vmtest::record((cond), #cond, __FILE__, __LINE__)
#define VM_CHECK_NEAR(actual, expected, tol) \
  ::vmtest::recordNear((actual), (expected), (tol), #actual " ~= " #expected, \
                       __FILE__, __LINE__)

#endif  // VEHICLE_MODELS_TEST_SUPPORT_HPP
