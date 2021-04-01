/**
 * @file units.h
 * @brief common units / constants
 * @author Gerry Chen
 */

#pragma once

#include <cmath>

namespace gtdynamics {
namespace cablerobot {

/// Kv motor velocity constant, in rad / V.s
struct Kv { double value; };
struct Kt { double value; };

constexpr Kv operator""_RPM_per_V(long double kv) {
  return Kv{static_cast<double>(kv) / 60.0 * 2 * M_PI};
}
constexpr Kv operator""_RPM_per_V(unsigned long long int kv) {
  return Kv{static_cast<double>(kv) / 60.0 * 2 * M_PI};
}
constexpr Kt operator/(double x, Kv kv) { return Kt{x / kv.value}; }

}  // namespace cablerobot
}  // namespace gtdynamics
