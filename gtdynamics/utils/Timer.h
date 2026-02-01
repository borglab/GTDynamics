/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Timer.h
 * @brief Timing
 * @author Yetong Zhang
 */

#include <chrono>
#include <gtsam/base/timing.h>

#pragma once

namespace gtsam {

/**
 * @class Timer
 * @brief Simple timer class using std::chrono.
 */
class Timer {
protected:
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  std::chrono::time_point<std::chrono::system_clock> stop_time_;

public:
  /// Constructor.
  Timer() {}

  /// Start timer.
  void start() { start_time_ = std::chrono::system_clock::now(); }

  /// Stop timer.
  void stop() { stop_time_ = std::chrono::system_clock::now(); }

  /// Return duration in seconds.
  double seconds() const {
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_time_ -
                                                                   start_time_);
    return ns.count() * 1e-9;
  }

  /// Return duration in milliseconds.
  double milliSeconds() const {
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_time_ -
                                                                   start_time_);
    return ns.count() * 1e-6;
  }

  /// Return duration in microseconds.
  double microSeconds() const {
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_time_ -
                                                                   start_time_);
    return ns.count() * 1e-3;
  }
};

} // namespace gtsam
