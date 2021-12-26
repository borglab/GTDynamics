/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Interval.h
 * @brief An interval from k_start to k_end.
 * @author Frank Dellaert
 */

#pragma once
namespace gtdynamics {

/// An interval from k_start to k_end.
struct Interval {
  size_t k_start, k_end;

 public:
  /// Constructor.
  explicit Interval(size_t k_start = 0, size_t k_end = 1)
      : k_start(k_start), k_end(k_end) {}

  size_t numTimeSteps() const { return (k_end - k_start);}
};

}  // namespace gtdynamics
