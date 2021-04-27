/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Slice.h
 * @brief A single discrete time slice.
 * @author: Frank Dellaert
 */

#pragma once

namespace gtdynamics {

/// A single discrete time slice.
struct Slice {
  size_t k;  // time index

 public:
  /// Construct from time index.
  explicit Slice(size_t k = 0) : k(k) {}
};

}  // namespace gtdynamics
