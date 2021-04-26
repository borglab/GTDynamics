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

namespace gtdynamics {

/// A single discrete time slice.
class Slice {
  size_t k_;  // time index

 public:
  /// Construct from time index.
  explicit Slice(size_t k = 0) : k_(k) {}

  /// Return time index.
  size_t k() const { return k_; }
};

}  // namespace gtdynamics