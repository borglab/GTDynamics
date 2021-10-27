/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Phase.h
 * @brief Utility methods for generating Phase objects.
 * @author: Disha Das, Frank Dellaert
 */

#pragma once

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/ConstraintSpec.h>
#include <gtdynamics/utils/Interval.h>
#include <gtdynamics/utils/FootContactConstraintSpec.h>

#include <iosfwd>

namespace gtdynamics {
/**
 * @class Phase class stores information about a robot stance
 * and its duration.
 */

class Phase : public Interval {
 protected:
  boost::shared_ptr<ConstraintSpec> constraint_spec_;

 public:
  /// Constructor
  Phase(size_t k_start, size_t k_end,
        const boost::shared_ptr<ConstraintSpec> &constraint_spec)
      : Interval(k_start, k_end), constraint_spec_(constraint_spec) {}

  ///Return Constraint Spec pointer
  const boost::shared_ptr<const ConstraintSpec> constraintSpec() const {
    return constraint_spec_;
  }

  /// Print to stream.
  friend std::ostream &operator<<(std::ostream &os, const Phase &phase);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string &s) const;

  /// Parse results into a matrix, in order: qs, qdots, qddots, taus, dt
  gtsam::Matrix jointMatrix(const Robot &robot, const gtsam::Values &results,
                            size_t k = 0,
                            boost::optional<double> dt = boost::none) const;
};
}  // namespace gtdynamics
