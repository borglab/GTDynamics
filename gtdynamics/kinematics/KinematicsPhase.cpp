/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsPhase.cpp
 * @brief Kinematics for a Phase with fixed contacts.
 * @author: Dan Barladeanu, Frank Dellaert
 */

#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/ContextUtils.h>
#include <gtdynamics/utils/Phase.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <stdexcept>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using std::string;
using std::vector;

template <>
NonlinearFactorGraph Kinematics::graph<Phase>(const Phase& phase,
                                              const Robot& robot) const {
  // Phase is an Interval with contact semantics, so delegate to the
  // Interval implementation for the kinematic graph.
  const Interval& interval = static_cast<const Interval&>(phase);
  return this->graph<Interval>(interval, robot);
}

template <>
NonlinearFactorGraph Kinematics::qFactors<Phase>(
    const Phase& phase, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points,
    const gtsam::Vector3& gravity, double ground_plane_height) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return qFactors(interval, robot, contact_points, gravity, ground_plane_height);
}

template <>
NonlinearFactorGraph Kinematics::vFactors<Phase>(
    const Phase& phase, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return vFactors(interval, robot, contact_points);
}

NonlinearFactorGraph Kinematics::contactHeightObjectives(
    const Phase& phase, const gtsam::Vector3& gravity,
    double ground_plane_height) const {
  const auto contacts = std::dynamic_pointer_cast<const FootContactConstraintSpec>(
      phase.constraintSpec());
  if (!contacts) {
    throw std::invalid_argument(
        "contactHeightObjectives requires a FootContactConstraintSpec");
  }
  return collectFactors(phase, [&](const Slice& slice) {
    return contactHeightObjectives(slice, contacts->contactPoints(), gravity,
                                   ground_plane_height);
  });
}

}  // namespace gtdynamics
