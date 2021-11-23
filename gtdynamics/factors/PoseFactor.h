/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PoseFactor.h
 * @brief Forward kinematics factor.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/assign/list_of.hpp>
#include <memory>
#include <string>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

using boost::assign::cref_list_of;

/**
 * Create single factor relating this link's pose (COM) with previous one.
 * Note: this function is provided for BW compatibility only, and will in time
 * be replaced with EqualityConstraint.
 *
 * PoseFactor is a three-way nonlinear factor between a joint's parent link
 * pose, child link pose, and the joint angle relating the two poses.
 *
 * Given the joint model, this factor optimizes for the underlying joint axis
 * and the corresponding poses of the parent and child links.
 *
 * @param cost_model The noise model for this factor.
 * @param joint The joint connecting the two poses.
 * @param time The timestep at which this factor is defined.
 */
inline gtsam::ExpressionFactor<gtsam::Vector6> PoseFactor(
    const gtsam::SharedNoiseModel &cost_model, const JointConstSharedPtr &joint,
    int time) {
  return gtsam::ExpressionFactor<gtsam::Vector6>(
      cost_model, gtsam::Vector6::Zero(), joint->poseConstraint(time));
}

/**
 * Create single factor relating this link's pose (COM) with previous one.
 * Note: this function is provided for BW compatibility only, and will in time
 * be replaced with EqualityConstraint.
 *
 * @param wTp_key Key for parent link's CoM pose in world frame.
 * @param wTc_key Key for child link's CoM pose in world frame.
 * @param q_key Key for joint value.
 * @param cost_model The noise model for this factor.
 * @param joint The joint connecting the two poses
 */
inline gtsam::ExpressionFactor<gtsam::Vector6> PoseFactor(
    DynamicsSymbol wTp_key, DynamicsSymbol wTc_key, DynamicsSymbol q_key,
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    JointConstSharedPtr joint) {
  return gtsam::ExpressionFactor<gtsam::Vector6>(
      cost_model, gtsam::Vector6::Zero(),
      joint->poseConstraint(wTp_key.time()));
}

}  // namespace gtdynamics
