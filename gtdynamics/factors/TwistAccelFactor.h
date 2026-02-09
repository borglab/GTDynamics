/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TwistAccelFactor.h
 * @brief twist acceleration factor, common between forward and inverse
 * dynamics.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtdynamics/universal_robot/Joint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>
#include <memory>
#include <string>

namespace gtdynamics {

/**
 * TwistAccelFactor is a six-way nonlinear factor which enforces relation
 * between acceleration on previous link and this link.
 */

/**
 * Factor linking child link's twist_accel, joint_coordinate, joint_vel,
 * joint_accel with previous link's twist_accel.
 *
 * Will create factor corresponding to Lynch & Park book: twist acceleration,
 * Equation 8.47, page 293
 *
 * @param joint JointConstSharedPtr to the joint
 */
inline gtsam::NoiseModelFactor::shared_ptr TwistAccelFactor(
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    JointConstSharedPtr joint, int time) {
  return std::make_shared<gtsam::ExpressionFactor<gtsam::Vector6>>(
      cost_model, gtsam::Vector6::Zero(), joint->twistAccelConstraint(time));
}

}  // namespace gtdynamics
