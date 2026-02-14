/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchEquivalenceFactor.h
 * @brief Wrench eq factor, enforce same wrench expressed in different link
 * frames.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>
#include <memory>
#include <string>
#include <vector>

namespace gtdynamics {

/** WrenchEquivalenceFactor is a 3-way nonlinear factor which enforces
 * relation between wrench expressed in two link frames*/

/**
 * Wrench eq factor, enforce same wrench expressed in different link frames.
 * @param joint JointConstSharedPtr to the joint
 */
inline gtsam::NoiseModelFactor::shared_ptr WrenchEquivalenceFactor(
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const JointConstSharedPtr &joint, size_t k = 0) {
  return std::make_shared<gtsam::ExpressionFactor<gtsam::Vector6>>(
      cost_model, gtsam::Vector6::Zero(),
      joint->wrenchEquivalenceConstraint(k));
}

}  // namespace gtdynamics
