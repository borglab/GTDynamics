/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TorqueFactor.h
 * @brief Torque factor, common between forward and inverse dynamics.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace gtdynamics {

/**
 * TorqueFactor is a two-way nonlinear factor which enforces relation between
 * wrench and torque on each link
 */

/**
 * Torque factor, common between forward and inverse dynamics.
 * Will create factor corresponding to Lynch & Park book:
 * Torque is always wrench projected on screw axis.
 * Equation 8.49, page 293 can be written as
 *  screw_axis.transpose() * F.transpose() == torque
 *
 * @param joint JointConstSharedPtr to the joint
 */
inline gtsam::NoiseModelFactor::shared_ptr TorqueFactor(
    const gtsam::noiseModel::Base::shared_ptr& cost_model,
    const JointConstSharedPtr& joint, size_t k = 0) {
  return std::make_shared<gtsam::ExpressionFactor<double>>(
      cost_model, 0.0, joint->torqueConstraint(k));
}

/** Include rotor inertia: transmitted screw-axis torque + armature*qddot.
 * Armature is nonnegative and finite, in kg m^2, and zero preserves the
 * original factor without introducing an acceleration key.
 */
inline gtsam::NoiseModelFactor::shared_ptr TorqueFactor(
    const gtsam::SharedNoiseModel& model, const JointConstSharedPtr& joint,
    size_t k, double armature) {
  if (!std::isfinite(armature) || armature < 0)
    throw std::invalid_argument("Armature must be finite and nonnegative");
  if (armature == 0) return TorqueFactor(model, joint, k);
  return std::make_shared<gtsam::ExpressionFactor<double>>(
      model, 0.0,
      joint->torqueConstraint(k) +
          gtsam::Double_(armature) *
              gtsam::Double_(JointAccelKey(joint->id(), k)));
}

}  // namespace gtdynamics
