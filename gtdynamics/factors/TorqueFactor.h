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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <memory>
#include <string>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/utils/values.h"

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
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const JointConstSharedPtr &joint, size_t k = 0) {
  return boost::make_shared<gtsam::ExpressionFactor<double>>(
      cost_model, 0.0, joint->torqueConstraint(k));
}

}  // namespace gtdynamics
