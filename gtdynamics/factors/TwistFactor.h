/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TwistFactor.h
 * @brief twist factor.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <string>

#include "gtdynamics/universal_robot/Joint.h"

namespace gtdynamics {

/**
 * TwistFactor is a four-way nonlinear factor which enforces relation
 * between twist on previous link and this link
 */

/**
 * Create single factor relating child link's twist with parent one.
 * Will create factor corresponding to Lynch & Park book:
 *  Equation 8.45, page 292
 *
 * @param joint a Joint
 */
inline gtsam::NoiseModelFactor::shared_ptr TwistFactor(
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    JointConstSharedPtr joint, int time) {
  return boost::make_shared<gtsam::ExpressionFactor<gtsam::Vector6>>(
      cost_model, gtsam::Vector6::Zero(), joint->twistConstraint(time));
}

}  // namespace gtdynamics
