/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchPlanarFactor.h
 * @brief Wrench planar factor, enforce the wrench to be planar.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/expressions.h>

#include <boost/optional.hpp>
#include <string>

#include "gtdynamics/dynamics/Dynamics.h"
#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/values.h"

namespace gtdynamics {

/**
 * Constraint that enforces the wrench to be planar.
 */
inline gtsam::Vector3_ WrenchPlanarConstraint(gtsam::Vector3 planar_axis,
                                              const JointConstSharedPtr &joint,
                                              size_t k = 0) {
  gtsam::Matrix36 H_wrench;
  if (planar_axis[0] == 1) {  // x axis
    H_wrench << 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0;
  } else if (planar_axis[1] == 1) {  // y axis
    H_wrench << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;
  } else if (planar_axis[2] == 1) {  // z axis
    H_wrench << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
  }

  auto wrench_key = WrenchKey(joint->child()->id(), joint->id(), k);
  gtsam::Vector6_ wrench(wrench_key);
  // TODO(yetong): maybe can be done easily with a functor, and/or
  // linearexpression (re-written with functor), and maybe this same pattern
  // could be used to clean up scalar multiply in Expression.h
  gtsam::Vector3_ error(std::bind(MatVecMult<3, 6>, H_wrench,
                                  std::placeholders::_1, std::placeholders::_2),
                        wrench);
  return error;
}

/**
 * WrenchPlanarFactor is a one-way nonlinear factor which enforces the
 * wrench to be planar
 */
inline gtsam::NoiseModelFactor::shared_ptr WrenchPlanarFactor(
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    gtsam::Vector3 planar_axis, const JointConstSharedPtr &joint,
    size_t k = 0) {
  return boost::make_shared<gtsam::ExpressionFactor<gtsam::Vector3>>(
      cost_model, gtsam::Vector3::Zero(),
      WrenchPlanarConstraint(planar_axis, joint, k));
}

}  // namespace gtdynamics
