/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchFactor.h
 * @brief Wrench balance factor, common between forward and inverse dynamics.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <boost/serialization/base_object.hpp>
#include <string>
#include <vector>

#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/utils.h"

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

/**
 * WrenchFactor is an n-way nonlinear factor which enforces relation
 * between wrenches on this link
 */

/**
 * Wrench balance factor, common between forward and inverse dynamics.
 * Will create factor corresponding to Lynch & Park book:
 *  wrench balance, Equation 8.48, page 293
 * @param gravity (optional) Create gravity wrench in link COM frame.
 */
inline gtsam::ExpressionFactor<gtsam::Vector6> WrenchFactor(
    const gtsam::SharedNoiseModel &cost_model, const LinkConstSharedPtr &link,
    const std::vector<DynamicsSymbol> &wrench_keys, int time,
    const boost::optional<gtsam::Vector3> &gravity = boost::none) {
  return gtsam::ExpressionFactor<gtsam::Vector6>(
      cost_model, gtsam::Vector6::Zero(),
      link->wrenchConstraint(wrench_keys, time, gravity));
}

}  // namespace gtdynamics
