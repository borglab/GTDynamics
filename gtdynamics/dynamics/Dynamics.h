/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Dynamics.h
 * @brief Wrench calculations for configurations in motion.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>

namespace gtdynamics {

/// calculate Coriolis term and jacobian w.r.t. joint coordinate twist
gtsam::Vector6 Coriolis(const gtsam::Matrix6 &inertia,
                        const gtsam::Vector6 &twist,
                        gtsam::OptionalJacobian<6, 6> H_twist = boost::none);

/// Matrix vector multiplication.
gtsam::Vector6 MatVecMult(const gtsam::Matrix6 &inertia,
                        const gtsam::Vector6 &twist,
                        gtsam::OptionalJacobian<6, 6> H_twist = boost::none);

}  // namespace gtdynamics
