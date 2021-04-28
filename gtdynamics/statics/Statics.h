/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Statics.h
 * @brief Wrench calculations for configurations at rest.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

/**
 * @fn Calculate gravity wrench
 * @param gravity 3-vector indicating gravity force, typically, [0,0,-g]
 * @param mass link mass, in kg.
 * @param wTcom pose of link center of mass frame
 * @param H_wTcom optional 6x6 Jacobian of wrench wrt COM pose
 * @returns 6x1 gravity wrench in CoM frame
 */
gtsam::Vector6 GravityWrench(
    const gtsam::Vector3 &gravity, double mass, const gtsam::Pose3 &wTcom,
    gtsam::OptionalJacobian<6, 6> H_wTcom = boost::none);

/// Calculate sum of wrenches with optional Jacobians (all identity!)
gtsam::Vector6 ResultantWrench(
    std::vector<gtsam::Vector6> wrenches,
    boost::optional<std::vector<gtsam::Matrix> &> H = boost::none);

}  // namespace gtdynamics
