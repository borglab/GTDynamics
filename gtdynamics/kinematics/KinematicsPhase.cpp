/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsPhase.cpp
 * @brief Kinematics for a Phase with fixed contacts.
 * @author: Dan Barladeanu, Frank Dellaert
 */

#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Phase.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using std::string;
using std::vector;

}  // namespace gtdynamics
