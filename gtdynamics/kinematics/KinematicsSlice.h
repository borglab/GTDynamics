/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsSlice.h
 * @brief Kinematics in single time slice.
 * @author: Frank Dellaert
 */

#pragma once

#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/ContactPoint.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

namespace gtdynamics {

struct ContactGoal {
  PointOnLink point_on_link;  ///< In COM.
  gtsam::Point3 goal_point;   ///< In world frame.

  /// Return link associated with contact point.
  const LinkSharedPtr& link() const { return point_on_link.link; }

  /// Return contact point in link COM frame.
  const gtsam::Point3& contact_in_com() const { return point_on_link.point; }

  /**
   * @fn Check that the contact goal has been achived for given values.
   * @param values a GTSAM Values instance that should contain link pose.
   * @param k time step to check (default 0).
   * @param tol tolerance in 3D (default 1e-9).
   */
  bool satisfied(const gtsam::Values& values, size_t k = 0,
                 double tol = 1e-9) const {
    return gtsam::distance3(point_on_link.predict(values, k), goal_point) < tol;
  }
};

///< Map of link name to ContactGoal
using ContactGoals = std::vector<ContactGoal>;

struct KinematicsSettings {
  using Isotropic = gtsam::noiseModel::Isotropic;
  const gtsam::SharedNoiseModel p_cost_model,     // pose factor
      g_cost_model,                               // goal point
      prior_q_cost_model;                         // joint angle prior factor
  gtsam::LevenbergMarquardtParams lm_parameters;  // LM parameters

  KinematicsSettings()
      : p_cost_model(Isotropic::Sigma(6, 1e-4)),
        g_cost_model(Isotropic::Sigma(3, 0.01)),
        prior_q_cost_model(Isotropic::Sigma(1, 0.5)) {}
};

/**
 * @fn Slice with kinematics constraints.
 * @param robot robot configuration
 * @param opt KinematicsSettings
 * @param k time step (default 0).
 * @returns factor graph..
 */
gtsam::NonlinearFactorGraph KinematicsSlice(
    const Robot& robot, const KinematicsSettings& opt = KinematicsSettings(),
    size_t k = 0);

/**
 * @fn Create point goal objectives.
 * @param robot robot configuration
 * @param contact_goals goals for contact points
 * @param opt KinematicsSettings
 * @param k time step to check (default 0).
 * @returns graph with point goal factors.
 */
gtsam::NonlinearFactorGraph PointGoalObjectives(
    const Robot& robot, const ContactGoals& contact_goals,
    const KinematicsSettings& opt = KinematicsSettings(), size_t k = 0);

/**
 * @fn Factors that minimize joint angles.
 * @param robot robot configuration
 * @param opt KinematicsSettings
 * @param k time step to check (default 0).
 * @returns graph with prior factors on joint angles.
 * TODO(frank): allow for a non-zero "rest" configuration.
 */
gtsam::NonlinearFactorGraph MinimumJointAngleSlice(
    const Robot& robot, const KinematicsSettings& opt = KinematicsSettings(),
    size_t k = 0);

/**
 * @fn Initialize kinematics.
 *
 * Use wTcom for poses and zero-mean noise for joint angles.
 *
 * @param robot robot configuration
 * @param k time step to check (default 0).
 * @param gaussian_noise stddev of noise to add to joint angles (default 0.1).
 * @returns values with identity poses and zero joint angles.
 */
gtsam::Values KinematicsSliceInitialValues(const Robot& robot, size_t k = 0,
                                           double gaussian_noise = 0.1);

/**
 * @fn Inverse kinematics given a set of contact goals.
 * @param robot robot configuration
 * @param contact_goals goals for contact points
 * @param opt KinematicsSettings
 * @param k time step to check (default 0).
 * @returns values with poses and joint angles.
 */
gtsam::Values InverseKinematics(
    const Robot& robot, const ContactGoals& contact_goals,
    const KinematicsSettings& opt = KinematicsSettings(), size_t k = 0);

}  // namespace gtdynamics
