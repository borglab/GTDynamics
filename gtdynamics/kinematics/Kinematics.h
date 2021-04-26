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

  /// Constructor
  ContactGoal(const PointOnLink& point_on_link, const gtsam::Point3& goal_point)
      : point_on_link(point_on_link), goal_point(goal_point) {}

  /// Return link associated with contact point.
  const LinkSharedPtr& link() const { return point_on_link.link; }

  /// Return contact point in link COM frame.
  const gtsam::Point3& contact_in_com() const { return point_on_link.point; }

  /// Print to stream.
  friend std::ostream& operator<<(std::ostream& os, const ContactGoal& cg);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string& s) const;

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

struct KinematicsParameters {
  using Isotropic = gtsam::noiseModel::Isotropic;
  const gtsam::SharedNoiseModel p_cost_model,     // pose factor
      g_cost_model,                               // goal point
      prior_q_cost_model;                         // joint angle prior factor
  gtsam::LevenbergMarquardtParams lm_parameters;  // LM parameters

  KinematicsParameters()
      : p_cost_model(Isotropic::Sigma(6, 1e-4)),
        g_cost_model(Isotropic::Sigma(3, 0.01)),
        prior_q_cost_model(Isotropic::Sigma(1, 0.5)) {}
};

/// All things kinematics, zero velocities/twists, and no forces.
template <class CONTEXT>
class Kinematics {
  Robot robot_;
  KinematicsParameters p_;

 public:
  /**
   * @fn Constructor.
   * @param context e.g., a Slice, Phase, WalkCycle, or Trajectory instance
   */
  Kinematics(const Robot& robot,
             const KinematicsParameters& parameters = KinematicsParameters())
      : robot_(robot), p_(parameters) {}

  /**
   * @fn Slice with kinematics constraints.
   * @returns factor graph..
   */
  gtsam::NonlinearFactorGraph graph(const CONTEXT& context);

  /**
   * @fn Create point goal objectives.
   * @param contact_goals goals for contact points
   * @returns graph with point goal factors.
   */
  gtsam::NonlinearFactorGraph pointGoalObjectives(
      const CONTEXT& context, const ContactGoals& contact_goals);

  /**
   * @fn Factors that minimize joint angles.
   * @returns graph with prior factors on joint angles.
   */
  gtsam::NonlinearFactorGraph jointAngleObjectives(const CONTEXT& context);

  /**
   * @fn Initialize kinematics.
   *
   * Use wTcom for poses and zero-mean noise for joint angles.
   *
   * @param gaussian_noise time step to check (default 0.1).
   * @returns values with identity poses and zero joint angles.
   */
  gtsam::Values initialValues(const CONTEXT& context,
                              double gaussian_noise = 0.1);

  /**
   * @fn Inverse kinematics given a set of contact goals.
   * @param contact_goals goals for contact points
   * @returns values with poses and joint angles.
   */
  gtsam::Values inverse(const CONTEXT& context,
                        const ContactGoals& contact_goals);
};
}  // namespace gtdynamics
