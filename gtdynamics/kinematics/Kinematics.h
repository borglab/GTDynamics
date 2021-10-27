/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Kinematics.h
 * @brief Kinematics optimizer.
 * @author: Frank Dellaert
 */

#pragma once

#include <gtdynamics/optimizer/Optimizer.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtdynamics/utils/Interval.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

namespace gtdynamics {

/**
 * Desired world position for a given point on some link.
 *
 * This simple struct stores a `point_on_link`, which specifies a 3D
 * point in the link's CoM frame, and a `goal_point` in world coordinate frames.
 * The goal is satisfied iff `point_on_link.predict(values, k) == goal_point`.
 */
struct ContactGoal {
  PointOnLink point_on_link;  ///< In COM.
  gtsam::Point3 goal_point;   ///< In world frame.

  /// Constructor
  ContactGoal(const PointOnLink& point_on_link, const gtsam::Point3& goal_point)
      : point_on_link(point_on_link), goal_point(goal_point) {}

  /// Return link associated with contact point.
  const LinkSharedPtr& link() const { return point_on_link.link; }

  /// Return contact point in link COM frame.
  const gtsam::Point3& contactInCoM() const { return point_on_link.point; }

  /// Print to stream.
  friend std::ostream& operator<<(std::ostream& os, const ContactGoal& cg);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string& s) const;

  /**
   * @fn Check that the contact goal has been achieved for given values.
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

/// Noise models etc specific to Kinematics class
struct KinematicsParameters : public OptimizationParameters {
  using Isotropic = gtsam::noiseModel::Isotropic;
  const gtsam::SharedNoiseModel p_cost_model,  // pose factor
      g_cost_model,                            // goal point
      prior_q_cost_model;                      // joint angle prior factor

  KinematicsParameters()
      : p_cost_model(Isotropic::Sigma(6, 1e-4)),
        g_cost_model(Isotropic::Sigma(3, 0.01)),
        prior_q_cost_model(Isotropic::Sigma(1, 0.5)) {}
};

/// All things kinematics, zero velocities/twists, and no forces.
class Kinematics : public Optimizer {
 protected:
  boost::shared_ptr<const KinematicsParameters> p_;  // overrides Base::p_

 public:
  /**
   * @fn Constructor.
   */
  Kinematics(const boost::shared_ptr<const KinematicsParameters>& parameters =
                 boost::make_shared<const KinematicsParameters>())
      : Optimizer(parameters), p_(parameters) {}

  /**
   * @fn Create graph with kinematics constraints.
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @returns factor graph..
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph graph(const CONTEXT& context,
                                    const Robot& robot) const;

  /**
   * @fn Create point goal objectives.
   * @param context Slice or Interval instance.
   * @param contact_goals goals for contact points
   * @returns graph with point goal factors.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph pointGoalObjectives(
      const CONTEXT& context, const ContactGoals& contact_goals) const;

  /**
   * @fn Factors that minimize joint angles.
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @returns graph with prior factors on joint angles.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph jointAngleObjectives(const CONTEXT& context,
                                                   const Robot& robot) const;

  /**
   * @fn Initialize kinematics.
   *
   * Use wTcom for poses and zero-mean noise for joint angles.
   *
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @param gaussian_noise time step to check (default 0.1).
   * @returns values with identity poses and zero joint angles.
   */
  template <class CONTEXT>
  gtsam::Values initialValues(const CONTEXT& context, const Robot& robot,
                              double gaussian_noise = 0.1) const;

  /**
   * @fn Inverse kinematics given a set of contact goals.
   * @fn This fuction does inverse kinematics seperately on each slice.
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @param contact_goals goals for contact points
   * @returns values with poses and joint angles.
   */
  template <class CONTEXT>
  gtsam::Values inverse(const CONTEXT& context, const Robot& robot,
                        const ContactGoals& contact_goals) const;

  /**
   * Interpolate using inverse kinematics: the goals are linearly interpolated.
   * @param interval Interval instance
   * @param robot Robot specification from URDF/SDF.
   * @param contact_goals1 goals for contact points for interval.k_start
   * @param contact_goals1 goals for contact points for interval.k_end
   * All results are return in values.
   */
  template <class CONTEXT>
  gtsam::Values interpolate(const CONTEXT& context, const Robot& robot,
                            const ContactGoals& contact_goals1,
                            const ContactGoals& contact_goals2) const;
};
}  // namespace gtdynamics
