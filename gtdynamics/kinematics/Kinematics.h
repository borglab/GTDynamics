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
#include <gtdynamics/utils/Interval.h>
#include <gtdynamics/utils/PointOnLink.h>
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

/**
 * Similar to the previous struct ContactGoal but with poses instead of
 * points.
 * Desired world pose for a end-effector pose.
 *
 * This simple struct stores the robot link that holds the end-effector, the
 * end-effector's pose in the final link's CoM frame, and a `goal_pose` in
 * world coordinate frames. The goal is satisfied iff
 * `pose_on_link.predict(values, k) == goal_pose`.
 */
struct PoseGoal {
  LinkSharedPtr ee_link;  ///< Link that hold end-effector
  gtsam::Pose3 comTgoal;  ///< Goal pose in link's CoM frame.
  gtsam::Pose3 wTgoal;    ///< Goal pose in world frame.

  /// Constructor
  PoseGoal(const LinkSharedPtr& ee_link, const gtsam::Pose3& comTgoal,
           const gtsam::Pose3& wTgoal)
      : ee_link(ee_link), comTgoal(comTgoal), wTgoal(wTgoal) {}

  /// Return link associated with contact pose.
  const LinkSharedPtr& link() const { return ee_link; }

  /// Return CoM pose needed to achieve goal pose.
  const gtsam::Pose3 wTcom() const {
    return wTgoal.compose(comTgoal.inverse());
  }

  /// Print to stream.
  friend std::ostream& operator<<(std::ostream& os, const PoseGoal& cg);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string& s) const;

  /**
   * @fn Check that the contact goal has been achieved for given values.
   * @param values a GTSAM Values instance that should contain link pose.
   * @param k time step to check (default 0).
   * @param tol tolerance in 3D (default 1e-9).
   */
  bool satisfied(const gtsam::Values& values, size_t k = 0,
                 double tol = 1e-9) const;
};

///< Map of time step k to PoseGoal for that time step
using PoseGoals = std::map<size_t, PoseGoal>;

/// Noise models etc specific to Kinematics class
struct KinematicsParameters : public OptimizationParameters {
  using Isotropic = gtsam::noiseModel::Isotropic;
  const gtsam::SharedNoiseModel p_cost_model,  // pose factor
      g_cost_model,                            // goal point
      prior_q_cost_model;                      // joint angle prior factor

  KinematicsParameters(
      const gtsam::SharedNoiseModel& p_cost_model = Isotropic::Sigma(6, 1e-4),
      const gtsam::SharedNoiseModel& g_cost_model = Isotropic::Sigma(3, 1e-2),
      const gtsam::SharedNoiseModel& prior_q_cost_model = Isotropic::Sigma(1,
                                                                           0.5))
      : p_cost_model(p_cost_model),
        g_cost_model(g_cost_model),
        prior_q_cost_model(prior_q_cost_model) {}

  // TODO(yetong): replace noise model with tolerance.
  KinematicsParameters(double p_cost_model_sigma,
                       double g_cost_model_sigma = 1e-2,
                       double prior_q_cost_model_sigma = 0.5)
      : KinematicsParameters(Isotropic::Sigma(6, p_cost_model_sigma),
                             Isotropic::Sigma(3, g_cost_model_sigma),
                             Isotropic::Sigma(1, prior_q_cost_model_sigma)) {}
};

/// All things kinematics, zero velocities/twists, and no forces.
class Kinematics : public Optimizer {
 protected:
  const KinematicsParameters p_;  // overrides Base::p_

 public:
  /**
   * @fn Constructor.
   */
  Kinematics(const KinematicsParameters& parameters = KinematicsParameters())
      : Optimizer(parameters), p_(parameters) {}

  /**
   * @fn Create graph with kinematics cost factors.
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @returns factor graph.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph graph(const CONTEXT& context,
                                    const Robot& robot) const;

  /**
   * @fn Create kinematics constraints.
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @returns Equality constraints.
   */
  template <class CONTEXT>
  gtsam::NonlinearEqualityConstraints constraints(const CONTEXT& context,
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
   * @fn Create contact-height objectives.
   * @param context Slice or Interval instance.
   * @param contact_points contact points on links.
   * @param gravity gravity vector used to define up direction.
   * @param cost_model noise model for each contact-height factor.
   * @returns graph with contact-height factors.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph contactHeightObjectives(
      const CONTEXT& context, const PointOnLinks& contact_points,
      const gtsam::Vector3& gravity,
      const gtsam::SharedNoiseModel& cost_model) const;

  /**
   * @fn Create point goal constraints.
   * @param context Slice or Interval instance.
   * @param contact_goals goals for contact points
   * @returns Equality constraints with point goal constraints.
   */
  template <class CONTEXT>
  gtsam::NonlinearEqualityConstraints pointGoalConstraints(
      const CONTEXT& context, const ContactGoals& contact_goals) const;

  /**
   * @fn Create hard constraints on joint angles.
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @param joint_targets Values containing desired joint angles.
   * Only keys present in joint_targets are constrained.
   * @returns Equality constraints on selected joint angles.
   */
  template <class CONTEXT>
  gtsam::NonlinearEqualityConstraints jointAngleConstraints(
      const CONTEXT& context, const Robot& robot,
      const gtsam::Values& joint_targets) const;

  /**
   * @fn Create a pose prior for a given link for each given pose.
   * @param context Slice or Interval instance.
   * @param pose_goals an object of PoseGoals: map of time step k to desired
   * pose at that time step, which will be used as mean of the prior
   * @returns graph with pose goal factors.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph poseGoalObjectives(
      const CONTEXT& context, const PoseGoals& pose_goals) const;

  /**
   * @fn Factors that minimize joint angles.
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @param joint_priors Values where the mean of the priors is specified. Uses
   * gtsam::Values to support different prior means for different time steps in
   * intervals/trajectories. The default is an empty Values, meaning that the
   * means will default to 0.
   * @returns graph with prior factors on joint angles.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph jointAngleObjectives(
      const CONTEXT& context, const Robot& robot,
      const gtsam::Values& joint_priors = gtsam::Values()) const;

  /**
   * @fn Factors that enforce joint angle limits.
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @return graph with prior factors on joint angles.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph jointAngleLimits(const CONTEXT& context,
                                               const Robot& robot) const;

  /**
   * @fn Initialize kinematics.
   *
   * If no values in initial_joints are given, use wTcom for poses and zero-mean
   * noise for joint angles.
   * If values are given, initialize joints with their given values, and
   * zero-mean noise to the ones that without a given value. Poses are
   * initialized with their fk values.
   *
   *
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @param gaussian_noise Gaussian noise standard deviation for initialization
   * (default 0.0).
   * @param initial_joints Initial values for joints.
   * @param use_fk If true, use forward kinematics to initialize poses based on
   * joint angles. If false, initialize poses near rest configuration with
   * Gaussian noise (default false).
   * @returns values with identity poses and zero joint angles.
   */
  template <class CONTEXT>
  gtsam::Values initialValues(
      const CONTEXT& context, const Robot& robot, double gaussian_noise = 0.0,
      const gtsam::Values& initial_joints = gtsam::Values(),
      bool use_fk = false) const;

  /**
   * @fn Inverse kinematics given a set of contact goals.
   * @fn This fuction does inverse kinematics seperately on each slice.
   * @param context Slice or Interval instance.
   * @param robot Robot specification from URDF/SDF.
   * @param contact_goals goals for contact points
   * @param contact_goals_as_constraints treat contact goal as hard constraints
   * @returns values with poses and joint angles.
   */
  template <class CONTEXT>
  gtsam::Values inverse(const CONTEXT& context, const Robot& robot,
                        const ContactGoals& contact_goals,
                        bool contact_goals_as_constraints = true) const;

  /**
   * @fn Inverse kinematics given a set of desired poses
   * @fn This function does inverse kinematics separately on each slice
   * @param context Slice or Interval instance
   * @param robot Robot specification from URDF/SDF
   * @param pose_goals goals for EE poses
   * @param joint_priors Optional argument to put priors centered on given
   * values. Uses gtsam::Values to support different prior means for different
   * time steps. If empty, the priors will be centered on the origin.
   * @return values with poses and joint angles
   */
  template <class CONTEXT>
  gtsam::Values inverse(
      const CONTEXT& context, const Robot& robot, const PoseGoals& pose_goals,
      const gtsam::Values& joint_priors = gtsam::Values()) const;

  /**
   * Interpolate using inverse kinematics: the goals are linearly interpolated.
   * @param context Interval instance
   * @param robot Robot specification from URDF/SDF.
   * @param contact_goals1 goals for contact points for context.k_start
   * @param contact_goals2 goals for contact points for context.k_end
   * All results are return in values.
   */
  template <class CONTEXT>
  gtsam::Values interpolate(const CONTEXT& context, const Robot& robot,
                            const ContactGoals& contact_goals1,
                            const ContactGoals& contact_goals2) const;
};
}  // namespace gtdynamics
