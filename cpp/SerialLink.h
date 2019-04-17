/**
 * @file  serial_link.h
 * @brief manipulator links
 * @Author: Frank Dellaert and Mandy Xie
 */

#ifndef SERIALLINK_H
#define SERIALLINK_H

#include <JointLimitVectorFactor.h>
#include <PoseGoalFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

extern gtsam::Key J(int j);

namespace manipulator {

template <typename T>
class SerialLink {
 private:
  std::vector<T> links_;
  gtsam::Pose3 base_;
  gtsam::Pose3 tool_;
  std::vector<gtsam::Vector6> screwAxes_;

 public:
  /**
   * Construct serial link manipulator from list of Link instances
   * Keyword arguments:
          calibration -- Link list
          base        -- optional wT0 base frame in world frame
          tool        -- optional tool frame in link N frame
   */
  SerialLink(const std::vector<T> &links,
             const gtsam::Pose3 &base = gtsam::Pose3(),
             const gtsam::Pose3 &tool = gtsam::Pose3());

  /* Copy constructor */
  SerialLink(const SerialLink &serial_link)
      : links_(serial_link.links_),
        base_(serial_link.base_),
        tool_(serial_link.tool_),
        screwAxes_(serial_link.screwAxes_) {}

  /* Return base pose in world frame */
  gtsam::Pose3 base() const { return base_; }

  /* Return tool pose in link N frame */
  gtsam::Pose3 tool() const { return tool_; }

  /* Return number of *moving* links */
  int numLinks() const { return links_.size(); }

  /* Return the ith link */
  T link(int i) const { return links_[i]; }

  /* Return all joint lower limits */
  gtsam::Vector jointLowerLimits() const;

  /* Return all joint uppper limits */
  gtsam::Vector jointUpperLimits() const;

  /* Return all joint limit thresholds */
  gtsam::Vector jointLimitThresholds() const;

  /** Calculate link transforms for all links
   * Keyword arguments:
          q (gtsam::Vector) -- optional joint angles (default all zero).
   */
  std::vector<gtsam::Pose3> linkTransforms(
      const gtsam::Vector &q = gtsam::Vector::Zero(1)) const;

  /** Forward kinematics.
   * Keyword arguments:
          q (gtsam::Vector) -- joint angles.
          J (Matrix) -- Jacobian matrix
     Returns tool frame in world frame.
   */
  std::vector<gtsam::Pose3> forwardKinematics(
      const gtsam::Vector &q,
      boost::optional<std::vector<gtsam::Matrix> &> J = boost::none) const;

  /** Return each link frame for given joint angles.
   * Note that frame Tj is aligned with the joint axis of joint j+1
          according to the Denavit-Hartenberg convention.
   * Keyword arguments:
          q (gtsam::Vector) -- optional joint angles (default all zero).
   */
  std::vector<gtsam::Pose3> linkFrames(
      const gtsam::Vector &q = gtsam::Vector::Zero(1)) const;

  /** Return each link's center of mass frame at rest, in the world frame.
   * Keyword arguments:
          q (gtsam::Vector) -- optional joint angles (default all zero).
   */
  std::vector<gtsam::Pose3> comFrames(
      const gtsam::Vector &q = gtsam::Vector::Zero(1)) const;

  /** calculate the rigid body transformation which takes the joint frames
   * from its reference configuration to the current configuration fo the
   * manipulator. R. Murray's book, page 116 about manipulator jacobian
   * Keyword arguments:
          q (gtsam::Vector) -- optional joint angles (default all zero).
   */
  std::vector<gtsam::Pose3> transformPOE(
      const gtsam::Vector &q = gtsam::Vector::Zero(1)) const;

  /* Return screw axes for all joints, expressed in their COM frame. */
  std::vector<gtsam::Vector6> screw_axes() const { return screwAxes_; }

  /* Return screw axes for all joints at rest configuration, expressed in world
   * frame.
   */
  std::vector<gtsam::Vector6> spatialScrewAxes() const;

  /** calculate spatial manipulator jacobian and joint poses
   * Keyword arguments:
   *      q (gtsam::Vector)  -- angles for revolution joint, distances for
   * prismatic joint
   */
  std::vector<gtsam::Matrix> spatialManipulatorJacobian(
      const gtsam::Vector &q) const;

  /** calculate "body manipulator jacobian" and joint poses
   * Keyword arguments:
   *      q (gtsam::Vector)  -- angles for revolution joint, distances for
   * prismatic joint sTb (gtsam::Pose3) -- eef body frame expressed base frame
   */
  std::vector<gtsam::Matrix> bodyManipulatorJacobian(
      const gtsam::Vector &q, std::vector<gtsam::Pose3> &sTb) const;

  /** Calculate velocity twists for all joints, expressed in their COM frame.
   * Keyword arguments:
   *      Ts -- link's center of mass frame expressed in the world frame
   *      joint_vecocities -- joint angular velocities (in rad/s)
   */
  std::vector<gtsam::Vector6> twists(
      const std::vector<gtsam::Pose3> &Ts,
      const gtsam::Vector &joint_velocities) const;

  /** Calculate list of transforms from COM frame j-1 relative to COM j.
   * Keyword arguments:
   *      q (gtsam::Vector) -- joint angles (in rad).
   *  Returns list of transforms, 2 more than number of links:
   *      - first transform is bT1, i.e. base expressed in link 1
   *      - last transform is tTnc, i.e., link N COM frame expressed in tool
   * frame
   */
  std::vector<gtsam::Pose3> jTi_list(const gtsam::Vector &q) const;

  /** Build factor graph for RR manipulator forward dynamics.
   * Keyword arguments:
   *      q (gtsam::Vector) -- joint angles (in rad).
   *      joint_vecocities -- joint angular velocities (in rad/s)
   *      torques (in Nm)
   *      gravity (np.array) -- if given, will create gravity forces
   *      base_twist_accel -- optional acceleration for base
   *      external_wrench  -- optional external wrench
   * Note: see Link.base_factor on use of base_twist_accel
   * Return Gaussian factor graph
   */
  gtsam::GaussianFactorGraph forwardDynamicsFactorGraph(
      const gtsam::Vector &q, const gtsam::Vector &joint_velocities,
      const gtsam::Vector &torques,
      const gtsam::Vector6 &base_twist_accel = gtsam::Vector6::Zero(),
      const gtsam::Vector6 &external_wrench = gtsam::Vector6::Zero(),
      boost::optional<gtsam::Vector3 &> gravity = boost::none) const;

  /** Build factor graph for RR manipulator inverse dynamics.
   * Keyword arguments:
   *      q (gtsam::Vector) -- joint angles (in rad).
   *      joint_vecocities -- joint angular velocities (in rad/s)
   *      joint_accelerations (np.array, in rad/s^2)
   *      gravity (np.array) -- if given, will create gravity forces
   *      base_twist_accel -- optional acceleration for base
   *      external_wrench  -- optional external wrench
   * Note: see Link.base_factor on use of base_twist_accel
   * Return Gaussian factor graph
   */
  gtsam::GaussianFactorGraph inverseDynamicsFactorGraph(
      const gtsam::Vector &q, const gtsam::Vector &joint_velocities,
      const gtsam::Vector &joint_accelerations,
      const gtsam::Vector6 &base_twist_accel = gtsam::Vector6::Zero(),
      const gtsam::Vector6 &external_wrench = gtsam::Vector6::Zero(),
      boost::optional<gtsam::Vector3 &> gravity = boost::none) const;

  /* Extract joint accelerations for all joints from gtsam::VectorValues. */
  gtsam::Vector extractJointAcceleraions(
      const gtsam::VectorValues &result) const;

  /* Extract torques for all joints from gtsam::VectorValues. */
  gtsam::Vector extractTorques(const gtsam::VectorValues &result) const;

  /** Optimize factor graph for manipulator forward dynamics.
   * Keyword arguments:
   *      fdynamics_factor_graph -- factor graph for forward dynamics.
   * Note: use extract_joint_accelerations to filter out jojt accelerations.
   * Returns gtsam::VectorValues with all unknowns:
          - N+1 twist accelerations (base + links)
          - N+1 torques (links + tool)
          - N joint accelerations.
  */
  gtsam::VectorValues factorGraphOptimization(
      const gtsam::GaussianFactorGraph &dynamics_factor_graph) const;

  /** Calculate joint accelerations from manipulator state and torques.
          See fdynamics_factor_graph for input arguments.
  */
  gtsam::Vector forwardDynamics(
      const gtsam::Vector &q, const gtsam::Vector &joint_velocities,
      const gtsam::Vector &torques,
      const gtsam::Vector6 &base_twist_accel = gtsam::Vector6::Zero(),
      const gtsam::Vector6 &external_wrench = gtsam::Vector6::Zero(),
      boost::optional<gtsam::Vector3 &> gravity = boost::none) const;

  /** Calculate joint accelerations from manipulator state and torques.
          See idynamics_factor_graph for input arguments.
  */
  gtsam::Vector inverseDynamics(
      const gtsam::Vector &q, const gtsam::Vector &joint_velocities,
      const gtsam::Vector &joint_accelerations,
      const gtsam::Vector6 &base_twist_accel = gtsam::Vector6::Zero(),
      const gtsam::Vector6 &external_wrench = gtsam::Vector6::Zero(),
      boost::optional<gtsam::Vector3 &> gravity = boost::none) const;

  /** joint limit factors
   * Returns joint limit factors.
   */
  JointLimitVectorFactor jointLimitVectorFactor() const;

  /** pose goal factor
   * Return pose goal factor
   * Keyword arguments:
          goal pose (gtsam::Pose3) -- eef pose goal.
  */
  PoseGoalFactor poseGoalFactor(const gtsam::Pose3 &goal_pose) const;

  /** return inverse kinematics factor graph
   * Keyword arguments:
          goal pose (gtsam::Pose3) -- eef pose goal.
  */
  gtsam::NonlinearFactorGraph inverseKinematicsFactorGraph(
      const gtsam::Pose3 &goal_pose) const;

  /* Extract joint postitions for all joints from gtsam::VectorValues. */
  gtsam::Vector extractJointCooridinates(const gtsam::Values &results) const;

  /** Set optimizer and optimize for ikine factor graph
   * Keyword arguments:
   *      graph(NonlinearFactorGraph) -- ikine_factor_graph
          init_values -- initial values for optimizer
  */
  gtsam::Values factorGraphOptimization(
      const gtsam::NonlinearFactorGraph &graph,
      const gtsam::Values &init_values) const;

  /** Inverse kinematics.
   * Keyword arguments:
          goal pose (gtsam::Pose3) -- eef pose goal.
          init_q -- initial guess for joint position values
     Returns joint angles gtsam::Vector.
   */
  gtsam::Vector inverseKinematics(const gtsam::Pose3 &goal_pose,
                                  const gtsam::Vector &init_q) const;
};
}  // namespace manipulator
#endif

