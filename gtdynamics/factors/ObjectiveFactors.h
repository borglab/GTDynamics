/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObjectiveFactors.h
 * @brief Support for adding PriorFactors to specify objectives.
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <iostream>

namespace gtdynamics {

/**
 * @brief Add objectives to graph for link i at time k using proxy class idiom
 * for keyword argument -like syntax.
 *
 * Example Usage:
 *  add_link_objectives(graph, id, k).pose(Pose3(), noise)
 *                                   .twist(Z_6x1, noise)
 *                                   .twistAccel(Z_6x1, noise);
 */
class add_link_objectives {
 private:
  gtsam::NonlinearFactorGraph* graph_;
  int i_;  // link id
  int k_;  // time step index
 public:
  /**
   * @brief General arguments:
   * @param graph to add to.
   * @param i The link id.
   * @param k Time step index (default 0).
   */
  add_link_objectives(gtsam::NonlinearFactorGraph* graph, int i, int k = 0)
      : graph_(graph), i_(i), k_(k) {}

  /// @name Optional arguments
  ///@{
  /**
   * @brief Add pose prior
   * @param pose target pose.
   * @param model noise model used in factor.
   */
  add_link_objectives& pose(
      gtsam::Pose3 pose,  //
      const gtsam::SharedNoiseModel& pose_model = nullptr) {
    graph_->addPrior<gtsam::Pose3>(internal::PoseKey(i_, k_),  //
                                   pose, pose_model);
    return *this;
  }
  /**
   * @brief Add twist prior
   * @param twist target twist.
   * @param model noise model used in factor.
   */
  add_link_objectives& twist(
      gtsam::Vector6 twist,
      const gtsam::SharedNoiseModel& twist_model = nullptr) {
    graph_->addPrior<gtsam::Vector6>(internal::TwistKey(i_, k_),  //
                                     twist, twist_model);
    return *this;
  }
  /**
   * @brief Add twist acceleration prior
   * @param twistAccel target twist acceleration.
   * @param model noise model used in factor.
   */
  add_link_objectives& twistAccel(
      gtsam::Vector6 twistAccel,
      const gtsam::SharedNoiseModel& twistAccel_model = nullptr) {
    graph_->addPrior<gtsam::Vector6>(internal::TwistAccelKey(i_, k_),
                                     twistAccel, twistAccel_model);
    return *this;
  }
  ///@}
};

/**
 * @brief Add objectives to graph for joint j at time k using proxy class idiom
 * for keyword argument -like syntax.
 *
 * Example Usage:
 *  add_joint_objectives(graph, id, k).angle(0, noise)
 *                                    .velocity(0, noise)
 *                                    .accel(0, noise);
 */
class add_joint_objectives {
 private:
  gtsam::NonlinearFactorGraph* graph_;
  int j_;  // joint id
  int k_;  // time step index
 public:
  /**
   * General arguments:
   * @param graph to add to.
   * @param j The joint id.
   * @param k Time step index (default 0).
   */
  add_joint_objectives(gtsam::NonlinearFactorGraph* graph, int j, int k = 0)
      : graph_(graph), j_(j), k_(k) {}

  /// @name Optional arguments
  ///@{
  /**
   * @brief Add joint angle prior
   * @param angle target angle.
   * @param model noise model used in factor.
   */
  add_joint_objectives& angle(  //
      double angle,             //
      const gtsam::SharedNoiseModel& angle_model = nullptr) {
    graph_->addPrior<double>(internal::JointAngleKey(j_, k_),  //
                             angle, angle_model);
    return *this;
  }
  /**
   * @brief Add joint velocity prior
   * @param velocity target velocity.
   * @param model noise model used in factor.
   */
  add_joint_objectives& velocity(
      double velocity,
      const gtsam::SharedNoiseModel& velocity_model = nullptr) {
    graph_->addPrior<double>(internal::JointVelKey(j_, k_),  //
                             velocity, velocity_model);
    return *this;
  }
  /**
   * @brief Add joint acceleration prior
   * @param acceleration target acceleration.
   * @param model noise model used in factor.
   */
  add_joint_objectives& acceleration(
      double acceleration,
      const gtsam::SharedNoiseModel& acceleration_model = nullptr) {
    graph_->addPrior<double>(internal::JointAccelKey(j_, k_),  //
                             acceleration, acceleration_model);
    return *this;
  }
};

void add_joints_at_rest_objectives(
    gtsam::NonlinearFactorGraph* graph, const Robot& robot,
    const gtsam::SharedNoiseModel& joint_velocity_model,
    const gtsam::SharedNoiseModel& joint_acceleration_model, int k = 0) {
  for (auto&& joint : robot.joints()) {
    add_joint_objectives(graph, joint->id(), k)
        .velocity(0, joint_velocity_model)
        .acceleration(0, joint_acceleration_model);
  }
}

/**
 * @brief  Add PointGoalFactors given a trajectory.
 * @param factors graph to add to.
 *
 * @param goal_trajectory end effector goal trajectory, in world coordinates
 *
 * @param cost_model noise model
 * @param point_com point on link, in COM coordinate frame
 * @param i The link id.
 * @param k starting time index (default 0).
 */
void AddPointGoalFactors(gtsam::NonlinearFactorGraph* factors,
                         const gtsam::SharedNoiseModel& cost_model,
                         const gtsam::Point3& point_com,
                         const std::vector<gtsam::Point3>& goal_trajectory,
                         unsigned char i, size_t k = 0) {
  gtsam::Key key = internal::PoseKey(i, k);
  factors->add(PointGoalFactors(key, cost_model, point_com, goal_trajectory));
}

/**
 * @brief Create stance foot trajectory.
 *
 * @param num_steps number of time steps
 * @param stance_point end effector goal, in world coordinates
 */
std::vector<gtsam::Point3> StanceTrajectory(const gtsam::Point3& stance_point,
                                            size_t num_steps) {
  return std::vector<gtsam::Point3>(num_steps, stance_point);
}

/**
 * @brief Create simple swing foot trajectory, from start to start + step.
 *
 * Swing feet is moved according to a pre-determined height trajectory, and
 * moved by the 3D vector step.
 * To see the curve, go to https://www.wolframalpha.com/ and type
 *    0.2 * pow(t, 1.1) * pow(1 - t, 0.7) for t from 0 to 1
 * Note the first goal point is *off* the ground and forwards of start.
 * Likewise the last goal point is off the ground and is not yet at start+step.
 *
 * @param start initial end effector goal, in world coordinates
 * @param step 3D vector to move by
 * @param num_steps number of time steps
 */
std::vector<gtsam::Point3> SimpleSwingTrajectory(const gtsam::Point3& start,
                                                 const gtsam::Point3& step,
                                                 size_t num_steps) {
  std::vector<gtsam::Point3> goal_trajectory;
  // if num_steps==3 -> 0--|--|--|--1
  const double dt = 1.0 / (num_steps + 1);
  const gtsam::Point3 delta_step = step * dt;
  gtsam::Point3 cp_goal = start + delta_step;
  for (int k = 0; k < num_steps; k++) {
    double t = dt * (k + 1);
    double h = 0.2 * pow(t, 1.1) * pow(1 - t, 0.7);  // reaches 6 cm height
    goal_trajectory.push_back(cp_goal + gtsam::Point3(0, 0, h));
    cp_goal = cp_goal + delta_step;
  }
  return goal_trajectory;
}

}  // namespace gtdynamics
