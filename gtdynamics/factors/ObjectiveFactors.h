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

#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <iostream>

namespace gtdynamics {

/**
 * @brief Add an pose objective to graph for the link i at time k.
 *
 * @param graph to add to.
 * @param pose target pose.
 * @param model noise model used in factor.
 * @param i The link id.
 * @param k Time step index (default 0).
 */
void add_pose_objective(gtsam::NonlinearFactorGraph* graph,
                        const gtsam::Pose3& pose,
                        const gtsam::SharedNoiseModel& model, int i,
                        int k = 0) {
  graph->addPrior<gtsam::Pose3>(internal::PoseKey(i, k), pose, model);
}

/**
 * @brief Add pose/twist objective to graph for the link i at time k.
 *
 * @param graph to add to.
 * @param pose target pose.
 * @param pose_model noise model used for pose factor.
 * @param twist target twist.
 * @param twist_model noise model used for twist factor.
 * @param i The link id.
 * @param k Time step index (default 0).
 */
void add_link_objective(gtsam::NonlinearFactorGraph* graph,
                        const gtsam::Pose3& pose,
                        const gtsam::SharedNoiseModel& pose_model,
                        const gtsam::Vector6& twist,
                        const gtsam::SharedNoiseModel& twist_model, int i,
                        int k = 0) {
  add_pose_objective(graph, pose, pose_model, i, k);
  graph->addPrior<gtsam::Vector6>(internal::TwistKey(i, k), twist, twist_model);
}

/**
 * @brief Add twist+derivatives objective to graph for the link i at time k.
 *
 * @param graph to add to.
 * @param twist target twist.
 * @param twist_model noise model used for twist.
 * @param twist_acceleration target twist acceleration.
 * @param twist_acceleration_model noise model used for twist acceleration.
 * @param i The link id.
 * @param k Time step index (default 0).
 */
void add_twist_objective(
    gtsam::NonlinearFactorGraph* graph, const gtsam::Vector6& twist,
    const gtsam::SharedNoiseModel& twist_model,  //
    const gtsam::Vector6& twist_acceleration,
    const gtsam::SharedNoiseModel& twist_acceleration_model,  //
    int i, int k = 0) {
  graph->addPrior<gtsam::Vector6>(internal::TwistKey(i, k), twist, twist_model);
  graph->addPrior<gtsam::Vector6>(internal::TwistAccelKey(i, k),
                                  twist_acceleration, twist_acceleration_model);
}

/**
 * @brief Add joint_angle objective to graph for joint j at time k.
 *
 * @param graph to add to.
 * @param joint_angle target joint= angle.
 * @param joint_angle_model noise model used for joint_angle.
 * @param j The joint id.
 * @param k Time step index (default 0).
 */
void add_joint_objective(gtsam::NonlinearFactorGraph* graph, double joint_angle,
                         const gtsam::SharedNoiseModel& joint_angle_model,  //
                         int j, int k = 0) {
  graph->addPrior<double>(internal::JointAngleKey(j, k), joint_angle,
                          joint_angle_model);
}

/**
 * @brief Add joint_angle derivatives objective to graph for joint j at time k.
 *
 * @param graph to add to.
 * @param joint_velocity target velocity.
 * @param joint_velocity_model noise model used for velocity.
 * @param joint_acceleration target acceleration.
 * @param joint_acceleration_model noise model used for acceleration.
 * @param j The joint id.
 * @param k Time step index (default 0).
 */
void add_joint_derivative_objectives(
    gtsam::NonlinearFactorGraph* graph,  //
    double joint_velocity,
    const gtsam::SharedNoiseModel& joint_velocity_model,  //
    double joint_acceleration,
    const gtsam::SharedNoiseModel& joint_acceleration_model,  //
    int j, int k = 0) {
  graph->addPrior<double>(internal::JointVelKey(j, k), joint_velocity,
                          joint_velocity_model);
  graph->addPrior<double>(internal::JointAccelKey(j, k), joint_acceleration,
                          joint_acceleration_model);
}

/**
 * @brief Add joint_angle+derivatives objective to graph for joint j at time k.
 *
 * @param graph to add to.
 * @param joint_angle target joint= angle.
 * @param joint_angle_model noise model used for joint_angle.
 * @param joint_velocity target velocity.
 * @param joint_velocity_model noise model used for velocity.
 * @param joint_acceleration target acceleration.
 * @param joint_acceleration_model noise model used for acceleration.
 * @param j The joint id.
 * @param k Time step index (default 0).
 */
void add_joint_objectives(
    gtsam::NonlinearFactorGraph* graph, double joint_angle,
    const gtsam::SharedNoiseModel& joint_angle_model,  //
    double joint_velocity,
    const gtsam::SharedNoiseModel& joint_velocity_model,  //
    double joint_acceleration,
    const gtsam::SharedNoiseModel& joint_acceleration_model,  //
    int j, int k = 0) {
  add_joint_objective(graph, joint_angle, joint_angle_model, j, k);
  add_joint_derivative_objectives(graph, joint_velocity, joint_velocity_model,
                                  joint_acceleration, joint_acceleration_model,
                                  j, k);
}



/**
 * @brief Proxy class idiom to add objectives to graph for link i at time k
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
  add_link_objectives& pose(gtsam::Pose3 pose,
                            const gtsam::SharedNoiseModel& pose_model) {
    graph_->addPrior<gtsam::Pose3>(internal::PoseKey(i_, k_),  //
                                   pose, pose_model);
    return *this;
  }
  /**
   * @brief Add twist prior
   * @param twist target twist.
   * @param model noise model used in factor.
   */
  add_link_objectives& twist(gtsam::Vector6 twist,
                             const gtsam::SharedNoiseModel& twist_model) {
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
      const gtsam::SharedNoiseModel& twistAccel_model) {
    graph_->addPrior<gtsam::Vector6>(internal::JointAccelKey(i_, k_),
                                     twistAccel, twistAccel_model);
    return *this;
  }
  ///@}
};

/**
 * @brief Proxy class idiom to add objectives to graph for joint j at time k
 * 
 * Example Usage:
 *  add_joint_objectives(graph, id, k).angle(0, noise)
 *                                    .vel(0, noise)
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
      double angle, const gtsam::SharedNoiseModel& angle_model) {
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
      double velocity, const gtsam::SharedNoiseModel& velocity_model) {
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
      double acceleration, const gtsam::SharedNoiseModel& acceleration_model) {
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
    add_joint_derivative_objectives(graph,                        //
                                    0, joint_velocity_model,      //
                                    0, joint_acceleration_model,  //
                                    joint->id(), k);
  }
}

}  // namespace gtdynamics
