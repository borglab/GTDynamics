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
