/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObjectiveFactors.cpp
 * @brief Support for adding PriorFactors to specify objectives.
 * @author Frank Dellaert
 */

#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <iostream>

namespace gtdynamics {

using gtsam::Point3;
using gtsam::SharedNoiseModel;

gtsam::NonlinearFactorGraph JointsAtRestObjectives(
    const Robot& robot, const SharedNoiseModel& joint_velocity_model,
    const SharedNoiseModel& joint_acceleration_model, int k) {
  gtsam::NonlinearFactorGraph graph;
  for (auto&& joint : robot.joints()) {
    graph.add(JointObjectives(joint->id(), k)
                  .velocity(0, joint_velocity_model)
                  .acceleration(0, joint_acceleration_model));
  }
  return graph;
}

gtsam::NonlinearFactorGraph PointGoalFactors(
    const SharedNoiseModel& cost_model, const Point3& point_com,
    const std::vector<Point3>& goal_trajectory, unsigned char i, size_t k) {
  gtsam::Key key = internal::PoseKey(i, k);
  return PointGoalFactors(key, cost_model, point_com, goal_trajectory);
}

std::vector<Point3> StanceTrajectory(const Point3& stance_point,
                                     size_t num_steps) {
  return std::vector<Point3>(num_steps, stance_point);
}

std::vector<Point3> SimpleSwingTrajectory(const Point3& start,
                                          const Point3& step,
                                          size_t num_steps) {
  std::vector<Point3> goal_trajectory;
  const double dt = 1.0 / (num_steps + 1);
  const Point3 delta_step = step * dt;
  Point3 cp_goal = start + delta_step;
  for (size_t k = 0; k < num_steps; k++) {
    double t = dt * (k + 1);
    double h = 0.2 * pow(t, 1.1) * pow(1 - t, 0.7);  // reaches 6 cm height
    goal_trajectory.push_back(cp_goal + Point3(0, 0, h));
    cp_goal = cp_goal + delta_step;
  }
  return goal_trajectory;
}

}  // namespace gtdynamics
