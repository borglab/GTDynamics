/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Trajectory.cpp
 * @brief Utility methods for generating Trajectory phases.
 * @author: Disha Das, Tarushree Gandhi
 * @author: Frank Dellaert, Gerry Chen
 */

#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/geometry/Point3.h>

#include <map>
#include <string>
#include <vector>

using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::SharedNoiseModel;
using gtsam::Values;
using gtsam::Z_6x1;
using std::map;
using std::string;
using std::vector;

namespace gtdynamics {

vector<NonlinearFactorGraph> Trajectory::getTransitionGraphs(
    DynamicsGraph &graph_builder, double mu) const {
  vector<NonlinearFactorGraph> transition_graphs;
  vector<ContactPoints> trans_cps = transitionContactPoints();
  vector<int> final_timesteps = finalTimeSteps();
  for (int p = 1; p < numPhases(); p++) {
    transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
        robot_, final_timesteps[p - 1], trans_cps[p - 1], mu));
  }
  return transition_graphs;
}

NonlinearFactorGraph Trajectory::multiPhaseFactorGraph(
    DynamicsGraph &graph_builder, const CollocationScheme collocation,
    double mu) const {
  // Graphs for transition between phases + their initial values.
  auto transition_graphs = getTransitionGraphs(graph_builder, mu);
  return graph_builder.multiPhaseTrajectoryFG(robot_, phaseDurations(),
                                              transition_graphs, collocation,
                                              phaseContactPoints(), mu);
}

vector<Values> Trajectory::transitionPhaseInitialValues(
    double gaussian_noise) const {
  vector<ContactPoints> trans_cps = transitionContactPoints();
  vector<Values> transition_graph_init;
  vector<int> final_timesteps = finalTimeSteps();
  for (int p = 1; p < numPhases(); p++) {
    transition_graph_init.push_back(ZeroValues(
        robot_, final_timesteps[p - 1], gaussian_noise, trans_cps[p - 1]));
  }
  return transition_graph_init;
}

Values Trajectory::multiPhaseInitialValues(double gaussian_noise,
                                           double dt) const {
  vector<Values> transition_graph_init =
      transitionPhaseInitialValues(gaussian_noise);
  return MultiPhaseZeroValuesTrajectory(robot_, phaseDurations(),
                                        transition_graph_init, dt,
                                        gaussian_noise, phaseContactPoints());
}

NonlinearFactorGraph Trajectory::contactLinkObjectives(
    const SharedNoiseModel &cost_model, const double ground_height) const {
  NonlinearFactorGraph factors;

  // Initials contact point goal.
  // TODO(frank): #179 make sure height is handled correctly.
  map<string, Point3> cp_goals = walk_cycle_.initContactPointGoal(robot_);

  // Distance to move contact point per time step during swing.
  // TODO(frank): this increases y. That can't be general in any way.
  // TODO(frank): figure out step from desired velocity
  const Point3 step(0, 0.4, 0);

  // Add contact point objectives to factor graph.
  for (int p = 0; p < numPhases(); p++) {
    int k_start = getStartTimeStep(p);

    const Phase &phase = this->phase(p);
    factors.add(phase.stanceObjectives(robot_, cp_goals, cost_model, k_start));

    factors.add(walk_cycle_.swingObjectives(robot_, phaseIndex(p), cp_goals,
                                            step, cost_model, k_start));

    // Update the goal point for the swing links.
    for (auto &&name : getPhaseSwingLinks(p)) {
      cp_goals[name] = cp_goals[name] + step;
    }
  }
  return factors;
}

void Trajectory::addBoundaryConditions(
    gtsam::NonlinearFactorGraph *graph, const Robot &robot,
    const SharedNoiseModel &pose_model, const SharedNoiseModel &twist_model,
    const SharedNoiseModel &twist_acceleration_model,
    const SharedNoiseModel &joint_velocity_model,
    const SharedNoiseModel &joint_acceleration_model) const {
  // Get final time step.
  int K = getEndTimeStep(numPhases() - 1);

  // Add link boundary conditions to FG.
  for (auto &&link : robot.links()) {
    // Initial link pose, twists.
    add_link_objectives(graph, link->id(), 0)
        .pose(link->wTcom(), pose_model)
        .twist(Z_6x1, twist_model);

    // Final link twists, accelerations.
    add_link_objectives(graph, link->id(), K)
        .twist(Z_6x1, twist_model)
        .twistAccel(Z_6x1, twist_acceleration_model);
  }

  // Add joint boundary conditions to FG.
  add_joints_at_rest_objectives(graph, robot, joint_velocity_model,
                                joint_acceleration_model, 0);
  add_joints_at_rest_objectives(graph, robot, joint_velocity_model,
                                joint_acceleration_model, K);
}

void Trajectory::addMinimumTorqueFactors(
    gtsam::NonlinearFactorGraph *graph, const Robot &robot,
    const SharedNoiseModel &cost_model) const {
  int K = getEndTimeStep(numPhases() - 1);
  for (auto &&joint : robot.joints()) {
    auto j = joint->id();
    for (int k = 0; k <= K; k++) {
      graph->emplace_shared<MinTorqueFactor>(internal::TorqueKey(j, k),
                                             cost_model);
    }
  }
}

}  // namespace gtdynamics
