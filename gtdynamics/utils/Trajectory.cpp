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
 * @author: Frank Dellaert, Gerry Chen, Frank Dellaert
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
    const SharedNoiseModel &cost_model, const Point3 &step) const {
  NonlinearFactorGraph factors;

  // Initials contact point goal.
  // TODO(frank): #179 make sure height is handled correctly.
  map<string, Point3> cp_goals = walk_cycle_.initContactPointGoal(robot_);

  size_t k_start = 0;
  for (int w = 0; w < repeat_; w++) {
    factors.add(walk_cycle_.contactLinkObjectives(robot_, cost_model, step,
                                                  k_start, &cp_goals));
    k_start += walk_cycle_.numTimeSteps();
  }
  return factors;
}

void Trajectory::addBoundaryConditions(
    gtsam::NonlinearFactorGraph *graph, const SharedNoiseModel &pose_model,
    const SharedNoiseModel &twist_model,
    const SharedNoiseModel &twist_acceleration_model,
    const SharedNoiseModel &joint_velocity_model,
    const SharedNoiseModel &joint_acceleration_model) const {
  // Get final time step.
  int K = getEndTimeStep(numPhases() - 1);

  // Add link boundary conditions to FG.
  for (auto &&link : robot_.links()) {
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
  add_joints_at_rest_objectives(graph, robot_, joint_velocity_model,
                                joint_acceleration_model, 0);
  add_joints_at_rest_objectives(graph, robot_, joint_velocity_model,
                                joint_acceleration_model, K);
}

void Trajectory::addMinimumTorqueFactors(
    gtsam::NonlinearFactorGraph *graph,
    const SharedNoiseModel &cost_model) const {
  int K = getEndTimeStep(numPhases() - 1);
  for (auto &&joint : robot_.joints()) {
    auto j = joint->id();
    for (int k = 0; k <= K; k++) {
      graph->emplace_shared<MinTorqueFactor>(internal::TorqueKey(j, k),
                                             cost_model);
    }
  }
}

void Trajectory::writePhaseToFile(std::ofstream &traj_file,
                                  const gtsam::Values &results,
                                  int phase) const {
  int k = getStartTimeStep(phase);
  auto phase_durations = phaseDurations();
  for (int time_step = 0; time_step < phase_durations[phase]; time_step++) {
    std::vector<std::string> vals;
    for (auto &&joint : robot_.joints())
      vals.push_back(std::to_string(JointAngle(results, joint->id(), k)));
    for (auto &&joint : robot_.joints())
      vals.push_back(std::to_string(JointVel(results, joint->id(), k)));
    for (auto &&joint : robot_.joints())
      vals.push_back(std::to_string(JointAccel(results, joint->id(), k)));
    for (auto &&joint : robot_.joints())
      vals.push_back(std::to_string(Torque(results, joint->id(), k)));
    vals.push_back(std::to_string(results.atDouble(PhaseKey(phase))));
    k++;
    std::string vals_str = boost::algorithm::join(vals, ",");
    traj_file << vals_str << "\n";
  }
}

}  // namespace gtdynamics
