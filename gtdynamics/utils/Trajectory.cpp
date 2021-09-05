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

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <iostream>
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
using std::to_string;
using std::vector;

namespace gtdynamics {

vector<NonlinearFactorGraph> Trajectory::getTransitionGraphs(
    const Robot &robot, const DynamicsGraph &graph_builder, double mu) const {
  vector<NonlinearFactorGraph> transition_graphs;
  const vector<int> final_timesteps = finalTimeSteps();
  const vector<PointOnLinks> trans_cps = transitionContactPoints();
  for (int p = 1; p < numPhases(); p++) {
    transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
        robot, final_timesteps[p - 1], trans_cps[p - 1], mu));
  }
  return transition_graphs;
}

NonlinearFactorGraph Trajectory::multiPhaseFactorGraph(
    const Robot &robot, const DynamicsGraph &graph_builder,
    const CollocationScheme collocation, double mu) const {
  // Graphs for transition between phases + their initial values.
  auto transition_graphs = getTransitionGraphs(robot, graph_builder, mu);
  return graph_builder.multiPhaseTrajectoryFG(robot, phaseDurations(),
                                              transition_graphs, collocation,
                                              phaseContactPoints(), mu);
}

vector<Values> Trajectory::transitionPhaseInitialValues(
    const Robot &robot, double gaussian_noise) const {
  vector<PointOnLinks> trans_cps = transitionContactPoints();
  vector<Values> transition_graph_init;
  vector<int> final_timesteps = finalTimeSteps();
  for (int p = 1; p < numPhases(); p++) {
    transition_graph_init.push_back(ZeroValues(
        robot, final_timesteps[p - 1], gaussian_noise, trans_cps[p - 1]));
  }
  return transition_graph_init;
}

Values Trajectory::multiPhaseInitialValues(const Robot &robot,
                                           double gaussian_noise,
                                           double dt) const {
  vector<Values> transition_graph_init =
      transitionPhaseInitialValues(robot, gaussian_noise);
  return MultiPhaseZeroValuesTrajectory(robot, phaseDurations(),
                                        transition_graph_init, dt,
                                        gaussian_noise, phaseContactPoints());
}

NonlinearFactorGraph Trajectory::contactPointObjectives(
    const Robot &robot, const SharedNoiseModel &cost_model, const Point3 &step,
    double ground_height) const {
  NonlinearFactorGraph factors;

  // Initialize contact point goals.
  ContactPointGoals cp_goals =
      walk_cycle_.initContactPointGoal(robot, ground_height);

  size_t k_start = 0;
  for (int w = 0; w < repeat_; w++) {
    factors.add(walk_cycle_.contactPointObjectives(step, cost_model, k_start,
                                                   &cp_goals));
    k_start += walk_cycle_.numTimeSteps();
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
    graph->add(LinkObjectives(link->id(), 0)
                   .pose(link->bMcom(), pose_model)
                   .twist(Z_6x1, twist_model));

    // Final link twists, accelerations.
    graph->add(LinkObjectives(link->id(), K)
                   .twist(Z_6x1, twist_model)
                   .twistAccel(Z_6x1, twist_acceleration_model));
  }

  // Add joint boundary conditions to FG.
  graph->add(JointsAtRestObjectives(robot, joint_velocity_model,
                                    joint_acceleration_model, 0));
  graph->add(JointsAtRestObjectives(robot, joint_velocity_model,
                                    joint_acceleration_model, K));
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

void Trajectory::writePhaseToFile(const Robot &robot, std::ofstream &file,
                                  const gtsam::Values &results, int p) const {
  using gtsam::Matrix;

  // Extract joint values.
  int k = getStartTimeStep(p);
  Matrix mat =
      phase(p).jointMatrix(robot, results, k, results.atDouble(PhaseKey(p)));

  // Write to file.
  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                         Eigen::DontAlignCols, ", ", "\n");
  file << mat.format(CSVFormat) << std::endl;
}

// Write results to traj file
void Trajectory::writeToFile(const Robot &robot, const std::string &name,
                             const gtsam::Values &results) const {
  vector<string> jnames;
  for (auto &&joint : robot.joints()) {
    jnames.push_back(joint->name());
  }
  string jnames_str = boost::algorithm::join(jnames, ",");

  std::ofstream file(name);

  // angles, vels, accels, torques, time.
  file << jnames_str << "," << jnames_str << "," << jnames_str << ","
       << jnames_str << ",t\n";
  for (int p = 0; p < numPhases(); p++) {
    writePhaseToFile(robot, file, results, p);
  }
  // Write the last 4 phases to disk n times
  for (int i = 0; i < 10; i++) {
    for (int p = 4; p < numPhases(); p++) {
      writePhaseToFile(robot, file, results, p);
    }
  }
}
}  // namespace gtdynamics
