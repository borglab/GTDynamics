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


void Trajectory::addPhaseContactPoints(const Phase &phase) {
  // Add unique PointOnLink objects to contact_points_
  for (auto &&kv : phase.footContactConstraintSpec()->contactPoints()) {
    int link_count =
        std::count_if(contact_points_.begin(), contact_points_.end(),
                      [&](const PointOnLink &contact_point) {
                        return contact_point.point == kv.point &&
                               contact_point.link == kv.link;
                      });
    if (link_count == 0) contact_points_.push_back(kv);
  }
}


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

ContactPointGoals Trajectory::initContactPointGoal(const Robot &robot,
                                                   double ground_height) const {
  ContactPointGoals cp_goals;
  const Point3 adjust(0, 0, -ground_height);

  // Go over all phases, and all contact points
  for (auto &&phase : phases_) {
    for (auto &&cp : phase.footContactConstraintSpec()->contactPoints()) {
      auto link_name = cp.link->name();
      // If no goal set yet, add it here
      if (cp_goals.count(link_name) == 0) {
        LinkSharedPtr link = robot.link(link_name);
        const Point3 foot_w = link->bMcom() * cp.point + adjust;
        cp_goals.emplace(link_name, foot_w);
      }
    }
  }

  return cp_goals;
}

NonlinearFactorGraph Trajectory::contactPointObjectives(
    const Robot &robot, const SharedNoiseModel &cost_model, const Point3 &step,
    ContactPointGoals &updated_cp_goals, double ground_height) const {
  NonlinearFactorGraph factors;

  // Initialize contact point goals.
  ContactPointGoals cp_goals = initContactPointGoal(robot, ground_height);

  size_t k_start = 0;

  for (const Phase &phase : phases_) {
    // Ask the Phase instance to anchor the stance legs
    factors.add(phase.footContactConstraintSpec()->
                contactPointObjectives(contact_points_, step, cost_model,
                                             k_start, cp_goals, phase.numTimeSteps()));
    // Update goals for swing legs
    cp_goals = phase.footContactConstraintSpec()->
                            updateContactPointGoals(contact_points_, step, cp_goals);

    // update the start time step for the next phase
    k_start += phase.numTimeSteps();
  }
  
  updated_cp_goals = cp_goals;
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
