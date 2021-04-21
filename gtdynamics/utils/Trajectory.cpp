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
  vector<Robot> phase_robots = phaseRobotModels();
  vector<int> final_timesteps = finalTimeSteps();
  for (int p = 1; p < numPhases(); p++) {
    transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
        phase_robots[p], final_timesteps[p - 1], trans_cps[p - 1], mu));
  }
  return transition_graphs;
}

NonlinearFactorGraph Trajectory::multiPhaseFactorGraph(
    DynamicsGraph &graph_builder,
    const CollocationScheme collocation, double mu) const {
  // Graphs for transition between phases + their initial values.
  auto transition_graphs = getTransitionGraphs(graph_builder, mu);
  return graph_builder.multiPhaseTrajectoryFG(
      phaseRobotModels(), phaseDurations(), transition_graphs, collocation,
      phaseContactPoints(), mu);
}

vector<Values> Trajectory::transitionPhaseInitialValues(
    double gaussian_noise) const {
  vector<ContactPoints> trans_cps = transitionContactPoints();
  vector<Values> transition_graph_init;
  vector<Robot> phase_robots = phaseRobotModels();
  vector<int> final_timesteps = finalTimeSteps();
  for (int p = 1; p < numPhases(); p++) {
    transition_graph_init.push_back(
        ZeroValues(phase_robots[p], final_timesteps[p - 1], gaussian_noise,
                   trans_cps[p - 1]));
  }
  return transition_graph_init;
}

Values Trajectory::multiPhaseInitialValues(double gaussian_noise,
                                           double dt) const {
  vector<Values> transition_graph_init =
      transitionPhaseInitialValues(gaussian_noise);
  return MultiPhaseZeroValuesTrajectory(phaseRobotModels(), phaseDurations(),
                                        transition_graph_init, dt,
                                        gaussian_noise, phaseContactPoints());
}

NonlinearFactorGraph Trajectory::contactLinkObjectives(
    const SharedNoiseModel &cost_model, const double ground_height) const {
  NonlinearFactorGraph factors;

  // Previous contact point goal.
  map<string, Point3> cp_goals = initContactPointGoal();

  // Distance to move contact point per time step during swing.
  auto contact_offset = Point3(0, 0.02, 0);

  // Add contact point objectives to factor graph.
  for (int p = 0; p < numPhases(); p++) {
    // if(p <2) contact_offset /=2 ;
    // Phase start and end timesteps.
    int k_start = getStartTimeStep(p);
    int k_end = getEndTimeStep(p);

    // Obtain the contact links and swing links for this phase.
    auto phase_contact_links = getPhaseContactLinks(p);
    auto phase_swing_links = getPhaseSwingLinks(p);

    for (int k = k_start; k <= k_end; k++) {
      for (auto &&kv : phase_contact_links) {
        auto contact_name = kv.first;
        Point3 goal_point(cp_goals[contact_name].x(),
                          cp_goals[contact_name].y(), ground_height - 0.05);
        factors.add(pointGoalFactor(contact_name, k, cost_model, goal_point));
      }

      // Normalized phase progress.
      double t_normed = (double)(k - k_start) / (double)(k_end - k_start);

      // Swing trajectory height over time.
      // TODO(frank): Alejandro should document this.
      double h = ground_height + pow(t_normed, 1.1) * pow(1 - t_normed, 0.7);

      for (auto &&psl : phase_swing_links) {
        Point3 goal_point(cp_goals[psl].x(), cp_goals[psl].y(), h);
        factors.add(pointGoalFactor(psl, k, cost_model, goal_point));

        // Update the goal point for the swing links.
        cp_goals[psl] = cp_goals[psl] + contact_offset;
      }
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
