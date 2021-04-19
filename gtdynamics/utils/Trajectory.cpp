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

#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/geometry/Point3.h>

#include <map>
#include <string>
#include <vector>

using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::noiseModel::Base;
using std::map;
using std::string;
using std::vector;

namespace gtdynamics {

const NonlinearFactorGraph Trajectory::contactLinkObjectives(
    const Base::shared_ptr &cost_model, const double ground_height) const {
  NonlinearFactorGraph factors;

  // Previous contact point goal.
  map<string, Point3> prev_cp = initContactPointGoal();

  // Distance to move contact point per time step during swing.
  auto contact_offset = Point3(0, 0.02, 0);

  // Add contact point objectives to factor graph.
  for (int p = 0; p < numPhases(); p++) {
    // if(p <2) contact_offset /=2 ;
    // Phase start and end timesteps.
    int t_p_i = getStartTimeStep(p);
    int t_p_f = getEndTimeStep(p);

    // Obtain the contact links and swing links for this phase.
    vector<string> phase_contact_links = getPhaseContactLinks(p);
    vector<string> phase_swing_links = getPhaseSwingLinks(p);

    for (int t = t_p_i; t <= t_p_f; t++) {
      // Normalized phase progress.
      double t_normed = (double)(t - t_p_i) / (double)(t_p_f - t_p_i);

      for (auto &&pcl : phase_contact_links) {
        Point3 goal_point(prev_cp[pcl].x(), prev_cp[pcl].y(),
                          ground_height - 0.05);
        factors.add(pointGoalFactor(pcl, t, cost_model, goal_point));
      }

      // Swing trajectory height over time.
      // TODO(frank): Alejandro should document this.
      double h = ground_height + pow(t_normed, 1.1) * pow(1 - t_normed, 0.7);

      for (auto &&psl : phase_swing_links) {
        Point3 goal_point(prev_cp[psl].x(), prev_cp[psl].y(), h);
        factors.add(pointGoalFactor(psl, t, cost_model, goal_point));
      }

      // Update the goal point for the swing links.
      for (auto &&psl : phase_swing_links)
        prev_cp[psl] = prev_cp[psl] + contact_offset;
    }
  }
  return factors;
}

}  // namespace gtdynamics
