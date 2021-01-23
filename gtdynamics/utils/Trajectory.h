/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Trajectory.h
 * @brief Utility methods for generating Trajectory phases.
 * @author: Disha Das, Tarushree Gandhi
 */

#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/utils/initialize_solution_utils.h>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/WalkCycle.h"

namespace gtdynamics {

/**
 * @class Trajectory class implements functions derived
 * from phase-related calculations.
 */
class Trajectory {
 protected:
  int repeat_;            ///< Number of repetitions of walk cycle
  WalkCycle walk_cycle_;  ///< Walk Cycle

  /// Gets the intersection between two ContactPoints objects
  const ContactPoints getIntersection(ContactPoints CPs_1,
                                      ContactPoints CPs_2) const {
    ContactPoints intersection;
    for (auto &&cp : CPs_1) {
      if (CPs_2.find(cp.first) != CPs_2.end()) {
        intersection.emplace(cp.first, cp.second);
      }
    }
    return intersection;
  }

 public:
  /// Default Constructor (for serialization)
  Trajectory(){};

  /**
   * Construct trajectory from WalkCycle and specified number of gait
   * repetitions.
   *
   * @param walk_cycle The Walk Cycle for the robot.
   * @param repeat The number of repetitions for each phase of the gait.
   */
  Trajectory(const WalkCycle &walk_cycle, const int &repeat)
      : repeat_(repeat), walk_cycle_(walk_cycle) {}

  /**
   * @fn Returns a vector of ContactPoints objects for all phases after
   * applying repetition on the walk cycle.
   * @return Phase CPs.
   */
  const std::vector<ContactPoints> phaseContactPoints() const {
    std::vector<ContactPoints> phase_cps;
    std::vector<Phase> phases = walk_cycle_.phases();
    for (int i = 0; i < repeat_; i++) {
      for (auto &&phase : phases) {
        phase_cps.push_back(phase.getAllContactPoints());
      }
    }
    return phase_cps;
  }

  /**
   * @fn Returns a vector of ContactPoints objects for all transitions between
   * phases after applying repetition on the original sequence.
   * @return Transition CPs.
   */
  const std::vector<ContactPoints> transitionContactPoints() const {
    std::vector<ContactPoints> trans_cps_orig;

    auto phases = walk_cycle_.phases();
    ContactPoints phase_1_cps;
    ContactPoints phase_2_cps;

    for (int p = 0; p < walk_cycle_.numPhases(); p++) {
      phase_1_cps = phases[p].getAllContactPoints();
      if (p == walk_cycle_.numPhases() - 1) {
        phase_2_cps = phases[0].getAllContactPoints();
      } else {
        phase_2_cps = phases[p + 1].getAllContactPoints();
      }

      ContactPoints intersection = getIntersection(phase_1_cps, phase_2_cps);
      trans_cps_orig.push_back(intersection);
    }

    // Copy the original transition contact point sequence
    // `repeat_` number of times.
    std::vector<ContactPoints> trans_cps(trans_cps_orig);
    for (int i = 0; i < repeat_ - 1; i++) {
      trans_cps.insert(trans_cps.end(), trans_cps_orig.begin(),
                       trans_cps_orig.end());
    }
    trans_cps.pop_back();

    return trans_cps;
  }

  /**
   * @fn Returns a vector of phase durations for all phases after
   * applying repetition on the walk cycle.
   * @return Phase durations.
   */
  const std::vector<int> phaseDurations() const {
    std::vector<int> phase_durations;
    std::vector<Phase> phases = walk_cycle_.phases();
    for (int i = 0; i < repeat_; i++) {
      for (auto &&phase : phases)
        phase_durations.push_back(phase.numTimeSteps());
    }
    return phase_durations;
  }

  /**
   * @fn Returns a vector of robot models for all phases after
   * applying repetition on the walk cycle.
   * @return Robot models.
   */
  const std::vector<Robot> phaseRobotModels() const {
    std::vector<Robot> robots;
    std::vector<Phase> phases = walk_cycle_.phases();
    for (int i = 0; i < repeat_; i++) {
      for (auto &&phase : phases)
        robots.push_back(phase.getRobotConfiguration());
    }
    return robots;
  }

  /**
   * @fn Returns the number of phases.
   * @return Number of phases.
   */
  const int numPhases() const { return walk_cycle_.numPhases() * repeat_; }

  /**
   * @fn Builds vector of Transition Graphs.
   * @param[in]graph_builder    Dynamics Graph
   * @param[in]gravity          Gravity
   * @param[in]mu               Coefficient of static friction
   * @return Vector of Transition Graphs
   */
  const std::vector<gtsam::NonlinearFactorGraph> getTransitionGraphs(
      gtdynamics::DynamicsGraph &graph_builder, const gtsam::Vector3 &gravity,
      const double &mu) const {
    std::vector<gtsam::NonlinearFactorGraph> transition_graphs;
    std::vector<ContactPoints> trans_cps = transitionContactPoints();
    std::vector<Robot> phase_robots = phaseRobotModels();
    std::vector<int> final_timesteps = finalTimeSteps();
    for (int p = 1; p < numPhases(); p++) {
      transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
          phase_robots[p], final_timesteps[p - 1], gravity, boost::none,
          trans_cps[p - 1], mu));
    }
    return transition_graphs;
  }

  /**
   * @fn Returns Initial values for transition graphs.
   * @param[in]gaussian_noise    Gaussian noise to add to initial values
   * @return Initial values for transition graphs
   */
  const std::vector<gtsam::Values> getInitTransitionValues(
      const double &gaussian_noise) const {
    std::vector<ContactPoints> trans_cps = transitionContactPoints();
    std::vector<gtsam::Values> transition_graph_init;
    std::vector<Robot> phase_robots = phaseRobotModels();
    std::vector<int> final_timesteps = finalTimeSteps();
    for (int p = 1; p < numPhases(); p++) {
      transition_graph_init.push_back(
          ZeroValues(phase_robots[p], final_timesteps[p - 1], gaussian_noise,
                     trans_cps[p - 1]));
    }
    return transition_graph_init;
  }

  /**
   * @fn Returns a vector of final time step for every phase.
   * @return Vector of final time steps.
   */
  const std::vector<int> finalTimeSteps() const {
    int final_timestep = 0;
    std::vector<int> final_timesteps;
    auto phases = walk_cycle_.phases();
    for (int i = 0; i < numPhases(); i++) {
      int phase_timestep = phases[i % walk_cycle_.numPhases()].numTimeSteps();
      final_timestep += phase_timestep;
      final_timesteps.push_back(final_timestep);
    }
    return final_timesteps;
  }

  /**
   * @fn Returns the start time step for a given phase.
   * @param[in]phase    Phase number.
   * @return Initial time step.
   */
  const int getStartTimeStep(const int &phase) const {
    std::vector<int> final_timesteps = finalTimeSteps();
    auto phases = walk_cycle_.phases();
    int t_p_i = final_timesteps[phase] -
                phases[phase % walk_cycle_.numPhases()].numTimeSteps();
    if (phase != 0) t_p_i += 1;
    return t_p_i;
  }

  /**
   * @fn Returns the end time step for a given phase.
   * @param[in]phase    Phase number.
   * @return Final time step.
   */
  const int getEndTimeStep(const int &phase) const {
    return finalTimeSteps()[phase];
  }

  const std::vector<std::string> getLinks() const {
    std::vector<std::string> link_list;
    for (auto &&elem : walk_cycle_.allContactPoints())
      link_list.push_back(elem.first);
    return link_list;
  }

  /**
   * @fn Returns the contact links for a given phase.
   * @param[in]phase    Phase number.
   * @return Vector of contact links.
   */
  const std::vector<std::string> getPhaseContactLinks(const int &phase) const {
    auto phases = walk_cycle_.phases();
    ContactPoints contact_points =
        phases[phase % walk_cycle_.numPhases()].getAllContactPoints();
    std::vector<std::string> contact_links;
    for (auto &&cp : contact_points) contact_links.push_back(cp.first);
    return contact_links;
  }

  /**
   * @fn Returns the swing links for a given phase.
   * @param[in]phase    Phase number.
   * @return Vector of swing links.
   */
  const std::vector<std::string> getPhaseSwingLinks(const int &phase) const {
    std::vector<std::string> phase_swing_links;
    std::vector<std::string> contact_links = getPhaseContactLinks(phase);
    for (auto &&l : getLinks()) {
      if (std::find(contact_links.begin(), contact_links.end(), l) ==
          contact_links.end())
        phase_swing_links.push_back(l);
    }
    return phase_swing_links;
  }

  /**
   * @fn Returns the initial contact point goal for every contact link.
   * @return Map of Contact Link and its goal point.
   */
  const std::map<std::string, gtsam::Point3> initContactPointGoal() const {
    std::map<std::string, gtsam::Point3> prev_cp;
    ContactPoints wc_cps = walk_cycle_.allContactPoints();
    for (auto &&cp : wc_cps) {
      gtdynamics::LinkSharedPtr link_ptr =
          walk_cycle_.phases()[0].getRobotConfiguration().getLinkByName(
              cp.first);
      prev_cp.insert(std::make_pair(
          cp.first, (link_ptr->wTcom() *
                     gtsam::Pose3(gtsam::Rot3(), cp.second.point))
                        .translation()));
    }
    return prev_cp;
  }

  /**
   * @fn Generates a PointGoalFactor object
   * @param[in] link              concerned link
   * @param[in] t                 time t
   * @param[in] cost_model        Noise model
   * @param[in] goal_point        target goal point
   */
  const PointGoalFactor pointGoalFactor(
      const std::string &link, const int &t,
      const gtsam::noiseModel::Base::shared_ptr &cost_model,
      const gtsam::Point3 &goal_point) const {
    gtdynamics::LinkSharedPtr link_ptr =
        walk_cycle_.phases()[0].getRobotConfiguration().getLinkByName(link);
    gtsam::Key pose_key = PoseKey(link_ptr->getID(), t);
    gtsam::Pose3 comTp = gtsam::Pose3(
        gtsam::Rot3(), walk_cycle_.allContactPoints()[link].point);
    return PointGoalFactor(pose_key, cost_model, comTp, goal_point);
  }

  /**
   * @fn Writes the angles, vels, accels, torques and time values for a single
   * phase to disk.
   * @param[in] traj_file    Trajectory File being written onto.
   * @param[in] results      Results of Optimization.
   * @param[in] phase        Phase number.
   */
  const void writePhaseToFile(std::ofstream &traj_file,
                              const gtsam::Values &results,
                              const int &phase) const {
    int t = getStartTimeStep(phase);
    auto phase_durations = phaseDurations();
    Robot robot = phaseRobotModels()[phase];
    for (int time_step = 0; time_step < phase_durations[phase]; time_step++) {
      std::vector<std::string> vals;
      for (auto &&joint : robot.joints())
        vals.push_back(
            std::to_string(results.atDouble(JointAngleKey(joint->getID(), t))));
      for (auto &&joint : robot.joints())
        vals.push_back(
            std::to_string(results.atDouble(JointVelKey(joint->getID(), t))));
      for (auto &&joint : robot.joints())
        vals.push_back(
            std::to_string(results.atDouble(JointAccelKey(joint->getID(), t))));
      for (auto &&joint : robot.joints())
        vals.push_back(
            std::to_string(results.atDouble(TorqueKey(joint->getID(), t))));
      vals.push_back(std::to_string(results.atDouble(PhaseKey(phase))));
      t++;
      std::string vals_str = boost::algorithm::join(vals, ",");
      traj_file << vals_str << "\n";
    }
  }
};
}  // namespace gtdynamics
