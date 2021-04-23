/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Trajectory.h
 * @brief Utility methods for generating Trajectory phases.
 * @author: Disha Das, Tarushree Gandhi, Varun Agrawal
 */

#pragma once

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/Phase.h>
#include <gtdynamics/utils/WalkCycle.h>
#include <gtdynamics/utils/initialize_solution_utils.h>

namespace gtdynamics {

/**
 * @class Trajectory class implements functions derived
 * from phase-related calculations.
 */
class Trajectory {
 protected:
  Robot robot_;           ///< Copy of the robot configuration
  int repeat_;            ///< Number of repetitions of walk cycle
  WalkCycle walk_cycle_;  ///< Walk Cycle

  /// Gets the intersection between two ContactPoints objects
  ContactPoints getIntersection(ContactPoints CPs_1,
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
  Trajectory(const Robot &robot, const WalkCycle &walk_cycle, int repeat)
      : robot_(robot), repeat_(repeat), walk_cycle_(walk_cycle) {}

  /**
   * @fn Returns a vector of ContactPoints objects for all phases after
   * applying repetition on the walk cycle.
   * @return Phase CPs.
   */
  std::vector<ContactPoints> phaseContactPoints() const {
    std::vector<ContactPoints> phase_cps;
    const auto &phases = walk_cycle_.phases();
    for (int i = 0; i < repeat_; i++) {
      for (auto &&phase : phases) {
        phase_cps.push_back(phase.contactPoints());
      }
    }
    return phase_cps;
  }

  /**
   * @fn Returns a vector of ContactPoints objects for all transitions between
   * phases after applying repetition on the original sequence.
   * @return Transition CPs.
   */
  std::vector<ContactPoints> transitionContactPoints() const {
    std::vector<ContactPoints> trans_cps_orig;

    auto phases = walk_cycle_.phases();
    ContactPoints phase_1_cps;
    ContactPoints phase_2_cps;

    for (int p = 0; p < walk_cycle_.numPhases(); p++) {
      phase_1_cps = phases[p].contactPoints();
      if (p == walk_cycle_.numPhases() - 1) {
        phase_2_cps = phases[0].contactPoints();
      } else {
        phase_2_cps = phases[p + 1].contactPoints();
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
  std::vector<int> phaseDurations() const {
    std::vector<int> phase_durations;
    const auto &phases = walk_cycle_.phases();
    for (int i = 0; i < repeat_; i++) {
      for (auto &&phase : phases)
        phase_durations.push_back(phase.numTimeSteps());
    }
    return phase_durations;
  }

  /**
   * @fn Returns the number of phases.
   * @return Number of phases.
   */
  int numPhases() const { return walk_cycle_.numPhases() * repeat_; }

  /**
   * @fn Builds vector of Transition Graphs.
   * @param[in]graph_builder    Dynamics Graph
   * @param[in]mu               Coefficient of static friction
   * @return Vector of Transition Graphs
   */
  std::vector<gtsam::NonlinearFactorGraph> getTransitionGraphs(
      DynamicsGraph &graph_builder, double mu) const;

  /**
   * @fn Builds multi-phase factor graph.
   * @param[in]graph_builder    Dynamics Graph
   * @param[in]mu               Coefficient of static friction
   * @return Multi-phase factor graph
   */
  gtsam::NonlinearFactorGraph multiPhaseFactorGraph(
      DynamicsGraph &graph_builder, const CollocationScheme collocation,
      double mu) const;

  /**
   * @fn Returns Initial values for transition graphs.
   * @param[in]gaussian_noise    Gaussian noise to add to initial values
   * @return Initial values for transition graphs
   */
  std::vector<gtsam::Values> transitionPhaseInitialValues(
      double gaussian_noise) const;

  /**
   * @fn Returns Initial values for multi-phase factor graph.
   * @param[in]gaussian_noise    Gaussian noise to add to initial values
   * @param[in]desired_dt        integration timestep
   * @return Initial values for multi-phase factor graph
   */
  gtsam::Values multiPhaseInitialValues(double gaussian_noise, double dt) const;

  /**
   * @fn Returns a vector of final time step for every phase.
   * @return Vector of final time steps.
   */
  std::vector<int> finalTimeSteps() const {
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
   * @fn Return phase for given phase number p.
   * @param[in]p    Phase number.
   * @return Phase instance.
   */
  const Phase &phase(int p) const {
    return walk_cycle_.phases().at(p % walk_cycle_.numPhases());
  }

  /**
   * @fn Returns the start time step for a given phase.
   * @param[in]p    Phase number.
   * @return Initial time step.
   */
  int getStartTimeStep(int p) const {
    std::vector<int> final_timesteps = finalTimeSteps();
    int k_start = final_timesteps[p] - phase(p).numTimeSteps();
    if (p != 0) k_start += 1;
    return k_start;
  }

  /**
   * @fn Returns the end time step for a given phase.
   * @param[in]p    Phase number.
   * @return Final time step.
   */
  int getEndTimeStep(int p) const { return finalTimeSteps()[p]; }

  /**
   * @fn Returns the contact links for a given phase.
   * @param[in]p    Phase number.
   * @return Vector of contact links.
   */
  ContactPoints getPhaseContactLinks(int p) const {
    return phase(p).contactPoints();
  }

  /**
   * @fn Returns the swing links for a given phase.
   * @param[in]p    Phase number.
   * @return Vector of swing links.
   */
  std::vector<std::string> getPhaseSwingLinks(int p) const {
    std::vector<std::string> phase_swing_links;
    auto contact_links = getPhaseContactLinks(p);
    for (auto &&kv : walk_cycle_.contactPoints()) {
      if (contact_links.count(kv.first) == 0)
        phase_swing_links.push_back(kv.first);
    }
    return phase_swing_links;
  }

  /**
   * @fn Generates a PointGoalFactor object
   * @param[in] link_name         concerned link
   * @param[in] k                 time index k
   * @param[in] cost_model        Noise model
   * @param[in] goal_point        target goal point
   */
  PointGoalFactor pointGoalFactor(const std::string &link_name,
                                  const ContactPoint &cp, int k,
                                  const gtsam::SharedNoiseModel &cost_model,
                                  const gtsam::Point3 &goal_point) const {
    LinkSharedPtr link = robot_.link(link_name);
    gtsam::Key pose_key = internal::PoseKey(link->id(), k);
    return PointGoalFactor(pose_key, cost_model, cp.point, goal_point);
  }

  /**
   * @fn Create desired stance and swing trajectories for all contact links.
   * @return All objective factors as a NonlinearFactorGraph
   */
  gtsam::NonlinearFactorGraph contactLinkObjectives(
      const gtsam::SharedNoiseModel &cost_model,
      double ground_height = 0.0) const;

  /**
   * @fn Add minimum torque objectives.
   * @return All MinTorqueFactor factors as a NonlinearFactorGraph
   */
  void addMinimumTorqueFactors(gtsam::NonlinearFactorGraph *graph,
                               const Robot &robot,
                               const gtsam::SharedNoiseModel &cost_model) const;

  /**
   * @fn Create objective factors for slice 0 and slice K.
   *
   * Links at time step 0 are constrainted to their wTcom poses and
   * zero twist, and zero twist and twist acceleration at last time step.
   * Joint angles velocities and accelerations are set to zero for all joints at
   * start *and* end.
   *
   * @return All factors as a NonlinearFactorGraph
   */
  void addBoundaryConditions(
      gtsam::NonlinearFactorGraph *graph, const Robot &robot,
      const gtsam::SharedNoiseModel &pose_model,
      const gtsam::SharedNoiseModel &twist_model,
      const gtsam::SharedNoiseModel &twist_acceleration_model,
      const gtsam::SharedNoiseModel &joint_velocity_model,
      const gtsam::SharedNoiseModel &joint_acceleration_model) const;

  /**
   * @fn Add priors on all variable time steps.
   * @param[in, out] graph NonlinearFactorGraph to add to
   * @param[in] desired_dt desired time step
   * @param[in] sigma      standard deviation (default 0: constrained)
   */
  void addIntegrationTimeFactors(gtsam::NonlinearFactorGraph *graph,
                                 double desired_dt, double sigma = 0) const {
    auto model = gtsam::noiseModel::Isotropic::Sigma(1, sigma);
    for (int phase = 0; phase < numPhases(); phase++)
      graph->addPrior<double>(PhaseKey(phase), desired_dt, model);
  }

  /**
   * @fn Writes the angles, vels, accels, torques and time values for a single
   * phase to disk.
   * @param[in] traj_file    Trajectory File being written onto.
   * @param[in] results      Results of Optimization.
   * @param[in] phase        Phase number.
   */
  void writePhaseToFile(const Robot &robot, std::ofstream &traj_file,
                        const gtsam::Values &results, int phase) const {
    int k = getStartTimeStep(phase);
    auto phase_durations = phaseDurations();
    for (int time_step = 0; time_step < phase_durations[phase]; time_step++) {
      std::vector<std::string> vals;
      for (auto &&joint : robot.joints())
        vals.push_back(std::to_string(JointAngle(results, joint->id(), k)));
      for (auto &&joint : robot.joints())
        vals.push_back(std::to_string(JointVel(results, joint->id(), k)));
      for (auto &&joint : robot.joints())
        vals.push_back(std::to_string(JointAccel(results, joint->id(), k)));
      for (auto &&joint : robot.joints())
        vals.push_back(std::to_string(Torque(results, joint->id(), k)));
      vals.push_back(std::to_string(results.atDouble(PhaseKey(phase))));
      k++;
      std::string vals_str = boost::algorithm::join(vals, ",");
      traj_file << vals_str << "\n";
    }
  }
};
}  // namespace gtdynamics
