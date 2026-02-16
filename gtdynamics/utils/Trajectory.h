/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Trajectory.h
 * @brief Utility methods for generating Trajectory phases.
 * @author: Disha Das, Tarushree Gandhi, Varun Agrawal, Frank Dellaert
 */

#pragma once

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/kinematics/PointGoalFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/Phase.h>
#include <gtdynamics/utils/WalkCycle.h>
#include <gtdynamics/utils/Initializer.h>

namespace gtdynamics {

/**
 * @class Trajectory class implements functions derived
 * from phase-related calculations.
 */
class Trajectory {
 protected:
  std::vector<Phase> phases_;  ///< All phases in the trajectory

 public:
  /// Default Constructor (for serialization)
  Trajectory() {}

  /**
   * Construct trajectory from WalkCycle and specified number of gait
   * repetitions. phases from walk_cycle will be added to Trajectory phases_
   * for repeat times. contact points from all phases then will be added to
   * contact_points_.
   *
   * @param walk_cycle  The Walk Cycle for the robot.
   * @param repeat      The number of repetitions for each phase of the gait.
   */
  Trajectory(const WalkCycle &walk_cycle, size_t repeat) {
    // Get phases of walk_cycle.
    auto phases_i = walk_cycle.phases();
    // Loop over `repeat` walk cycles W_i
    for (size_t i = 0; i < repeat; i++) {
      // Append phases_i of walk_cycle to phases_ vector member.
      phases_.insert(phases_.end(), phases_i.begin(), phases_i.end());
    }
  }

  /// Returns vector of phases in the trajectory
  const std::vector<Phase> &phases() const { return phases_; }

  /**
   * @fn Returns a vector of PointOnLinks objects for all phases after
   * applying repetition on the walk cycle.
   * This function returns all contact points from all phases
   * and may have repetitions, as opposed to contact_points_.
   * @return Phase CPs.
   */
  std::vector<PointOnLinks> phaseContactPoints() const {
    WalkCycle wc = WalkCycle(phases_);
    return wc.allPhasesContactPoints();
  }

  /**
   * @fn Returns a vector of PointOnLinks objects for all transitions between
   * phases after applying repetition on the original sequence.
   * @return Transition CPs.
   */
  std::vector<PointOnLinks> transitionContactPoints() const {
    WalkCycle wc = WalkCycle(phases_);
    return wc.transitionContactPoints();
  }

  /**
   * @fn Returns a vector of phase durations for all phases after
   * applying repetition on the walk cycle.
   * @return Phase durations.
   */
  std::vector<int> phaseDurations() const {
    std::vector<int> phase_durations;
    for (auto &&phase : phases_) {
      phase_durations.push_back(phase.numTimeSteps());
    }
    return phase_durations;
  }

  /**
   * @fn Returns the number of phases.
   * @return Number of phases.
   */
  size_t numPhases() const { return phases_.size(); }

  /**
   * @fn Builds vector of Transition Graphs.
   * @param[in] robot            Robot specification from URDF/SDF.
   * @param[in] graph_builder    Dynamics Graph
   * @param[in] mu               Coefficient of static friction
   * @param[in] ground_plane_height Contact ground-plane height in world frame.
   * @return Vector of Transition Graphs
   */
  std::vector<gtsam::NonlinearFactorGraph> getTransitionGraphs(
      const Robot &robot, const DynamicsGraph &graph_builder, double mu,
      double ground_plane_height = 0.0) const;

  /**
   * @fn Builds multi-phase factor graph.
   * @param[in] robot            Robot specification from URDF/SDF.
   * @param[in] graph_builder    GraphBuilder instance.
   * @param[in] collocation      Which collocation scheme to use.
   * @param[in] mu               Coefficient of static friction.
   * @param[in] ground_plane_height Contact ground-plane height in world frame.
   * @return Multi-phase factor graph
   */
  gtsam::NonlinearFactorGraph multiPhaseFactorGraph(
      const Robot &robot, const DynamicsGraph &graph_builder,
      const CollocationScheme collocation, double mu,
      double ground_plane_height = 0.0) const;

  /**
   * @fn Returns Initial values for transition graphs.
   * @param[in] robot             Robot specification from URDF/SDF.
   * @param[in] initializer       Initializer class to initialize with
   * @param[in] gaussian_noise    Gaussian noise to add to initial values
   * @return Initial values for transition graphs
   */
  std::vector<gtsam::Values> transitionPhaseInitialValues(
      const Robot &robot, const Initializer &initializer, double gaussian_noise) const;

  /**
   * @fn Returns Initial values for multi-phase factor graph.
   * @param[in] robot             Robot specification from URDF/SDF.
   * @param[in] initializer       Initializer class to initialize with
   * @param[in] gaussian_noise    Gaussian noise to add to initial values
   * @param[in] desired_dt        integration timestep
   * @return Initial values for multi-phase factor graph
   */
  gtsam::Values multiPhaseInitialValues(const Robot &robot,
                                        const Initializer &initializer, 
                                        double gaussian_noise, double dt) const;

  /**
   * @fn Returns a vector of final time step for every phase.
   * @return Vector of final time steps.
   */
  std::vector<int> finalTimeSteps() const {
    int final_timestep = 0;
    std::vector<int> final_timesteps;
    for (size_t i = 0; i < phases_.size(); i++) {
      int phase_timestep = phases_[i].numTimeSteps();
      final_timestep += phase_timestep;
      final_timesteps.push_back(final_timestep);
    }
    return final_timesteps;
  }

  /**
   * @fn Return phase for given phase number p.
   * @param[in] p    Phase number \in [0..repeat * numPhases()[.
   * @return Phase instance.
   */
  const Phase &phase(size_t p) const { return phases_[p]; }

  /**
   * @fn Returns the start time step for a given phase.
   * @param[in] p    Phase number.
   * @return Initial time step.
   */
  int getStartTimeStep(size_t p) const {
    std::vector<int> final_timesteps = finalTimeSteps();
    int k_start = final_timesteps[p] - phase(p).numTimeSteps();
    if (p != 0) k_start += 1;
    return k_start;
  }

  /**
   * @fn Returns the end time step for a given phase.
   * @param[in] p    Phase number.
   * @return Final time step.
   */
  int getEndTimeStep(size_t p) const { return finalTimeSteps()[p]; }

  /**
   * @fn Generates a PointGoalFactor object
   * @param[in] robot             Robot specification from URDF/SDF.
   * @param[in] link_name         concerned link
   * @param[in] cp                contact point on that link
   * @param[in] k                 time index k
   * @param[in] cost_model        Noise model
   * @param[in] goal_point        target goal point
   */
  gtsam::ExpressionFactor<gtsam::Vector3> pointGoalFactor(
      const Robot &robot, const std::string &link_name, const PointOnLink &cp,
      int k, const gtsam::SharedNoiseModel &cost_model,
      const gtsam::Point3 &goal_point) const {
    LinkSharedPtr link = robot.link(link_name);
    gtsam::Key pose_key = PoseKey(link->id(), k);
    return PointGoalFactor(pose_key, cost_model, cp.point, goal_point);
  }

  /**
   * @fn Create desired stance and swing trajectories for all contact links.
   * @fn This function creates a WalkCycle object from all phases in the
   * trajectory
   * @fn and uses WalkCycle functionality
   * @param[in] robot Robot specification from URDF/SDF.
   * @param[in] cost_model Noise model
   * @param[in] step The 3D vector the foot moves in a step.
   * @param[in] ground_height z-coordinate of ground plane in world frame.
   * @return All objective factors as a NonlinearFactorGraph
   */
  gtsam::NonlinearFactorGraph contactPointObjectives(
      const Robot &robot, const gtsam::SharedNoiseModel &cost_model,
      const gtsam::Point3 &step, double ground_height = {}) const;

  /**
   * @fn Add minimum torque objectives.
   * @param[in,out] graph nonlinear factor graph to add to.
   * @param[in] robot Robot specification from URDF/SDF.
   * @param[in] cost_model Noise model
   * @return All MinTorqueFactor factors as a NonlinearFactorGraph
   */
  void addMinimumTorqueFactors(gtsam::NonlinearFactorGraph *graph,
                               const Robot &robot,
                               const gtsam::SharedNoiseModel &cost_model) const;

  /**
   * @fn Create objective factors for slice 0 and slice K.
   *
   * Links at time step 0 are constrained to their wTcom poses and
   * zero twist, and zero twist and twist acceleration at last time step.
   * Joint angles velocities and accelerations are set to zero for all joints at
   * start *and* end.
   *
   * @param[in,out] graph nonlinear factor graph to add to.
   * @param[in] robot Robot specification from URDF/SDF.
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
    for (size_t phase = 0; phase < numPhases(); phase++)
      graph->addPrior<double>(PhaseKey(phase), desired_dt, model);
  }

  /**
   * @fn Writes the angles, vels, accels, torques and time values for a single
   * phase to disk.
   * @param[in] robot        Robot specification from URDF/SDF.
   * @param[in] traj_file    Trajectory File being written onto.
   * @param[in] results      Results of Optimization.
   * @param[in] phase        Phase number.
   */
  void writePhaseToFile(const Robot &robot, std::ofstream &traj_file,
                        const gtsam::Values &results, int phase) const;

  /**
   * @fn Writes the angles, vels, accels, torques and time values to disk.
   * @param[in] robot     Robot specification from URDF/SDF.
   * @param[in] name      Trajectory File name.
   * @param[in] results   Results of Optimization.
   */
  void writeToFile(const Robot &robot, const std::string &name,
                   const gtsam::Values &results) const;
};
}  // namespace gtdynamics
