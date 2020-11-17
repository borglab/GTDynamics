/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Trajectory.h
 * @brief Utility methods for generating Trajectory phases.
 * @Author: Disha Das, Tarushree Gandhi
 */

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include <gtdynamics/utils/initialize_solution_utils.h>
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include <gtdynamics/factors/PointGoalFactor.h>

#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

using gtdynamics::ContactPoints, gtdynamics::ContactPoint,
    std::vector, std::string, gtsam::Rot3, gtsam::Pose3;

namespace gtdynamics
{

  struct Phase
  {
    int duration;
    int final_timestep;
    Stance stance;
  };

  /**
 * @class Trajectory class implements functions derived 
 * from phase-related calculations.
 */
  class Trajectory
  {

  protected:
    int repeat_ = 1;                ///< Number of repetitions of walk cycle
    int num_phases_ = 0;            ///< Number of phases in trajectory
    vector<Phase> phases_;          ///< Vector of Phase structs
    TrajectoryParams traj_params_;  ///<TrajectoryParams object

    /** Gets the intersection between two string vectors of contact points */
    const vector<string> getIntersection(vector<string> CPs_1, vector<string> CPs_2) const
    {
      std::sort(CPs_1.begin(), CPs_1.end());
      std::sort(CPs_2.begin(), CPs_2.end());
      std::vector<string> intersection;
      std::set_intersection(CPs_1.begin(), CPs_1.end(),
                            CPs_2.begin(), CPs_2.end(),
                            std::back_inserter(intersection));
      return intersection;
    }

    /** Returns the ContactPoints object from vector of string of CP names */
    const ContactPoints getContactPoints(const vector<std::string> &CPs) const
    {
      ContactPoints CP_vec;
      for (auto &&CP : CPs)
        CP_vec.push_back(traj_params_.getContactPointMap()[CP]);
      return CP_vec;
    }

  public:
    /** Default Constructor */
    Trajectory(const TrajectoryParams &traj_params, const int &repeat)
        : repeat_(repeat),
          traj_params_(traj_params)
    {
      int walk_cycle_size = traj_params_.getWalkCycle().size();
      vector<int> durations = traj_params_.getDurations();
      num_phases_ = repeat_ * walk_cycle_size;
      int final_timestep = 0;
      for (int i = 0; i < num_phases_; i++)
      {
        string current_stance = traj_params_.getWalkCycle()[i % walk_cycle_size];
        int current_duration = traj_params_.getDurations()[i % walk_cycle_size];
        final_timestep += current_duration;
        phases_.push_back(Phase{current_duration, final_timestep, traj_params_.getStanceMap()[current_stance]});
      }
    }

    /** @fn Returns a vector of ContactPoints objects for all phases after 
     * applying repetition on the original sequence.
     * @return Phase CPs.
     */
    const vector<ContactPoints> phaseCPs() const
    {
      vector<ContactPoints> phase_cps;
      for (auto &&phase : phases_)
      {
        vector<std::string> CPs = phase.stance.contact_points;
        phase_cps.push_back(getContactPoints(CPs));
      }
      return phase_cps;
    }

    /** @fn Returns a vector of ContactPoints objects for all transitions between phases after 
     * applying repetition on the original sequence.
     * @return Transition CPs.
     */
    const vector<ContactPoints> transitionCPs() const
    {
      vector<ContactPoints> trans_cps;
      for (int p = 0; p < num_phases_ - 1; p++)
      {
        vector<string> CPs_1 = phases_[p].stance.contact_points;
        vector<string> CPs_2 = phases_[p + 1].stance.contact_points;
        vector<string> intersection = getIntersection(CPs_1, CPs_2);
        trans_cps.push_back(getContactPoints(intersection));
      }
      return trans_cps;
    }

    /** @fn Returns a vector of phase durations for all phases after 
     * applying repetition on the walk cycle.
     * @return Phase durations.
     */
    const vector<int> phaseDurations() const
    {
      vector<int> phase_durations;
      for (auto &&phase : phases_)
        phase_durations.push_back(phase.duration);
      return phase_durations;
    }

    /** @fn Returns a vector of robot models for all phases after 
     * applying repetition on the walk cycle.
     * @return Robot models.
     */
    const vector<Robot> phaseRobotModels() const
    {
      vector<Robot> robots;
      for (auto &&phase : phases_)
        robots.push_back(phase.stance.robot_configuration);
      return robots;
    }

    /** @fn Returns the number of phases.
     * @return Number of phases.
     */
    const int numPhases() const { return num_phases_; }

    /** @fn Builds vector of Transition Graphs.
     * @param[in]graph_builder    Dynamics Graph
     * @param[in]gravity          Gravity
     * @param[in]mu               Coefficient of static friction
     * @return Vector of Transition Graphs
     */
    const vector<gtsam::NonlinearFactorGraph> getTransitionGraphs(gtdynamics::DynamicsGraph &graph_builder,
                                                            const gtsam::Vector3 &gravity, const double &mu) const
    {
      vector<gtsam::NonlinearFactorGraph> transition_graphs;
      vector<ContactPoints> trans_cps = transitionCPs();
      for (int p = 1; p < num_phases_; p++)
      {
        transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
            phases_[p].stance.robot_configuration, phases_[p - 1].final_timestep, gravity,
            boost::none, trans_cps[p - 1], mu));
      }
      return transition_graphs;
    }

    /** @fn Returns Initial values for transition graphs.
     * @param[in]gaussian_noise    Gaussian noise to add to initial values
     * @return Initial values for transition graphs
     */
    const vector<gtsam::Values> getInitTransitionValues(const double &gaussian_noise) const
    {
      vector<ContactPoints> trans_cps = transitionCPs();
      vector<gtsam::Values> transition_graph_init;
      for (int p = 1; p < num_phases_; p++)
      {
        transition_graph_init.push_back(ZeroValues(
            phases_[p].stance.robot_configuration,
            phases_[p - 1].final_timestep, gaussian_noise, trans_cps[p - 1]));
      }
      return transition_graph_init;
    }

    /** @fn Returns the start time step for a given phase.
     * @param[in]phase    Phase number.
     * @return Initial time step.
     */
    const int getStartTimeStep(const int &phase) const
    {
      int t_p_i = phases_[phase].final_timestep - phases_[phase].duration;
      if (phase != 0)
        t_p_i += 1;
      return t_p_i;
    }

    /** @fn Returns the end time step for a given phase.
     * @param[in]phase    Phase number.
     * @return Final time step.
     */
    const int getEndTimeStep(const int &phase) const
    {
      return phases_[phase].final_timestep;
    }

    /** @fn Returns the contact links for a given phase.
     * @param[in]phase    Phase number.
     * @return Vector of contact links.
     */
    const vector<string> getPhaseContactLinks(const int &phase) const
    {
      return phases_[phase].stance.contact_points;
    }

    /** @fn Returns the swing links for a given phase.
     * @param[in]phase    Phase number.
     * @return Vector of swing links.
     */
    const vector<string> getPhaseSwingLinks(const int &phase) const
    {
      vector<string> phase_swing_links;
      vector<string> contact_links = phases_[phase].stance.contact_points;
      for (auto &&l : traj_params_.getLinks())
      {
        if (std::find(contact_links.begin(),
                      contact_links.end(), l) == contact_links.end())
          phase_swing_links.push_back(l);
      }
      return phase_swing_links;
    }

    const std::map<string, gtsam::Point3> initContactPointGoal() const
    {
      std::map<string, gtsam::Point3> prev_cp;
      for (auto &&link : traj_params_.getLinks())
      {
        gtdynamics::ContactPoint CP = traj_params_.getContactPointMap()[link];
        gtdynamics::LinkSharedPtr link_ptr = phases_[0].stance.robot_configuration.getLinkByName(link);
        prev_cp.insert(std::make_pair(link,
                                      (link_ptr->wTcom() * Pose3(Rot3(), CP.contact_point)).translation()));
      }
      return prev_cp;
    }

    /** @fn Generates a PointGoalFactor object
     * @param[in]link              concerned link
     * @param[in]t                 time t
     * @param[in]cost_model        Noise model
     * @param[in]goal_point        target goal point
     */
    const PointGoalFactor pointGoalFactor(const string &link, const int &t,
                                                const gtsam::noiseModel::Base::shared_ptr &cost_model,
                                                const gtsam::Point3 &goal_point) const
    {
      gtdynamics::LinkSharedPtr link_ptr = phases_[0].stance.robot_configuration.getLinkByName(link);
      gtsam::Key pose_key = PoseKey(link_ptr->getID(), t);
      gtsam::Pose3 comTp = Pose3(Rot3(), traj_params_.getContactPointMap()[link].contact_point);
      return PointGoalFactor(pose_key, cost_model, comTp ,goal_point);
    }

    /** @fn Writes the angles, vels, accels, torques and time values for a single phase to disk.
     * @param[in]traj_file    Trajectory File being written onto.
     * @param[in]results      Results of Optimization.
     * @param[in]phase        Phase number.
     */
    const void writePhaseToFile(std::ofstream &traj_file, const gtsam::Values &results, const int &phase) const
    {
      int t = getStartTimeStep(phase);
      Robot robot = phases_[phase].stance.robot_configuration;
      // robot.removeLink(robot.getLinkByName("fix"));
      for (int time_step = 0; time_step < phases_[phase].duration; time_step++)
      {
        vector<string> vals;
        for (auto &&joint : robot.joints())
          vals.push_back(std::to_string(results.atDouble(JointAngleKey(joint->getID(), t))));
        for (auto &&joint : robot.joints())
          vals.push_back(std::to_string(results.atDouble(JointVelKey(joint->getID(), t))));
        for (auto &&joint : robot.joints())
          vals.push_back(std::to_string(results.atDouble(JointAccelKey(joint->getID(), t))));
        for (auto &&joint : robot.joints())
          vals.push_back(std::to_string(results.atDouble(TorqueKey(joint->getID(), t))));
        vals.push_back(std::to_string(results.atDouble(PhaseKey(phase))));
        t++;
        string vals_str = boost::algorithm::join(vals, ",");
        traj_file << vals_str << "\n";
      }
    }
  };
} //namespace gtdynamics