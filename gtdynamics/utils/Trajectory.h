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
#include "gtdynamics/utils/WalkCycle.h"

#include <string>
#include <algorithm>
#include <boost/algorithm/string/join.hpp>

using gtdynamics::ContactPoints, gtdynamics::ContactPoint,
    std::vector, std::string, gtsam::Rot3, gtsam::Pose3;

namespace gtdynamics
{
  /**
  * @class Trajectory class implements functions derived 
  * from phase-related calculations.
  */
  class Trajectory
  {
  protected:
    int repeat_;                   ///< Number of repetitions of walk cycle
    WalkCycle walk_cycle_;         ///< Walk Cycle 

    static bool comparator(ContactPoint i,ContactPoint j){
       return (i.name < j.name); 
    }

    /// Gets the intersection between two string vectors of contact points
    const ContactPoints getIntersection(ContactPoints CPs_1, ContactPoints CPs_2) const
    {
      std::sort(CPs_1.begin(), CPs_1.end(), comparator);
      std::sort(CPs_2.begin(), CPs_2.end(), comparator);
      ContactPoints intersection;
      ContactPoints::iterator it;
      bool found = false;
      for(auto &&cp : CPs_1){
        it = std::find_if(CPs_2.begin(), CPs_2.end(), 
                            [cp](const ContactPoint& cp_){
                                    return cp_.name==cp.name;});
        if(it != CPs_2.end()){
            intersection.push_back(it[0]); 
            found = true;
        }
        else if(found) break;
      }
      return intersection;
    }

  public:
    /// Default Constructor
    Trajectory(const WalkCycle &walk_cycle, const int &repeat)
        : repeat_(repeat),
          walk_cycle_(walk_cycle){}

    /** @fn Returns a vector of ContactPoints objects for all phases after 
       * applying repetition on the walk cycle.
       * @return Phase CPs.
       */
    const vector<ContactPoints> phaseCPs() const
    {
      vector<ContactPoints> phase_cps;
      vector<Phase> phases = walk_cycle_.phases();
      for(int i = 0; i < repeat_; i++){
        for (auto &&phase : phases){
          phase_cps.push_back(phase.getAllContactPoints());
        }
      }
      return phase_cps;
    }

    /** @fn Returns a vector of ContactPoints objects for all transitions between phases after 
       * applying repetition on the original sequence.
       * @return Transition CPs.
       */
    const vector<ContactPoints> transitionCPs() const
    {
      vector<ContactPoints> trans_cps_orig;

      auto phases = walk_cycle_.phases();
      ContactPoints phase_1_cps;
      ContactPoints phase_2_cps;
      for(int p = 0; p < walk_cycle_.numPhases(); p++){
        phase_1_cps = phases[p].getAllContactPoints();
        if(p == walk_cycle_.numPhases() - 1)
          phase_2_cps = phases[0].getAllContactPoints();
        else
          phase_2_cps = phases[p+1].getAllContactPoints();
        ContactPoints intersection = getIntersection(phase_1_cps, phase_2_cps);
        trans_cps_orig.push_back(intersection);
      }

      vector<ContactPoints> trans_cps(trans_cps_orig) ;
      for(int i = 0; i < repeat_ - 1; i++)
        trans_cps.insert(trans_cps.end(), trans_cps_orig.begin(), trans_cps_orig.end());
      trans_cps.pop_back();

      return trans_cps;
    }

    /** @fn Returns a vector of phase durations for all phases after 
       * applying repetition on the walk cycle.
       * @return Phase durations.
       */
    const vector<int> phaseDurations() const
    {
      vector<int> phase_durations;
      vector<Phase> phases = walk_cycle_.phases();
      for(int i = 0; i < repeat_; i++){
        for (auto &&phase : phases)
          phase_durations.push_back(phase.numTimeSteps());
      }
      return phase_durations;
    }

    /** @fn Returns a vector of robot models for all phases after 
       * applying repetition on the walk cycle.
       * @return Robot models.
       */
    const vector<Robot> phaseRobotModels() const
    {
      vector<Robot> robots;
      vector<Phase> phases = walk_cycle_.phases();
      for(int i = 0; i < repeat_; i++){
        for (auto &&phase : phases)
          robots.push_back(phase.getRobotConfiguration());
      }
      return robots;
    }

    /** @fn Returns the number of phases.
       * @return Number of phases.
       */
    const int numPhases() const { return walk_cycle_.numPhases()*repeat_; }

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
      vector<Robot> phase_robots = phaseRobotModels();
      vector<int> final_timesteps = finalTimeSteps();
      for (int p = 1; p < numPhases(); p++)
      {
        transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
            phase_robots[p], final_timesteps[p-1], gravity,
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
      vector<Robot> phase_robots = phaseRobotModels();
      vector<int> final_timesteps = finalTimeSteps();
      for (int p = 1; p < numPhases(); p++)
      {
        transition_graph_init.push_back(ZeroValues(
            phase_robots[p],
            final_timesteps[p-1], gaussian_noise, trans_cps[p - 1]));
      }
      return transition_graph_init;
    }

    /** @fn Returns a vector of final time step for every phase.
     * @return Vector of final time steps.
     */
    const vector<int> finalTimeSteps() const
    {
      int final_timestep = 0;
      vector<int> final_timesteps;
      auto phases = walk_cycle_.phases();
      for (int i = 0; i < numPhases(); i++)
      {
        int phase_timestep = phases[i % walk_cycle_.numPhases()].numTimeSteps();
        final_timestep += phase_timestep;
        final_timesteps.push_back(final_timestep);
      }
      return final_timesteps;
    }

    /** @fn Returns the start time step for a given phase.
       * @param[in]phase    Phase number.
       * @return Initial time step.
       */
    const int getStartTimeStep(const int &phase) const
    {
      vector<int> final_timesteps = finalTimeSteps();
      auto phases = walk_cycle_.phases();
      int t_p_i = final_timesteps[phase] - 
                    phases[phase % walk_cycle_.numPhases()].numTimeSteps();
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
      return finalTimeSteps()[phase];
    }

    const vector<string> getLinks() const{
      vector<string> link_list;
      for(auto &&elem : walk_cycle_.links())
        link_list.push_back(elem.first);
      return link_list;
    }

    /** @fn Returns the contact links for a given phase.
       * @param[in]phase    Phase number.
       * @return Vector of contact links.
       */
    const vector<string> getPhaseContactLinks(const int &phase) const
    {
      auto phases = walk_cycle_.phases();
      ContactPoints contact_points = phases[phase % walk_cycle_.numPhases()].getAllContactPoints();
      vector<string> contact_links;
      for(auto &&cp : contact_points) contact_links.push_back(cp.name);
      return contact_links;
    }

    /** @fn Returns the swing links for a given phase.
       * @param[in]phase    Phase number.
       * @return Vector of swing links.
       */
    const vector<string> getPhaseSwingLinks(const int &phase) const
    {
      vector<string> phase_swing_links;
      vector<string> contact_links = getPhaseContactLinks(phase);
      for (auto &&l : getLinks())
      {
        if (std::find(contact_links.begin(),
                      contact_links.end(), l) == contact_links.end())
          phase_swing_links.push_back(l);
      }
      return phase_swing_links;
    }

    /** @fn Returns the initial contact point goal for every contact link.
       * @return Map of Contact Link and its goal point.
       */
    const std::map<string, gtsam::Point3> initContactPointGoal() const
    {
      std::map<string, gtsam::Point3> prev_cp;
      std::map<string, ContactPoint> link_map = walk_cycle_.links();
      for (auto &&link : link_map){
        gtdynamics::LinkSharedPtr link_ptr = walk_cycle_.phases()[0].
                                        getRobotConfiguration().getLinkByName(link.second.name);
          prev_cp.insert(std::make_pair(link.second.name,
                                        (link_ptr->wTcom() * Pose3(Rot3(), link.second.contact_point)).translation()));
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
      gtdynamics::LinkSharedPtr link_ptr = walk_cycle_.phases()[0].
                                              getRobotConfiguration().getLinkByName(link);
      gtsam::Key pose_key = PoseKey(link_ptr->getID(), t);
      gtsam::Pose3 comTp = Pose3(Rot3(), walk_cycle_.links()[link].contact_point);
      return PointGoalFactor(pose_key, cost_model, comTp, goal_point);
    }

    /** @fn Writes the angles, vels, accels, torques and time values for a single phase to disk.
       * @param[in]traj_file    Trajectory File being written onto.
       * @param[in]results      Results of Optimization.
       * @param[in]phase        Phase number.
       */
    const void writePhaseToFile(std::ofstream &traj_file, const gtsam::Values &results, const int &phase) const
    {
      int t = getStartTimeStep(phase);
      auto phase_durations = phaseDurations();
      Robot robot = phaseRobotModels()[phase];
      for (int time_step = 0; time_step < phase_durations[phase]; time_step++)
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

    // void blahBlah() const
    // {
    //   //Get phase information
    //   vector<CPs> phase_cps = this->phaseCPs();
    //   vector<int> phase_durations = this->phaseDurations();
    //   vector<Robot> robots = this->phaseRobotModels();

    //   // Define noise to be added to initial values, desired timestep duration,
    //   // vector of link name strings, robot model for each phase, and
    //   // phase transition initial values.
    //   double gaussian_noise = 1e-5;
    //   double dt_des = 1. / 240;
    //   vector<Values> transition_graph_init = this->getInitTransitionValues(gaussian_noise);

    //   // Get final time step.
    //   int t_f = this->getEndTimeStep(this->numPhases() - 1); // Final timestep.

    //   // Collocation scheme.
    //   auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Euler;

    //   // Graphs for transition between phases + their initial values.
    //   vector<gtsam::NonlinearFactorGraph> transition_graphs = this->getTransitionGraphs(graph_builder, gravity, mu);

    //   // Construct the multi-phase trajectory factor graph.
    //   //TODO: Pass Trajectory here
    //   std::cout << "Creating dynamics graph" << std::endl;
    //   auto graph = graph_builder.multiPhaseTrajectoryFG(
    //       robots, phase_durations, transition_graphs, collocation, gravity, boost::none, phase_cps, mu);

    //   // Build the objective factors.
    //   gtsam::NonlinearFactorGraph objective_factors;
    //   auto base_link = spider.getLinkByName("body");

    //   std::map<string, gtdynamics::LinkSharedPtr> link_map;
    //   for (auto &&link : links)
    //     link_map.insert(std::make_pair(link, spider.getLinkByName(link)));

    //   // Previous contact point goal.
    //   std::map<string, Point3> prev_cp = this->initContactPointGoal();

    //   // Distance to move contact point per time step during swing.
    //   auto contact_offset = Point3(0, 0.02, 0);

    //   // Add contact point objectives to factor graph.
    //   for (int p = 0; p < this->numPhases(); p++)
    //   {
    //     // if(p <2) contact_offset /=2 ;
    //     // Phase start and end timesteps.
    //     int t_p_i = this->getStartTimeStep(p);
    //     int t_p_f = this->getEndTimeStep(p);

    //     // Obtain the contact links and swing links for this phase.
    //     vector<string> phase_contact_links = this->getPhaseContactLinks(p);
    //     vector<string> phase_swing_links = this->getPhaseSwingLinks(p);

    //     //Setting the contact point goals for one time 
    //     for (int t = t_p_i; t <= t_p_f; t++)
    //     {
    //       // Normalized phase progress.
    //       double t_normed = (double)(t - t_p_i) / (double)(t_p_f - t_p_i);

    //       for (auto &&pcl : phase_contact_links)
    //         objective_factors.add(this->pointGoalFactor(
    //             pcl, t, Isotropic::Sigma(3, 1e-7), //1e-7
    //             Point3(prev_cp[pcl].x(), prev_cp[pcl].y(), GROUND_HEIGHT - 0.05)));

    //       double h = GROUND_HEIGHT + std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

    //       for (auto &&psl : phase_swing_links)
    //         objective_factors.add(this->pointGoalFactor(
    //             psl, t, Isotropic::Sigma(3, 1e-7),
    //             Point3(prev_cp[psl].x(), prev_cp[psl].y(), h)));

    //       // Update the goal point for the swing links.
    //       for (auto &&psl : phase_swing_links)
    //         prev_cp[psl] = prev_cp[psl] + contact_offset;
    //     }
    //   }

    //   // Add base goal objectives to the factor graph.
    //   for (int t = 0; t <= t_f; t++)
    //   {
    //     objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
    //         PoseKey(base_link->getID(), t),
    //         gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.0, 0.5)), //0.5
    //         Isotropic::Sigma(6, 5e-5)));                             //6.2e-5 //5e-5
    //     objective_factors.add(gtsam::PriorFactor<Vector6>(
    //         TwistKey(base_link->getID(), t), Vector6::Zero(), Isotropic::Sigma(6, 5e-5)));
    //   }

    //   // Add link boundary conditions to FG.
    //   for (auto &&link : spider.links())
    //   {
    //     // Initial link pose, twists.
    //     objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
    //         PoseKey(link->getID(), 0), link->wTcom(), dynamics_model_6));
    //     objective_factors.add(gtsam::PriorFactor<Vector6>(
    //         TwistKey(link->getID(), 0), Vector6::Zero(), dynamics_model_6));

    //     // Final link twists, accelerations.
    //     objective_factors.add(gtsam::PriorFactor<Vector6>(
    //         TwistKey(link->getID(), t_f), Vector6::Zero(), objectives_model_6));
    //     objective_factors.add(gtsam::PriorFactor<Vector6>(
    //         TwistAccelKey(link->getID(), t_f), Vector6::Zero(),
    //         objectives_model_6));
    //   }

    //   // Add joint boundary conditions to FG.
    //   for (auto &&joint : spider.joints())
    //   {
    //     //Add priors to joint angles
    //     for (int t = 0; t <= t_f; t++)
    //     {
    //       if (joint->name().find("hip2") == 0)
    //         objective_factors.add(gtsam::PriorFactor<double>(JointAngleKey(joint->getID(), t), 2.5, dynamics_model_1_2));
    //     }
    //     objective_factors.add(gtsam::PriorFactor<double>(
    //         JointVelKey(joint->getID(), 0), 0.0, dynamics_model_1));
    //     objective_factors.add(gtsam::PriorFactor<double>(
    //         JointVelKey(joint->getID(), t_f), 0.0, objectives_model_1));
    //     objective_factors.add(gtsam::PriorFactor<double>(
    //         JointAccelKey(joint->getID(), t_f), 0.0, objectives_model_1));
    //   }

    //   // Add prior factor constraining all Phase keys to have duration of 1 / 240.
    //   for (int phase = 0; phase < this->numPhases(); phase++)
    //     objective_factors.add(gtsam::PriorFactor<double>(
    //         PhaseKey(phase), dt_des,
    //         gtsam::noiseModel::Isotropic::Sigma(1, 1e-30)));

    //   // Add min torque objectives.
    //   for (int t = 0; t <= t_f; t++)
    //   {
    //     for (auto &&joint : spider.joints())
    //       objective_factors.add(gtdynamics::MinTorqueFactor(
    //           TorqueKey(joint->getID(), t),
    //           gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
    //   }
    //   graph.add(objective_factors);

    //   //TODO: Pass Trajectory here
    //   // Initialize solution.
    //   gtsam::Values init_vals;
    //   init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
    //       robots, phase_durations, transition_graph_init, dt_des, gaussian_noise,
    //       phase_cps);

    //   // Optimize!
    //   gtsam::LevenbergMarquardtParams params;
    //   params.setVerbosityLM("SUMMARY");
    //   params.setlambdaInitial(1e0);
    //   params.setlambdaLowerBound(1e-7);
    //   params.setlambdaUpperBound(1e10);
    //   gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
    //   auto results = optimizer.optimize();

    //   //Write results to traj file
    //   vector<string> jnames;
    //   for (auto &&joint : spider.joints())
    //     jnames.push_back(joint->name());
    //   std::cout << jnames.size() << std::endl;
    //   string jnames_str = boost::algorithm::join(jnames, ",");
    //   std::ofstream traj_file;

    //   //Get current directory to save the generated traj.csv file
    //   char cwd[PATH_MAX];
    //   char *fgh = getcwd(cwd, PATH_MAX);
    //   string example_directory = strcat(cwd, "/..");

    //   traj_file.open(example_directory + "/forward_traj.csv");
    //   // angles, vels, accels, torques, time.
    //   traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
    //             << jnames_str << ",t"
    //             << "\n";
    //   for (int phase = 0; phase < this->numPhases(); phase++)
    //     this->writePhaseToFile(traj_file, results, phase);

    //   //Write the last 4 phases to disk n times
    //   for (int i = 0; i < 10; i++)
    //   {
    //     for (int phase = 4; phase < phase_durations.size(); phase++)
    //       this->writePhaseToFile(traj_file, results, phase);
    //   }
    //   traj_file.close();
    //   return 0;
    // }
  };
} //namespace gtdynamics