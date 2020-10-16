/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  phase_utils.h
 * @brief Utility methods for extracting information from Phases.
 * @Author: Disha Das
 * @Author: Tarushree Gandhi
 */

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"

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


namespace gtdynamics {

using gtdynamics::ContactPoints, gtdynamics::ContactPoint, 
      std::vector, std::string;

/**
 * @class Phase is a class that stores information related
 * to Contact Points, stances and sequence of stances and implements 
 * functions derived from phase-related calculations.
 */
class Phase {

  protected:
    //Map from CP name to CP object
    std::map<std::string, gtdynamics::ContactPoint > contact_point_map;

    //Map from stance name to constituent CP names
    std::map<std::string, vector<std::string> > stance_map;

    //List of stances
    std::vector<string> stances;

    //Number of repetitions of stance sequence
    int repeat; 

    //Time step for each phase
    int step_size;

    //Vector of constructed walk cycle time steps
    vector<int> phase_steps;

    //Vector of cumulative time steps
    vector<int> cum_phase_steps;

    //List of all links or CP names
    vector<string> links;

    //Repeats a sequence of stances
    vector<gtdynamics::ContactPoints> repeatSequence(vector<gtdynamics::ContactPoints> cps_, int repetition){
        vector<gtdynamics::ContactPoints> cps;
        for(int i = 0; i < repetition; i++){
            cps.insert(std::end(cps), std::begin(cps_), std::end(cps_));
        }
        if (repetition == 0)
          return cps_;
        else
          return cps;
    }
    
    //Gets the intersection between two string vectors of contact links
    vector<string> getIntersection(vector<string> CPs_1, vector<string> CPs_2){
        std::sort(CPs_1.begin(), CPs_1.end());
        std::sort(CPs_2.begin(), CPs_2.end());
        std::vector<string> intersection;
        std::set_intersection(CPs_1.begin(), CPs_1.end(),
                                CPs_2.begin(), CPs_2.end(),
                                std::back_inserter(intersection));
        return intersection;
    }

    //Returns the ContactPoints object from vector of string of CP names
    gtdynamics::ContactPoints getCPs(vector<std::string> CPs){
        gtdynamics::ContactPoints CP_vec;
        for (auto&& CP : CPs)
            CP_vec.push_back(contact_point_map[CP]);
        return CP_vec;
    }

  public:
    /** Default Constructor */
    Phase(){}

    /** @fn Maps a Contact Point name to its ContactPoint object.
     * @param[in] CP   Single ContactPoint object.
     */
    void addContactPoint(gtdynamics::ContactPoint CP){
        contact_point_map.insert(std::make_pair(CP.name, CP));
        links.push_back(CP.name);
    }

    /** @fn Maps a vector of ContactPoint names to their ContactPoint objects.
     * @param[in] CPs   Vector of ContactPoint objects.
     */
    void addContactPoints(gtdynamics::ContactPoints CPs){
        for (auto&& cp : CPs){
          contact_point_map.insert(std::make_pair(cp.name, cp));
          links.push_back(cp.name);
        }
    }

    /** @fn Maps a stance to a vector of constituent Contact Point names.
     * @param[in] name   Name of the stance.
     * @param[in] CPs    Constituent Contact points of the stance.
     */
    void addStance(std::string name, vector<string> CPs){
        stance_map.insert(std::make_pair(name, CPs));
    }

    /** @fn Stores a sequence of stances or phases.
     * @param[in] phases   Sequence of stances.
     */
    void addSequence(vector<string> phases){
        stances = phases;
    }

    /** @fn Returns a vector of ContactPoints objects for all phases after 
     * applying repetition on the original sequence.
     * @param[in]repetitions    Number of repetitions of stance sequence.
     * @return Phase CPs.
     */
    vector<gtdynamics::ContactPoints> getPhaseCPs(int repetitions=0){
      repeat = repetitions;
      vector<gtdynamics::ContactPoints> phase_cps_;

      for (auto&& stance : stances){
          vector<std::string> CPs = stance_map[stance];
          phase_cps_.push_back(getCPs(CPs));
      }
      vector<gtdynamics::ContactPoints> phase_cps = repeatSequence(phase_cps_, repeat);
      return phase_cps;
    }

    /** @fn Returns a vector of ContactPoints objects for all transitions between phases after 
     * applying repetition on the original sequence.
     * @return Transition CPs.
     */
    vector<gtdynamics::ContactPoints> getTransitionCPs(){
        vector<gtdynamics::ContactPoints> trans_cps_;

        for(int p = 0; p < stances.size(); p++ ){
            vector<string> stance_1 = stance_map[stances[p]];
            vector<string> stance_2;
            if(p == stances.size()-1)
              stance_2 = stance_map[stances[0]];
            else
              stance_2 = stance_map[stances[p + 1]];
            std::vector<string> intersection = getIntersection(stance_1, stance_2);
            trans_cps_.push_back(getCPs(intersection));
        }
        vector<gtdynamics::ContactPoints> trans_cps = repeatSequence(trans_cps_, repeat);
        trans_cps.erase(trans_cps.end());
        
        return trans_cps;
    }

    /** @fn Returns a vector of phase steps for all phases after 
     * applying repetition on the original sequence.
     * @param[in]step    Time step for each phase.
     * @return Phase steps.
     */
    vector<int> getPhaseSteps(int step){
        step_size = step;
        phase_steps.resize(stances.size()*repeat);
        std::fill (phase_steps.begin(), phase_steps.end(), step_size);
        return phase_steps;
    }

    /** @fn Returns cumulative phase steps. 
     * @return Cumulative phase steps.
     */
    vector<int> getCumulativePhaseSteps(){
      for (int i = 0; i < phase_steps.size(); i++) {
          int cum_val = i == 0 ? phase_steps[0] : phase_steps[i] + cum_phase_steps[i-1];
          cum_phase_steps.push_back(cum_val);
      }
      return cum_phase_steps;
    }

    /** @fn Returns the start time step for a given phase.
     * @param[in]phase    Phase number.
     * @return Initial time step.
     */
    int getStartTimeStep(int phase){
      int t_p_i = cum_phase_steps[phase] - phase_steps[phase];
      if (phase != 0) t_p_i += 1;
      return t_p_i;
    }

    /** @fn Returns the end time step for a given phase.
     * @param[in]phase    Phase number.
     * @return Final time step.
     */
    int getEndTimeStep(int phase){
      return cum_phase_steps[phase];
    }

    /** @fn Returns the contact links for a given phase.
     * @param[in]phase    Phase number.
     * @return Vector of contact links.
     */
    vector<string> getPhaseContactLinks(int phase){
      std::string current_stance = stances[phase%stances.size()];
      vector<string> stance_CPs = stance_map[current_stance];
      return stance_CPs;
    }

    /** @fn Returns the swing links for a given phase.
     * @param[in]phase    Phase number.
     * @return Vector of swing links.
     */
    vector<string> getPhaseSwingLinks(int phase){
      vector<string> phase_swing_links;
      std::string current_stance = stances[phase%stances.size()];
      vector<string> stance_CPs = stance_map[current_stance];
      for (auto&& l : links) {
        if (std::find(stance_CPs.begin(),
                stance_CPs.end(), l) == stance_CPs.end())
            phase_swing_links.push_back(l); 
      }
      return phase_swing_links;
    }

    /** @fn Writes the angles, vels, accels, torques and time values for a single phase to disk.
     * @param[in]traj_file    Trajectory File being written onto.
     * @param[in]results      Results of Optimization.
     * @param[in]robot        Robot model.
     * @param[in]phase        Phase number.
     */
    void writePhaseToFile(std::ofstream& traj_file, gtsam::Values results, 
                          gtdynamics::Robot robot, int phase){
      int t = getStartTimeStep(phase);
      for (int phase_step = 0; phase_step < phase_steps[phase]; phase_step++) {

        vector<string> vals;
        for (auto&& joint : robot.joints())
          vals.push_back(std::to_string(results.atDouble(JointAngleKey(joint->getID(), t))));
        for (auto&& joint : robot.joints())
          vals.push_back(std::to_string(results.atDouble(JointVelKey(joint->getID(), t))));
        for (auto&& joint : robot.joints())
          vals.push_back(std::to_string(results.atDouble(JointAccelKey(joint->getID(), t))));
        for (auto&& joint : robot.joints())
          vals.push_back(std::to_string(results.atDouble(TorqueKey(joint->getID(), t))));
        vals.push_back(std::to_string(results.atDouble(PhaseKey(phase))));
        t++;
        string vals_str = boost::algorithm::join(vals, ",");
        traj_file << vals_str << "\n";
      }
    }
};
}