/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Phase.h
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

using gtdynamics::ContactPoints, gtdynamics::ContactPoint, 
      std::vector, std::string;

namespace gtdynamics {

/**
 * @class Phase is a class that stores information related
 * to Contact Points, stances and sequence of stances and 
 * implements functions derived from phase-related calculations.
 */
class Phase {

  protected:
    /** Map from CP name to CP object */
    std::map<std::string, ContactPoint > contact_point_map_;

    /** Map from stance name to constituent CP names */
    std::map<std::string, vector<std::string> > stance_map_;

    const std::vector<string> stances_; ///< List of stances

    int repeat_ = 1;                    ///< Number of repetitions of stance sequence

    int step_size_ = 20;                ///< Time step for each phase

    vector<int> phase_steps_;           ///< Vector of constructed walk cycle time steps

    vector<int> cum_phase_steps_;       ///< Vector of cumulative time steps

    vector<string> links_;              ///< List of all links or CP names

    /** Repeats a sequence of stances */
    const vector<ContactPoints> repeatSequence(const vector<ContactPoints>& cps_original, const int& repetition) {
        vector<ContactPoints> cps;
        for(int i = 0; i < repetition; i++){
            cps.insert(std::end(cps), std::begin(cps_original), std::end(cps_original));
        }
        return cps;
    }
    
    /** Gets the intersection between two string vectors of contact links */
    const vector<string> getIntersection(vector<string> CPs_1, vector<string> CPs_2) {
        std::sort(CPs_1.begin(), CPs_1.end());
        std::sort(CPs_2.begin(), CPs_2.end());
        std::vector<string> intersection;
        std::set_intersection(CPs_1.begin(), CPs_1.end(),
                                CPs_2.begin(), CPs_2.end(),
                                std::back_inserter(intersection));
        return intersection;
    }

    /** Returns the ContactPoints object from vector of string of CP names */
    const ContactPoints getContactPoints(const vector<std::string>& CPs) {
        ContactPoints CP_vec;
        for (auto&& CP : CPs)
            CP_vec.push_back(contact_point_map_[CP]);
        return CP_vec;
    }

  public:
    /** Default Constructor */
    Phase(const vector<string>& sequence):stances_(sequence){}

    /** @fn Maps a Contact Point name to its ContactPoint object.
     * @param[in] CP   Single ContactPoint object.
     */
    void addContactPoint(const ContactPoint& CP){
        contact_point_map_.insert(std::make_pair(CP.name, CP));
        links_.push_back(CP.name);
    }

    /** @fn Maps a vector of ContactPoint names to their ContactPoint objects.
     * @param[in] CPs   Vector of ContactPoint objects.
     */
    void addContactPoints(const ContactPoints& CPs){
        for (auto&& cp : CPs){
          contact_point_map_.insert(std::make_pair(cp.name, cp));
          links_.push_back(cp.name);
        }
    }

    /** @fn Maps a stance to a vector of constituent Contact Point names.
     * @param[in] name   Name of the stance.
     * @param[in] CPs    Constituent Contact points of the stance.
     */
    void addStance(const std::string& name, const vector<string>& CPs){
      if(std::find (stances_.begin(), stances_.end(), name) != stances_.end())
        stance_map_.insert(std::make_pair(name, CPs));
    }

    /** @fn Sets the number of repetitions to be made of the sequence.
     * @param[in] repetitions   Number of repetitions.
     */
    void setRepetitions(const int& repetitions){
      repeat_ = repetitions;
    }

    /** @fn Sets the time step for each phase.
     * @param[in] time_step   Time step for each phase.
     */
    void setPhaseStep(const int& time_step){
      step_size_ = time_step;
      phase_steps_.resize(stances_.size()*repeat_);
      std::fill (phase_steps_.begin(), phase_steps_.end(), step_size_);

      for (int i = 0; i < (int)phase_steps_.size(); i++) {
        int cum_val = i == 0 ? phase_steps_[0] : phase_steps_[i] + cum_phase_steps_[i-1];
        cum_phase_steps_.push_back(cum_val);
      }
    }

    /** @fn Returns a vector of ContactPoints objects for all phases after 
     * applying repetition on the original sequence.
     * @param[in]repetitions    Number of repetitions of stance sequence.
     * @return Phase CPs.
     */
    const vector<ContactPoints> getPhaseCPs(){
      vector<ContactPoints> phase_cps_original;

      for (auto&& stance : stances_){
          vector<std::string> CPs = stance_map_[stance];
          phase_cps_original.push_back(getContactPoints(CPs));
      }
      vector<ContactPoints> phase_cps = repeatSequence(phase_cps_original, repeat_);
      return phase_cps;
    }

    /** @fn Returns a vector of ContactPoints objects for all transitions between phases after 
     * applying repetition on the original sequence.
     * @return Transition CPs.
     */
    const vector<ContactPoints> getTransitionCPs() {
        vector<ContactPoints> trans_cps_original;

        for(int p = 0; p < (int)stances_.size(); p++ ){
            vector<string> stance_1 = stance_map_[stances_[p]];
            vector<string> stance_2;
            if(p == (int)stances_.size()-1)
              stance_2 = stance_map_[stances_[0]];
            else
              stance_2 = stance_map_[stances_[p + 1]];
            std::vector<string> intersection = getIntersection(stance_1, stance_2);
            trans_cps_original.push_back(getContactPoints(intersection));
        }
        vector<ContactPoints> trans_cps = repeatSequence(trans_cps_original, repeat_);
        trans_cps.erase(trans_cps.end());
        
        return trans_cps;
    }

    /** @fn Returns a vector of phase steps for all phases after 
     * applying repetition on the original sequence.
     * @param[in]step    Time step for each phase.
     * @return Phase steps.
     */
    const vector<int> getPhaseSteps(){
        return phase_steps_;
    }

    /** @fn Returns cumulative phase steps. 
     * @return Cumulative phase steps.
     */
    const vector<int> getCumulativePhaseSteps(){
      return cum_phase_steps_;
    }

    /** @fn Returns the start time step for a given phase.
     * @param[in]phase    Phase number.
     * @return Initial time step.
     */
    const int getStartTimeStep(const int& phase){
      int t_p_i = cum_phase_steps_[phase] - phase_steps_[phase];
      if (phase != 0) t_p_i += 1;
      return t_p_i;
    }

    /** @fn Returns the end time step for a given phase.
     * @param[in]phase    Phase number.
     * @return Final time step.
     */
    const int getEndTimeStep(const int& phase){
      return cum_phase_steps_[phase];
    }

    /** @fn Returns the contact links for a given phase.
     * @param[in]phase    Phase number.
     * @return Vector of contact links.
     */
    const vector<string> getPhaseContactLinks(const int& phase) {
      std::string current_stance = stances_[phase%stances_.size()];
      vector<string> stance_CPs = stance_map_[current_stance];
      return stance_CPs;
    }

    /** @fn Returns the swing links for a given phase.
     * @param[in]phase    Phase number.
     * @return Vector of swing links.
     */
    const vector<string> getPhaseSwingLinks(const int& phase) {
      vector<string> phase_swing_links;
      std::string current_stance = stances_[phase%stances_.size()];
      vector<string> stance_CPs = stance_map_[current_stance];
      for (auto&& l : links_) {
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
    const void writePhaseToFile(std::ofstream& traj_file, const gtsam::Values& results, 
                          const Robot& robot, const int& phase) {
      int t = getStartTimeStep(phase);
      for (int phase_step = 0; phase_step < phase_steps_[phase]; phase_step++) {

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
} //namespace gtdynamics