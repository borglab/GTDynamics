/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WalkCycle.h
 * @brief Class to store walk cycle.
 * @Author: Disha Das
 */

#include <string>
#include <stdio.h>
#include <vector>

using std::vector, std::string;

namespace std
{
    /**
     * @class WalkCycle class stores the sequence of stances
     * in a walk cycle and their respective durations.
     */
    class WalkCycle{
        protected:
            string name_ = "";                            ///< Name of Walk cycle
            vector<int> walk_cycle_durations_ = {};       ///< Stance durations in walk cycle
            vector<string> walk_cycle_stances_ = {};      ///< Sequence of stances in walk cycle

        public:
            /// Default Constructor
            WalkCycle(){}

            /// Constructor for naming the walk cycle
            WalkCycle(const string& name) : name_(name) {}

            /** @fn Adds next stance in se quence
             * @param[in] name       Name of stance
             * @param[in] duration   Duration of stance
             */
            void addNextStance(const string& name, const int& duration){
                walk_cycle_stances_.push_back(name);
                walk_cycle_durations_.push_back(duration);
            }

            /// Returns name of walk cycle 
            string name() const { return name_; }

            /// Returns walk cycle 
            vector<string> getWalkCycle() const { return walk_cycle_stances_; }

            /// Returns walk cycle durations
            vector<int> getDurations() const {return walk_cycle_durations_; }
    };
} //namespace std
