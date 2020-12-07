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

#pragma once

#include "gtdynamics/utils/Phase.h"

#include <string>
#include <stdio.h>
#include <vector>

using std::vector;

namespace gtdynamics
{
    /**
     * @class WalkCycle class stores the sequence of phases
     * in a walk cycle.
     */
    class WalkCycle{
        protected:
            std::vector<gtdynamics::Phase> phases_ = {};       ///< Phases in walk cycle
            std::map<string, ContactPoint> links_;

        public:
            /// Default Constructor
            WalkCycle(){}

            /** @fn Adds phase in walk cycle
             * @param[in] phase      Phase object
             */
            void addPhase(const Phase& phase){
                auto CPs_1 = phase.getAllContactPoints();
                ContactPoints CPs_2;
                ContactPoints::iterator it;
                for(auto p: phases_){
                    CPs_2 = p.getAllContactPoints();
                    for(auto cp : CPs_1){
                        it = std::find_if(CPs_2.begin(), CPs_2.end(), 
                            [cp](const ContactPoint& cp_){
                                    if(cp_.name == cp.name){
                                        if(cp_.contact_point != cp.contact_point)
                                            throw std::runtime_error("Multiple contact points for link " +cp.name+"!");
                                        else if(cp_.contact_height != cp.contact_height)
                                            throw std::runtime_error("Multiple contact points for link " +cp.name+"!");
                                        else return true;
                                    }
                                    else return false;});
                    }
                }
                phases_.push_back(phase);
                for(auto &&cp: phase.getAllContactPoints()){
                    if(links_.find(cp.name) == links_.end())
                        links_[cp.name] = cp;
                }
            }

            /// Returns vector of phases in the walk cycle
            std::vector<Phase> phases() const { return phases_; }

            /// Returns count of phases in the walk cycle
            int numPhases() const { return phases_.size(); }

            std::map<string, ContactPoint> links() const { return links_;}
    };
} //namespace gtdynamics
