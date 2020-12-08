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
            ContactPoints walk_cycle_contact_points_ {};          ///< All Contact points in the walk cycle

        public:
            /// Default Constructor
            WalkCycle(){}

            /** @fn Adds phase in walk cycle
             * @param[in] phase      Phase object
             */
            void addPhase(const Phase& phase){
                auto phase_contact_points = phase.getAllContactPoints();
                for(auto &&contact_point : phase_contact_points){
                    if(walk_cycle_contact_points_.find(contact_point.first) == 
                                                    walk_cycle_contact_points_.end()){
                        walk_cycle_contact_points_.emplace(
                                            contact_point.first, contact_point.second);
                    }else{
                        auto cp = walk_cycle_contact_points_[contact_point.first];
                        if(cp.contact_point != contact_point.second.contact_point ||
                           cp.contact_id != contact_point.second.contact_id ||
                           cp.contact_height != contact_point.second.contact_height)
                           throw std::runtime_error("Multiple Contact points for Link "
                                                         + contact_point.first + " found!");
                    }
                }
                phases_.push_back(phase);
            }

            /// Returns vector of phases in the walk cycle
            std::vector<Phase> phases() const { return phases_; }

            /// Returns count of phases in the walk cycle
            int numPhases() const { return phases_.size(); }

            ContactPoints allContactPoints() const { return walk_cycle_contact_points_;}
    };
} //namespace gtdynamics
