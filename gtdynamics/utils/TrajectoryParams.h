/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TrajectoryParams.h
 * @brief Class to store params for generating Trajectory.
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

namespace gtdynamics
{
    struct Stance
    {
        string name;
        Robot robot_configuration;
        vector<string> contact_points;
    };

    /**
 * @class TrajectoryParams class stores information related
 * to Contact Points, stances and walk cycles required
 * for calculation of Trajectory phases in the Trajectory class
 */
    class TrajectoryParams
    {
    protected:
        /** Map from CP name to CP object */
        std::map<std::string, ContactPoint> contact_point_map_;

        /** Map from stance name to Stance struct */
        std::map<std::string, Stance> stance_map_;

        std::vector<string> walk_cycle_ = {};   ///< Sequence of stances
        std::vector<int> durations_ = {};       ///< Stance durations in walk cycle
        vector<string> links_ = {};             ///< List of all contact links

    public:
        /** Default Constructor */
        TrajectoryParams() {}

        /** @fn Stores contact links
         * @param[in] links   List of contact links
         */
        void addLinks(const vector<string> &links){links_ = links;}

        /** @fn Maps a Contact Point name to its ContactPoint object.
         * @param[in] CP   Single ContactPoint object.
         */
        void addContactPoint(const ContactPoint &CP)
        {
            if (std::find(links_.begin(), links_.end(), CP.name) != links_.end())
                contact_point_map_.insert(std::make_pair(CP.name, CP));
            else
                throw std::runtime_error("Contact Point \"" + CP.name +
                                         "\" does not belong to a contact link!");
        }

        /** @fn Maps a vector of ContactPoint names to their ContactPoint objects.
         * @param[in] CPs   Vector of ContactPoint objects.
         */
        void addContactPoints(const ContactPoints &CPs)
        {
            for (auto &&cp : CPs)
            {
                if (std::find(links_.begin(), links_.end(), cp.name) != links_.end())
                    contact_point_map_.insert(std::make_pair(cp.name, cp));
                else
                    throw std::runtime_error("Contact Point \"" + cp.name +
                                             "\" does not belong to a contact link!");
            }
        }

        /** @fn Generates Contact Point object for each contact link
         * @param[in] point            Location of contact point in link frame.
         * @param[in] contact_height   Height of contact point 
         */
        void generateContactPoints (const gtsam::Point3 &point, const double &contact_height)
        {
            int i = 0;
            for (auto &&l : links_)
                addContactPoint(ContactPoint{l, point, i++, contact_height});
        }

        /** @fn Stores walk cycle
         * @param[in] walk_cycle   List of stances and their durations in a sequence that make up a walk cycle
         */
        void setWalkCycle(const vector<pair<string, int>>& walk_cycle){
            for (auto elem : walk_cycle){
                walk_cycle_.push_back(elem.first);
                durations_.push_back(elem.second);
            }
        }

        /** @fn Maps a stance to a Stance struct.
         * @param[in] name   Name of the stance.
         * @param[in] robot  Robot model used for this stance.
         * @param[in] CPs    Constituent Contact points of the stance.
         */
        void addStance(const std::string &name,
                       const Robot &robot, const vector<string> &CPs)
        {
            if (std::find(walk_cycle_.begin(), walk_cycle_.end(), name) == walk_cycle_.end())
                throw std::runtime_error("Stance " + name + " not found in walk cycle.");
            if (stance_map_.find(name) != stance_map_.end())
                stance_map_.erase(name);
            stance_map_.insert(std::make_pair(name, Stance{name, robot, CPs}));
        }

        /** Returns list of links */
        vector<string> getLinks() const { return links_; }

        /** Returns CP name to CP map */
        std::map<std::string, ContactPoint> getContactPointMap() const { return contact_point_map_; }

        /** Returns walk cycle */
        vector<string> getWalkCycle() const { return walk_cycle_; }

        /** Returns walk cycle durations */
        vector<int> getDurations() const {return durations_; }

        /** Returns stance name to Stance struct map */
        std::map<std::string,Stance> getStanceMap() const { return stance_map_; }
    };
} //namespace gtdynamics
