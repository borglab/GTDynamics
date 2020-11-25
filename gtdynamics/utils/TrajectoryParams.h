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

#include <string>
#include <stdio.h>

using gtdynamics::ContactPoints, gtdynamics::ContactPoint,
    std::vector, std::string;

namespace gtdynamics
{
    /// Stance struct stores data for a single stance in a walk cycle
    struct Stance
    {
        Robot robot_configuration;
        vector<string> contact_points;
    };

    /**
     * @class TrajectoryParams class stores information related
     * to Contact Points, stances and walk cycles required
     * for calculation of Trajectory phases in the Trajectory class
     */
    class TrajectoryParams{
        protected:
            /// Contact points, indexed by contact point name (a string)
            std::map<std::string, ContactPoint> contact_points_;

            /// Stances, indexed by stance name (a string)
            std::map<std::string, Stance> stances_;

            WalkCycle walk_cycle_;                  ///< WalkCycle
            vector<string> links_ = {};             ///< List of all contact links

        public:
            /// Default Constructor
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
                    contact_points_.insert(std::make_pair(CP.name, CP));
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
                        contact_points_.insert(std::make_pair(cp.name, cp));
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
             * @param[in] walk_cycle   WalkCycle object
             */
            void setWalkCycle(const WalkCycle& walk_cycle){
                walk_cycle_ = walk_cycle;
                for (auto elem : walk_cycle.getWalkCycle()){
                    if (stances_.find(elem) == stances_.end())
                        throw std::runtime_error("Stance " + elem + " not found.");
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
                if (stances_.find(name) != stances_.end())
                    stances_.erase(name);
                for (auto link : CPs){
                    if (std::find(links_.begin(), links_.end(), link) == links_.end())
                        throw std::runtime_error("Contact Link " + link + " not found.");
                }
                stances_.insert(std::make_pair(name, Stance{robot, CPs}));
            }

            /// Returns list of links
            vector<string> getLinks() const { return links_; }

            /// Returns CP name to CP map
            std::map<std::string, ContactPoint> getContactPointMap() const { return contact_points_; }

            /// Returns walk cycle 
            vector<string> getWalkCycle() const { return walk_cycle_.getWalkCycle(); }

            /// Returns walk cycle durations
            vector<int> getDurations() const {return walk_cycle_.getDurations(); }

            /// Returns stance name to Stance struct map 
            std::map<std::string,Stance> getStanceMap() const { return stances_; }
    };
} //namespace gtdynamics
