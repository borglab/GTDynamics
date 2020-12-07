/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Phase.h
 * @brief Utility methods for generating Phase objects.
 * @Author: Disha Das
 */

#pragma once

#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <algorithm>
#include <boost/algorithm/string/join.hpp>

using gtdynamics::ContactPoints, gtdynamics::ContactPoint,
    std::vector, std::string;

namespace gtdynamics
{
  /**
  * @class Phase class stores information about a robot stance
  * and its duration.
  */
  class Phase
  {
  protected:
    Robot robot_configuration_ ;        ///< Robot configuration of this stance
    ContactPoints contact_points_ = {}; ///< Contact Points
    int num_time_steps_;                ///< Number of time steps in this phase

  public:
    /// Constructor
    Phase(const Robot& robot_configuration, const int& time_steps)
                            : robot_configuration_(robot_configuration), 
                              num_time_steps_(time_steps) {}

    /** @fn Adds a contact point in the phase.
     * 
     * @param[in] link             Name of link in the robot_configuration.
     * @param[in] point            Point of contact on link.
     * @param[in] contact_height   Height of contact point from ground.
     */
    void addContactPoint(const string& link , const gtsam::Point3& point, const double& contact_height){
        robot_configuration_.getLinkByName(link);
        vector<string> links;
        for(auto &&cp: contact_points_) links.push_back(cp.name);
        if(std::find(links.begin(), links.end(), link) == links.end() ){
          contact_points_.push_back(ContactPoint{link, point, 0, contact_height});
        }
    }


    /** @fn Generates contact points.
     * 
     * @param[in] links            List of links.
     * @param[in] point            Point of contact on link.
     * @param[in] contact_height   Height of contact point from ground.
     */
    void generateContactPoints(const vector<string>& links , const gtsam::Point3& point, const double& contact_height){
      for(auto &&l : links){
        robot_configuration_.getLinkByName(l);
        vector<string> link_list;
        for(auto &&cp: contact_points_) link_list.push_back(cp.name);
        if(std::find(link_list.begin(), link_list.end(), l) == link_list.end() )
            contact_points_.push_back(ContactPoint{l, point, 0, contact_height});
      }
    }

    /// Returns the robot configuration of the stance
    Robot getRobotConfiguration() const { return robot_configuration_;}

    /// Returns all the contact points in the stance
    ContactPoints getAllContactPoints() const { return contact_points_;}

    /// Returns the contact point object of link.
    ContactPoint getContactPointAtLink(const string& link) {
        ContactPoints::iterator it = std::find_if(contact_points_.begin(), contact_points_.end(), 
                                                    [link](const ContactPoint& cp){return cp.name == link;});
        if(it == contact_points_.end())
            throw std::runtime_error("Link " + link + " has no contact point!");
        return it[0];
    }

    /// Returns the number of time steps in this phase
    int numTimeSteps() const { return num_time_steps_;}

  };
} //namespace gtdynamics