/**
 * @file UniversalRobot.h
 * @brief Robot structure.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

// TODO(aescontrela): implement LinkBody getLinkByName(std::string name) and 
//  LinkJoint getJointByName(std::string name) methods.

#pragma once

#include <LinkTypes.h>
#include <LinkBody.h>
#include <LinkJoint.h>

#include <boost/optional.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <urdf_model/model.h>

#include <stdexcept>
#include <vector>

namespace robot {

/** Construct all LinkBody and LinkJoint objects from an input urdf::ModelInterfaceSharedPtr.
 * Keyword arguments:
 *    urdf_ptr         -- a shared pointer to a urdf::ModelInterface object.
 *    joint_params     -- a vector contanining optional params for joints.
 * 
 */
typedef std::pair<std::vector<robot::LinkBodySharedPtr>,
                  std::vector<robot::LinkJointSharedPtr>> LinkBodyJointPair;
LinkBodyJointPair extract_structure_from_urdf(
    const urdf::ModelInterfaceSharedPtr urdf_ptr,
    const boost::optional<std::vector<robot::LinkJointParams>> joint_params);

class UniversalRobot {

private:
    urdf::ModelInterfaceSharedPtr robot_urdf_;
    
    gtsam::Pose3 base_;


public:
    /**
     * Construct a robot structure using a URDF model interface.
     * Keyword Arguments:
     *  robot_urdf    -- URDF model interface.
     *  base          -- wT0 transform from parent link to world frame.
     */    
    UniversalRobot(const urdf::ModelInterfaceSharedPtr robot_urdf,
                   const gtsam::Pose3 &base);

    // Return parent link pose in world frame.
    const gtsam::Pose3& base() const;

};    
} // namespace UniversalRobot
