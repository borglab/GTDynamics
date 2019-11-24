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

// TODO(aescontrela): Add `const` to instance methods that don't modify the object's
// data members.

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
    const boost::optional<std::vector<robot::LinkJointParams>> joint_params = boost::none);

class UniversalRobot {

private:
    std::vector<LinkBodySharedPtr> link_bodies_;
    std::vector<LinkJointSharedPtr> link_joints_;
    
    // The robot's world position specified via a single link.
    std::string base_name_;
    gtsam::Pose3 base_;

    // For quicker/easier access to links and joints.
    std::map<std::string, robot::LinkBodySharedPtr> name_to_link_body_;
    std::map<std::string, robot::LinkJointSharedPtr> name_to_link_joint_;


public:
    /**
     * Construct a robot structure using a URDF model interface.
     * Keyword Arguments:
     *  robot_links_and_joints    -- LinkBodyJointPair containing links and joints.
     *  base          -- wT0 transform from parent link to world frame.
     */    
    UniversalRobot(const LinkBodyJointPair urdf_links_and_joints,
                   const std::string base_name, const gtsam::Pose3 &base);

    /// Return parent link pose in world frame.
    const gtsam::Pose3& base() const;

    /// Return the link corresponding to the input string.
    LinkBodySharedPtr getLinkByName(std::string name);

    /// Return the joint corresponding to the input string.
    LinkJointSharedPtr getJointByName(std::string name);

    /// Return number of *moving* links.
    int numLinks() const;

    /// Return number of joints.
    int numJoints() const;

    /// Return each link's length.
    std::map<std::string, double> lengths() const;

    /// Return each joint's screw axis in their COM frame.
    std::map<std::string, gtsam::Vector6> screwAxes() const;

    /// Return all joint lower limits.
    std::map<std::string, double> jointLowerLimits() const;

    /// Return all joint upper limits.
    std::map<std::string, double> jointUpperLimits() const;

    /// Return all joint limit thresholds.
    std::map<std::string, double> jointLimitThresholds() const;

    /** Calculate link transforms for all links. 
     * 
     * Each link can have multiple transforms. Consider the following:
     *     \
     *      \ l0
     *       \ j0      
     *  j1 o--o-----o
     *    /     l2
     *   / l1
     *  /
     * 
     * In this case, link l2 has two parents: l0 and l1, which connect via
     * joints j0 and j1, respectively. Therefore link l2 contains two transforms:
     * l0->l2 and l1->l2
     * 
     * This method returns the transforms of each link relative to its parent(s).
     * The return value is a nested map. The outer layer maps from the link name to
     * a mapping of the parent link name to the transform. For the above case, the
     * return value is:
     * 
     * l2:
     *   l0: [R0|t0]
     *   l1: [R1|t1]
     * 
     * Non-specified joint angles are assumed to be the rest angle.
     * 
     * Keyword arguments:
     *   joint_name_to_angle -- Map from joint name to desired angle.
     */
    std::map<std::string, std::map<std::string, gtsam::Pose3>> linkTransforms(
        boost::optional<std::map<std::string, double>> joint_name_to_angle = boost::none
    ) const;
    
    /** Calculate the transform from the destination link COM to the source
     * link COM frame of a particular joint.
     * 
     * Keyword arguments:
     *   name -- the joint's name.
     *   q    -- joint angle (in rad).
     */
    gtsam::Pose3 cTpCOM(std::string name, double q);

    /** Calculate transforms from the destination link COM frame to the source
     * link COM frame for all the joints.
     * 
     * In the following case, l2 is the destination link while l0 and l1 are
     * the source links:
     * 
     *     \
     *      \ l0
     *       \ j0      
     *  j1 o--o-----o
     *    /     l2
     *   / l1
     *  /
     * 
     * This method returns the COM transform from the source links to the
     * destination link. The return value is a nested map. The outer
     * layer maps from the link name to a mapping of the parent link name to the
     * transform. For the above case, the return value is:
     * 
     * l2:
     *   l0: [R0|t0]
     *   l1: [R1|t1]
     * 
     * Keyword arguments:
     *   joint_name_to_angle -- Map from joint name to desired angle.
     */
    std::map<std::string, std::map<std::string, gtsam::Pose3>> cTpCOMs(
        boost::optional<std::map<std::string, double>> joint_name_to_angle = boost::none
    );
    
    /**
     * Return each link's center of mass frame at rest, in the world frame.
     * 
     * Return value is a map from the link name to the world frame COM pose.
     * 
     * Keyword arguments:
     *   joint_name_to_angle -- Optional map from joint name to desired angle.
     */
    std::map<std::string, gtsam::Pose3> COMFrames(
        boost::optional<std::map<std::string, double>> joint_name_to_angle = boost::none
    );

    /** Calculate list of transforms from COM frame j-1 relative to COM j.
     * 
     * In the following case, l2 is the destination link while l0 and l1 are
     * the source links:
     * 
     *     \
     *      \ l0
     *       \ j0      
     *  j1 o--o-----o
     *    /     l2
     *   / l1
     *  /
     * 
     * This method returns the COM transform from the destination link to the
     * source links. The return value is a nested map. The outer
     * layer maps from the link name to a mapping of the child link name to the
     * transform. For the above case, the return value is:
     * 
     * l0:
     *   l2: [R0|t0]^{-1}
     * l1:
     *   l2: [R0|t0]^{-1}
     * 
     * Keyword arguments:
     *   joint_name_to_angle -- Map from joint name to desired angle.
     */
    std::map<std::string, std::map<std::string, gtsam::Pose3>> jTiTransforms(
        boost::optional<std::map<std::string, double>> joint_name_to_angle = boost::none
    );
    
    /** Return screw axes for all joints at rest configuration, expressed in
     * world frame. Return value is a map from joint name to spatial screw
     * axis.
     */
    std::map<std::string, gtsam::Vector6> spatialScrewAxes();

    /** Calculate the rigid body transformation which takes the joint frames
     * from its reference configuration to the current configuration for the
     * manipulator. R. Murray's book, page 116 about manipulator jacobian.
     * 
     * Keyword arguments:
     *   joint_name_to_angle -- Optional map from joint name to desired angle.
     */
    std::map<std::string, gtsam::Pose3> transformPOE(
         boost::optional<std::map<std::string, double>> joint_name_to_angle = boost::none
    ) const;

    /** Calculate spatial manipulator jacobian and joint poses.
     * 
     * Keyword arguments:
     *   joint_name_to_angle -- map from joint name to angle (for revolute
     *      joint) or distance (for prismatic joint).
     */
    // std::vector<gtsam::Matrix> spatialManipulatorJacobian(
    //   const gtsam::Vector &q) const;

    /** Forward kinematics.
     * 
     * Keyword arguments:
     *   q -- joint angles.
     *   J -- Jacobian matrix.
     * 
     * Returns link COM frames in world frame.
     */
    // std::vector<gtsam::Pose3> forwardKinematics(
    //   const gtsam::Vector &q,
    //   boost::optional<std::vector<gtsam::Matrix> &> J = boost::none) const;

    /** calculate inverse kinematics.
     *
     * Keyword arguments:
     *   poseGoal      -- goal pose
     *   init_q        -- initial value for joint angles
     */
    // gtsam::Vector inverseKinematics(
    //     const gtsam::Pose3 &poseGoal, const gtsam::Vector &init_q) const;

    /** Returns joint limit factor.
     * 
     * Keyword arguments:
     *   cost_model -- noise model.
     *   i          -- timestep index.
     */
    gtsam::NonlinearFactorGraph jointLimitFactors(
      const gtsam::noiseModel::Base::shared_ptr &cost_model, int i) const;

};    
} // namespace UniversalRobot
