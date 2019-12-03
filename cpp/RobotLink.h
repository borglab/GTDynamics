/**
 * @file  RobotLink.h
 * @brief only link part of a manipulator, does not include joint part
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <RobotTypes.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <urdf_model/link.h>

#include <string>
#include <vector>
#include <memory>

namespace robot {
/**
 * RobotLink is the base class for links taking different format of parameters
 */
class RobotLink : public std::enable_shared_from_this<RobotLink> {
 private:

  std::string name_;

  unsigned char id_ = 0;

  // Inertial elements.
  double mass_;
  gtsam::Pose3 centerOfMass_;
  gtsam::Matrix3 inertia_;

  // pose elements
  gtsam::Pose3 link_pose_;  // pose of link expressed in root link frame
  gtsam::Pose3 com_pose_;   // com pose of link expressed in root link frame
  bool pose_set_;           // indicate whether the pose has been set

  // option to fix the link, used for ground link
  bool is_fixed_;

  // Parent information.
  std::vector<RobotLinkSharedPtr> parent_links_;
  std::vector<RobotJointSharedPtr> parent_joints_;

  // Child information. References to child objects are stored as weak pointers
  // to prevent circular references.
  std::vector<RobotJointWeakPtr> child_joints_;
  std::vector<RobotLinkWeakPtr> child_links_;

 public:
  RobotLink() {}
  
  /**
   * Create RobotLink from a urdf::LinkSharedPtr instance, as described in 
   * ROS/urdfdom_headers:
   * https://github.com/ros/urdfdom_headers/blob/master/urdf_model/include/urdf_model/link.h
   * 
   * Keyword arguments:
   *   urdf_link_ptr   -- (ptr to) object containing relevant link information.
   *   parent_link     -- link connected to this link via the parent joint.
   *   parent_joint    -- joint which connects this link to the parent link.
   */
  RobotLink(urdf::LinkSharedPtr urdf_link_ptr) 
      : name_(urdf_link_ptr->name), mass_(urdf_link_ptr->inertial->mass),
        centerOfMass_(gtsam::Pose3(
            gtsam::Rot3(
                gtsam::Quaternion(
                    urdf_link_ptr->inertial->origin.rotation.w,
                    urdf_link_ptr->inertial->origin.rotation.x,
                    urdf_link_ptr->inertial->origin.rotation.y,
                    urdf_link_ptr->inertial->origin.rotation.z
                )
            ), 
            gtsam::Point3(
                urdf_link_ptr->inertial->origin.position.x,
                urdf_link_ptr->inertial->origin.position.y,
                urdf_link_ptr->inertial->origin.position.z
            )
        )),
        inertia_((gtsam::Matrix(3,3) << 
            urdf_link_ptr->inertial->ixx, urdf_link_ptr->inertial->ixy, urdf_link_ptr->inertial->ixz,
            urdf_link_ptr->inertial->ixy, urdf_link_ptr->inertial->iyy, urdf_link_ptr->inertial->iyz,
            urdf_link_ptr->inertial->ixz, urdf_link_ptr->inertial->iyz, urdf_link_ptr->inertial->izz).finished()),
        pose_set_(false), is_fixed_(false){}

  virtual ~RobotLink() = default;

  RobotLinkSharedPtr getSharedPtr(void) {
      return shared_from_this(); 
  }

  RobotLinkWeakPtr getWeakPtr(void) {
      return shared_from_this();
  }

  void setID(unsigned char id) {
      if (id == 0)
          throw std::runtime_error("ID cannot be 0");
      id_ = id;
  }

  unsigned char getID() {
      if (id_ == 0)
          throw std::runtime_error(
              "Calling getID on a link whose ID has not been set");
      return id_;
  }

  void addChildJoint(RobotJointWeakPtr child_joint_ptr) {
      child_joints_.push_back(child_joint_ptr);
  }

  void addChildLink(RobotLinkWeakPtr child_link_ptr) {
      child_links_.push_back(child_link_ptr);
  }

  void addParentJoint(RobotJointSharedPtr parent_joint_ptr) {
      parent_joints_.push_back(parent_joint_ptr);
  }

  void addParentLink(RobotLinkSharedPtr parent_link_ptr) {
      parent_links_.push_back(parent_link_ptr);
  }

  void setPose(const gtsam::Pose3& link_pose) {
    link_pose_ = link_pose;
    com_pose_ = link_pose_ * centerOfMass_ ;
    pose_set_ = true;
  }

  const gtsam::Pose3& getLinkPose() {
    return link_pose_;
  }

  const gtsam::Pose3& getComPose() {
    return com_pose_;
  }

  bool isPoseSet() {
    return pose_set_;
  }

  bool isFixed() {
    return is_fixed_;
  }

  void fix() {
    is_fixed_ = true;
  }

  void unfix() {
    is_fixed_ = false;
  }

  std::vector<RobotJointWeakPtr> getChildJoints(void) {
      return child_joints_;
  }

  std::vector<RobotLinkWeakPtr> getChildLinks(void) {
      return child_links_;
  }

  std::vector<RobotJointSharedPtr> getParentJoints(void) {
      return parent_joints_;
  }

  std::vector<RobotLinkSharedPtr> getParentLinks(void) {
      return parent_links_;
  }

  std::vector<RobotJointSharedPtr> getJoints(void) {
      std::vector<RobotJointSharedPtr> parent_joints = getParentJoints();
      std::vector<RobotJointWeakPtr> child_joints_weak = getChildJoints();
      
      std::vector<RobotJointSharedPtr> child_joints;
      for (auto&& child_joint : child_joints_weak)
        child_joints.push_back(child_joint.lock());
    
      parent_joints.insert(parent_joints.begin(), child_joints.begin(),
        child_joints.end());
    
      return parent_joints;
  }

  // Reutrn link name.
  std::string name() const { return name_; }

  /// Return link mass.
  double mass() const { return mass_; }

  /// Return center of mass (gtsam::Pose3)
  const gtsam::Pose3 &centerOfMass() const { return centerOfMass_; }

  /// Return inertia.
  const gtsam::Matrix3 &inertia() const { return inertia_; }

  /// Return general mass gtsam::Matrix
  gtsam::Matrix6 inertiaMatrix() const {
    std::vector<gtsam::Matrix> gmm;
    gmm.push_back(inertia_);
    gmm.push_back(gtsam::I_3x3 * mass_);

    return gtsam::diag(gmm);
  }
};
}  // namespace robot
