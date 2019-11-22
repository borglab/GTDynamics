/**
 * @file  LinkBody.h
 * @brief only link part of a manipulator, does not include joint part
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <LinkTypes.h>

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
 * LinkBody is the base class for links taking different format of parameters
 */
class LinkBody : public std::enable_shared_from_this<LinkBody> {
 private:

  std::string name_;

  // Inertial elements.
  double mass_;
  gtsam::Pose3 centerOfMass_;
  gtsam::Matrix3 inertia_;

  // Parent information.
  std::vector<LinkBodySharedPtr> parent_links_;
  std::vector<LinkJointSharedPtr> parent_joints_;

  // Child information. References to child objects are stored as weak pointers
  // to prevent circular references.
  std::vector<LinkJointWeakPtr> child_joints_;
  std::vector<LinkBodyWeakPtr> child_links_;

 public:
  LinkBody() {}
  
  /**
   * Create LinkBody from a urdf::LinkSharedPtr instance, as described in 
   * ROS/urdfdom_headers:
   * https://github.com/ros/urdfdom_headers/blob/master/urdf_model/include/urdf_model/link.h
   * 
   * Keyword arguments:
   *   urdf_link_ptr   -- (ptr to) object containing relevant link information.
   *   parent_link     -- link connected to this link via the parent joint.
   *   parent_joint    -- joint which connects this link to the parent link.
   */
  LinkBody(urdf::LinkSharedPtr urdf_link_ptr) 
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
            urdf_link_ptr->inertial->ixz, urdf_link_ptr->inertial->iyz, urdf_link_ptr->inertial->izz).finished()) {}

  virtual ~LinkBody() = default;

  LinkBodySharedPtr getSharedPtr(void) {
      return shared_from_this(); 
  }

  LinkBodyWeakPtr getWeakPtr(void) {
      return shared_from_this();
  }

  void addChildJoint(LinkJointWeakPtr child_joint_ptr) {
      child_joints_.push_back(child_joint_ptr);
  }

  void addChildLink(LinkBodyWeakPtr child_link_ptr) {
      child_links_.push_back(child_link_ptr);
  }

  void addParentJoint(LinkJointSharedPtr parent_joint_ptr) {
      parent_joints_.push_back(parent_joint_ptr);
  }

  void addParentLink(LinkBodySharedPtr parent_link_ptr) {
      parent_links_.push_back(parent_link_ptr);
  }

  std::vector<LinkJointWeakPtr> getChildJoints(void) {
      return child_joints_;
  }

  std::vector<LinkBodyWeakPtr> getChildLinks(void) {
      return child_links_;
  }

  std::vector<LinkJointSharedPtr> getParentJoints(void) {
      return parent_joints_;
  }

  std::vector<LinkBodySharedPtr> getParentLinks(void) {
      return parent_links_;
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
