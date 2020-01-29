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

#include <sdf/sdf.hh>

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

  // SDF Elements.
  gtsam::Pose3 Twl_; // Link frame defined in the world frame.
  gtsam::Pose3 Tlcom_; // CoM frame defined in the link frame.
  gtsam::Pose3 Twcom_; // CoM frame defined in the world frame.

  // option to fix the link, used for ground link
  bool is_fixed_;
  gtsam::Pose3 fixed_pose_;

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
   * Initialize RobotLink's inertial properties with a sdf::Link instance, as
   * described in the sdformat8 documentation: 
   * https://bitbucket.org/osrf/sdformat/src/7_to_gz11/include/sdf/Link.hh
   * 
   * Keyword arguments:
   *    sdf_link -- sdf::Link object containing link information.
   */
  RobotLink(sdf::Link sdf_link)
      : name_(sdf_link.Name()),
        mass_(sdf_link.Inertial().MassMatrix().Mass()),
        inertia_((gtsam::Matrix(3,3) <<
          sdf_link.Inertial().Moi()(0, 0), sdf_link.Inertial().Moi()(0, 1), sdf_link.Inertial().Moi()(0, 2),
          sdf_link.Inertial().Moi()(1, 0), sdf_link.Inertial().Moi()(1, 1), sdf_link.Inertial().Moi()(1, 2),
          sdf_link.Inertial().Moi()(2, 0), sdf_link.Inertial().Moi()(2, 1), sdf_link.Inertial().Moi()(2, 2)
        ).finished()),
        Twl_(gtsam::Pose3(
          gtsam::Rot3(
            gtsam::Quaternion(
              sdf_link.Pose().Rot().W(),
              sdf_link.Pose().Rot().X(),
              sdf_link.Pose().Rot().Y(),
              sdf_link.Pose().Rot().Z()
            )
          ),
          gtsam::Point3(
            sdf_link.Pose().Pos()[0],
            sdf_link.Pose().Pos()[1],
            sdf_link.Pose().Pos()[2]
          )
        )),
        Tlcom_(gtsam::Pose3(
          gtsam::Rot3(
            gtsam::Quaternion(
              sdf_link.Inertial().Pose().Rot().W(),
              sdf_link.Inertial().Pose().Rot().X(),
              sdf_link.Inertial().Pose().Rot().Y(),
              sdf_link.Inertial().Pose().Rot().Z()
            )
          ),
          gtsam::Point3(
            sdf_link.Inertial().Pose().Pos()[0],
            sdf_link.Inertial().Pose().Pos()[1],
            sdf_link.Inertial().Pose().Pos()[2]
          )
        )),
        Twcom_(Twl_ * Tlcom_),
        is_fixed_(false) {}

  virtual ~RobotLink() = default;

  RobotLinkSharedPtr getSharedPtr(void) {
      return shared_from_this(); 
  }

  RobotLinkWeakPtr getWeakPtr(void) {
      return shared_from_this();
  }

  // remove the parent joint and corresponding parent link
  void removeParentJoint(RobotJointSharedPtr joint, RobotLinkSharedPtr parent_link) {
    parent_joints_.erase(std::find(parent_joints_.begin(), parent_joints_.end(), joint));
    parent_links_.erase(std::find(parent_links_.begin(), parent_links_.end(), parent_link));
  }

  // remove the child joint and corresponding child link
  void removeChildJoint(RobotJointSharedPtr joint, RobotLinkSharedPtr child_link) {
    for (auto joint_it = child_joints_.begin(); joint_it != child_joints_.end(); joint_it++) {
      if ((*joint_it).lock() == joint) {
        child_joints_.erase(joint_it);
        break;
      }
    }

    for (auto link_it = child_links_.begin(); link_it != child_links_.end(); link_it++) {
      if ((*link_it).lock() == child_link) {
        child_links_.erase(link_it);
        break;
      }
    }
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

  // TODO(aescontrela): Kill the children and the parents. Just remove them entirely?
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

  const gtsam::Pose3& Twl() { return Twl_; }
  const gtsam::Pose3& Tlcom() { return Tlcom_; }
  const gtsam::Pose3& Twcom() { return Twcom_; }

  const gtsam::Pose3& getFixedPose() {
    return fixed_pose_;
  }

  bool isFixed() {
    return is_fixed_;
  }

  // fix the link to fixed_pose, if fixed_pose not specify, fix the link to default pose
  void fix(const boost::optional<gtsam::Pose3&> fixed_pose = boost::none) {
    is_fixed_ = true;
    fixed_pose_ = fixed_pose ? *fixed_pose : Twcom_;
  }

  void unfix() {
    is_fixed_ = false;
  }

  // TODO(aescontrela): Remove these.
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

  /// Return the frame at link's end in the link com frame.
  // TODO(aescontrela): Comment this better. Nuke this later.
  gtsam::Pose3 leTl_com() const {
    gtsam::Pose3 l_comTle = gtsam::Pose3(
    gtsam::Rot3::identity(), Tlcom_.translation());
    return l_comTle.inverse();
  }

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
