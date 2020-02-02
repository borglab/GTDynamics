/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RobotLink.h
 * @brief only link part of a robot, does not include joint part
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <RobotTypes.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <sdf/sdf.hh>

#include <memory>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace robot {
/**
 * RobotLink is the base class for links taking different format of parameters
 */
class RobotLink : public std::enable_shared_from_this<RobotLink> {
 private:
  std::string name_;

  int id_ = -1;

  // Inertial elements.
  double mass_;
  gtsam::Pose3 centerOfMass_;
  gtsam::Matrix3 inertia_;

  // SDF Elements.
  gtsam::Pose3 Twl_;    // Link frame defined in the world frame.
  gtsam::Pose3 Tlcom_;  // CoM frame defined in the link frame.
  gtsam::Pose3 Twcom_;  // CoM frame defined in the world frame.

  // option to fix the link, used for ground link
  bool is_fixed_;
  gtsam::Pose3 fixed_pose_;

  std::vector<RobotJointWeakPtr> joints_;  // joints connected to the link

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
  explicit RobotLink(sdf::Link sdf_link)
      : name_(sdf_link.Name()),
        mass_(sdf_link.Inertial().MassMatrix().Mass()),
        inertia_(
            (gtsam::Matrix(3, 3) << sdf_link.Inertial().Moi()(0, 0),
             sdf_link.Inertial().Moi()(0, 1), sdf_link.Inertial().Moi()(0, 2),
             sdf_link.Inertial().Moi()(1, 0), sdf_link.Inertial().Moi()(1, 1),
             sdf_link.Inertial().Moi()(1, 2), sdf_link.Inertial().Moi()(2, 0),
             sdf_link.Inertial().Moi()(2, 1), sdf_link.Inertial().Moi()(2, 2))
                .finished()),
        Twl_(gtsam::Pose3(
            gtsam::Rot3(gtsam::Quaternion(
                sdf_link.Pose().Rot().W(), sdf_link.Pose().Rot().X(),
                sdf_link.Pose().Rot().Y(), sdf_link.Pose().Rot().Z())),
            gtsam::Point3(sdf_link.Pose().Pos()[0], sdf_link.Pose().Pos()[1],
                          sdf_link.Pose().Pos()[2]))),
        Tlcom_(
            gtsam::Pose3(gtsam::Rot3(gtsam::Quaternion(
                             sdf_link.Inertial().Pose().Rot().W(),
                             sdf_link.Inertial().Pose().Rot().X(),
                             sdf_link.Inertial().Pose().Rot().Y(),
                             sdf_link.Inertial().Pose().Rot().Z())),
                         gtsam::Point3(sdf_link.Inertial().Pose().Pos()[0],
                                       sdf_link.Inertial().Pose().Pos()[1],
                                       sdf_link.Inertial().Pose().Pos()[2]))),
        Twcom_(Twl_ * Tlcom_),
        is_fixed_(false) {}

  virtual ~RobotLink() = default;

  // return a shared pointer of the link
  RobotLinkSharedPtr getSharedPtr(void) { return shared_from_this(); }

  // return a weak pointer of the link
  RobotLinkWeakPtr getWeakPtr(void) { return shared_from_this(); }

  // remove the joint
  void removeJoint(RobotJointWeakPtr joint) {
    for (auto joint_it = joints_.begin(); joint_it != joints_.end(); joint_it++) {
      if ((*joint_it).lock() == joint.lock()) {
        joints_.erase(joint_it);
        break;
      }
    }
  }

  // set ID for the link
  void setID(unsigned char id) {
    // if (id == 0) throw std::runtime_error("ID cannot be 0");
    id_ = id;
  }

  // return ID of the link
  int getID() const {
    if (id_ == -1)
      throw std::runtime_error(
          "Calling getID on a link whose ID has not been set");
    return id_;
  }

  // add joint to the link
  void addJoint(RobotJointWeakPtr joint_ptr) {
    joints_.push_back(joint_ptr);
  }

  // transform from link to world frame
  const gtsam::Pose3& Twl() { return Twl_; }

  // transfrom from link com frame to link frame
  const gtsam::Pose3& Tlcom() { return Tlcom_; }

  // transform from link com frame to world frame
  const gtsam::Pose3& Twcom() { return Twcom_; }

  // the fixed pose of the link
  const gtsam::Pose3& getFixedPose() { return fixed_pose_; }

  // whether the link is fixed
  bool isFixed() { return is_fixed_; }

  // fix the link to fixed_pose, if fixed_pose not specify, fix the link to
  // default pose
  void fix(const boost::optional<gtsam::Pose3&> fixed_pose = boost::none) {
    is_fixed_ = true;
    fixed_pose_ = fixed_pose ? *fixed_pose : Twcom_;
  }

  // unfix the link
  void unfix() { is_fixed_ = false; }

  // return all joints of the link
  const std::vector<RobotJointWeakPtr>& getJoints(void) const { return joints_; }

  // Reutrn link name.
  std::string name() const { return name_; }

  /// Return link mass.
  double mass() const { return mass_; }

  /// Return center of mass (gtsam::Pose3)
  const gtsam::Pose3& centerOfMass() const { return centerOfMass_; }

  /// Return the frame at link's end in the link com frame.
  gtsam::Pose3 leTl_com() const {
    gtsam::Pose3 l_comTle =
        gtsam::Pose3(gtsam::Rot3::identity(), Tlcom_.translation());
    return l_comTle.inverse();
  }

  /// Return inertia.
  const gtsam::Matrix3& inertia() const { return inertia_; }

  /// Return general mass gtsam::Matrix
  gtsam::Matrix6 inertiaMatrix() const {
    std::vector<gtsam::Matrix> gmm;
    gmm.push_back(inertia_);
    gmm.push_back(gtsam::I_3x3 * mass_);
    return gtsam::diag(gmm);
  }
};
}  // namespace robot
