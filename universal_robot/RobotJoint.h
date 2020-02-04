/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RobotJoint.h
 * @brief Representation of a robot joint.
 * @Author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, Yetong Zhang
 */

#pragma once

#include <RobotLink.h>
#include <RobotTypes.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <utils.h>

#include <memory>
#include <string>
#include <vector>

namespace robot {
/**
 * RobotJoint is the base class for a joint connecting two RobotLink objects.
 */
class RobotJoint : public std::enable_shared_from_this<RobotJoint> {
 public:
  /** joint effort types
   * Actuated: motor powered
   * Unactuated: not powered, free to move, exert zero torque
   * Impedence: with spring resistance
   */
  enum JointEffortType { Actuated, Unactuated, Impedence };

 private:
  // This joint's name, as described in the URDF file.
  std::string name_;

  // ID reference to gtsam::LabeledSymbol.
  int id_ = -1;
  // 'R' for revolute, 'P' for prismatic
  char joint_type_;

  JointEffortType jointEffortType_;
  gtsam::Vector3 axis_;

  // Joint parameters.
  double jointLowerLimit_;
  double jointUpperLimit_;
  double jointLimitThreshold_;

  double dampCoefficient_;
  double springCoefficient_;

  double velocityLimit_;
  double velocityLimitThreshold_;

  double accelerationLimit_;
  double accelerationLimitThreshold_;

  double torqueLimit_;
  double torqueLimitThreshold_;

  RobotLinkWeakPtr parent_link_;
  RobotLinkWeakPtr child_link_;

  // SDF Elements.
  gtsam::Pose3 Twj_;  // Joint frame defined in world frame.
  gtsam::Pose3
      Tjpcom_;  // Rest transform to parent link CoM frame from joint frame.
  gtsam::Pose3
      Tjccom_;  // Rest transform to child link CoM frame from joint frame.
  gtsam::Pose3 com_Mpc_;  // Rest transform to parent link com frame from child
                          // link com frame at rest.
  gtsam::Vector6 pScrewAxis_;  // Joint axis expressed in COM frame of parent link
  gtsam::Vector6 cScrewAxis_;  // Joint axis expressed in COM frame of child link

  /// Return transform of child link com frame w.r.t parent link com frame
  gtsam::Pose3 MpcCom(boost::optional<double> q = boost::none) const {
    if (q)
      return com_Mpc_ * gtsam::Pose3::Expmap(cScrewAxis_ * (*q));
    else
      return com_Mpc_;
  }

  /// Return transform of parent link com frame w.r.t child link com frame
  gtsam::Pose3 McpCom(boost::optional<double> q = boost::none) const {
    if (q)
      // return gtsam::Pose3::Expmap(screwAxis_ * (*q)).inverse() *
      //        (com_Mpc_.inverse());
      return com_Mpc_.inverse() * gtsam::Pose3::Expmap(pScrewAxis_ * (*q));
    else
      return com_Mpc_.inverse();
  }

  /// Return the joint axis. Rotational axis for revolute and translation
  /// direction for prismatic in the joint frame.
  const gtsam::Vector3& axis() const { return axis_; }

  /// Transform from the world frame to the joint frame.
  const gtsam::Pose3& Twj() const { return Twj_; }

  /// Transform from the joint frame to the parent's center of mass.
  const gtsam::Pose3& Tjpcom() const { return Tjpcom_; }

  /// Transform from the joint frame to the child's center of mass.
  const gtsam::Pose3& Tjccom() const { return Tjccom_; }

  /// check if the link is Child link, throw an error if link is not connected to this joint
  bool isChildLink (const RobotLinkWeakPtr link) const
  {
    RobotLinkSharedPtr link_ptr = link.lock();
    if (link_ptr != child_link_.lock() && link_ptr != parent_link_.lock())
    {
      throw std::runtime_error("link " + link_ptr->name() + " is not connected to this joint " + name_);
    }
    return link_ptr == child_link_.lock();
  }

 public:
  RobotJoint() {}

  /**
   * Create RobotJoint from a sdf::Joint instance, as described in
   * ROS/urdfdom_headers:
   * https://bitbucket.org/osrf/sdformat/src/7_to_gz11/include/sdf/Joint.hh
   *
   * Keyword arguments:
   *   sdf_joint                  -- sdf::Joint instance to derive joint
   * attributes from. jointEffortType_           -- joint effort type.
   *   springCoefficient          -- spring coefficient for Impedence joint.
   *   jointLimitThreshold        -- joint angle limit threshold.
   *   velocityLimitThreshold     -- joint velocity limit threshold.
   *   accelerationLimit          -- joint acceleration limit
   *   accelerationLimitThreshold -- joint acceleration limit threshold
   *   torqueLimitThreshold       -- joint torque limit threshold
   *   parent_link                -- shared pointer to the parent RobotLink.
   *   child_link                 -- shared pointer to the child RobotLink.
   */
  RobotJoint(sdf::Joint sdf_joint, JointEffortType joint_effort_type,
             double springCoefficient, double jointLimitThreshold,
             double velocityLimitThreshold, double accelerationLimit,
             double accelerationLimitThreshold, double torqueLimitThreshold,
             RobotLinkSharedPtr parent_link, RobotLinkSharedPtr child_link)
      : name_(sdf_joint.Name()),
        jointEffortType_(joint_effort_type),
        axis_(gtsam::Vector3(sdf_joint.Axis()->Xyz()[0],
                             sdf_joint.Axis()->Xyz()[1],
                             sdf_joint.Axis()->Xyz()[2])),
        jointLowerLimit_(sdf_joint.Axis()->Lower()),
        jointUpperLimit_(sdf_joint.Axis()->Upper()),
        jointLimitThreshold_(jointLimitThreshold),
        dampCoefficient_(sdf_joint.Axis()->Damping()),
        springCoefficient_(springCoefficient),
        velocityLimit_(sdf_joint.Axis()->MaxVelocity()),
        velocityLimitThreshold_(velocityLimitThreshold),
        accelerationLimit_(accelerationLimit),
        accelerationLimitThreshold_(accelerationLimitThreshold),
        torqueLimit_(sdf_joint.Axis()->Effort()),
        torqueLimitThreshold_(torqueLimitThreshold),
        parent_link_(parent_link),
        child_link_(child_link) {
    if ((sdf_joint.PoseFrame() == "") &&
        (sdf_joint.Pose() == ignition::math::Pose3d()))
      Twj_ = child_link->Twl();
    else
      Twj_ = gtsam::Pose3(
          gtsam::Rot3(gtsam::Quaternion(
              sdf_joint.Pose().Rot().W(), sdf_joint.Pose().Rot().X(),
              sdf_joint.Pose().Rot().Y(), sdf_joint.Pose().Rot().Z())),
          gtsam::Point3(sdf_joint.Pose().Pos()[0], sdf_joint.Pose().Pos()[1],
                        sdf_joint.Pose().Pos()[2]));

    Tjpcom_ = Twj_.inverse() * parent_link->Twcom();
    Tjccom_ = Twj_.inverse() * child_link->Twcom();
    com_Mpc_ = parent_link->Twcom().inverse() * child_link->Twcom();

    pScrewAxis_ = robot::unit_twist(
        Tjpcom_.rotation().inverse() * -axis_,
        Tjpcom_.rotation().inverse() * (-Tjpcom_.translation().vector()));
    cScrewAxis_ = robot::unit_twist(
        Tjccom_.rotation().inverse() * axis_,
        Tjccom_.rotation().inverse() * (-Tjccom_.translation().vector()));
    if (sdf_joint.Type() == sdf::JointType::REVOLUTE) {
      joint_type_ = 'R';
    }
    else if (sdf_joint.Type() == sdf::JointType::PRISMATIC) {
      joint_type_ = 'P';
    }
  }

  /// Return a shared ptr to this joint.
  RobotJointSharedPtr getSharedPtr() { return shared_from_this(); }

  /// Return a weak ptr to this joint.
  RobotJointWeakPtr getWeakPtr() { return shared_from_this(); }

  /// Set the joint's ID to track reference to gtsam::LabeledSymbol.
  void setID(unsigned char id) {
    id_ = id;
  }

  /// Get the joint's ID to track reference to gtsam::LabeledSymbol.
  int getID() const {
    if (id_ == -1)
      throw std::runtime_error(
          "Calling getID on a link whose ID has not been set");
    return id_;
  }

  // Return joint name.
  std::string name() const { return name_; }

  /// Return jointType
  char jointType() const { return joint_type_; }

  /// Return joint effort type
  JointEffortType jointEffortType() const { return jointEffortType_; }

  RobotLinkWeakPtr otherLink(const RobotLinkWeakPtr link) const
  {
    return isChildLink(link) ? parent_link_ : child_link_;
  }

  /// Return the transform from this link com to the other link com frame
  gtsam::Pose3 transformFrom(const RobotLinkWeakPtr link, boost::optional<double> q = boost::none) const
  {
    return isChildLink(link) ? MpcCom(q) : McpCom(q);
  }

  /// Return the transform from the other link com to this link com frame
  gtsam::Pose3 transformTo(const RobotLinkWeakPtr link, boost::optional<double> q = boost::none) const
  {
    return isChildLink(link) ? McpCom(q) : MpcCom(q);
  }

  /// Return screw axis expressed in the specified link frame
  const gtsam::Vector6& screwAxis(const RobotLinkWeakPtr link) const { 
    return isChildLink(link) ? cScrewAxis_ : pScrewAxis_; 
  }

  std::vector<RobotLinkWeakPtr> links() const {
    return std::vector<RobotLinkWeakPtr> {parent_link_, child_link_};
  }

  /// Return a shared ptr to the parent link.
  RobotLinkWeakPtr parentLink() { return parent_link_; }

  /// Return a weak ptr to the child link.
  RobotLinkWeakPtr childLink() { return child_link_; }

  /// Return joint angle lower limit.
  double jointLowerLimit() const { return jointLowerLimit_; }

  /// Return joint angle upper limit.
  double jointUpperLimit() const { return jointUpperLimit_; }

  /// Return joint angle limit threshold.
  double jointLimitThreshold() const { return jointLimitThreshold_; }

  /// Return joint damping coefficient
  double dampCoefficient() const { return dampCoefficient_; }

  /// Return joint spring coefficient
  double springCoefficient() const { return springCoefficient_; }

  /// Return joint velocity limit.
  double velocityLimit() const { return velocityLimit_; }

  /// Return joint velocity limit threshold.
  double velocityLimitThreshold() const { return velocityLimitThreshold_; }

  /// Return joint acceleration limit.
  double accelerationLimit() const { return accelerationLimit_; }

  /// Return joint acceleration limit threshold.
  double accelerationLimitThreshold() const {
    return accelerationLimitThreshold_;
  }

  /// Return joint torque limit.
  double torqueLimit() const { return torqueLimit_; }

  /// Return joint torque limit threshold.
  double torqueLimitThreshold() const { return torqueLimitThreshold_; }

  virtual ~RobotJoint() = default;
};

struct RobotJointParams {
  std::string name;  // Name of this joint as described in the URDF file.

  RobotJoint::JointEffortType jointEffortType =
      RobotJoint::JointEffortType::Actuated;
  double springCoefficient = 0;      // spring coefficient for Impedence joint.
  double jointLimitThreshold = 0.0;  // joint angle limit threshold.
  double velocityLimitThreshold = 0.0;  // joint velocity limit threshold.
  double accelerationLimit = 10000;     // joint acceleration limit.
  double accelerationLimitThreshold =
      0.0;                            // joint acceleration limit threshold.
  double torqueLimitThreshold = 0.0;  // joint torque limit threshold.
};

}  // namespace robot
