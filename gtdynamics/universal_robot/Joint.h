/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Joint.h
 * @brief Representation of a robot joint.
 * @Author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, Yetong Zhang
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_JOINT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_JOINT_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>

#include <memory>
#include <string>
#include <vector>

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RobotTypes.h"
#include "gtdynamics/utils/Utils.h"

namespace gtdynamics {

/**
 * Joint is the base class for a joint connecting two Link objects.
 */
class Joint : public std::enable_shared_from_this<Joint> {
 public:
  /** joint effort types
   * Actuated: motor powered
   * Unactuated: not powered, free to move, exert zero torque
   * Impedence: with spring resistance
   */
  enum JointEffortType { Actuated, Unactuated, Impedence };

  /**
   * JointParams contains all parameters to construct a joint
   */
  struct Params {
    std::string name;                   // name of the joint
    char joint_type;                    // type of joint
    Joint::JointEffortType effort_type;  // joint effor type
    LinkSharedPtr parent_link;          // shared pointer to parent link
    LinkSharedPtr child_link;           // shared pointer to child link
    gtsam::Vector3 axis;                // joint axis expressed in joint frame
    gtsam::Pose3 wTj;                   // joint pose expressed in world frame
    double joint_lower_limit;
    double joint_upper_limit;
    double joint_limit_threshold;
  };

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
  double joint_lower_limit_;
  double joint_upper_limit_;
  double joint_limit_threshold_;

  double damping_coeff_;
  double spring_coeff_;

  double velocity_limit_;
  double velocity_limit_threshold_;

  double acceleration_limit_;
  double acceleration_limit_threshold_;

  double torque_limit_;
  double torque_limit_threshold_;

  LinkSharedPtr parent_link_;
  LinkSharedPtr child_link_;

  // SDF Elements.
  gtsam::Pose3 wTj_;  // Joint frame defined in world frame.
  gtsam::Pose3
      jTpcom_;  // Rest transform to parent link CoM frame from joint frame.
  gtsam::Pose3
      jTccom_;  // Rest transform to child link CoM frame from joint frame.
  gtsam::Pose3 pMccom_;  // Rest transform to parent link com frame from child
                          // link com frame at rest.
  gtsam::Vector6
      pScrewAxis_;  // Joint axis expressed in COM frame of parent link
  gtsam::Vector6
      cScrewAxis_;  // Joint axis expressed in COM frame of child link

  /// Return transform of child link com frame w.r.t parent link com frame
  gtsam::Pose3 pMcCom(boost::optional<double> q = boost::none) const {
    if (q)
      return pMccom_ * gtsam::Pose3::Expmap(cScrewAxis_ * (*q));
    else
      return pMccom_;
  }

  /// Return transform of parent link com frame w.r.t child link com frame
  gtsam::Pose3 cMpCom(boost::optional<double> q = boost::none) const {
    if (q)
      // return gtsam::Pose3::Expmap(screwAxis_ * (*q)).inverse() *
      //        (pMccom_.inverse());
      return pMccom_.inverse() * gtsam::Pose3::Expmap(pScrewAxis_ * (*q));
    else
      return pMccom_.inverse();
  }

  /// Return the joint axis. Rotational axis for revolute and translation
  /// direction for prismatic in the joint frame.
  const gtsam::Vector3& axis() const { return axis_; }

  /// Transform from the world frame to the joint frame.
  const gtsam::Pose3& wTj() const { return wTj_; }

  /// Transform from the joint frame to the parent's center of mass.
  const gtsam::Pose3& jTpcom() const { return jTpcom_; }

  /// Transform from the joint frame to the child's center of mass.
  const gtsam::Pose3& jTccom() const { return jTccom_; }

  /// check if the link is Child link, throw an error if link is not connected
  /// to this joint
  bool isChildLink(const LinkSharedPtr link) const {
    LinkSharedPtr link_ptr = link;
    if (link_ptr != child_link_ && link_ptr != parent_link_) {
      throw std::runtime_error("link " + link_ptr->name() +
                               " is not connected to this joint " + name_);
    }
    return link_ptr == child_link_;
  }

  void setScrewAxis() {
    jTpcom_ = wTj_.inverse() * parent_link_->wTcom();
    jTccom_ = wTj_.inverse() * child_link_->wTcom();

    gtsam::Rot3 pcomRj = jTpcom_.rotation().inverse();
    gtsam::Rot3 ccomRj = jTccom_.rotation().inverse();

    pMccom_ = parent_link_->wTcom().inverse() * child_link_->wTcom();

    pScrewAxis_ = gtdynamics::unit_twist(pcomRj * -axis_,
        pcomRj * (-jTpcom_.translation().vector()));
    cScrewAxis_ = gtdynamics::unit_twist(ccomRj * axis_,
        ccomRj * (-jTccom_.translation().vector()));
  }

 public:
  Joint() {}

  /**
   * Create Joint from a sdf::Joint instance, as described in
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
   *   parent_link                -- shared pointer to the parent Link.
   *   child_link                 -- shared pointer to the child Link.
   */
  Joint(const sdf::Joint& sdf_joint, JointEffortType joint_effort_type,
        double springCoefficient, double jointLimitThreshold,
        double velocityLimitThreshold, double accelerationLimit,
        double accelerationLimitThreshold, double torqueLimitThreshold,
        LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : name_(sdf_joint.Name()),
        jointEffortType_(joint_effort_type),
        axis_(gtsam::Vector3(sdf_joint.Axis()->Xyz()[0],
                             sdf_joint.Axis()->Xyz()[1],
                             sdf_joint.Axis()->Xyz()[2])),
        joint_lower_limit_(sdf_joint.Axis()->Lower()),
        joint_upper_limit_(sdf_joint.Axis()->Upper()),
        joint_limit_threshold_(jointLimitThreshold),
        damping_coeff_(sdf_joint.Axis()->Damping()),
        spring_coeff_(springCoefficient),
        velocity_limit_(sdf_joint.Axis()->MaxVelocity()),
        velocity_limit_threshold_(velocityLimitThreshold),
        acceleration_limit_(accelerationLimit),
        acceleration_limit_threshold_(accelerationLimitThreshold),
        torque_limit_(sdf_joint.Axis()->Effort()),
        torque_limit_threshold_(torqueLimitThreshold),
        parent_link_(parent_link),
        child_link_(child_link) {
    if ((sdf_joint.PoseFrame() == "") &&
        (sdf_joint.Pose() == ignition::math::Pose3d()))
      wTj_ = child_link->wTl();
    else
      wTj_ = parse_ignition_pose(sdf_joint.Pose());

    setScrewAxis();
    if (sdf_joint.Type() == sdf::JointType::REVOLUTE) {
      joint_type_ = 'R';
    } else if (sdf_joint.Type() == sdf::JointType::PRISMATIC) {
      joint_type_ = 'P';
    }
  }

  /** constructor using JointParams */
  explicit Joint(const Params& params)
      : name_(params.name),
        joint_type_(params.joint_type),
        jointEffortType_(params.effort_type),
        axis_(params.axis),
        joint_lower_limit_(params.joint_lower_limit),
        joint_upper_limit_(params.joint_upper_limit),
        joint_limit_threshold_(params.joint_limit_threshold),
        parent_link_(params.parent_link),
        child_link_(params.child_link),
        wTj_(params.wTj) {
    setScrewAxis();
  }

  /// Return a shared ptr to this joint.
  JointSharedPtr getSharedPtr() { return shared_from_this(); }

  /// Set the joint's ID to track reference to gtsam::LabeledSymbol.
  void setID(unsigned char id) { id_ = id; }

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

  LinkSharedPtr otherLink(const LinkSharedPtr link) const {
    return isChildLink(link) ? parent_link_ : child_link_;
  }

  /// Return the transform from this link com to the other link com frame
  gtsam::Pose3 transformFrom(const LinkSharedPtr link,
                             boost::optional<double> q = boost::none) const {
    return isChildLink(link) ? pMcCom(q) : cMpCom(q);
  }

  /// Return the transform from the other link com to this link com frame
  gtsam::Pose3 transformTo(const LinkSharedPtr link,
                           boost::optional<double> q = boost::none) const {
    return isChildLink(link) ? cMpCom(q) : pMcCom(q);
  }

  /// Return screw axis expressed in the specified link frame
  const gtsam::Vector6& screwAxis(const LinkSharedPtr link) const {
    return isChildLink(link) ? cScrewAxis_ : pScrewAxis_;
  }

  std::vector<LinkSharedPtr> links() const {
    return std::vector<LinkSharedPtr>{parent_link_, child_link_};
  }

  /// Return a shared ptr to the parent link.
  LinkSharedPtr parentLink() { return parent_link_; }

  /// Return a shared ptr to the child link.
  LinkSharedPtr childLink() { return child_link_; }

  /// Return joint angle lower limit.
  double jointLowerLimit() const { return joint_lower_limit_; }

  /// Return joint angle upper limit.
  double jointUpperLimit() const { return joint_upper_limit_; }

  /// Return joint angle limit threshold.
  double jointLimitThreshold() const { return joint_limit_threshold_; }

  /// Return joint damping coefficient
  double dampCoefficient() const { return damping_coeff_; }

  /// Return joint spring coefficient
  double springCoefficient() const { return spring_coeff_; }

  /// Return joint velocity limit.
  double velocityLimit() const { return velocity_limit_; }

  /// Return joint velocity limit threshold.
  double velocityLimitThreshold() const { return velocity_limit_threshold_; }

  /// Return joint acceleration limit.
  double accelerationLimit() const { return acceleration_limit_; }

  /// Return joint acceleration limit threshold.
  double accelerationLimitThreshold() const {
    return acceleration_limit_threshold_;
  }

  /// Return joint torque limit.
  double torqueLimit() const { return torque_limit_; }

  /// Return joint torque limit threshold.
  double torqueLimitThreshold() const { return torque_limit_threshold_; }

  virtual ~Joint() = default;
};

struct JointParams {
  std::string name;  // Name of this joint as described in the URDF file.

  Joint::JointEffortType jointEffortType = Joint::JointEffortType::Actuated;
  double springCoefficient = 0;      // spring coefficient for Impedence joint.
  double jointLimitThreshold = 0.0;  // joint angle limit threshold.
  double velocityLimitThreshold = 0.0;  // joint velocity limit threshold.
  double accelerationLimit = 10000;     // joint acceleration limit.
  double accelerationLimitThreshold =
      0.0;                            // joint acceleration limit threshold.
  double torqueLimitThreshold = 0.0;  // joint torque limit threshold.
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_JOINT_H_
