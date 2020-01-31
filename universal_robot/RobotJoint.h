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
  unsigned char id_;

  char jointType_;
  JointEffortType jointEffortType_;

  // Rotational axis for 'R' jointType_ (revolute joint). Translational
  // direction for 'P' jointType_ (prismatic joint).
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

  // Parent link.
  RobotLinkSharedPtr parent_link_;

  // Child link. References to child objects are stored as weak pointers
  // to prevent circular references.
  RobotLinkWeakPtr child_link_;

  // SDF Elements.
  gtsam::Pose3 Twj_;  // Joint frame defined in world frame.
  gtsam::Pose3
      Tjpcom_;  // Rest transform to parent link CoM frame from joint frame.
  gtsam::Pose3
      Tjccom_;  // Rest transform to child link CoM frame from joint frame.
  gtsam::Pose3 com_Mpc_;  // Rest transform to parent link com frame from child
                          // link com frame at rest.
  gtsam::Vector6 screwAxis_;  // Joint axis expressed in COM frame of child link

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
   *   child_link                 -- weak pointer to the child RobotLink.
   */
  RobotJoint(sdf::Joint sdf_joint, JointEffortType joint_effort_type,
             double springCoefficient, double jointLimitThreshold,
             double velocityLimitThreshold, double accelerationLimit,
             double accelerationLimitThreshold, double torqueLimitThreshold,
             RobotLinkSharedPtr parent_link, RobotLinkWeakPtr child_link)
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
      Twj_ = child_link.lock()->Twl();
    else
      Twj_ = gtsam::Pose3(
          gtsam::Rot3(gtsam::Quaternion(
              sdf_joint.Pose().Rot().W(), sdf_joint.Pose().Rot().X(),
              sdf_joint.Pose().Rot().Y(), sdf_joint.Pose().Rot().Z())),
          gtsam::Point3(sdf_joint.Pose().Pos()[0], sdf_joint.Pose().Pos()[1],
                        sdf_joint.Pose().Pos()[2]));

    Tjpcom_ = Twj_.inverse() * parent_link->Twcom();
    Tjccom_ = Twj_.inverse() * child_link.lock()->Twcom();
    com_Mpc_ = parent_link->Twcom().inverse() * child_link.lock()->Twcom();

    // RobotLinkSharedPtr child_link_strong_ = child_link_.lock();
    screwAxis_ = manipulator::unit_twist(
        Tjccom_.rotation().inverse() * axis_,
        Tjccom_.rotation().inverse() * (-Tjccom_.translation().vector()));
  }

  /// Return a shared ptr to this joint.
  RobotJointSharedPtr getSharedPtr() { return shared_from_this(); }

  /// Return a weak ptr to this joint.
  RobotJointWeakPtr getWeakPtr() { return shared_from_this(); }

  /// Set the joint's ID to track reference to gtsam::LabeledSymbol.
  void setID(unsigned char id) {
    if (id == 0) throw std::runtime_error("ID cannot be 0");
    id_ = id;
  }

  /// Get the joint's ID to track reference to gtsam::LabeledSymbol.
  unsigned char getID() {
    if (id_ == 0)
      throw std::runtime_error(
          "Calling getID on a link whose ID has not been set");
    return id_;
  }

  // Return joint name.
  std::string name() const { return name_; }

  /// Return jointType
  char jointType() const { return jointType_; }

  /// Return joint effort type
  JointEffortType jointEffortType() const { return jointEffortType_; }

  /// Return the joint axis. Rotational axis for revolute and translation
  /// direction for prismatic in the joint frame.
  const gtsam::Vector3& axis() const { return axis_; }

  /// Transform from the world frame to the joint frame.
  const gtsam::Pose3& Twj() const { return Twj_; }

  /// Transform from the joint frame to the parent's center of mass.
  const gtsam::Pose3& Tjpcom() const { return Tjpcom_; }

  /// Transform from the joint frame to the child's center of mass.
  const gtsam::Pose3& Tjccom() const { return Tjccom_; }

  /// Return transform of child link com frame w.r.t parent link com frame
  gtsam::Pose3 MpcCom(boost::optional<double> q = boost::none) const {
    if (q)
      return com_Mpc_ * gtsam::Pose3::Expmap(screwAxis_ * (*q));
    else
      return com_Mpc_;
  }

  /// Return transform of parent link com frame w.r.t child link com frame
  gtsam::Pose3 McpCom(boost::optional<double> q = boost::none) const {
    if (q)
      return gtsam::Pose3::Expmap(screwAxis_ * (*q)).inverse() *
             (com_Mpc_.inverse());
    else
      return com_Mpc_.inverse();
  }

  /// Return screw axis.
  const gtsam::Vector6& screwAxis() const { return screwAxis_; }

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

  /// Return a shared ptr to the parent link.
  RobotLinkSharedPtr parentLink() { return parent_link_; }

  /// Return a weak ptr to the child link.
  RobotLinkWeakPtr childLink() { return child_link_; }

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
