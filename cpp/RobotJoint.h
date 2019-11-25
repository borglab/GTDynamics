/**
 * @file  RobotJoint.h
 * @brief only joint part of a manipulator
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <utils.h>

#include <RobotLink.h>
#include <RobotTypes.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>

#include <urdf_model/joint.h>

#include <string>
#include <vector>
#include <memory>

// TODO(aescontrela): Verify that the `effort` parameter refers to the max torque/force
// applied at the joint.
namespace robot {
/**
 * RobotJoint is the base class for a joint connecting two RobotLink objects.
 */
class RobotJoint : public std::enable_shared_from_this<RobotJoint>{
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

  unsigned char id_;

  char jointType_;
  JointEffortType jointEffortType_;

  // Rotational axis for 'R' jointType_ (revolute joint). Translational
  // direction for 'P' jointType_ (prismatic joint).
  gtsam::Vector3 axis_;
  // x-y-z and r-p-y coords of link frame w.r.t.
  // the former link frame.
  gtsam::Pose3 origin_;
  // Rest transform to link frame from source link frame at rest.
  gtsam::Pose3 pMc_;
  // Joint axis expressed in COM frame of dest link
  gtsam::Vector6 screwAxis_;

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
  
 public:
  RobotJoint() {}
    /**
     * Create RobotJoint from a urdf::JointSharedPtr instance, as described in
     * ROS/urdfdom_headers:
     * https://github.com/ros/urdfdom_headers/blob/master/urdf_model/include/urdf_model/joint.h
     * 
     * Keyword arguments:
     *   urdf_joint_ptr             -- urdf::JointSharedPtr instance to derive joint attributes from.
     *   jointEffortType_           -- joint effort type.
     *   springCoefficient          -- spring coefficient for Impedence joint.
     *   jointLimitThreshold        -- joint angle limit threshold.
     *   velocityLimitThreshold     -- joint velocity limit threshold.
     *   accelerationLimit          -- joint acceleration limit
     *   accelerationLimitThreshold -- joint acceleration limit threshold
     *   torqueLimitThreshold       -- joint torque limit threshold
     *   parent_link                -- shared pointer to the parent RobotLink.
     *   child_link                 -- weak pointer to the child RobotLink.
     */
    RobotJoint(urdf::JointSharedPtr urdf_joint_ptr, JointEffortType joint_effort_type,
              double springCoefficient, double jointLimitThreshold,
              double velocityLimitThreshold, double accelerationLimit, double accelerationLimitThreshold,
              double torqueLimitThreshold, RobotLinkSharedPtr parent_link, RobotLinkWeakPtr child_link) 
              : name_(urdf_joint_ptr->name),
                jointEffortType_(joint_effort_type),
                axis_(gtsam::Vector3(urdf_joint_ptr->axis.x, urdf_joint_ptr->axis.y,
                  urdf_joint_ptr->axis.z)),
                origin_(gtsam::Pose3(
                  gtsam::Rot3(gtsam::Quaternion(
                    urdf_joint_ptr->parent_to_joint_origin_transform.rotation.w,
                    urdf_joint_ptr->parent_to_joint_origin_transform.rotation.x,
                    urdf_joint_ptr->parent_to_joint_origin_transform.rotation.y,
                    urdf_joint_ptr->parent_to_joint_origin_transform.rotation.z
                    )),
                  gtsam::Point3(
                    urdf_joint_ptr->parent_to_joint_origin_transform.position.x,
                    urdf_joint_ptr->parent_to_joint_origin_transform.position.y,
                    urdf_joint_ptr->parent_to_joint_origin_transform.position.z
                ))),
                jointLowerLimit_(urdf_joint_ptr->limits->lower),
                jointUpperLimit_(urdf_joint_ptr->limits->upper),
                jointLimitThreshold_(jointLimitThreshold),
                dampCoefficient_(urdf_joint_ptr->dynamics->damping),
                springCoefficient_(springCoefficient),
                velocityLimit_(urdf_joint_ptr->limits->velocity),
                velocityLimitThreshold_(velocityLimitThreshold),
                accelerationLimit_(accelerationLimit),
                accelerationLimitThreshold_(accelerationLimitThreshold),
                torqueLimit_(urdf_joint_ptr->limits->effort),
                torqueLimitThreshold_(torqueLimitThreshold),
                parent_link_(parent_link), child_link_(child_link) {
        if (urdf_joint_ptr->type == urdf::Joint::PRISMATIC) {
            jointType_ = 'P';
        } else if (urdf_joint_ptr->type == urdf::Joint::REVOLUTE) {
            jointType_ = 'R';
        }

        if (jointType_ == 'R') {
          pMc_ = origin_ *
                gtsam::Pose3(gtsam::Rot3::Rodrigues(axis_ * 0), gtsam::Point3());
        } else {
          pMc_ = origin_ * gtsam::Pose3(gtsam::Rot3(), axis_ * 0);
        }

        RobotLinkSharedPtr child_link_strong_ = child_link_.lock();
        screwAxis_ = manipulator::unit_twist(
          child_link_strong_->centerOfMass().rotation().inverse() * axis_,
          child_link_strong_->centerOfMass().translation().vector());
    }

  /// Return a shared ptr to this joint.
  RobotJointSharedPtr getSharedPtr() { return shared_from_this(); }

  /// Return a weak ptr to this joint.
  RobotJointWeakPtr getWeakPtr() { return shared_from_this(); }

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

  // Return joint name.
  std::string name() const { return name_; }

  /// Return jointType
  char jointType() const { return jointType_; }

  /// Return joint effort type
  JointEffortType jointEffortType() const { return jointEffortType_; }

  /// Return the joint axis. Rotational axis for revolute and translation
  /// direction for prismatic.
  gtsam::Vector3 axis() const { return axis_; }

  // x-y-z and r-p-y coords of link frame w.r.t. the former link frame.
  gtsam::Pose3 origin() const { return origin_; }

  /// Return transfrom of dest link frame w.r.t. source link frame at rest
  gtsam::Pose3 pMc() const { return pMc_; }

  /// Return screw axis.
  const gtsam::Vector6 &screwAxis() const { return screwAxis_; }

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

  /** Return transfrom of dest link frame w.r.t. source link frame at any joint
   *  angle
   * Keyword arguments:
        q -- optional joint angles.
  */
  gtsam::Pose3 pTc(double q) const {
    // screw axis in dest link frame
    gtsam::Vector6 screwAxis = manipulator::unit_twist(axis_, gtsam::Point3(0, 0, 0));
    return pMc_ * gtsam::Pose3::Expmap(screwAxis * q);
  }

  virtual ~RobotJoint() = default;
};

struct RobotJointParams {
  std::string name; // Name of this joint as described in the URDF file.

  RobotJoint::JointEffortType jointEffortType = RobotJoint::JointEffortType::Actuated;
  double springCoefficient = 0; // spring coefficient for Impedence joint.
  double jointLimitThreshold = 0.0; // joint angle limit threshold.
  double velocityLimitThreshold = 0.0; // joint velocity limit threshold.
  double accelerationLimit = 10000; // joint acceleration limit.
  double accelerationLimitThreshold = 0.0; // joint acceleration limit threshold.
  double torqueLimitThreshold = 0.0; // joint torque limit threshold.
};

}  // namespace robot
