/**
 * @file  LinkJoint.h
 * @brief only joint part of a manipulator
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <utils.h>


#include <LinkTypes.h>

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
// TODO(aescontrela): Add pMc transform.
// TODO(aescontrela): Add screw axis.
namespace robot {
/**
 * LinkBody is the base class for links taking different format of parameters
 */
class LinkJoint : public std::enable_shared_from_this<LinkJoint>{
 public:
 
  /** joint effort types
   * Actuated: motor powered
   * Unactuated: not powered, free to move, exert zero torque
   * Impedence: with spring resistance
   */
  enum JointEffortType { Actuated, Unactuated, Impedence };

 private:

  std::string name_;

  char jointType_;
  JointEffortType jointEffortType_;

  gtsam::Vector3 axis_;

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

  uint jointIndex_;
  uint sourceLinkIndex_;
  uint destLinkIndex_;

  gtsam::Pose3 pMc_;
  gtsam::Vector6 screwAxis_;

  // Parent link.
  LinkBodySharedPtr parent_link_;

  // Child link. References to child objects are stored as weak pointers
  // to prevent circular references.
  LinkBodyWeakPtr child_link_;
  
 public:
  LinkJoint() {}
    /**
     * Create LinkJoint from a urdf::JointSharedPtr instance, as described in
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
     *   parent_link                -- shared pointer to the parent LinkBody.
     *   child_link                 -- weak pointer to the child LinkBody.
     */
    LinkJoint(urdf::JointSharedPtr urdf_joint_ptr, JointEffortType joint_effort_type,
              double springCoefficient, double jointLimitThreshold,
              double velocityLimitThreshold, double accelerationLimit, double accelerationLimitThreshold,
              double torqueLimitThreshold, LinkBodySharedPtr parent_link, LinkBodyWeakPtr child_link) 
              : name_(urdf_joint_ptr->name),
                jointEffortType_(joint_effort_type),
                axis_(gtsam::Vector3(urdf_joint_ptr->axis.x, urdf_joint_ptr->axis.y,
                  urdf_joint_ptr->axis.z)),
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
    }

  /**
   * Construct from jointType, source link, dest links,
   * Keyword arguments:
      jointType                  -- 'R': revolute,
                                    'P': prismatic
      jointEffortType_           -- joint effort type
      sourceLinkIndex            -- index of source link
      destLinkIndex             -- index dest link
      axis                       -- for 'R' joint, rotational axis
                                    for 'P' joint, translational direction
      pMc                        -- dest link frame w.r.t. source
                                    link frame at rest
      screwAxis                  -- joint axis expressed in
                                    COM frame of dest link
      sprintCoefficient          -- spring coefficient for Impedence joint
      dampCoefficient            -- damping coefficient for others
      jointLowerLimit            -- joint angle lower limit
      jointUpperLimit            -- joint angle upper limit
      jointLimitThreshold        -- joint angle limit threshold
      velocityLimit              -- joint velocity limit
      velocityLimitThreshold     -- joint velocity limit threshold
      accelerationLimit          -- joint acceleration limit
      accelerationLimitThreshold -- joint acceleration limit threshold
      torqueLimit                -- joint torque limit
      torqueLimitThreshold       -- joint torque limit threshold
    * Note: joint angle limits are given in radians.
   */
  // LinkJoint(char jointType, JointEffortType jointEffortType, uint jointIndex,
  //           uint sourceLinkIndex, uint destLinkIndex, const gtsam::Vector3 axis,
  //           const gtsam::Pose3 &pMc, const gtsam::Vector6 &screwAxis,
  //           double springCoefficient = 0, double dampCoefficient = 0,
  //           double jointLowerLimit = -M_PI, double jointUpperLimit = M_PI,
  //           double jointLimitThreshold = 0.0, double velocity_limit = 10000,
  //           double velocity_limit_threshold = 0.0,
  //           double acceleration_limit = 10000,
  //           double acceleration_limit_threshold = 0.0,
  //           double torque_limit = 10000, double torque_limit_threshold = 0.0)
  //     : jointType_(jointType),
  //       jointEffortType_(jointEffortType),
  //       jointIndex_(jointIndex),
  //       sourceLinkIndex_(sourceLinkIndex),
  //       destLinkIndex_(destLinkIndex),
  //       axis_(axis),
  //       pMc_(pMc),
  //       screwAxis_(screwAxis),
  //       springCoefficient_(springCoefficient),
  //       dampCoefficient_(dampCoefficient),
  //       jointLowerLimit_(jointLowerLimit),
  //       jointUpperLimit_(jointUpperLimit),
  //       jointLimitThreshold_(jointLimitThreshold),
  //       velocityLimit_(velocity_limit),
  //       velocityLimitThreshold_(velocity_limit_threshold),
  //       accelerationLimit_(acceleration_limit),
  //       accelerationLimitThreshold_(acceleration_limit_threshold),
  //       torqueLimit_(torque_limit),
  //       torqueLimitThreshold_(torque_limit_threshold) {}

  std::shared_ptr<LinkJoint> getSharedPtr(void) {
      // return std::make_shared<LinkBody>(this);
      return shared_from_this();
  }

  std::weak_ptr<LinkJoint> getWeakPtr(void) {
      return shared_from_this();
  }

  // Return joint name.
  std::string name() const { return name_; }

  /// Return jointType
  char jointType() const { return jointType_; }

  /// Return joint effort type
  JointEffortType jointEffortType() const { return jointEffortType_; }

  /// Return joint index
  uint jointIndex() const { return jointIndex_; }

  /// Return sourceLink index
  uint sourceLinkIndex() const { return sourceLinkIndex_; };

  /// Return destLink index
  uint destLinkIndex() const { return destLinkIndex_; };

  /// Return screw axis.
  const gtsam::Vector6 &screwAxis() const { return screwAxis_; }

  /// Return joint spring coefficient
  double springCoefficient() const { return springCoefficient_; }

  /// Return joint damping coefficient
  double dampCoefficient() const { return dampCoefficient_; }

  /// Return joint angle lower limit.
  double jointLowerLimit() const { return jointLowerLimit_; }

  /// Return joint angle upper limit.
  double jointUpperLimit() const { return jointUpperLimit_; }

  /// Return joint angle limit threshold.
  double jointLimitThreshold() const { return jointLimitThreshold_; }

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

  /// Return transfrom of dest link frame w.r.t. source link frame at rest
  gtsam::Pose3 pMc() const { return pMc_; }

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

  virtual ~LinkJoint() = default;
};

struct LinkJointParams {
  std::string name; // Name of this joint.

  LinkJoint::JointEffortType jointEffortType = LinkJoint::JointEffortType::Actuated;
  double springCoefficient = 0;
  double jointLimitThreshold = 0.0;
  double velocityLimitThreshold = 0.0;
  double accelerationLimit = 10000;
  double accelerationLimitThreshold = 0.0;
  double torqueLimitThreshold = 0.0;
};

}  // namespace robot


