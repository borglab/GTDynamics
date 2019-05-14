/**
 * @file  urdf_link.h
 * @brief URDF parameter link class, inheritate from Link class
 * taking universal robot discription format parameters.
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <Link.h>
#include <utils.h>

#include <gtsam/geometry/Pose3.h>

namespace manipulator {

/**
 * DH_Link is a link taking URDF parameters
 */
class URDF_Link : public Link {
 private:
  gtsam::Pose3 origin_;
  gtsam::Vector3 axis_;

 public:
  /**
  * Construct from joint_type, mass, center_of_mass, and inertia
  * Keyword arguments:
         origin                      -- the x-y-z and roll-pitch-yaw
                                        coords of link frame w.r.t.
                                        the former link frame
         axis                        -- the x-y-z unit vector along
                                        the rotation axis in the link frame
         joint_type                  --'R': revolute,  'P' prismatic
         mass                        -- mass of link
         center_of_mass              -- the position and orientation
                                        of the center of mass frame w.r.t.
                                        link frame
         inertia                     -- principal inertias
         joint_lower_limit           -- joint angle lower limit
         joint_upper_limit           -- joint angle upper limit
         joint_limit_threshold       -- joint angle limit threshold
         velocityLimit               -- joint velocity limit
         velocityLimitThreshold      -- velocity limit threshold
         accelerationLimit           -- joint acceleration limit
         accelerationLimitThreshold  -- joint acceleration limit threshold
         torqueLimit                 -- joint torque limit
         torqueLimitThreshold        -- joint torque limit threshold
  * Note: angles are given in degrees, but converted to radians internally.
  */
  URDF_Link(const gtsam::Pose3 &origin, const gtsam::Vector3 &axis,
            char joint_type, double mass, const gtsam::Pose3 &center_of_mass,
            const gtsam::Matrix3 &inertia, double joint_lower_limit = -180,
            double joint_upper_limit = 180, double joint_limit_threshold = 0.0,
            double velocity_limit = 10000,
            double velocity_limit_threshold = 0.0,
            double acceleration_limit = 10000,
            double acceleration_limit_threshold = 0.0,
            double torque_limit = 10000, double torque_limit_threshold = 0.0)
      : Link(joint_type, mass, center_of_mass, inertia,
             unit_twist(center_of_mass.rotation().inverse() * axis,
                        center_of_mass.inverse().translation().vector()),
             radians(joint_lower_limit), radians(joint_upper_limit),
             radians(joint_limit_threshold), velocity_limit,
             velocity_limit_threshold, acceleration_limit,
             acceleration_limit_threshold, torque_limit,
             torque_limit_threshold),
        origin_(origin),
        axis_(axis) {}

  /* Copy constructor */
  URDF_Link(const URDF_Link &urdf_link)
      : Link(urdf_link.jointType_, urdf_link.mass(), urdf_link.centerOfMass(),
             urdf_link.inertia(), urdf_link.screwAxis(),
             urdf_link.jointLowerLimit(), urdf_link.jointUpperLimit(),
             urdf_link.jointLimitThreshold(), urdf_link.velocityLimit(),
             urdf_link.velocityLimitThreshold(), urdf_link.accelerationLimit(),
             urdf_link.accelerationLimitThreshold(), urdf_link.torqueLimit(),
             urdf_link.torqueLimitThreshold()),
        origin_(urdf_link.origin_),
        axis_(urdf_link.axis_) {}

  /** Calculate link transform
  Keyword argument:
      q -- optional generalized joint angle (default 0)
  Return Link transform.
  */
  gtsam::Pose3 A(double q = 0) const override {
    if (jointType_ == 'R') {
      return origin_ *
             gtsam::Pose3(gtsam::Rot3::Rodrigues(axis_ * q), gtsam::Point3());
    } else {
      return origin_ * gtsam::Pose3(gtsam::Rot3(), axis_ * q);
    }
  }

  double length() const { return origin_.x(); }
};
}  // namespace manipulator
