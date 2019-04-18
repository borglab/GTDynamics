/**
 * @file  dh_link.h
 * @brief link taking Denavit-Harternberg parameters
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <Link.h>
#include <utils.h>

namespace manipulator {

/**
 * DH_Link is a link taking Denavit-Harternberg parameters
 */
class DH_Link : public Link {
 private:
  double theta_;
  double d_;
  double a_;
  double alpha_;

 public:
  /**
  * Construct from joint_type, mass, center_of_mass, inertia, and joint limits
  * Keyword arguments:
     d (m)                   -- link offset, i.e., distance between two joints
     theta (degrees)         -- angle between two joint frame x-axes (theta)
     a (m)                   -- link length. i.e., distance between two joints
     alpha (degrees)         -- link twist, i.e., angle between joint axes
     joint_type (char)       -- 'R': revolute,  'P' prismatic
     mass (double)            -- mass of link
     center_of_mass (Point3) -- center of mass location expressed in link frame
     inertia (vector)        -- principal inertias
     joint_lower_limit (degrees)       -- joint angle lower limit
     joint_upper_limit (degrees)       -- joint angle upper limit
     joint_limit_threshold (degrees)   -- joint angle limit threshold
     accelerationLimit -- joint acceleration limit
     accelerationLimitThreshold -- acceleration limit threshold
     torqueLimit -- joint torque limit
     torqueLimitThreshold -- torque limit threshold
  * Note: angles are given in degrees, but converted to radians internally.
  */
  DH_Link(double theta, double d, double a, double alpha, char joint_type,
          double mass, const gtsam::Point3 &center_of_mass,
          const gtsam::Vector3 &inertia, double joint_lower_limit = -180,
          double joint_upper_limit = 180, double joint_limit_threshold = 0.0,
          double velocity_limit = 10000, double velocity_limit_threshold = 0.0,
          double acceleration_limit = 10000,
          double acceleration_limit_threshold = 0.0,
          double torque_limit = 10000, double torque_limit_threshold = 0.0)
      : Link(joint_type, mass, center_of_mass, inertia,
             unit_twist(gtsam::Vector3(0, sin(radians(alpha)), cos(radians(alpha))),
                        gtsam::Vector3(-a, 0, 0) - center_of_mass.vector()),
             radians(joint_lower_limit), radians(joint_upper_limit),
             radians(joint_limit_threshold), velocity_limit,
             velocity_limit_threshold, acceleration_limit,
             acceleration_limit_threshold, torque_limit,
             torque_limit_threshold),
        theta_(radians(theta)),
        d_(d),
        a_(a),
        alpha_(radians(alpha)) {}

  /* Copy constructor */
  DH_Link(const DH_Link &dh_link)
      : Link(dh_link.jointType_, dh_link.mass(), dh_link.centerOfMass(),
             dh_link.inertia(), dh_link.screwAxis(), dh_link.jointLowerLimit(),
             dh_link.jointUpperLimit(), dh_link.jointLimitThreshold(),
             dh_link.velocityLimit(), dh_link.velocityLimitThreshold(),
             dh_link.accelerationLimit(), dh_link.accelerationLimitThreshold(),
             dh_link.torqueLimit(), dh_link.torqueLimitThreshold()),
        theta_(dh_link.theta_),
        d_(dh_link.d_),
        a_(dh_link.a_),
        alpha_(dh_link.alpha_) {}

  /** Calculate link transform
   * Keyword argument:
      q -- optional generalized joint angle (default 0)
   * Return Link transform.
  */
  gtsam::Pose3 A(double q = 0) const override {
    double theta = theta_;
    double d = d_;
    if (jointType_ == 'R') {
      theta = q;
    } else {
      d = q;
    }
    return gtsam::Pose3(gtsam::Rot3::Yaw(theta), gtsam::Point3(0, 0, d)) *
           gtsam::Pose3(gtsam::Rot3::Roll(alpha_), gtsam::Point3(a_, 0, 0));
  }

  /* return approximate length of link */
  double length() { return sqrt(d_ * d_ + a_ * a_); }
};
}  // namespace manipulator
