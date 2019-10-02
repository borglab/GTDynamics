namespace manipulator {

#include <cpp/Link.h>

class Link {
  Link(char joint_type, double mass, const gtsam::Pose3 &center_of_mass,
       const gtsam::Matrix3 &inertia, const gtsam::Vector6 &screwAxis,
       double joint_lower_limit = -M_PI, double joint_upper_limit = M_PI,
       double joint_limit_threshold = 0.0, double velocity_limit = 10000,
       double velocity_limit_threshold = 0.0, double acceleration_limit = 10000,
       double acceleration_limit_threshold = 0.0, double torque_limit = 10000,
       double torque_limit_threshold = 0.0)
      : jointType_(joint_type),
        mass_(mass),
        centerOfMass_(center_of_mass),
        inertia_(inertia),
        screwAxis_(screwAxis),
        jointLowerLimit_(joint_lower_limit),
        jointUpperLimit_(joint_upper_limit),
        jointLimitThreshold_(joint_limit_threshold),
        velocityLimit_(velocity_limit),
        velocityLimitThreshold_(velocity_limit_threshold),
        accelerationLimit_(acceleration_limit),
        accelerationLimitThreshold_(acceleration_limit_threshold),
        torqueLimit_(torque_limit),
        torqueLimitThreshold_(torque_limit_threshold);

  Link(char joint_type, double mass, const gtsam::Point3 &center_of_mass,
       const gtsam::Matrix3 &inertia, const gtsam::Vector6 &screwAxis,
       double joint_lower_limit = -M_PI, double joint_upper_limit = M_PI,
       double joint_limit_threshold = 0.0, double velocity_limit = 10000,
       double velocity_limit_threshold = 0.0, double acceleration_limit = 10000,
       double acceleration_limit_threshold = 0.0, double torque_limit = 10000,
       double torque_limit_threshold = 0.0)
      : jointType_(joint_type),
        mass_(mass),
        centerOfMass_(gtsam::Pose3(gtsam::Rot3(), center_of_mass)),
        inertia_(inertia),
        screwAxis_(screwAxis),
        jointLowerLimit_(joint_lower_limit),
        jointUpperLimit_(joint_upper_limit),
        jointLimitThreshold_(joint_limit_threshold),
        velocityLimit_(velocity_limit),
        velocityLimitThreshold_(velocity_limit_threshold),
        accelerationLimit_(acceleration_limit),
        accelerationLimitThreshold_(acceleration_limit_threshold),
        torqueLimit_(torque_limit),
        torqueLimitThreshold_(torque_limit_threshold);

  const gtsam::Vector6 &screwAxis() const;
  double mass() const;
};
