class gtsam::Vector6;
class gtsam::Matrix3;

#include <Link.h>

namespace manipulator {

class Link {
  Link(char joint_type, double mass, const gtsam::Pose3 &center_of_mass,
       const gtsam::Matrix3 &inertia, const gtsam::Vector6 &screwAxis,
       double joint_lower_limit, double joint_upper_limit,
       double joint_limit_threshold, double velocity_limit,
       double velocity_limit_threshold, double acceleration_limit,
       double acceleration_limit_threshold, double torque_limit,
       double torque_limit_threshold);

  Link(char joint_type, double mass, const gtsam::Point3 &center_of_mass,
       const gtsam::Matrix3 &inertia, const gtsam::Vector6 &screwAxis,
       double joint_lower_limit, double joint_upper_limit,
       double joint_limit_threshold, double velocity_limit,
       double velocity_limit_threshold, double acceleration_limit,
       double acceleration_limit_threshold, double torque_limit,
       double torque_limit_threshold);

  gtsam::Vector6 screwAxis() const;
  gtsam::Matrix3 inertia() const;
  double mass() const;
};
}