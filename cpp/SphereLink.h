/**
 * @file  SphereLink.h
 * @brief model link as spheres for coliision check
 * @Author: Mandy Xie and Frank Dellaert
 */
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace manipulator {

/** SphereLink is a class simplies a robotic arm by representing it as a set of
 * spheres*/
class SphereLink {
 private:
  double radius_;  // sphere radius
  std::vector<gtsam::Point3>
      sphere_centers_;  // sphere center position, expressed in base frame;

 public:
  /** constructor for sphere robot model
   * key argument
   * radius         -- sphere radius
   * sphere_centers -- sphere center positions, expressed in
   *                   link COM frame;
   */
  SphereLink(double radius, std::vector<gtsam::Point3> &sphere_centers)
      : radius_(radius), sphere_centers_(sphere_centers) {}

  ~SphereLink() {}

  /** get sphere centers expressed in base frame
   * key argument
   * index  -- sphere index
   * com_pose  -- link center of mass frame pose in base frame
   */
  gtsam::Point3 sphereCenter(
      size_t index, const gtsam::Pose3 &com_pose,
      gtsam::OptionalJacobian<3, 6> H = boost::none) const {
    return com_pose.transform_from(sphere_centers_[index], H);
  }

  // return the number of spheres to model this link
  size_t num_sphere() const { return sphere_centers_.size(); }

  // return radius of sphere
  double radius() const { return radius_; }
};
}  // namespace manipulator
