/**
 * @file  ObstacleSDFFactor.h
 * @brief obstacle signed distance field factor used for collision check
 * @Author: Mandy Xie and Frank Dellaert
 */

#pragma once

#include <ObstacleSDFFactor.h>
#include <SignedDistanceField.h>
#include <SphereLink.h>

namespace manipulator {

/** hinge loss obstacle cost function */
double hingeLossObstacleCost(
    const Point3 &point, const SignedDistanceField &sdf, double eps,
    OptionalJacobian<1, 3> H_point = boost::none) {
  Vector3 field_gradient;
  double dist_signed;
  try {
    dist_signed = sdf.getSignedDistance(point, field_gradient);
  } catch (std::runtime_error) {
    if (H_point) *H_point = Matrix13::Zero();
    return 0.0;
  }

  if (dist_signed > eps) {
    // faraway no error
    if (H_point) *H_point = Matrix13::Zero();
    return 0.0;

  } else {
    // outside but < eps or inside object
    if (H_point) *H_point = -field_gradient.transpose();
    return eps - dist_signed;
  }
}

Vector ObstacleSDFFactor::evaluateError(
    const Pose3 &pose,
    boost::optional<Matrix &> H_pose) const override {
  // if Jacobians used, initialize as zeros
  auto num = sphere_link_.num_sphere();
  auto radius = sphere_link_.radius();
  if (H_pose) *H_pose = Matrix::Zero(num, 6);
  // allocate cost vector
  Vector error = Vector::Zero(num);

  for (size_t i = 0; i < num; ++i) {
    if (H_pose) {
      Matrix36 H_point;
      auto center = sphere_link_.sphereCenter(i, pose, H_point);
      Matrix13 H_hingeLoss;
      error[i] =
          hingeLossObstacleCost(center, sdf_, epsilon_ + radius, H_hingeLoss);
      H_pose->row(i) = H_hingeLoss * H_point;
    } else {
      auto center = sphere_link_.sphereCenter(i, pose);
      error[i] = hingeLossObstacleCost(center, sdf_, epsilon_ + radius);
    }
  }
  return error;
}
