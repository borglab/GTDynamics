/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file WorldVelocityFactor.h
 * @brief Tie a COM body twist to its world-frame linear velocity.
 * @author GTDynamics contributors
 */
#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
namespace gtdynamics {
/** Residual R_world_COM * v_body - v_world, in m/s. Body twist is angular
 * first. Pose translation has zero Jacobian; the right-local rotation block
 * is -R*skew(v_body). Jacobians are 3x6, 3x6 and 3x3 respectively.
 */
class WorldVelocityFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector6,
                                      gtsam::Vector3> {
 public:
  /** Relate a COM pose, its body twist and world linear velocity. */
  WorldVelocityFactor(gtsam::Key poseKey, gtsam::Key twistKey,
                      gtsam::Key velocityKey,
                      const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector6, gtsam::Vector3>(
            model, poseKey, twistKey, velocityKey) {}
  /** Evaluate the frame-consistency residual and optional Jacobians. */
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Vector6& twist,
      const gtsam::Vector3& velocity, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr) const override {
    const gtsam::Matrix3 rotation = pose.rotation().matrix();
    if (H1) {
      *H1 = gtsam::Matrix36::Zero();
      H1->leftCols<3>() = -rotation * gtsam::skewSymmetric(twist.tail<3>());
    }
    if (H2) {
      *H2 = gtsam::Matrix36::Zero();
      H2->rightCols<3>() = rotation;
    }
    if (H3) *H3 = -gtsam::Matrix3::Identity();
    return rotation * twist.tail<3>() - velocity;
  }
  /** Copy this immutable factor. */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<WorldVelocityFactor>(*this);
  }
};
}  // namespace gtdynamics
