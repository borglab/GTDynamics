/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtdynamics/utils/PoseOnLink.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtdynamics {

/** Temporal material-frame displacement: Log((wTcom1*comTcontact)^-1 *
 * (wTcom2*comTcontact)). Noise order is rotation (radians), translation
 * (metres), expressed in the first contact frame. This is a displacement, not a
 * velocity. Analytic Jacobians are 6x6 in Pose3 right-local coordinates.
 */
class ContactPoseEqualityFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
  gtsam::Pose3 comTcontact_;

 public:
  ContactPoseEqualityFactor(const PoseOnLink& contact,
                            const gtsam::SharedNoiseModel& model, size_t first,
                            size_t second)
      : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>(
            model, PoseKey(contact.link->id(), first),
            PoseKey(contact.link->id(), second)),
        comTcontact_(contact.comTcontact) {
    if (!model || model->dim() != 6)
      throw std::invalid_argument("ContactPoseEqualityFactor needs 6D noise");
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ContactPoseEqualityFactor>(*this);
  }
  gtsam::Vector evaluateError(
      const gtsam::Pose3& first, const gtsam::Pose3& second,
      gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr) const override {
    gtsam::Matrix6 A, B, C, D, L;
    const auto a = first.compose(comTcontact_, A);
    const auto b = second.compose(comTcontact_, B);
    const auto delta = a.between(b, C, D);
    const auto error = gtsam::Pose3::Logmap(delta, L);
    if (H1) *H1 = L * C * A;
    if (H2) *H2 = L * D * B;
    return error;
  }
};

/** Align contact +Z to a unit world normal, without fixing yaw or translation.
 * The 3D unit-vector difference has rank two; sigma is a small-angle scale in
 * radians. Its analytic Jacobian is 3x6. Height is a separate objective.
 */
class ContactNormalFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  gtsam::Vector3 comNormal_, worldNormal_;

 public:
  ContactNormalFactor(gtsam::Key key, const gtsam::Pose3& comTcontact,
                      const gtsam::Vector3& worldNormal,
                      const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactorN<gtsam::Pose3>(model, key),
        comNormal_(comTcontact.rotation().rotate(gtsam::Vector3::UnitZ())),
        worldNormal_(worldNormal.normalized()) {
    if (!worldNormal.allFinite() || worldNormal.norm() == 0 || !model ||
        model->dim() != 3)
      throw std::invalid_argument(
          "ContactNormalFactor needs a normal and 3D noise");
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ContactNormalFactor>(*this);
  }
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      gtsam::OptionalMatrixType H = nullptr) const override {
    gtsam::Matrix3 rotation;
    const auto normal = pose.rotation().rotate(comNormal_, rotation);
    if (H) {
      *H = gtsam::Matrix36::Zero();
      H->leftCols<3>() = rotation;
    }
    return normal - worldNormal_;
  }
};
}  // namespace gtdynamics
