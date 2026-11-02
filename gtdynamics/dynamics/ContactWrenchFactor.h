/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <stdexcept>

namespace gtdynamics {

/** Map a torque-first wrench from a material contact frame to a link COM.
 * comTcontact maps contact coordinates into COM coordinates. Wrenches are
 * [torque (N m), force (N)], acting ON the robot. The result is
 * [R*torque + p cross R*force, R*force], with p the COM-to-contact displacement
 * expressed in the COM frame. It preserves wrench/twist virtual power.
 */
inline gtsam::Matrix6 contactWrenchTransform(const gtsam::Pose3& comTcontact) {
  return comTcontact.inverse().AdjointMap().transpose();
}

/** Express a contact-frame wrench at the link COM, retaining all six entries.
 * This changes both frame orientation and moment reference point. It applies
 * to arbitrary external wrenches, without assuming a point contact or zero
 * moment at the sole.
 */
inline gtsam::Vector6 contactWrenchToCom(const gtsam::Pose3& comTcontact,
                                         const gtsam::Vector6& contactWrench) {
  return contactWrenchTransform(comTcontact) * contactWrench;
}

/** Affine consistency between a sole wrench and an external link-COM wrench.
 * The six-dimensional residual is comWrench - T*contactWrench, ordered torque
 * (N m) then force (N); use corresponding rotational and force noise scales.
 * comTcontact is fixed. Analytic Jacobians with respect to the sole and COM
 * Vector6 variables are the constant matrices -T and I. A constrained noise
 * model makes this an exact mapping for fixed-kinematics dynamics inference.
 */
class ContactWrenchFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector6, gtsam::Vector6> {
  gtsam::Matrix6 transform_;

 public:
  ContactWrenchFactor(gtsam::Key contactWrenchKey, gtsam::Key comWrenchKey,
                      const gtsam::Pose3& comTcontact,
                      const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactorN<gtsam::Vector6, gtsam::Vector6>(
            model, contactWrenchKey, comWrenchKey),
        transform_(contactWrenchTransform(comTcontact)) {
    if (!model || model->dim() != 6)
      throw std::invalid_argument("ContactWrenchFactor needs 6D noise");
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ContactWrenchFactor>(*this);
  }

  gtsam::Vector evaluateError(
      const gtsam::Vector6& contactWrench, const gtsam::Vector6& comWrench,
      gtsam::OptionalMatrixType HContact = nullptr,
      gtsam::OptionalMatrixType HCom = nullptr) const override {
    if (HContact) *HContact = -transform_;
    if (HCom) *HCom = gtsam::Matrix6::Identity();
    return comWrench - transform_ * contactWrench;
  }
};

}  // namespace gtdynamics
