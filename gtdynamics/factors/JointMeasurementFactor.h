/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  JointMeasurementFactor.h
 * @brief Factor relating a parent link and a child link given joint
 * measurement.
 * @author Varun Agrawal
 */

#pragma once
#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtdynamics {

/**
 * @brief A 2-way factor to relate the parent and child links of a joint given
 * the joint coordinate as a measurement.
 */
class JointMeasurementFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
 private:
  using This = JointMeasurementFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>;

  JointConstSharedPtr joint_;
  double measured_joint_coordinate_;

 public:
  // shorthand for a smart pointer to a factor
  using shared_ptr = std::shared_ptr<JointMeasurementFactor>;

  /**
   * @brief Construct a new Link Pose Factor object
   *
   * @param wTp_key Key to the parent link.
   * @param wTc_key Key to the child link.
   * @param model The noise model for this factor.
   * @param joint The joint connecting the links.
   * @param joint_coordinate The coordinates of the joint motion.
   * @param k The time index.
   */
  JointMeasurementFactor(gtsam::Key wTp_key, gtsam::Key wTc_key,
                         const gtsam::noiseModel::Base::shared_ptr& model,
                         const JointConstSharedPtr joint,
                         double joint_coordinate)
      : Base(model, wTp_key, wTc_key),
        joint_(joint),
        measured_joint_coordinate_(joint_coordinate) {}

  /**
   * @brief Convenience constructor
   *
   * @param model The noise model for this factor.
   * @param joint The joint connecting the links.
   * @param joint_coordinate The coordinates of the joint motion.
   * @param k The time index.
   */
  JointMeasurementFactor(const gtsam::noiseModel::Base::shared_ptr& model,
                         const JointConstSharedPtr joint,
                         double joint_coordinate, size_t k)
      : Base(model, PoseKey(joint->parent()->id(), k),
             PoseKey(joint->child()->id(), k)),
        joint_(joint),
        measured_joint_coordinate_(joint_coordinate) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& wTp, const gtsam::Pose3& wTc,
      gtsam::OptionalMatrixType H_wTp = nullptr,
      gtsam::OptionalMatrixType H_wTc = nullptr) const override {
    gtsam::Matrix6 H;
    gtsam::Pose3 wTc_hat =
        joint_->poseOf(joint_->child(), wTp, measured_joint_coordinate_, H_wTp);

    gtsam::Vector6 error = wTc.logmap(wTc_hat, H_wTc, H_wTp ? &H : 0);
    if (H_wTp) {
      *H_wTp = H * (*H_wTp);
    }
    return error;
  }

  /// Return measurement
  double measured() { return measured_joint_coordinate_; }

  /// print contents
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "JointMeasurementFactor(" << keyFormatter(key1()) << ","
              << keyFormatter(key2()) << ")\n";
    gtsam::traits<double>::Print(measured_joint_coordinate_, "  measured: ");
    this->noiseModel_->print("  noise model: ");
  }

  bool equals(const gtsam::NonlinearFactor& other,
              double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&other);
    return e != nullptr && Base::equals(*e, tol) &&
           joint_->equals(*e->joint_) &&
           measured_joint_coordinate_ == e->measured_joint_coordinate_;
  }
};
}  // namespace gtdynamics

namespace gtsam {

template <>
struct traits<gtdynamics::JointMeasurementFactor>
    : public Testable<gtdynamics::JointMeasurementFactor> {};

}  // namespace gtsam
