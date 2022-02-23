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
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

/**
 * @brief A 2-way factor to relate the parent and child links of a joint given
 * the joint coordinate as a measurement.
 */
class JointMeasurementFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 private:
  using This = JointMeasurementFactor;
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

  JointConstSharedPtr joint_;
  double measured_joint_coordinate_;

 public:
  /**
   * @brief Construct a new Joint Measurement Factor object
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
      boost::optional<gtsam::Matrix&> H_wTp = boost::none,
      boost::optional<gtsam::Matrix&> H_wTc = boost::none) const override {
    gtsam::Matrix6 H;
    gtsam::Pose3 wTc_hat =
        joint_->poseOf(joint_->child(), wTp, measured_joint_coordinate_, H_wTp);

    gtsam::Vector6 error = wTc.logmap(wTc_hat, H_wTc, H_wTp ? &H : 0);
    if (H_wTp) {
      *H_wTp = H * (*H_wTp);
    }
    return error;
  }

  /// print contents
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "JointMeasurementFactor(" << keyFormatter(key1()) << ","
              << keyFormatter(key2()) << ")\n";
    gtsam::traits<double>::Print(measured_joint_coordinate_, "  measured: ");
    this->noiseModel_->print("  noise model: ");
  }
};
}  // namespace gtdynamics
