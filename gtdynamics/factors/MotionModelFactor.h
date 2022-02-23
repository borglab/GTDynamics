/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MotionModelFactor.h
 * @brief Factor to enforce a motion model of a link.
 * @author Varun Agrawal
 */

#pragma once

#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtdynamics {

/**
 * @brief A 4-way factor which evaluates the motion model of a link in the
 * robot's base frame.
 */
class MotionModelFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3,
                                      gtsam::Pose3> {
 private:
  using This = MotionModelFactor;
  using Base = gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3,
                                        gtsam::Pose3, gtsam::Pose3>;

  gtsam::Pose3 motion_model_;

 public:
  // shorthand for a smart pointer to a factor
  using shared_ptr = boost::shared_ptr<MotionModelFactor>;

  /**
   * @brief Construct a new Motion Model Factor object
   *
   * @param wTb1_key
   * @param wTl1_key
   * @param wTb2_key
   * @param wTl2_key
   * @param model
   * @param motion_model_mean
   */
  MotionModelFactor(gtsam::Key wTb1_key, gtsam::Key wTl1_key,
                    gtsam::Key wTb2_key, gtsam::Key wTl2_key,
                    const gtsam::noiseModel::Base::shared_ptr& model,
                    const gtsam::Pose3& motion_model_mean)
      : Base(model, wTb1_key, wTl1_key, wTb2_key, wTl2_key),
        motion_model_(motion_model_mean) {}

  MotionModelFactor(const gtsam::noiseModel::Base::shared_ptr& model,
                    const LinkConstSharedPtr base,
                    const LinkConstSharedPtr link,
                    const gtsam::Pose3& motion_model_mean, size_t t0, size_t t1)
      : Base(model, PoseKey(base->id(), t0), PoseKey(link->id(), t0),
             PoseKey(base->id(), t1), PoseKey(link->id(), t1)),
        motion_model_(motion_model_mean) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& wTb1, const gtsam::Pose3& wTl1,
      const gtsam::Pose3& wTb2, const gtsam::Pose3& wTl2,
      boost::optional<gtsam::Matrix&> H_wTb1 = boost::none,
      boost::optional<gtsam::Matrix&> H_wTl1 = boost::none,
      boost::optional<gtsam::Matrix&> H_wTb2 = boost::none,
      boost::optional<gtsam::Matrix&> H_wTl2 = boost::none) const override {
    // Compute link poses in the body frame
    gtsam::Pose3 bTl1 = wTb1.between(wTl1, H_wTb1, H_wTl1);
    gtsam::Pose3 bTl2 = wTb2.between(wTl2, H_wTb2, H_wTl2);

    gtsam::Matrix6 H1, H2;
    gtsam::Pose3 l1Tl2 = bTl1.between(bTl2, (H_wTb1 || H_wTl1) ? &H1 : nullptr,
                                      (H_wTb2 || H_wTl2) ? &H2 : nullptr);

    gtsam::Vector6 error =
        gtsam::traits<gtsam::Pose3>::Local(motion_model_, l1Tl2);

    if (H_wTb1) {
      *H_wTb1 = H1 * (*H_wTb1);
    }
    if (H_wTl1) {
      *H_wTl1 = H1 * (*H_wTl1);
    }
    if (H_wTb2) {
      *H_wTb2 = H2 * (*H_wTb2);
    }
    if (H_wTl2) {
      *H_wTl2 = H2 * (*H_wTl2);
    }
    return error;
  }

  /// print contents
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "MotionModelFactor(" << keyFormatter(key1()) << ","
              << keyFormatter(key2()) << ")\n";
    this->noiseModel_->print("  noise model: ");
  }
};
}  // namespace gtdynamics
