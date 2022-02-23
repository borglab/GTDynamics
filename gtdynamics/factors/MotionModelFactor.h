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
#include <gtsam/slam/BetweenFactor.h>

namespace gtdynamics {

/**
 * @brief A 4-way factor which evaluates the motion model of a link in the
 * robot's base frame.
 */
class MotionModelFactor : public gtsam::BetweenFactor<gtsam::Pose3> {
 private:
  using This = MotionModelFactor;
  using Base = gtsam::BetweenFactor<gtsam::Pose3>;

 public:
  // shorthand for a smart pointer to a factor
  using shared_ptr = boost::shared_ptr<MotionModelFactor>;

  /**
   * @brief Construct a new motion model factor.
   *
   * @param wTp_key Key to the parent link.
   * @param wTc_key Key to the child link.
   * @param model The noise model for this factor.
   * @param joint The joint connecting the links.
   * @param joint_coordinate The coordinates of the joint motion.
   * @param k The time index.
   */
  MotionModelFactor(gtsam::Key wTl1_key, gtsam::Key wTl2_key,
                    const gtsam::noiseModel::Base::shared_ptr& model,
                    const gtsam::Pose3& motion_model_mean)
      : Base(wTl1_key, wTl2_key, motion_model_mean, model) {}

  /**
   * @brief Construct a new motion model factor.
   *
   * @param wTp_key Key to the parent link.
   * @param wTc_key Key to the child link.
   * @param model The noise model for this factor.
   * @param joint The joint connecting the links.
   * @param joint_coordinate The coordinates of the joint motion.
   * @param k The time index.
   */
  MotionModelFactor(gtsam::Key wTb1_key, gtsam::Key wTl1_key,
                    gtsam::Key wTb2_key, gtsam::Key wTl2_key,
                    const gtsam::noiseModel::Base::shared_ptr& model,
                    const gtsam::Pose3& motion_model_mean)
      : Base(wTl1_key, wTl2_key, motion_model_mean, model) {
    // Also add the extra wTb keys
    keys_.push_back(wTb1_key);
    keys_.push_back(wTb2_key);
  }

  /// Evaluate error when only
  using Base::evaluateError;

  gtsam::Vector evaluateError(
      const gtsam::Pose3& wTb1, const gtsam::Pose3& wTl1,
      const gtsam::Pose3& wTb2, const gtsam::Pose3& wTl2,
      boost::optional<gtsam::Matrix&> H_wTb1 = boost::none,
      boost::optional<gtsam::Matrix&> H_wTl1 = boost::none,
      boost::optional<gtsam::Matrix&> H_wTb2 = boost::none,
      boost::optional<gtsam::Matrix&> H_wTl2 = boost::none) const {
    gtsam::Matrix6 H_b1, H_b2, H_l1, H_l2;
    gtsam::Pose3 bTl1 =
        wTb1.between(wTl1, H_wTb1 ? &H_b1 : nullptr, H_wTl1 ? &H_l1 : nullptr);
    gtsam::Pose3 bTl2 =
        wTb2.between(wTl2, H_wTb2 ? &H_b2 : nullptr, H_wTl2 ? &H_l2 : nullptr);
    gtsam::Matrix6 H1, H2;
    gtsam::Vector6 error = Base::evaluateError(bTl1, bTl2, H_wTl1, H_wTl2);
    // gtsam::Vector6 error = Base::evaluateError(bTl1, bTl2, &H1, &H2);

    if (H_wTb1) {
      *H_wTb1 = (*H_wTl1) * H_b1;
    }
    if (H_wTl1) {
      *H_wTl1 = (*H_wTl1) * H_l1;
    }
    if (H_wTb2) {
      *H_wTb2 = (*H_wTl2) * H_b2;
    }
    if (H_wTl2) {
      *H_wTl2 = (*H_wTl2) * H_l2;
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
