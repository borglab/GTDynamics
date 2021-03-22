/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PoseFactor.h
 * @brief Forward kinematics factor.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <string>

#include "gtdynamics/universal_robot/JointTyped.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

/**
 * PoseFactor is a three-way nonlinear factor between the previous link pose and
 * this link pose
 */
class PoseFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3,
                                      typename JointTyped::JointCoordinate> {
 private:
  using JointCoordinate = typename JointTyped::JointCoordinate;
  using This = PoseFactor;
  using Base =
      gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, JointCoordinate>;
  enum { N = JointTyped::N };

  boost::shared_ptr<const JointTyped> joint_;

 public:
  /**
   * Create single factor relating this link's pose (COM) with previous one.
   *
   * @param cost_model The noise model for this factor.
   * @param joint The joint connecting the two poses.
   * @param time The timestep at which this factor is defined.
   */
  PoseFactor(const gtsam::SharedNoiseModel &cost_model,
             const JointConstSharedPtr &joint, int time)
      : Base(cost_model, internal::PoseKey(joint->parent()->id(), time),
             internal::PoseKey(joint->child()->id(), time),
             internal::JointAngleKey(joint->id(), time)),
        joint_(boost::static_pointer_cast<const JointTyped>(joint)) {}

  /**
   * Create single factor relating this link's pose (COM) with previous one.
   *
   * Please use the joint based constructor above if possible.
   *
   * @param wTp_key Key for parent link's CoM pose in world frame.
   * @param wTc_key Key for child link's CoM pose in world frame.
   * @param q_key Key for joint value.
   * @param cost_model The noise model for this factor.
   * @param joint The joint connecting the two poses
   */
  PoseFactor(gtsam::Key wTp_key, gtsam::Key wTc_key, gtsam::Key q_key,
             const gtsam::noiseModel::Base::shared_ptr &cost_model,
             JointConstSharedPtr joint)
      : Base(cost_model, wTp_key, wTc_key, q_key),
        joint_(boost::static_pointer_cast<const JointTyped>(joint)) {}

  virtual ~PoseFactor() {}

  /**
   * Evaluate link pose errors
   * @param wTp previous (parent) link CoM pose
   * @param wTc this (child) link CoM pose
   * @param q joint angle
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wTp, const gtsam::Pose3 &wTc,
      const JointCoordinate &q,
      boost::optional<gtsam::Matrix &> H_wTp = boost::none,
      boost::optional<gtsam::Matrix &> H_wTc = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    Eigen::Matrix<double, 6, 6> wTc_hat_H_wTp, H_wTc_hat;
    Eigen::Matrix<double, 6, N> wTc_hat_H_q;
    auto wTc_hat =
        joint_->poseOf(joint_->child(), wTp, q, H_wTp ? &wTc_hat_H_wTp : 0,
                       H_q ? &wTc_hat_H_q : 0);
    // TODO(frank): logmap derivative is close to identity when error is small
    gtsam::Vector6 error =
        wTc.logmap(wTc_hat, H_wTc, (H_q || H_wTp) ? &H_wTc_hat : 0);
    if (H_wTp) *H_wTp = H_wTc_hat * wTc_hat_H_wTp;
    if (H_q) *H_q = H_wTc_hat * wTc_hat_H_q;
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "pose factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
