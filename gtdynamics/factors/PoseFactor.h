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
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <string>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

/**
 * \deprecated NOTE This factor has been deprecated and will be removed in the
 * future.
 *
 * PoseFactor is a three-way nonlinear factor between a joint's parent link
 * pose, child link pose, and the joint angle relating the two poses.
 *
 * Given the joint model, this factor optimizes for the underlying joint axis
 * and the corresponding poses of the parent and child links.
 */
class TempPoseFactor : public gtsam::NoiseModelFactor {
 private:
  using This = TempPoseFactor;
  using Base = gtsam::NoiseModelFactor;

  int t_;
  JointConstSharedPtr joint_;

 public:
  /**
   * Create single factor relating this link's pose (COM) with previous one.
   *
   * @param cost_model The noise model for this factor.
   * @param joint The joint connecting the two poses.
   * @param time The timestep at which this factor is defined.
   */
  TempPoseFactor(const gtsam::SharedNoiseModel &cost_model,
                 const JointConstSharedPtr &joint, int time)
      : Base(cost_model,
             cref_list_of<3>(
                 internal::PoseKey(joint->parent()->id(), time).key())(
                 internal::PoseKey(joint->child()->id(), time).key())(
                 internal::JointAngleKey(joint->id(), time).key())),
        t_(time),
        joint_(joint) {}

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
  TempPoseFactor(DynamicsSymbol wTp_key, DynamicsSymbol wTc_key,
                 DynamicsSymbol q_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 JointConstSharedPtr joint)
      : Base(cost_model,
             cref_list_of<3>(wTp_key.key())(wTc_key.key())(q_key.key())),
        t_(wTp_key.time()),
        joint_(joint) {}

  virtual ~TempPoseFactor() {}

  /**
   * Evaluate link pose errors
   * @param x Values containing:
   *  wTp - previous (parent) link CoM pose
   *  wTc - this (child) link CoM pose
   *  q - joint angle
   */
  gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                boost::optional<std::vector<gtsam::Matrix> &>
                                    H = boost::none) const override {
    if (!this->active(x)) return gtsam::Vector6::Zero();

    const gtsam::Pose3 wTp = x.at<gtsam::Pose3>(keys_[0]),
                       wTc = x.at<gtsam::Pose3>(keys_[1]);
    // TODO(frank): logmap derivative is close to identity when error is small
    gtsam::Matrix6 wTc_hat_H_wTp, H_wTc_hat, H_wTc;
    // TODO(gerry): figure out how to make this work better for dynamic matrices
    gtsam::Matrix wTc_hat_H_q;
    boost::optional<gtsam::Matrix &> wTc_hat_H_q_ref;
    if (H) wTc_hat_H_q_ref = wTc_hat_H_q;

    auto wTc_hat =
        joint_->poseOf(joint_->child(), wTp, JointAngle(x, joint_->id(), t_),
                       H ? &wTc_hat_H_wTp : 0, wTc_hat_H_q_ref);
    gtsam::Vector6 error =
        wTc.logmap(wTc_hat, H ? &H_wTc : 0, H ? &H_wTc_hat : 0);
    if (H) {
      (*H)[0] = H_wTc_hat * wTc_hat_H_wTp;
      (*H)[1] = H_wTc;
      (*H)[2] = H_wTc_hat * wTc_hat_H_q;
    }
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
    std::cout << (s.empty() ? s : s + " ") << "Pose Factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};

/**
 * Create single factor relating this link's pose (COM) with previous one.
 * Note: this function is provided for BW compatibility only, and will in time
 * be replaced with EqualityConstraint.
 *
 * PoseFactor is a three-way nonlinear factor between a joint's parent link
 * pose, child link pose, and the joint angle relating the two poses.
 *
 * Given the joint model, this factor optimizes for the underlying joint axis
 * and the corresponding poses of the parent and child links.
 *
 * @param cost_model The noise model for this factor.
 * @param joint The joint connecting the two poses.
 * @param time The timestep at which this factor is defined.
 */
inline gtsam::NoiseModelFactor::shared_ptr PoseFactor(
    const gtsam::SharedNoiseModel &cost_model, const JointConstSharedPtr &joint,
    int time) {
  return boost::make_shared<gtsam::ExpressionFactor<gtsam::Vector6>>(
      cost_model, gtsam::Vector6::Zero(), joint->poseConstraint(time));
}

  /**
   * Create single factor relating this link's pose (COM) with previous one.
   * Note: this function is provided for BW compatibility only, and will in time
   * be replaced with EqualityConstraint.
   * 
   * @param wTp_key Key for parent link's CoM pose in world frame.
   * @param wTc_key Key for child link's CoM pose in world frame.
   * @param q_key Key for joint value.
   * @param cost_model The noise model for this factor.
   * @param joint The joint connecting the two poses
   */
inline gtsam::NoiseModelFactor::shared_ptr PoseFactor(
    DynamicsSymbol wTp_key, DynamicsSymbol wTc_key, DynamicsSymbol q_key,
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    JointConstSharedPtr joint) {
  return boost::make_shared<gtsam::ExpressionFactor<gtsam::Vector6>>(
      cost_model, gtsam::Vector6::Zero(),
      joint->poseConstraint(wTp_key.time()));
}

}  // namespace gtdynamics
