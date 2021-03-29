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

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/assign/list_of.hpp>

#include <memory>
#include <string>

namespace gtdynamics {

/**
 * PoseFactor is a three-way nonlinear factor between the previous link pose and
 * this link pose
 */
class PoseFactor : public gtsam::NoiseModelFactor {
 private:
  using This = PoseFactor;
  using Base = gtsam::NoiseModelFactor;
  // using boost::assign::cref_list_of;

  gtsam::Key wTp_key_, wTc_key_, q_key_;
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
  PoseFactor(const gtsam::SharedNoiseModel &cost_model,
             const JointConstSharedPtr &joint, int time)
      : Base(cost_model,
             boost::assign::cref_list_of<3>(
                 internal::PoseKey(joint->parent()->id(), time).key())(
                 internal::PoseKey(joint->child()->id(), time).key())(
                 internal::JointAngleKey(joint->id(), time).key())),
        wTp_key_(internal::PoseKey(joint->parent()->id(), time)),
        wTc_key_(internal::PoseKey(joint->child()->id(), time)),
        q_key_(internal::JointAngleKey(joint->id(), time)),
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
  PoseFactor(gtsam::Key wTp_key, gtsam::Key wTc_key, gtsam::Key q_key,
             const gtsam::noiseModel::Base::shared_ptr &cost_model,
             JointConstSharedPtr joint)
      : Base(cost_model, boost::assign::cref_list_of<3>(wTp_key)(wTc_key)(q_key)),
        wTp_key_(wTp_key),
        wTc_key_(wTc_key),
        q_key_(q_key),
        t_(wTp_key & 0xFFFFFFFF),  // hack
        joint_(joint) {}

  virtual ~PoseFactor() {}


  /**
   * Evaluate link pose errors
   * @param wTp previous (parent) link CoM pose
   * @param wTc this (child) link CoM pose
   * @param q joint angle
   */
  gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                boost::optional<std::vector<gtsam::Matrix> &>
                                    H = boost::none) const override {
    const gtsam::Pose3 &wTp = x.at<gtsam::Pose3>(wTp_key_),
                       &wTc = x.at<gtsam::Pose3>(wTc_key_);
    // TODO(frank): logmap derivative is close to identity when error is small
    if (!H) return wTc.logmap(joint_->poseOf(joint_->child(), wTp, x, t_));

    gtsam::Matrix wTc_hat_H_wTp, H_wTc_hat, wTc_hat_H_q;
    auto wTc_hat = joint_->poseOf(joint_->child(), wTp, x, t_,
                                  wTc_hat_H_wTp, wTc_hat_H_q);
    gtsam::Vector6 error = wTc.logmap(wTc_hat, (*H)[1], H_wTc_hat);
    (*H)[0] = H_wTc_hat * wTc_hat_H_wTp;
    (*H)[2] = H_wTc_hat * wTc_hat_H_q;
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
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
