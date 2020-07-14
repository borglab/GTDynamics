/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PoseFactor.h
 * @brief Forward kinematics factor.
 * @Author: Frank Dellaert and Mandy Xie
 */

#ifndef GTDYNAMICS_FACTORS_POSEFACTOR_H_
#define GTDYNAMICS_FACTORS_POSEFACTOR_H_

#include <memory>
#include <string>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "gtdynamics/utils/utils.h"
#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

/** PoseFactor is a three-way nonlinear factor between the previous link pose
 * and this link pose*/
class PoseFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3,
                                      typename JointTyped::AngleType> {
 private:
  typedef PoseFactor This;
  typedef gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3,
                                   typename JointTyped::AngleType>
      Base;
  typedef typename JointTyped::AngleType JointAngleType;

  std::shared_ptr<const JointTyped> joint_;

 public:
  /** Create single factor relating this link's pose (COM) with previous one.
      Keyword arguments:
          joint -- the joint connecting the two poses
   */
  PoseFactor(gtsam::Key pose_key_i, gtsam::Key pose_key_j, gtsam::Key q_key,
             const gtsam::noiseModel::Base::shared_ptr &cost_model,
             JointConstSharedPtr joint)
      : Base(cost_model, pose_key_i, pose_key_j, q_key),
        joint_(std::static_pointer_cast<const JointTyped>(joint)) {}

  virtual ~PoseFactor() {}

  /** evaluate link pose errors
      Keyword argument:
          pose_i         -- previous link pose
          pose_j         -- this link pose
          q              -- joint coordination
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose_i, const gtsam::Pose3 &pose_j,
      const JointAngleType &q,
      boost::optional<gtsam::Matrix &> H_pose_i = boost::none,
      boost::optional<gtsam::Matrix &> H_pose_j = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    gtsam::Pose3 pose_j_hat;
    if (H_q) {
      gtsam::Matrix6 pose_j_hat_H_iTjhat;
      gtsam::Matrix iTjhat_H_q;
      gtsam::Pose3 iTjhat =
          joint_->transformTo(joint_->parentLink(), q, iTjhat_H_q);
      pose_j_hat = pose_i.compose(iTjhat, H_pose_i, pose_j_hat_H_iTjhat);
      *H_q = pose_j_hat_H_iTjhat * iTjhat_H_q;
    } else {
      gtsam::Pose3 iTjhat = joint_->transformTo(joint_->parentLink(), q);
      pose_j_hat = pose_i.compose(iTjhat, H_pose_i);
    }
    gtsam::Vector6 error;
    if (!(H_q || H_pose_i)) {
      error = pose_j.logmap(pose_j_hat, H_pose_j);
    } else {
      gtsam::Matrix6 H_pose_j_hat;
      error = pose_j.logmap(pose_j_hat, H_pose_j, H_pose_j_hat);

      if (H_pose_i)
        *H_pose_i = H_pose_j_hat * (*H_pose_i);
      if (H_q)
        *H_q = H_pose_j_hat * (*H_q);
    }

    return error;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "pose factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_POSEFACTOR_H_
