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

#include "gtdynamics/universal_robot/JointTyped.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <string>

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
  PoseFactor(gtsam::Key wTp_key, gtsam::Key wTc_key, gtsam::Key q_key,
             const gtsam::noiseModel::Base::shared_ptr &cost_model,
             JointConstSharedPtr joint)
      : Base(cost_model, wTp_key, wTc_key, q_key),
        joint_(std::static_pointer_cast<const JointTyped>(joint)) {}

  virtual ~PoseFactor() {}

  /** evaluate link pose errors
      Keyword argument:
          wTp         -- previous (parent) link pose
          wTc         -- this (child) link pose
          q           -- joint angle
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wTp, const gtsam::Pose3 &wTc,
      const JointAngleType &q,
      boost::optional<gtsam::Matrix &> H_wTp = boost::none,
      boost::optional<gtsam::Matrix &> H_wTc = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    gtsam::Pose3 wTc_hat;
    if (H_q) {
      gtsam::Matrix6 wTc_hat_H_pTjhat;
      gtsam::Matrix pTc_hat_H_q;
      gtsam::Pose3 pTc_hat =
          joint_->transformTo(joint_->parentLink(), q, pTc_hat_H_q);
      wTc_hat = wTp.compose(pTc_hat, H_wTp, wTc_hat_H_pTjhat);
      *H_q = wTc_hat_H_pTjhat * pTc_hat_H_q;
    } else {
      gtsam::Pose3 pTc_hat = joint_->transformTo(joint_->parentLink(), q);
      wTc_hat = wTp.compose(pTc_hat, H_wTp);
    }
    gtsam::Vector6 error;
    if (!(H_q || H_wTp)) {
      error = wTc.logmap(wTc_hat, H_wTc);
    } else {
      gtsam::Matrix6 H_wTc_hat;
      error = wTc.logmap(wTc_hat, H_wTc, H_wTc_hat);

      if (H_wTp)
        *H_wTp = H_wTc_hat * (*H_wTp);
      if (H_q)
        *H_q = H_wTc_hat * (*H_q);
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
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_POSEFACTOR_H_
