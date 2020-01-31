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

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <utils.h>

#include <iostream>
#include <string>

namespace manipulator {

/** PoseFunctor is functor predicting link's pose (COM) with previous one*/
class PoseFunctor {
 private:
  gtsam::Pose3 iMj_;
  gtsam::Vector6 screw_axis_;

 public:
  /** Create functor predicting this link's pose (COM) with previous one.
      Keyword arguments:
          jMi        -- previous COM frame, expressed in this link's COM frame,
     at rest configuration screw_axis -- screw axis expressed in link's COM
     frame
   */
  PoseFunctor(const gtsam::Pose3 &jMi, const gtsam::Vector6 &screw_axis)
      : iMj_(jMi.inverse()), screw_axis_(screw_axis) {}

  /** predict link pose
      Keyword argument:
          pose_i        -- previous link pose
          q             -- joint coordination
      Returns:
          pose_j        -- this link pose
  */
  gtsam::Pose3 operator()(
      const gtsam::Pose3 &pose_i, const double &q,
      gtsam::OptionalJacobian<6, 6> H_pose_i = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none) const {
    gtsam::Matrix6 Hexp;
    gtsam::Pose3 jTi = iMj_ * gtsam::Pose3::Expmap(screw_axis_ * q, Hexp);

    gtsam::Matrix6 pose_j_H_jTi;
    auto pose_j = pose_i.compose(jTi, H_pose_i, pose_j_H_jTi);
    if (H_q) {
      *H_q = pose_j_H_jTi * (Hexp * screw_axis_);
    }

    return pose_j;
  }
};

/** PoseFactor is a three-way nonlinear factor between the previuse link pose
 * and this link pose*/
class PoseFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, double> {
 private:
  typedef PoseFactor This;
  typedef gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, double> Base;

  PoseFunctor predict_;

 public:
  /** Create single factor relating this link's pose (COM) with previous one.
      Keyword arguments:
          jMi -- previous COM frame, expressed in this link's COM frame, at rest
     configuration screw_axis -- screw axis expressed in link's COM frame
   */
  PoseFactor(gtsam::Key pose_key_i, gtsam::Key pose_key_j, gtsam::Key q_key,
             const gtsam::noiseModel::Base::shared_ptr &cost_model,
             const gtsam::Pose3 &jMi, const gtsam::Vector6 &screw_axis)
      : Base(cost_model, pose_key_i, pose_key_j, q_key),
        predict_(jMi, screw_axis) {}

  virtual ~PoseFactor() {}

  /** evaluate link pose errors
      Keyword argument:
          pose_i         -- previous link pose
          pose_j         -- this link pose
          q              -- joint coordination
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose_i, const gtsam::Pose3 &pose_j, const double &q,
      boost::optional<gtsam::Matrix &> H_pose_i = boost::none,
      boost::optional<gtsam::Matrix &> H_pose_j = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    auto pose_j_hat = predict_(pose_i, q, H_pose_i, H_q);
    gtsam::Vector6 error = pose_j.logmap(pose_j_hat);
    if (H_pose_j) {
      *H_pose_j = -gtsam::I_6x6;
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
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace manipulator
