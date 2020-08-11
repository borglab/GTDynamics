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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

#include "gtdynamics/utils/utils.h"

namespace gtdynamics {

/** PoseFunctor is functor predicting link's pose (COM) with previous one*/
class PoseFunctor {
 private:
  gtsam::Pose3 pMc_;
  gtsam::Vector6 screw_axis_;

 public:
  /** Create functor predicting this link's pose (COM) with previous one.
      Keyword arguments:
          cMp        -- previous (parent) COM frame, expressed in this (child)
     link's COM frame, at rest configuration
          screw_axis -- screw axis expressed in link's COM frame
   */
  PoseFunctor(const gtsam::Pose3 &cMp, const gtsam::Vector6 &screw_axis)
      : pMc_(cMp.inverse()), screw_axis_(screw_axis) {}

  /** predict link pose
      Keyword argument:
          wTp        -- previous (parent) link pose
          q          -- joint angle
      Returns:
          wTc        -- this (child) link pose
  */
  gtsam::Pose3 operator()(
      const gtsam::Pose3 &wTp, const double &q,
      gtsam::OptionalJacobian<6, 6> H_wTp = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none) const {
    gtsam::Matrix6 Hexp;
    gtsam::Pose3 cTp = pMc_ * gtsam::Pose3::Expmap(screw_axis_ * q, Hexp);

    gtsam::Matrix6 wTc_H_cTp;
    auto wTc = wTp.compose(cTp, H_wTp, wTc_H_cTp);
    if (H_q) {
      *H_q = wTc_H_cTp * (Hexp * screw_axis_);
    }

    return wTc;
  }
};

/** PoseFactor is a three-way nonlinear factor between the previous link pose
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
          cMp -- previous (parent) COM frame, expressed in this (child) link's
     COM frame, at rest configuration 
          screw_axis -- screw axis expressed in link's COM frame
   */
  PoseFactor(gtsam::Key wTp_key, gtsam::Key wTc_key, gtsam::Key q_key,
             const gtsam::noiseModel::Base::shared_ptr &cost_model,
             const gtsam::Pose3 &cMp, const gtsam::Vector6 &screw_axis)
      : Base(cost_model, wTp_key, wTc_key, q_key),
        predict_(cMp, screw_axis) {}

  virtual ~PoseFactor() {}

  /** evaluate link pose errors
      Keyword argument:
          wTp         -- previous (parent) link pose
          wTc         -- this (child) link pose
          q           -- joint angle
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wTp, const gtsam::Pose3 &wTc, const double &q,
      boost::optional<gtsam::Matrix &> H_wTp = boost::none,
      boost::optional<gtsam::Matrix &> H_wTc = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    auto wTc_hat = predict_(wTp, q, H_wTp, H_q);
    gtsam::Vector6 error = wTc.logmap(wTc_hat);
    if (H_wTc) {
      *H_wTc = -gtsam::I_6x6;
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
