/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TwistFactor.h
 * @brief twist factor.
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

#include <boost/optional.hpp>

#include "gtdynamics/utils/utils.h"

namespace gtdynamics {

/** TwistFactor is a four-way nonlinear factor which enforces relation
 * between twist on previous link and this link*/
class TwistFactor
    : public gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6, double,
                                      double> {
 private:
  typedef TwistFactor This;
  typedef gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6, double,
                                   double>
      Base;
  gtsam::Pose3 jMi_;
  gtsam::Vector6 screw_axis_;

 public:
  /** Create single factor relating this link's twist with previous one.
      Keyword arguments:
          jMi -- previous COM frame, expressed in this link's COM frame, at rest
     configuration screw_axis -- screw axis expressed in link's COM frame Will
     create factor corresponding to Lynch & Park book: -Equation 8.45, page 292
   */
  TwistFactor(gtsam::Key twistI_key, gtsam::Key twistJ_key, gtsam::Key q_key,
              gtsam::Key qVel_key,
              const gtsam::noiseModel::Base::shared_ptr &cost_model,
              const gtsam::Pose3 &jMi, const gtsam::Vector6 &screw_axis)
      : Base(cost_model, twistI_key, twistJ_key, q_key, qVel_key),
        jMi_(jMi),
        screw_axis_(screw_axis) {}
  virtual ~TwistFactor() {}

 private:
  /* calculate jacobian of AdjointMap term w.r.t. joint coordinate q */
  gtsam::Matrix61 qJacobian_(const double &q,
                             const gtsam::Vector6 &twist_i) const {
    auto H = AdjointMapJacobianQ(q, jMi_, screw_axis_);
    return H * twist_i;
  }

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          twsit_i       -- twist on the previous link
          twsit_j       -- twist on this link
          q             -- joint coordination
          qVel          -- joint velocity
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_i, const gtsam::Vector6 &twist_j,
      const double &q, const double &qVel,
      boost::optional<gtsam::Matrix &> H_twist_i = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_j = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel = boost::none) const override {
    gtsam::Pose3 jTi = gtsam::Pose3::Expmap(-screw_axis_ * q) * jMi_;
    if (H_twist_i) {
      *H_twist_i = -jTi.AdjointMap();
    }
    if (H_twist_j) {
      *H_twist_j = gtsam::I_6x6;
    }
    if (H_q) {
      *H_q = -qJacobian_(q, twist_i);
    }
    if (H_qVel) {
      *H_qVel = -screw_axis_;
    }

    return twist_j - jTi.AdjointMap() * twist_i - screw_axis_ * qVel;
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
    std::cout << s << "twist factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
