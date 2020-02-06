/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchEquivalenceFactor.h
 * @brief Wrench eq factor, enforce same wrench expressed in different link frames.
 * @Author: Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <utils.h>

#include <vector>
#include <string>

#include <boost/optional.hpp>

namespace robot {

/** WrenchEquivalenceFactor is a 3-way nonlinear factor which enforces
 * relation between wrench expressed in two link frames*/
class WrenchEquivalenceFactor
    : public gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6, double> {
 private:
  typedef WrenchEquivalenceFactor This;
  typedef gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6, double> Base;
  gtsam::Pose3 M_21_;
  gtsam::Vector6 screw_axis_;

 public:
  /** wrench eq factor, enforce same wrench expressed in different link frames.
      Keyword argument:
          M_21        -- rest transform between the com frame of two links
          screw_axis  -- screw axis expressed in link2's COM frame
   */
  WrenchEquivalenceFactor(gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                          gtsam::Key q_key,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          const gtsam::Pose3 &M_21,
                          const gtsam::Vector6 &screw_axis)
      : Base(cost_model, wrench_key_1, wrench_key_2, q_key),
        M_21_(M_21),
        screw_axis_(screw_axis) {}
  virtual ~WrenchEquivalenceFactor() {}

 private:
  /* calculate joint coordinate q jacobian */
  gtsam::Matrix61 qJacobian_(double q, const gtsam::Vector6 &wrench_2) const {
    auto H = robot::AdjointMapJacobianQ(q, M_21_, screw_axis_);
    return H.transpose() * wrench_2;
  }

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          twsit         -- twist on this link
          twsit_accel   -- twist acceleration on this link
          wrench_1      -- wrench on Link 1 expressed in link 1 com frame
          wrench_2      -- wrench on Link 2 expressed in link 2 com frame
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench_1, const gtsam::Vector6 &wrench_2,
      const double &q,
      boost::optional<gtsam::Matrix &> H_wrench_1 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_2 = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    gtsam::Pose3 T_21 = gtsam::Pose3::Expmap(-screw_axis_ * q) * M_21_;

    gtsam::Vector6 error = wrench_1 + T_21.AdjointMap().transpose() * wrench_2;

    if (H_wrench_1) {
      *H_wrench_1 = gtsam::I_6x6;
    }
    if (H_wrench_2) {
      *H_wrench_2 = T_21.AdjointMap().transpose();
    }
    if (H_q) {
      *H_q = qJacobian_(q, wrench_2);
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
    std::cout << s << "wrench equivalence factor" << std::endl;
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
}  // namespace robot
