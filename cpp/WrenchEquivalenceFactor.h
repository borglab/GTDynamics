/**
 * @file  WrenchEquivalenceFactor.h
 * @brief wrench balance factor, common between forward and inverse dynamics.
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <utils.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <vector>

namespace manipulator {

/** WrenchEquivalenceFactor is a six-way nonlinear factor which enforces relation
 * between wrenches on this link and the next link*/
class WrenchEquivalenceFactor
    : public gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6, double> {
 private:
  typedef WrenchEquivalenceFactor This;
  typedef gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6, double>
      Base;
  gtsam::Pose3 kMj_;
  gtsam::Matrix6 inertia_;
  gtsam::Vector6 screw_axis_;
  gtsam::Vector3 gravity_;

 public:
  /** wrench balance factor, common between forward and inverse dynamics.
      Keyword argument:
          kMj        -- this COM frame, expressed in next link's COM frame
          inertia    -- moment of inertia and mass for this link
          screw_axis -- screw axis expressed in kth link's COM frame
          gravity    -- if given, will create gravity wrench. In link
     COM frame. Will create factor corresponding to Lynch & Park book:
          - wrench balance, Equation 8.48, page 293
   */
  WrenchEquivalenceFactor(gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
               gtsam::Key q_key,
               const gtsam::noiseModel::Base::shared_ptr &cost_model,
               const gtsam::Pose3 &kMj, const gtsam::Vector6 &screw_axis)
      : Base(cost_model, wrench_key_1, wrench_key_2, q_key),
        kMj_(kMj),
        screw_axis_(screw_axis) {}
  virtual ~WrenchEquivalenceFactor() {}

 private:
  /* calculate joint coordinate q jacobian */
  gtsam::Matrix61 qJacobian_(double q, const gtsam::Vector6 &wrench_2) const {
    auto H = AdjointMapJacobianQ(q, kMj_, screw_axis_);
    return H.transpose() * wrench_2;
  }

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          twsit         -- twist on this link
          twsit_accel   -- twist acceleration on this link
          wrench_1      -- wrench expressed in link 1
          wrench_2      -- wrench expressed in link 2
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench_1, const gtsam::Vector6 &wrench_2,
      const double &q,
      boost::optional<gtsam::Matrix &> H_wrench_1 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_2 = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    gtsam::Pose3 T_21 = gtsam::Pose3::Expmap(-screw_axis_ * q) * kMj_;

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
  gtsam::NonlinearFactor::shared_ptr clone() const override{
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
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace manipulator
