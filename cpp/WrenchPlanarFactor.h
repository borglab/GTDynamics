/**
 * @file  WrenchPlanarFactor.h
 * @brief wrench balance factor, common between forward and inverse dynamics.
 * @Author: Yetong Zhang
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

namespace robot {

/** WrenchPlanarFactor is a six-way nonlinear factor which enforces relation
 * between wrenches on this link and the next link*/
class WrenchPlanarFactor : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
private:
  typedef WrenchPlanarFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector6> Base;
  gtsam::Vector3 plannar_axis_;

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
  WrenchPlanarFactor(gtsam::Key wrench_key,
                      const gtsam::noiseModel::Base::shared_ptr &cost_model,
                      gtsam::Vector3 plannar_axis)
      : Base(cost_model, wrench_key), plannar_axis_(plannar_axis) {}
  virtual ~WrenchPlanarFactor() {}

  /** evaluate wrench balance errors
      Keyword argument:
          twsit         -- twist on this link
          twsit_accel   -- twist acceleration on this link
          wrench_j      -- wrench on this link
          wrench_k      -- wrench from the next link
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench,
      boost::optional<gtsam::Matrix &> H_wrench = boost::none) const override {

    gtsam::Matrix36 H_wrench_;
    H_wrench_ << 0, 1, 0, 0, 0, 0,            //
                 0, 0, 1, 0, 0, 0,            //
                 0, 0, 0, 1, 0, 0;

    gtsam::Vector3 error = H_wrench_ * wrench;

    if (H_wrench) {
      *H_wrench = H_wrench_;
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
    std::cout << s << "wrench plannar factor" << std::endl;
    Base::print("", keyFormatter);
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
} // namespace robot
