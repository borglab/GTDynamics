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
  gtsam::Matrix36 H_wrench_;

public:
  /** wrench balance factor, common between forward and inverse dynamics.
      Keyword argument:
          planar_axis        -- axis of the plane
   */
  WrenchPlanarFactor(gtsam::Key wrench_key,
                      const gtsam::noiseModel::Base::shared_ptr &cost_model,
                      gtsam::Vector3 planar_axis)
      : Base(cost_model, wrench_key) {
    if (planar_axis[0] == 1) {  // x axis
      H_wrench_ << 0, 1, 0, 0, 0, 0,
                  0, 0, 1, 0, 0, 0,
                  0, 0, 0, 1, 0, 0;
    }
    else if (planar_axis[1] == 1) {   // y axis
      H_wrench_ << 1, 0, 0, 0, 0, 0,
                  0, 0, 1, 0, 0, 0,
                  0, 0, 0, 0, 1, 0;
    }
    else if (planar_axis[2] == 1) {   // z axis
      H_wrench_ << 1, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 1;
    }
  }
  virtual ~WrenchPlanarFactor() {
  }

  /** evaluate wrench balance errors
      Keyword argument:
          wrench      -- wrench on the link
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench,
      boost::optional<gtsam::Matrix &> H_wrench = boost::none) const override {

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
