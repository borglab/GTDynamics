/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchPlanarFactor.h
 * @brief Wrench planar factor, enforce the wrench to be planar.
 * @Author: Yetong Zhang
 */

#ifndef GTDYNAMICS_FACTORS_WRENCHPLANARFACTOR_H_
#define GTDYNAMICS_FACTORS_WRENCHPLANARFACTOR_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

#include <boost/optional.hpp>

#include "gtdynamics/utils/Utils.h"

namespace gtdynamics {

/** WrenchPlanarFactor is a one-way nonlinear factor which enforces the
 * wrench to be planar*/
class WrenchPlanarFactor : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 private:
  typedef WrenchPlanarFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector6> Base;
  gtsam::Matrix36 H_wrench_;

 public:
  /** Constructor
      Keyword argument:
          planar_axis        -- axis of the plane
   */
  WrenchPlanarFactor(gtsam::Key wrench_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     gtsam::Vector3 planar_axis)
      : Base(cost_model, wrench_key) {
    if (planar_axis[0] == 1) {  // x axis
      H_wrench_ << 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0;
    } else if (planar_axis[1] == 1) {  // y axis
      H_wrench_ << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    } else if (planar_axis[2] == 1) {  // z axis
      H_wrench_ << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    }
  }
  virtual ~WrenchPlanarFactor() {}

  /** evaluate error
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
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_WRENCHPLANARFACTOR_H_
