/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchPlanarFactor.h
 * @brief Wrench planar factor, enforce the wrench to be planar.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <string>

#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/values.h"

namespace gtdynamics {

/**
 * WrenchPlanarFactor is a one-way nonlinear factor which enforces the
 * wrench to be planar
 */
class WrenchPlanarFactor : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 private:
  using This = WrenchPlanarFactor;
  using Base = gtsam::NoiseModelFactor1<gtsam::Vector6>;
  gtsam::Matrix36 H_wrench_;

  /// Private constructor with arbitrary keys
  WrenchPlanarFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     gtsam::Vector3 planar_axis, gtsam::Key wrench_key)
      : Base(cost_model, wrench_key) {
    if (planar_axis[0] == 1) {  // x axis
      H_wrench_ << 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0;
    } else if (planar_axis[1] == 1) {  // y axis
      H_wrench_ << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    } else if (planar_axis[2] == 1) {  // z axis
      H_wrench_ << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    }
  }

 public:
  /** Constructor
   * @param planar_axis axis of the plane
   */
  WrenchPlanarFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     gtsam::Vector3 planar_axis,
                     const boost::shared_ptr<const JointTyped> &joint,
                     size_t k = 0)
      : WrenchPlanarFactor(
            cost_model, planar_axis,
            internal::WrenchKey(joint->child()->id(), joint->id(), k)) {}

  virtual ~WrenchPlanarFactor() {}

  /**
   * Evaluate error
   * @param wrench wrench on the link
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

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "wrench plannar factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
