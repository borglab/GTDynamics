/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  StaticWrenchFactor.h
 * @brief Wrench balance factor for a stationary link.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <boost/serialization/base_object.hpp>
#include <string>
#include <vector>

#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/utils.h"

namespace gtdynamics {

/**
 * StaticWrenchFactor is an n-way nonlinear factor which enforces that the sum
 * of the gravity wrench and the external wrenches should be zero for a
 * stationary link.
 */
class StaticWrenchFactor : public gtsam::NoiseModelFactor {
  using This = StaticWrenchFactor;
  using Base = gtsam::NoiseModelFactor;
  double mass_;
  boost::optional<gtsam::Vector3> gravity_;

 public:
  /**
   * Static wrench balance factor.
   * @param wrench_keys Keys for unknown external wrenches.
   * @param pose_key Key for link CoM pose.
   * @param cost_model Cost model to regulate constraint.
   * @param mass Mass for this link.
   * @param gravity (optional) Gravity vector in world frame.
   */
  StaticWrenchFactor(
      const std::vector<DynamicsSymbol> &wrench_keys, gtsam::Key pose_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model, double mass,
      const boost::optional<gtsam::Vector3> &gravity = boost::none);

 public:
  /**
   * Evaluate TotalExternalWrench, which should be zero and is factor error.
   * @param values contains the pose and wrenches acting on the link.
   * @param H Jacobians, in the order: *wrenches, pose
   */
  gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                boost::optional<std::vector<gtsam::Matrix> &>
                                    H = boost::none) const override;

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "static wrench factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
