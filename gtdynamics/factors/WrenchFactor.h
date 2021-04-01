/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchFactor.h
 * @brief Wrench balance factor, common between forward and inverse dynamics.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/assign/list_of.hpp>
#include <boost/optional.hpp>
#include <boost/serialization/base_object.hpp>
#include <string>
#include <vector>

#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/DynamicsSymbol.h"

namespace gtdynamics {

/**
 * WrenchFactor is an n-way nonlinear factor which enforces relation
 * between wrenches on this link
 */
class WrenchFactor : public gtsam::NoiseModelFactor {
  using This = WrenchFactor;
  using Base = gtsam::NoiseModelFactor;
  gtsam::Matrix6 inertia_;
  gtsam::Vector3 gravity_;

 public:
  /**
   * Wrench balance factor, common between forward and inverse dynamics.
   * Will create factor corresponding to Lynch & Park book:
   *  wrench balance, Equation 8.48, page 293
   * @param inertia Moment of inertia and mass for this link
   * @param gravity (optional) Create gravity wrench in link COM frame.
   */
  WrenchFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key,
               const std::vector<DynamicsSymbol> &wrench_keys,
               gtsam::Key pose_key,
               const gtsam::noiseModel::Base::shared_ptr &cost_model,
               const gtsam::Matrix6 &inertia,
               const boost::optional<gtsam::Vector3> &gravity = boost::none)
      : Base(cost_model),
        inertia_(inertia),
        gravity_(gravity ? *gravity : gtsam::Vector3::Zero()) {
    keys_.reserve(wrench_keys.size() + 3);
    keys_.push_back(twist_key);
    keys_.push_back(twistAccel_key);
    keys_.push_back(pose_key);
    keys_.insert(keys_.end(), wrench_keys.cbegin(), wrench_keys.cend());
  }

 private:
  /// calculate jacobian of coriolis term w.r.t. joint coordinate twist
  gtsam::Matrix6 twistJacobian_(const gtsam::Vector6 &twist) const;

 public:
  /**
   * Evaluate wrench balance errors
   * @param values contains the twist of the link, twistAccel of the link, pose
   * of the link, and wrenches acting on the link.
   * @param H Jacobians, in the order: twist, twistAccel, pose, *wrenches twist
   * twist of this link
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
    std::cout << s << "wrench factor" << std::endl;
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
