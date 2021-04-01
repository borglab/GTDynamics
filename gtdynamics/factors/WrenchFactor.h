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
  gtsam::Matrix6 twistJacobian_(const gtsam::Vector6 &twist) const {
    auto g1 = inertia_(0, 0), g2 = inertia_(1, 1), g3 = inertia_(2, 2),
         m = inertia_(3, 3);
    auto w1 = twist(0), w2 = twist(1), w3 = twist(2), v1 = twist(3),
         v2 = twist(4), v3 = twist(5);
    gtsam::Matrix6 H_twist;
    H_twist << 0, (g2 - g3) * w3, (g2 - g3) * w2, 0, 0, 0,  //
        (g3 - g1) * w3, 0, (g3 - g1) * w1, 0, 0, 0,         //
        (g1 - g2) * w2, (g1 - g2) * w1, 0, 0, 0, 0,         //
        0, -m * v3, m * v2, 0, m * w3, -m * w2,             //
        m * v3, 0, -m * v1, -m * w3, 0, m * w1,             //
        -m * v2, m * v1, 0, m * w2, -m * w1, 0;
    return H_twist;
  }

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
                                    H = boost::none) const override {
    if (!this->active(x)) {
      return gtsam::Vector::Zero(this->dim());
    }

    // `keys_` order: twist, twistAccel, pose, *wrenches
    const gtsam::Vector6 twist = x.at<gtsam::Vector6>(keys_.at(0));
    const gtsam::Vector6 twistAccel = x.at<gtsam::Vector6>(keys_.at(1));
    const gtsam::Pose3 pose = x.at<gtsam::Pose3>(keys_.at(2));
    gtsam::Vector6 wrenchSum = std::accumulate(  // C++17: reduce
        keys_.cbegin() + 3, keys_.cend(),
        static_cast<gtsam::Vector6>(gtsam::Z_6x1),
        [&x](gtsam::Vector6 v1, gtsam::Key key) -> gtsam::Vector6 {
          return v1 + x.at<gtsam::Vector6>(key);
        });

    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate);
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    // Equation 8.48 (F = ma)
    gtsam::Vector6 error =
        (inertia_ * twistAccel) - wrenchSum - gravity_wrench -
        (gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist);

    if (H) {
      (*H)[0] = -twistJacobian_(twist);
      (*H)[1] = inertia_;
      (*H)[2] = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
      std::fill(H->begin()+3, H->end(), -gtsam::I_6x6);
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
