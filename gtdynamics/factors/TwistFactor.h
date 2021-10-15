/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TwistFactor.h
 * @brief twist factor.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <string>

#include "gtdynamics/universal_robot/JointTyped.h"

namespace gtdynamics {

/**
 * TwistFactor is a four-way nonlinear factor which enforces relation
 * between twist on previous link and this link
 */
class TwistFactor
    : public gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                      typename JointTyped::JointCoordinate,
                                      typename JointTyped::JointVelocity> {
 private:
  using JointCoordinate = typename JointTyped::JointCoordinate;
  using JointVelocity = typename JointTyped::JointVelocity;
  using This = TwistFactor;
  using Base = gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                        JointCoordinate, JointVelocity>;

  JointConstSharedPtr joint_;

 public:
  /**
   * Create single factor relating child link's twist with parent one.
   * Will create factor corresponding to Lynch & Park book:
   *  Equation 8.45, page 292
   *
   * @param joint a Joint
   */
  TwistFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
              JointConstSharedPtr joint, int t)
      : Base(cost_model,  //
             internal::TwistKey(joint->parent()->id(), t),
             internal::TwistKey(joint->child()->id(), t),
             internal::JointAngleKey(joint->id(), t),
             internal::JointVelKey(joint->id(), t)),
        joint_(joint) {}
  virtual ~TwistFactor() {}

 public:
  /**
   * Evaluate wrench balance errors
   * @param twist_p twist on the previous link
   * @param twist_c twist on this link
   * @param q joint coordination
   * @param qVel joint velocity
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_p, const gtsam::Vector6 &twist_c,
      const JointCoordinate &q, const JointVelocity &qVel,
      boost::optional<gtsam::Matrix &> H_twist_p = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_c = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel = boost::none) const override {
    auto error =
        boost::static_pointer_cast<const JointTyped>(joint_)->transformTwistTo(
            joint_->child(), q, qVel, twist_p, H_q, H_qVel, H_twist_p) -
        twist_c;

    if (H_twist_c) {
      *H_twist_c = -gtsam::I_6x6;
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
    std::cout << s << "twist factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
