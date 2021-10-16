/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TwistAccelFactor.h
 * @brief twist acceleration factor, common between forward and inverse
 * dynamics.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <memory>
#include <string>

#include "gtdynamics/universal_robot/JointTyped.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

/**
 * TwistAccelFactor is a six-way nonlinear factor which enforces relation
 * between acceleration on previous link and this link.
 */
class TwistAccelFactor
    : public gtsam::NoiseModelFactor6<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, double, double, double> {
 private:
  using This = TwistAccelFactor;
  using Base = gtsam::NoiseModelFactor6<gtsam::Vector6, gtsam::Vector6,
                                        gtsam::Vector6, double,
                                        double, double>;
  using JointTypedConstSharedPtr = boost::shared_ptr<const JointTyped>;
  JointTypedConstSharedPtr joint_;

 public:
  /**
   * Factor linking child link's twist_accel, joint_coordinate, joint_vel,
   * joint_accel with previous link's twist_accel.
   *
   * Will create factor corresponding to Lynch & Park book: twist acceleration,
   * Equation 8.47, page 293
   *
   * @param joint JointConstSharedPtr to the joint
   */
  TwistAccelFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
                   JointConstSharedPtr joint, int t)
      : Base(cost_model,  //
             internal::TwistKey(joint->child()->id(), t),
             internal::TwistAccelKey(joint->parent()->id(), t),
             internal::TwistAccelKey(joint->child()->id(), t),
             internal::JointAngleKey(joint->id(), t),
             internal::JointVelKey(joint->id(), t),
             internal::JointAccelKey(joint->id(), t)),
        joint_(boost::static_pointer_cast<const JointTyped>(joint)) {}
  virtual ~TwistAccelFactor() {}

 private:
 public:
  /**
   * Evaluate twist acceleration errors
   * @param twistAccel_p twist acceleration on parent link
   * @param twistAccel_c twist acceleration on child link
   * @param twist_c twist on child link
   * @param q joint coordination
   * @param qVel joint velocity
   * @param qAccel joint acceleration
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_c, const gtsam::Vector6 &twistAccel_p,
      const gtsam::Vector6 &twistAccel_c, const double &q,
      const double &qVel, const double &qAccel,
      boost::optional<gtsam::Matrix &> H_twist_c = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel_p = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel_c = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel = boost::none,
      boost::optional<gtsam::Matrix &> H_qAccel = boost::none) const override {
    auto error =
        joint_->transformTwistAccelTo(joint_->child(), q, qVel, qAccel,
                                      twist_c, twistAccel_p, H_q, H_qVel,
                                      H_qAccel, H_twist_c, H_twistAccel_p) -
        twistAccel_c;

    if (H_twistAccel_c) {
      *H_twistAccel_c = -gtsam::I_6x6;
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
    std::cout << s << "twist acceleration factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor6", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
