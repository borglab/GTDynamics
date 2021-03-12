/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchEquivalenceFactor.h
 * @brief Wrench eq factor, enforce same wrench expressed in different link
 * frames.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <memory>
#include <string>
#include <vector>

#include "gtdynamics/universal_robot/JointTyped.h"

namespace gtdynamics {

/** WrenchEquivalenceFactor is a 3-way nonlinear factor which enforces
 * relation between wrench expressed in two link frames*/
class WrenchEquivalenceFactor
    : public gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6,
                                      typename JointTyped::JointCoordinate> {
 private:
  using JointCoordinate = typename JointTyped::JointCoordinate;
  using This = WrenchEquivalenceFactor;
  using Base =
      gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6, JointCoordinate>;

  using JointTypedConstSharedPtr = boost::shared_ptr<const JointTyped>;
  JointTypedConstSharedPtr joint_;

 public:
  /**
   * Wrench eq factor, enforce same wrench expressed in different link frames.
   * @param joint JointConstSharedPtr to the joint
   */
  WrenchEquivalenceFactor(gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                          gtsam::Key q_key,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          JointTypedConstSharedPtr joint)
      : Base(cost_model, wrench_key_1, wrench_key_2, q_key), joint_(joint) {}
  virtual ~WrenchEquivalenceFactor() {}

 public:
  /**
   * Evaluate wrench balance errors
   * @param twist twist on this link
   * @param twist_accel twist acceleration on this link
   * @param wrench_1 wrench on Link 1 expressed in link 1 com frame
   * @param wrench_2 wrench on Link 2 expressed in link 2 com frame
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench_1, const gtsam::Vector6 &wrench_2,
      const JointCoordinate &q,
      boost::optional<gtsam::Matrix &> H_wrench_1 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_2 = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    auto link = joint_->child();
    gtsam::Pose3 T_21 = joint_->transformTo(link, q);

    gtsam::Vector6 error = wrench_1 + T_21.AdjointMap().transpose() * wrench_2;

    if (H_wrench_1) {
      *H_wrench_1 = gtsam::I_6x6;
    }
    if (H_wrench_2) {
      *H_wrench_2 = T_21.AdjointMap().transpose();
    }
    if (H_q) {
      *H_q =
          joint_->AdjointMapJacobianJointAngle(link, q).transpose() * wrench_2;
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
    std::cout << s << "wrench equivalence factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
