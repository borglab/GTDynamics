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

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/utils/values.h"

namespace gtdynamics {

/** WrenchEquivalenceFactor is a 3-way nonlinear factor which enforces
 * relation between wrench expressed in two link frames*/
class WrenchEquivalenceFactor
    : public gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6, double> {
 private:
  using This = WrenchEquivalenceFactor;
  using Base = gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6, double>;

  JointConstSharedPtr joint_;

  /// Private constructor with arbitrary keys
  WrenchEquivalenceFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                          gtsam::Key q_key,
                          const JointConstSharedPtr &joint)
      : Base(cost_model, wrench_key_1, wrench_key_2, q_key), joint_(joint) {}

 public:
  /**
   * Wrench eq factor, enforce same wrench expressed in different link frames.
   * @param joint JointConstSharedPtr to the joint
   */
  WrenchEquivalenceFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          const JointConstSharedPtr &joint, size_t k = 0)
      : WrenchEquivalenceFactor(
            cost_model,
            internal::WrenchKey(joint->parent()->id(), joint->id(), k),
            internal::WrenchKey(joint->child()->id(), joint->id(), k),
            internal::JointAngleKey(joint->id(), k), joint) {}

  virtual ~WrenchEquivalenceFactor() {}

  /**
   * Evaluate wrench balance errors
   * @param twist twist on this link
   * @param twist_accel twist acceleration on this link
   * @param wrench_1 wrench on Link 1 expressed in link 1 com frame
   * @param wrench_2 wrench on Link 2 expressed in link 2 com frame
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench_1, const gtsam::Vector6 &wrench_2,
      const double &q,
      boost::optional<gtsam::Matrix &> H_wrench_1 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_2 = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    gtsam::Pose3 T_21 = joint_->childTparent(q);
    gtsam::Matrix6 Ad_21_T = T_21.AdjointMap().transpose();
    gtsam::Vector6 error = wrench_1 + Ad_21_T * wrench_2;

    if (H_wrench_1) {
      *H_wrench_1 = gtsam::I_6x6;
    }
    if (H_wrench_2) {
      *H_wrench_2 = Ad_21_T;
    }
    if (H_q) {
      // TODO(frank): really, child? Double-check derivatives
      *H_q = AdjointMapJacobianQ(q, joint_->childTparent(0.0),
                                 joint_->screwAxis(joint_->child()))
                 .transpose() *
             wrench_2;
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
