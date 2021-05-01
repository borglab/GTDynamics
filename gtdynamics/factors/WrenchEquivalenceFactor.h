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
#include "gtdynamics/universal_robot/ScrewJointBase.h"

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

 private:

  gtsam::Matrix6 AdjointMapJacobianQ(double q, const gtsam::Pose3 &jMi,
                                    const gtsam::Vector6 &screw_axis) const {
    // taking opposite value of screw_axis_ is because
    // jTi = Pose3::Expmap(-screw_axis_ * q) * jMi;
    gtsam::Vector3 w = -screw_axis.head<3>();
    gtsam::Vector3 v = -screw_axis.tail<3>();
    gtsam::Pose3 kTj = gtsam::Pose3::Expmap(-screw_axis * q) * jMi;
    auto w_skew = gtsam::skewSymmetric(w);
    gtsam::Matrix3 H_expo = w_skew * cosf(q) + w_skew * w_skew * sinf(q);
    gtsam::Matrix3 H_R = H_expo * jMi.rotation().matrix();
    gtsam::Vector3 H_T = H_expo * (jMi.translation() - w_skew * v) +
                        w * w.transpose() * v;
    gtsam::Matrix3 H_TR = gtsam::skewSymmetric(H_T) * kTj.rotation().matrix() +
                          gtsam::skewSymmetric(kTj.translation()) * H_R;
    gtsam::Matrix6 H = gtsam::Z_6x6;
    gtsam::insertSub(H, H_R, 0, 0);
    gtsam::insertSub(H, H_TR, 3, 0);
    gtsam::insertSub(H, H_R, 3, 3);
    return H;
  }

   /* calculate joint coordinate q jacobian */
  gtsam::Matrix61 qJacobian_(double q, const gtsam::Vector6 &wrench_2) const {

    boost::shared_ptr<const ScrewJointBase> screw_joint = boost::dynamic_pointer_cast<const ScrewJointBase>(joint_);
    gtsam::Pose3 M_21_ = screw_joint->parentTchild(0).inverse();
    gtsam::Vector6 screw_axis_ = screw_joint->screwAxis(screw_joint->child());
    auto H = AdjointMapJacobianQ(q, M_21_, screw_axis_);
    return H.transpose() * wrench_2;
  }

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
    gtsam::Pose3 T_21 = joint_->relativePoseOf(joint_->parent(), q);
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
      *H_q =
          joint_->AdjointMapJacobianJointAngle(joint_->child(), q).transpose() *
          wrench_2;
      // *H_q = qJacobian_(q, wrench_2);
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
