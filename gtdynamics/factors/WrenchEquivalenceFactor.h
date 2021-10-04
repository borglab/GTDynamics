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

#include <boost/assign/list_of.hpp>
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
class WrenchEquivalenceFactor : public gtsam::NoiseModelFactor {
 private:
  using This = WrenchEquivalenceFactor;
  using Base = gtsam::NoiseModelFactor;

  JointConstSharedPtr joint_;
  int k_;

 public:
  /**
   * Wrench eq factor, enforce same wrench expressed in different link frames.
   * @param joint JointConstSharedPtr to the joint
   */
  WrenchEquivalenceFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          const JointConstSharedPtr &joint, size_t k = 0)
      : Base(cost_model,
             boost::assign::cref_list_of<3>(
                 internal::WrenchKey(joint->parent()->id(), joint->id(), k)
                     .key())(
                 internal::WrenchKey(joint->child()->id(), joint->id(), k)
                     .key())(internal::JointAngleKey(joint->id(), k).key())),
        joint_(joint),
        k_(k) {}

  virtual ~WrenchEquivalenceFactor() {}

  /**
   * Evaluate wrench balance errors
   */
  gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                boost::optional<std::vector<gtsam::Matrix> &>
                                    H = boost::none) const override {
    const gtsam::Vector6 &wrench_1 = x.at<gtsam::Vector6>(keys_[0]),
                         &wrench_2 = x.at<gtsam::Vector6>(keys_[1]);
    boost::optional<gtsam::Matrix &> T_21_H_q;
    if (H) T_21_H_q = (*H)[2];  // after gtsam#884 merge, H ? &(*H)[2] : nullptr
    gtsam::Matrix6 H_T_21;

    gtsam::Pose3 T_21 =
        joint_->relativePoseOf(joint_->parent(), x, k_, T_21_H_q);
    gtsam::Vector6 wrench_from_2 =
        H ? T_21.AdjointTranspose(wrench_2, H_T_21, (*H)[1])  // H_wrench_2
          : T_21.AdjointTranspose(wrench_2);

    if (H) {
      (*H)[0] = gtsam::I_6x6;  // H_wrench_1
      (*H)[2] = H_T_21 * (*H)[2];  // H_q
    }

    return wrench_1 + wrench_from_2;
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
