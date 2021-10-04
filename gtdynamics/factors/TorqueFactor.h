/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TorqueFactor.h
 * @brief Torque factor, common between forward and inverse dynamics.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/assign/list_of.hpp>
#include <boost/optional.hpp>
#include <memory>
#include <string>

#include "gtdynamics/universal_robot/JointTyped.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/utils/values.h"

namespace gtdynamics {

/**
 * TorqueFactor is a two-way nonlinear factor which enforces relation between
 * wrench and torque on each link
 */
class TorqueFactor : public gtsam::NoiseModelFactor {
 private:
  using This = TorqueFactor;
  using Base = gtsam::NoiseModelFactor;
  using MyJointConstSharedPtr = boost::shared_ptr<const Joint>;
  MyJointConstSharedPtr joint_;

  /// Private constructor with arbitrary keys
  TorqueFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
               gtsam::Key wrench_key, gtsam::Key torque_key,
               const MyJointConstSharedPtr &joint)
      : Base(cost_model,
             boost::assign::cref_list_of<2>(wrench_key)(torque_key)),
        joint_(joint) {}

 public:
  /**
   * Torque factor, common between forward and inverse dynamics.
   * Will create factor corresponding to Lynch & Park book:
   * Torque is always wrench projected on screw axis.
   * Equation 8.49, page 293 can be written as
   *  screw_axis.transpose() * F.transpose() == torque
   *
   * @param joint JointConstSharedPtr to the joint
   */
  TorqueFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
               const MyJointConstSharedPtr &joint, size_t k = 0)
      : TorqueFactor(cost_model,
                     internal::WrenchKey(joint->child()->id(), joint->id(), k),
                     internal::TorqueKey(joint->id(), k), joint) {}

  virtual ~TorqueFactor() {}

 public:
  /**
   * Evaluate wrench balance errors
   */
  gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                boost::optional<std::vector<gtsam::Matrix> &>
                                    H = boost::none) const override {
    const gtsam::Vector6 &wrench = x.at<gtsam::Vector6>(keys_[0]);
    gtsam::Vector torque;
    try {
      torque = x.at<gtsam::Vector>(keys_[1]);
    } catch (const gtsam::ValuesIncorrectType &e) {
      torque = gtsam::Vector1(x.at<double>(keys_[1]));
    }

    if (H) {
      (*H)[1] = -gtsam::Matrix::Identity(torque.size(), torque.size());
      return joint_->transformWrenchToTorque(joint_->child(), wrench, (*H)[0]) -
             torque;
    } else {
      return joint_->transformWrenchToTorque(joint_->child(), wrench) - torque;
    }
  }

  /// Returns the joint
  MyJointConstSharedPtr getJoint() const { return joint_; }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "torque factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
