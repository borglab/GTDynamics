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
class TorqueFactor : public gtsam::NoiseModelFactor2<gtsam::Vector6, double> {
 private:
  using This = TorqueFactor;
  using Base = gtsam::NoiseModelFactor2<gtsam::Vector6, double>;
  using MyJointConstSharedPtr = boost::shared_ptr<const JointTyped>;
  MyJointConstSharedPtr joint_;

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
               const JointConstSharedPtr &joint, size_t k = 0)
      : Base(cost_model,
             internal::WrenchKey(joint->child()->id(), joint->id(), k),
             internal::TorqueKey(joint->id(), k)),
        joint_(boost::static_pointer_cast<const JointTyped>(joint)) {}

  virtual ~TorqueFactor() {}

 public:
  /**
   * Evaluate wrench balance errors
   * @param wrench wrench on this link
   * @param torque torque on this link joint
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench, const double &torque,
      boost::optional<gtsam::Matrix &> H_wrench = boost::none,
      boost::optional<gtsam::Matrix &> H_torque = boost::none) const override {
    if (H_torque) {
      *H_torque = -gtsam::Matrix1::Identity();
    }
    // TODO(G+S): next PR will generalize this from Vector1
    return gtsam::Vector1(
        joint_->transformWrenchToTorque(joint_->child(), wrench, H_wrench) -
        torque);
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
