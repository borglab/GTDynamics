/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TorqueFactor.h
 * @brief Torque factor, common between forward and inverse dynamics.
 * @Author: Frank Dellaert and Mandy Xie
 */

#ifndef GTDYNAMICS_FACTORS_TORQUEFACTOR_H_
#define GTDYNAMICS_FACTORS_TORQUEFACTOR_H_

#include <string>

#include <boost/optional.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "gtdynamics/universal_robot/JointTyped.h"

namespace gtdynamics {

/** TorqueFactor is a two-way nonlinear factor which enforces relation between
 * wrench and torque on each link*/
class TorqueFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector6,
        typename JointTyped::AngleTangentType> {
 private:
  typedef typename JointTyped::AngleTangentType JointAngleTangentType;
  typedef typename gtsam::traits<typename JointTyped::AngleType>::TangentVector
      JointAngleTangentVector;
  typedef TorqueFactor This;
  typedef gtsam::NoiseModelFactor2<gtsam::Vector6, JointAngleTangentType> Base;
  typedef std::shared_ptr<const JointTyped> MyJointConstSharedPtr;
  MyJointConstSharedPtr joint_;

 public:
  /** torque factor, common between forward and inverse dynamics.
      Keyword argument:
          joint         -- JointConstSharedPtr to the joint
      Will create factor corresponding to Lynch & Park book:
      Torque is always wrench projected on screw axis.
      Equation 8.49, page 293 can be written as
      screw_axis.transpose() * F.transpose() == torque
   */
  TorqueFactor(gtsam::Key wrench_key, gtsam::Key torque_key,
               const gtsam::noiseModel::Base::shared_ptr &cost_model,
               MyJointConstSharedPtr joint)
      : Base(cost_model, wrench_key, torque_key), joint_(joint) {}
  virtual ~TorqueFactor() {}

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          wrench       -- wrench on this link
          torque       -- torque on this link joint
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench, const JointAngleTangentType &torque,
      boost::optional<gtsam::Matrix &> H_wrench = boost::none,
      boost::optional<gtsam::Matrix &> H_torque = boost::none) const override {
    if (H_torque) {
      *H_torque = -JointTyped::MatrixN::Identity();
    }
    // TODO(G+S): next PR will generalize this from Vector1
    return gtsam::Vector1(
        joint_->transformWrenchToTorque(joint_->childLink(), wrench, H_wrench) -
        torque);
  }

  // Returns the joint
  MyJointConstSharedPtr getJoint() const { return joint_; }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "torque factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_TORQUEFACTOR_H_
