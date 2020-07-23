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
 * @Author: Yetong Zhang
 */

#ifndef GTDYNAMICS_FACTORS_WRENCHEQUIVALENCEFACTOR_H_
#define GTDYNAMICS_FACTORS_WRENCHEQUIVALENCEFACTOR_H_

#include <string>
#include <vector>
#include <memory>

#include <boost/optional.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

/** WrenchEquivalenceFactor is a 3-way nonlinear factor which enforces
 * relation between wrench expressed in two link frames*/
template <class JointTypedClass>
class WrenchEquivalenceFactor
    : public gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6,
                                      typename JointTypedClass::AngleType> {
 private:
  typedef typename JointTypedClass::AngleType JointAngleType;
  typedef WrenchEquivalenceFactor This;
  typedef gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6,
                                   typename JointTypedClass::AngleType>
      Base;
  typedef std::shared_ptr<const JointTypedClass> MyJointConstSharedPtr;
  MyJointConstSharedPtr joint_;

 public:
  /** wrench eq factor, enforce same wrench expressed in different link frames.
      Keyword argument:
          joint       -- JointConstSharedPtr to the joint
   */
  WrenchEquivalenceFactor(gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                          gtsam::Key q_key,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          MyJointConstSharedPtr joint)
      : Base(cost_model, wrench_key_1, wrench_key_2, q_key),
        joint_(joint) {}
  virtual ~WrenchEquivalenceFactor() {}

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          twist         -- twist on this link
          twist_accel   -- twist acceleration on this link
          wrench_1      -- wrench on Link 1 expressed in link 1 com frame
          wrench_2      -- wrench on Link 2 expressed in link 2 com frame
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench_1, const gtsam::Vector6 &wrench_2,
      const JointAngleType &q,
      boost::optional<gtsam::Matrix &> H_wrench_1 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_2 = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    auto link = joint_->childLink();
    gtsam::Pose3 T_21 = joint_->transformFrom(link, q);

    gtsam::Vector6 error = wrench_1 + T_21.AdjointMap().transpose() * wrench_2;

    if (H_wrench_1) {
      *H_wrench_1 = gtsam::I_6x6;
    }
    if (H_wrench_2) {
      *H_wrench_2 = T_21.AdjointMap().transpose();
    }
    if (H_q) {
      *H_q = joint_->AdjointMapJacobianJointAngle(link, q).transpose() *
             wrench_2;
    }
    return error;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "wrench equivalence factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_WRENCHEQUIVALENCEFACTOR_H_
