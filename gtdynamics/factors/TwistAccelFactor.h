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
 * @Author: Frank Dellaert and Mandy Xie
 */

#ifndef GTDYNAMICS_FACTORS_TWISTACCELFACTOR_H_
#define GTDYNAMICS_FACTORS_TWISTACCELFACTOR_H_

#include <string>

#include <boost/optional.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "gtdynamics/utils/utils.h"
#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"
#include "gtdynamics/universal_robot/RobotTypes.h"

namespace gtdynamics {

/** TwistAccelFactor is a six-way nonlinear factor which enforces relation
 * between acceleration on previous link and this link*/
template <class JointTypedClass>
class TwistAccelFactor : public gtsam::NoiseModelFactor6<
                             gtsam::Vector6, gtsam::Vector6, gtsam::Vector6,
                             typename JointTypedClass::AngleType,
                             typename JointTypedClass::AngleTangentType,
                             typename JointTypedClass::AngleTangentType> {
 private:
  typedef typename JointTypedClass::AngleType JointAngleType;
  typedef typename JointTypedClass::AngleTangentType JointAngleTangentType;
  typedef TwistAccelFactor This;
  typedef gtsam::NoiseModelFactor6<gtsam::Vector6, gtsam::Vector6,
                                   gtsam::Vector6, JointAngleType,
                                   JointAngleTangentType, JointAngleTangentType>
      Base;
  typedef std::shared_ptr<const JointTypedClass> MyJointConstSharedPtr;
  MyJointConstSharedPtr joint_;

 public:
  /** factor linking child link's twist_accel, joint_coordinate, joint_vel,
     joint_accel with previous link's twist_accel.
     Keyword arguments:
        joint         -- JointConstSharedPtr to the joint
        
    Will create factor corresponding to Lynch & Park book: twist acceleration,
    Equation 8.47, page 293
   */
  TwistAccelFactor(gtsam::Key twist_key_c, gtsam::Key twistAccel_key_p,
                   gtsam::Key twistAccel_key_c, gtsam::Key q_key,
                   gtsam::Key qVel_key, gtsam::Key qAccel_key,
                   const gtsam::noiseModel::Base::shared_ptr &cost_model,
                   MyJointConstSharedPtr joint)
      : Base(cost_model, twist_key_c, twistAccel_key_p, twistAccel_key_c, q_key,
             qVel_key, qAccel_key),
        joint_(joint) {}
  virtual ~TwistAccelFactor() {}

 private:

 public:
  /** evaluate twist acceleration errors
      Keyword argument:
          twistAccel_p          -- twist acceleration on parent link
          twistAccel_c          -- twist acceleration on child link
          twist_c               -- twist on child link
          q                     -- joint coordination
          qVel                  -- joint velocity
          qAccel                -- joint acceleration
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_c, const gtsam::Vector6 &twistAccel_p,
      const gtsam::Vector6 &twistAccel_c, const JointAngleType &q,
      const JointAngleTangentType &qVel, const JointAngleTangentType &qAccel,
      boost::optional<gtsam::Matrix &> H_twist_c = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel_p = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel_c = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel = boost::none,
      boost::optional<gtsam::Matrix &> H_qAccel = boost::none) const override {
    auto error =
        joint_->transformTwistAccelTo(joint_->childLink(), q, qVel, qAccel,
                                      twist_c, twistAccel_p, H_q, H_qVel,
                                      H_qAccel, H_twist_c, H_twistAccel_p) -
        twistAccel_c;

    if (H_twistAccel_c) {
      *H_twistAccel_c = -gtsam::I_6x6;
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
    std::cout << s << "twist acceleration factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor6", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_TWISTACCELFACTOR_H_
