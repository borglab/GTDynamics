/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TwistFactor.h
 * @brief twist factor.
 * @Author: Frank Dellaert and Mandy Xie
 */

#ifndef GTDYNAMICS_FACTORS_TWISTFACTOR_H_
#define GTDYNAMICS_FACTORS_TWISTFACTOR_H_

#include "gtdynamics/universal_robot/JointTyped.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <string>

namespace gtdynamics {

/** TwistFactor is a four-way nonlinear factor which enforces relation
 * between twist on previous link and this link*/
class TwistFactor
    : public gtsam::NoiseModelFactor4<
          gtsam::Vector6, gtsam::Vector6, typename JointTyped::JointCoordinate,
          typename JointTyped::JointVelocity> {
 private:
  typedef typename JointTyped::JointCoordinate JointCoordinate;
  typedef typename JointTyped::JointVelocity JointVelocity;
  typedef TwistFactor This;
  typedef gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                   JointCoordinate, JointVelocity>
      Base;
  gtsam::Pose3 cMp_;
  JointConstSharedPtr joint_;
  gtsam::Vector6 screw_axis_;

 public:
  /** Create single factor relating child link's twist with parent one.
      Keyword arguments:
          joint -- a Joint
      Will create factor corresponding to Lynch & Park book:
        -Equation 8.45, page 292
   */
  TwistFactor(gtsam::Key twistP_key, gtsam::Key twistC_key, gtsam::Key q_key,
              gtsam::Key qVel_key,
              const gtsam::noiseModel::Base::shared_ptr &cost_model,
              JointConstSharedPtr joint)
      : Base(cost_model, twistP_key, twistC_key, q_key, qVel_key),
        joint_(joint) {}
  virtual ~TwistFactor() {}

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          twist_p       -- twist on the previous link
          twist_c       -- twist on this link
          q             -- joint coordination
          qVel          -- joint velocity
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_p, const gtsam::Vector6 &twist_c,
      const JointCoordinate &q, const JointVelocity &qVel,
      boost::optional<gtsam::Matrix &> H_twist_p = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_c = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel = boost::none) const override {
    auto error =
        std::static_pointer_cast<const JointTyped>(joint_)
            ->transformTwistTo(joint_->childLink(), q, qVel,
                                    twist_p, H_q, H_qVel,
                                    H_twist_p) -
        twist_c;

    if (H_twist_c) {
      *H_twist_c = -gtsam::I_6x6;
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
    std::cout << s << "twist factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_TWISTFACTOR_H_
