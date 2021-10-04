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
 * @author Frank Dellaert and Mandy Xie
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

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

/**
 * TwistAccelFactor is a six-way nonlinear factor which enforces relation
 * between acceleration on previous link and this link.
 */
class TwistAccelFactor : public gtsam::NoiseModelFactor {
 private:
  using This = TwistAccelFactor;
  using Base = gtsam::NoiseModelFactor;

  JointConstSharedPtr joint_;
  int t_;

 public:
  /**
   * Factor linking child link's twist_accel, joint_coordinate, joint_vel,
   * joint_accel with previous link's twist_accel.
   *
   * Will create factor corresponding to Lynch & Park book: twist acceleration,
   * Equation 8.47, page 293
   *
   * @param joint JointConstSharedPtr to the joint
   */
  TwistAccelFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
                   JointConstSharedPtr joint, int t)
      : Base(cost_model,
             boost::assign::cref_list_of<6>(
                 internal::TwistKey(joint->child()->id(), t).key())(
                 internal::TwistAccelKey(joint->parent()->id(), t).key())(
                 internal::TwistAccelKey(joint->child()->id(), t).key())(
                 internal::JointAngleKey(joint->id(), t).key())(
                 internal::JointVelKey(joint->id(), t).key())(
                 internal::JointAccelKey(joint->id(), t).key())),
        joint_(joint),
        t_(t) {}
  virtual ~TwistAccelFactor() {}

 private:
 public:
  /**
   * Evaluate twist acceleration errors
   */
  gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                boost::optional<std::vector<gtsam::Matrix> &>
                                    H = boost::none) const override {
    const gtsam::Vector6 &twist_c = x.at<gtsam::Vector6>(keys_[0]);
    const gtsam::Vector6 &twistAccel_p = x.at<gtsam::Vector6>(keys_[1]);
    const gtsam::Vector6 &twistAccel_c = x.at<gtsam::Vector6>(keys_[2]);

    // Due to quirks of OptionalJacobian, this is the cleanest way (Gerry)
    if (H) {
      (*H)[2] = -gtsam::I_6x6;  // H_twistAccel_c
      return joint_->transformTwistAccelTo(t_, joint_->child(), x, twist_c,
                                           twistAccel_p,
                                           (*H)[3],  // H_q
                                           (*H)[4],  // H_qVel
                                           (*H)[5],  // H_qAccel
                                           (*H)[0],  // H_twist_c
                                           (*H)[1])  // H_twist_p
             - twistAccel_c;
    } else {
      return joint_->transformTwistAccelTo(t_, joint_->child(), x, twist_c,
                                           twistAccel_p)  //
             - twistAccel_c;
    }
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
    std::cout << s << "twist acceleration factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor6", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
