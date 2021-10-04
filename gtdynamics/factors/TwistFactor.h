/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TwistFactor.h
 * @brief twist factor.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/assign/list_of.hpp>
#include <boost/optional.hpp>
#include <string>

#include "gtdynamics/universal_robot/Joint.h"

namespace gtdynamics {

/**
 * TwistFactor is a four-way nonlinear factor which enforces relation
 * between twist on previous link and this link
 */
class TwistFactor : public gtsam::NoiseModelFactor {
 private:
  using This = TwistFactor;
  using Base = gtsam::NoiseModelFactor;

  JointConstSharedPtr joint_;
  int t_;

 public:
  /**
   * Create single factor relating child link's twist with parent one.
   * Will create factor corresponding to Lynch & Park book:
   *  Equation 8.45, page 292
   *
   * @param joint a Joint
   */
  TwistFactor(const gtsam::noiseModel::Base::shared_ptr &cost_model,
              JointConstSharedPtr joint, int t)
      : Base(cost_model,  //
             boost::assign::cref_list_of<4>(
                 internal::TwistKey(joint->parent()->id(), t).key())(
                 internal::TwistKey(joint->child()->id(), t).key())(
                 internal::JointAngleKey(joint->id(), t).key())(
                 internal::JointVelKey(joint->id(), t).key())),
        joint_(joint),
        t_(t) {}
  virtual ~TwistFactor() {}

 public:
  /**
   * Evaluate wrench balance errors
   * Order of keys: twist_p, twist_c, q, qdot
   */
  gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                boost::optional<std::vector<gtsam::Matrix> &>
                                    H = boost::none) const override {
    const gtsam::Vector6 &twist_p = x.at<gtsam::Vector6>(keys_[0]);
    const gtsam::Vector6 &twist_c = x.at<gtsam::Vector6>(keys_[1]);

    // Due to quirks of OptionalJacobian, this is the cleanest way (Gerry)
    if (H) {
      (*H)[1] = -gtsam::I_6x6;  // H_twist_c
      return joint_->transformTwistTo(t_, joint_->child(), x, twist_p,
                                      (*H)[2],  // H_q
                                      (*H)[3],  // H_qdot
                                      (*H)[0])  // H_twist_p
             - twist_c;
    } else {
      return joint_->transformTwistTo(t_, joint_->child(), x, twist_p)  //
             - twist_c;
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
    std::cout << s << "twist factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
