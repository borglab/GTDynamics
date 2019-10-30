/**
 * @file  ToolWrenchFactor.h
 * @brief Factor enforcing external wrench at tool frame.
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>
#include <vector>

namespace manipulator {

/** ToolWrenchFactor is a four-way nonlinear factor which enforces the tool
 * wrench*/
class ToolWrenchFactor
    : public gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Pose3> {
 private:
  typedef ToolWrenchFactor This;
  typedef gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                   gtsam::Vector6, gtsam::Pose3>
      Base;
  gtsam::Pose3 tTn_;
  gtsam::Matrix6 inertia_;
  gtsam::Vector6 external_wrench_;
  gtsam::Vector3 gravity_;

 public:
  /** wrench balance factor, common between forward and inverse dynamics.
      Keyword argument:
          tTn     -- tool frame, expressed in last link's COM frame
          inertia -- moment of inertia and mass for this link
          gravity -- if given, will create gravity wrench
                     In link COM frame.
      Will create factor corresponding to Lynch & Park book:
          - wrench balance, Equation 8.48, page 293
   */
  ToolWrenchFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                   gtsam::Key wrench_key_j, gtsam::Key pose_key,
                   const gtsam::noiseModel::Base::shared_ptr &cost_model,
                   const gtsam::Pose3 &tTn, const gtsam::Matrix6 &inertia,
                   const gtsam::Vector6 &external_wrench,
                   const boost::optional<gtsam::Vector3> &gravity = boost::none)
      : Base(cost_model, twist_key, twistAccel_key, wrench_key_j, pose_key),
        tTn_(tTn),
        inertia_(inertia),
        external_wrench_(-external_wrench) {
    gravity_.setZero();
    if (gravity) {
      gravity_ = *gravity;
    }
  }
  virtual ~ToolWrenchFactor() {}

 private:
  /* calculate jacobian of coriolis term w.r.t. joint coordinate twist */
  gtsam::Matrix6 twistJacobian_(const gtsam::Vector6 &twist) const {
    // TODO(Mandy): figure out if this can be done with vector math
    auto g1 = inertia_(0, 0), g2 = inertia_(1, 1), g3 = inertia_(2, 2),
         m = inertia_(3, 3);
    auto w1 = twist(0), w2 = twist(1), w3 = twist(2), v1 = twist(3),
         v2 = twist(4), v3 = twist(5);
    gtsam::Matrix6 H_twist;
    H_twist << 0, (g2 - g3) * w3, (g2 - g3) * w2, 0, 0, 0,  //
        (g3 - g1) * w3, 0, (g3 - g1) * w1, 0, 0, 0,         //
        (g1 - g2) * w2, (g1 - g2) * w1, 0, 0, 0, 0,         //
        0, -m * v3, m * v2, 0, m * w3, -m * w2,             //
        m * v3, 0, -m * v1, -m * w3, 0, m * w1,             //
        -m * v2, m * v1, 0, m * w2, -m * w1, 0;
    return H_twist;
  }

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          twsit          -- twist on this link
          twsit_accel    -- twist acceleration on this link
          wrench_j       -- wrench on this link
          pose           -- link com pose expressed in base frame
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist, const gtsam::Vector6 &twistAccel,
      const gtsam::Vector6 &wrench_j, const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_j = boost::none,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_, H_unrotate).vector();
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    gtsam::Vector6 error =
        inertia_ * twistAccel - wrench_j +
        tTn_.AdjointMap().transpose() * external_wrench_ -
        gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist -
        gravity_wrench;

    if (H_twist) {
      *H_twist = -twistJacobian_(twist);
    }
    if (H_twistAccel) {
      *H_twistAccel = inertia_;
    }
    if (H_wrench_j) {
      *H_wrench_j = -gtsam::I_6x6;
    }
    if (H_pose) {
      *H_pose = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
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
    std::cout << s << "tool wrench factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace manipulator
