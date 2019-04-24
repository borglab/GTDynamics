/**
 * @file  WrenchFactor.h
 * @brief wrench balance factor, common between forward and inverse dynamics.
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <utils.h>

#include <boost/optional.hpp>
#include <iostream>
#include <vector>

namespace manipulator {

/** WrenchFactor is a six-way nonlinear factor which enforces relation
 * between wrenches on this link and the next link*/
class WrenchFactor
    : public gtsam::NoiseModelFactor6<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Pose3, double> {
 private:
  typedef WrenchFactor This;
  typedef gtsam::NoiseModelFactor6<gtsam::Vector6, gtsam::Vector6,
                                   gtsam::Vector6, gtsam::Vector6, gtsam::Pose3,
                                   double>
      Base;
  gtsam::Pose3 kMj_;
  gtsam::Matrix6 inertia_;
  gtsam::Vector6 screw_axis_;
  gtsam::Vector3 gravity_;

 public:
  /** wrench balance factor, common between forward and inverse dynamics.
      Keyword argument:
          kMj        -- this COM frame, expressed in next link's COM frame
          inertia    -- moment of inertia and mass for this link
          screw_axis -- screw axis expressed in kth link's COM frame
          gravity    -- if given, will create gravity wrench. In link
     COM frame. Will create factor corresponding to Lynch & Park book:
          - wrench balance, Equation 8.48, page 293
   */
  WrenchFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key,
               gtsam::Key wrench_key_j, gtsam::Key wrench_key_k,
               gtsam::Key pose_key, gtsam::Key q_key,
               const gtsam::noiseModel::Base::shared_ptr &cost_model,
               const gtsam::Pose3 &kMj, const gtsam::Matrix6 &inertia,
               const gtsam::Vector6 &screw_axis,
               boost::optional<gtsam::Vector3 &> gravity = boost::none)
      : Base(cost_model, twist_key, twistAccel_key, wrench_key_j, wrench_key_k,
             pose_key, q_key),
        kMj_(kMj),
        inertia_(inertia),
        screw_axis_(screw_axis) {
    gravity_ = (gtsam::Vector(3) << 0, 0, 0).finished();
    if (gravity) {
      gravity_ = *gravity;
    }
  }
  virtual ~WrenchFactor() {}

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
         (g3 - g1) * w3, 0, (g3 - g1) * w1, 0, 0, 0,                         //
         (g1 - g2) * w2, (g1 - g2) * w1, 0, 0, 0, 0,                         //
         0, -m * v3, m * v2, 0, m * w3, -m * w2,                             //
         m * v3, 0, -m * v1, -m * w3, 0, m * w1,                             //
         -m * v2, m * v1, 0, m * w2, -m * w1, 0;
    return H_twist;
  }

  /* calculate joint coordinate q jacobian */
  gtsam::Matrix61 qJacobian_(double q, const gtsam::Vector6 &wrench_k) const {
    auto H = AdjointMapJacobianQ(q, kMj_, screw_axis_);
    return H.transpose() * wrench_k;
  }

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          twsit         -- twist on this link
          twsit_accel   -- twist acceleration on this link
          wrench_j      -- wrench on this link
          wrench_k      -- wrench from the next link
          H_twist       -- jacobian matrix w.r.t. twist
          H_twist_accel -- jacobian matrix w.r.t. twist acceleration
          H_wrench_j    -- jacobian matrix w.r.t. wrench on this link
          H_wrench_k    -- jacobian matrix w.r.t. wrench from the next
                           link
          H_pose        -- jacobian matrix w.r.t. link com pose
          H_q           -- jacobian matrix w.r.t. joint angle
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist, const gtsam::Vector6 &twistAccel,
      const gtsam::Vector6 &wrench_j, const gtsam::Vector6 &wrench_k,
      const gtsam::Pose3 &pose, const double &q,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_j = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_k = boost::none,
      boost::optional<gtsam::Matrix &> H_pose = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const {
    gtsam::Pose3 kTj = gtsam::Pose3::Expmap(-screw_axis_ * q) * kMj_;
    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate).vector();
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    gtsam::Vector6 error =
        inertia_ * twistAccel - wrench_j +
        kTj.AdjointMap().transpose() * wrench_k -
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
    if (H_wrench_k) {
      *H_wrench_k = kTj.AdjointMap().transpose();
    }
    if (H_q) {
      *H_q = qJacobian_(q, wrench_k);
    }
    if (H_pose) {
      *H_pose = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
    }

    return error;
  }

  // overload evaluateError
  /** evaluate wrench balance errors
    Keyword argument:
        twsit         -- twist on this link
        twsit_accel   -- twist acceleration on this link
        wrench_j      -- wrench on this link
        wrench_k      -- wrench from the next link
        pose          -- link COM pose expressed in base frame
        q             -- joint angle
*/
  gtsam::Vector evaluateError(const gtsam::Vector6 &twist,
                              const gtsam::Vector6 &twistAccel,
                              const gtsam::Vector6 &wrench_j,
                              const gtsam::Vector6 &wrench_k,
                              const gtsam::Pose3 pose, const double &q) const {
    gtsam::Pose3 kTj = gtsam::Pose3::Expmap(-screw_axis_ * q) * kMj_;
    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    auto gravity = pose.rotation().unrotate(gravity_point).vector();
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    return inertia_ * twistAccel - wrench_j +
           kTj.AdjointMap().transpose() * wrench_k -
           gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist -
           gravity_wrench;
  }

  // for unit test in checking jacobian matrix with numericalDerivative11, boost
  // bind can bind no more than 9 arguments

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override{
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "wrench factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor6", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace manipulator
