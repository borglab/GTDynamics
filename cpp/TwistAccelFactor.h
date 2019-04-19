/**
 * @file  TwistAccelFactor.h
 * @brief twist acceleration factor, common between forward and inverse
 * dynamics.
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

/** TwistAccelFactor is a six-way nonlinear factor which enforces relation
 * between acceleration on previous link and this link*/
class TwistAccelFactor
    : public gtsam::NoiseModelFactor6<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, double, double, double> {
 private:
  typedef TwistAccelFactor This;
  typedef gtsam::NoiseModelFactor6<gtsam::Vector6, gtsam::Vector6, gtsam::Vector6,
                                   double, double, double>
      Base;
  gtsam::Pose3 jMi_;
  gtsam::Vector6 screw_axis_;

 public:
  /** factor linking this link's twist_accel, joint_coordinate, joint_vel,
     joint_accel with previous link's twist_accel. Keyword arguments: jMi --
     previous COM frame, expressed in this link's COM frame, at rest
     configuration screw_axis -- screw axis expressed in link's COM frame Will
     create factor corresponding to Lynch & Park book:
          - twist acceleration, Equation 8.47, page 293
   */
  TwistAccelFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key_i,
                   gtsam::Key twistAccel_key_j, gtsam::Key q_key,
                   gtsam::Key qVel_key, gtsam::Key qAccel_key,
                   const gtsam::noiseModel::Base::shared_ptr &cost_model,
                   const gtsam::Pose3 &jMi, const gtsam::Vector6 &screw_axis)
      : Base(cost_model, twist_key, twistAccel_key_i, twistAccel_key_j, q_key,
             qVel_key, qAccel_key),
        jMi_(jMi),
        screw_axis_(screw_axis) {}
  virtual ~TwistAccelFactor() {}

 private:
  /* calculate jacobian of adjointV term w.r.t. joint coordinate twist */
  gtsam::Matrix6 twistJacobian_(const double &qVel) const {
    gtsam::Matrix6 H_twist = -gtsam::Pose3::adjointMap(screw_axis_ * qVel);
    return H_twist;
  }

  /* calculate jacobian of AdjointMap term w.r.t. joint coordinate q */
  gtsam::Matrix61 qJacobian_(const double &q,
                           const gtsam::Vector6 &twist_accel_i) const {
    auto H = AdjointMapJacobianQ(q, jMi_, screw_axis_);
    return H * twist_accel_i;
  }

 public:
  /** evaluate twist acceleration errors
      Keyword argument:
          twistAccel_i          -- twist acceleration on previous link
          twistAccel_j          -- twist acceleration on this link
          twist                 -- twist on this link
          q                     -- joint coordination
          qVel                  -- joint velocity
          qAccel                -- joint acceleration
          H_twistAccel_i        -- jacobian matrix w.r.t. twistAccel_i
          H_twistAccel_j        -- jacobian matrix w.r.t. twistAccel_j
          H_twist               -- jacobian matrix w.r.t. twist
          H_q                   -- jacobian matrix w.r.t. joint coordinate
          H_qVel                -- jacobian matrix w.r.t. joint velocity
          H_qAccel              -- jacobian matrix w.r.t. joint acceleration
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist, const gtsam::Vector6 &twistAccel_i,
      const gtsam::Vector6 &twistAccel_j, const double &q, const double &qVel,
      const double &qAccel,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel_i = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel_j = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel = boost::none,
      boost::optional<gtsam::Matrix &> H_qAccel = boost::none) const {
    gtsam::Pose3 jTi = gtsam::Pose3::Expmap(-screw_axis_ * q) * jMi_;
    gtsam::Matrix6 H_adjoint;
    gtsam::Vector6 error =
        twistAccel_j - jTi.AdjointMap() * twistAccel_i -
        gtsam::Pose3::adjoint(twist, screw_axis_ * qVel, H_adjoint) -
        screw_axis_ * qAccel;
    if (H_twist) {
      *H_twist = -H_adjoint;
    }
    if (H_twistAccel_i) {
      *H_twistAccel_i = -jTi.AdjointMap();
    }
    if (H_twistAccel_j) {
      *H_twistAccel_j = gtsam::I_6x6;
    }
    if (H_q) {
      *H_q = -qJacobian_(q, twistAccel_i);
    }
    if (H_qVel) {
      *H_qVel = -gtsam::Pose3::adjointMap(twist) * screw_axis_;
    }
    if (H_qAccel) {
      *H_qAccel = -screw_axis_;
    }

    return error;
  }

  // overload evaluateError
  /** evaluate twist acceleration errors
    Keyword argument:
        twistAccel_i          -- twist acceleration on previous link
        twistAccel_j          -- twist acceleration on this link
        twist                 -- twist on this link
        q                     -- joint coordination
        qVel                  -- joint velocity
        qAccel                -- joint acceleration
*/
  gtsam::Vector evaluateError(const gtsam::Vector6 &twist,
                              const gtsam::Vector6 &twistAccel_i,
                              const gtsam::Vector6 &twistAccel_j,
                              const double &q, const double &qVel,
                              const double &qAccel) const {
    gtsam::Pose3 jTi = gtsam::Pose3::Expmap(-screw_axis_ * q) * jMi_;
    return twistAccel_j - jTi.AdjointMap() * twistAccel_i -
           gtsam::Pose3::adjointMap(twist) * screw_axis_ * qVel -
           screw_axis_ * qAccel;
  }

  // @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "twist acceleration factor" << std::endl;
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
