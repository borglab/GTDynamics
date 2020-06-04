/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CollocationFactors.h
 * @brief collocation factor on link poses and twists.
 * @Author: Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtdynamics {

/** predict link pose
    Keyword argument:
        pose_i        -- link pose at current time step
        twistdt       -- link twist * dt
    Returns:
        pose_j        -- link pose at next time step
*/
gtsam::Pose3 predictPose(
    const gtsam::Pose3 &pose_i, const gtsam::Vector6 &twistdt,
    gtsam::OptionalJacobian<6, 6> H_pose_i = boost::none,
    gtsam::OptionalJacobian<6, 6> H_twistdt = boost::none) {
  gtsam::Matrix6 Hexp;
  gtsam::Pose3 jTi = gtsam::Pose3::Expmap(twistdt, Hexp);

  gtsam::Matrix6 pose_j_H_jTi;
  auto pose_j = pose_i.compose(jTi, H_pose_i, H_twistdt ? & pose_j_H_jTi : nullptr);
  if (H_twistdt) {
    *H_twistdt = pose_j_H_jTi * (Hexp);
  }
  return pose_j;
}

/** EulerPoseColloFactor is a four-way nonlinear factor between link pose of
 * current and next time steps*/
class EulerPoseColloFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3,
                                      gtsam::Vector6, double> {
 private:
  typedef EulerPoseColloFactor This;
  using Base = gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3,
                                        gtsam::Vector6, double>;

 public:
  EulerPoseColloFactor(gtsam::Key pose_key_i, gtsam::Key pose_key_j,
                       gtsam::Key twist_key, gtsam::Key dt_key,
                       const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, pose_key_i, pose_key_j, twist_key, dt_key) {}

  virtual ~EulerPoseColloFactor() {}

  /** evaluate link pose errors
      Keyword argument:
          pose_i         -- link pose of current step
          pose_j         -- link pose of next step
          twist          -- link twist
          dt             -- duration of time step
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose_i, const gtsam::Pose3 &pose_j,
      const gtsam::Vector6 &twist, const double &dt,
      boost::optional<gtsam::Matrix &> H_pose_i = boost::none,
      boost::optional<gtsam::Matrix &> H_pose_j = boost::none,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_dt = boost::none) const override {
    gtsam::Vector6 twistdt = twist * dt;
    gtsam::Matrix6 H_twistdt;
    auto pose_j_hat = predictPose(pose_i, twistdt, H_pose_i, H_twistdt);
    gtsam::Vector6 error = pose_j.logmap(pose_j_hat);
    if (H_pose_j) {
      *H_pose_j = -gtsam::I_6x6;
    }
    if (H_twist) {
      *H_twist = H_twistdt * dt;
    }
    if (H_dt) {
      *H_dt = H_twistdt * twist;
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
    std::cout << s << "Euler collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};

/** TrapezoidalPoseColloFactor is a five-way nonlinear factor between link pose
 * of current and next time steps*/
class TrapezoidalPoseColloFactor
    : public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3,
                                      gtsam::Vector6, gtsam::Vector6, double> {
 private:
  typedef TrapezoidalPoseColloFactor This;
  using Base = gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Vector6,
                                   gtsam::Vector6, double>;

 public:
  TrapezoidalPoseColloFactor(
      gtsam::Key pose_key_i, gtsam::Key pose_key_j, gtsam::Key twist_i_key,
      gtsam::Key twist_j_key, gtsam::Key dt_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, pose_key_i, pose_key_j, twist_i_key, twist_j_key,
             dt_key) {}

  virtual ~TrapezoidalPoseColloFactor() {}

  /** evaluate link pose errors
      Keyword argument:
          pose_i         -- link pose of current step
          pose_j         -- link pose of next step
          twist_i        -- link twist of current step
          twist_j        -- link twist of next step
          dt             -- duration of time step
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose_i, const gtsam::Pose3 &pose_j,
      const gtsam::Vector6 &twist_i, const gtsam::Vector6 &twist_j,
      const double &dt, boost::optional<gtsam::Matrix &> H_pose_i = boost::none,
      boost::optional<gtsam::Matrix &> H_pose_j = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_i = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_j = boost::none,
      boost::optional<gtsam::Matrix &> H_dt = boost::none) const override {
    gtsam::Vector6 twistdt = 0.5 * dt * (twist_i + twist_j);
    gtsam::Matrix6 H_twistdt;
    auto pose_j_hat = predictPose(pose_i, twistdt, H_pose_i, H_twistdt);
    gtsam::Vector6 error = pose_j.logmap(pose_j_hat);
    if (H_pose_j) {
      *H_pose_j = -gtsam::I_6x6;
    }
    if (H_twist_i) {
      *H_twist_i = 0.5 * dt * H_twistdt;
    }
    if (H_twist_j) {
      *H_twist_j = 0.5 * dt * H_twistdt;
    }
    if (H_dt) {
      *H_dt = H_twistdt * (0.5 * (twist_i + twist_j));
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
    std::cout << s << "trapezoidal collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};

/** EulerTwistColloFactor is a four-way nonlinear factor between link twist of
 * current and next time steps*/
class EulerTwistColloFactor
    : public gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, double> {
 private:
  typedef EulerTwistColloFactor This;
  typedef gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                   gtsam::Vector6, double>
      Base;

 public:
  EulerTwistColloFactor(gtsam::Key twist_key_i, gtsam::Key twist_key_j,
                        gtsam::Key accel_key, gtsam::Key dt_key,
                        const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, twist_key_i, twist_key_j, accel_key, dt_key) {}

  virtual ~EulerTwistColloFactor() {}

  /** evaluate link twist errors
      Keyword argument:
          twist_i        -- link twist of current step
          twist_j        -- link twist of next step
          accel          -- link twist acceleration
          dt             -- duration of time step
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_i, const gtsam::Vector6 &twist_j,
      const gtsam::Vector6 &accel, const double &dt,
      boost::optional<gtsam::Matrix &> H_twist_i = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_j = boost::none,
      boost::optional<gtsam::Matrix &> H_accel = boost::none,
      boost::optional<gtsam::Matrix &> H_dt = boost::none) const override {
    gtsam::Vector6 error = twist_i + dt * accel - twist_j;
    if (H_twist_j) {
      *H_twist_j = -gtsam::I_6x6;
    }
    if (H_twist_i) {
      *H_twist_i = gtsam::I_6x6;
    }
    if (H_accel) {
      *H_accel = gtsam::I_6x6 * dt;
    }
    if (H_dt) {
      *H_dt = accel;
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
    std::cout << s << "Euler twist collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};

/** TrapezoidalTwistColloFactor is a four-way nonlinear factor between link
 * twist of current and next time steps*/
class TrapezoidalTwistColloFactor
    : public gtsam::NoiseModelFactor5<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Vector6, double> {
 private:
  typedef TrapezoidalTwistColloFactor This;
  typedef gtsam::NoiseModelFactor5<gtsam::Vector6, gtsam::Vector6,
                                   gtsam::Vector6, gtsam::Vector6, double>
      Base;

 public:
  TrapezoidalTwistColloFactor(
      gtsam::Key twist_key_i, gtsam::Key twist_key_j, gtsam::Key accel_key_i,
      gtsam::Key accel_key_j, gtsam::Key dt_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, twist_key_i, twist_key_j, accel_key_i, accel_key_j,
             dt_key) {}

  virtual ~TrapezoidalTwistColloFactor() {}

  /** evaluate link twist errors
      Keyword argument:
          twist_i        -- link twist of current step
          twist_j        -- link twist of next step
          accel_i        -- link twist acceleration of current step
          accel_j        -- link twist acceleration of next step
          dt             -- duration of time step
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_i, const gtsam::Vector6 &twist_j,
      const gtsam::Vector6 &accel_i, const gtsam::Vector6 &accel_j,
      const double &dt,
      boost::optional<gtsam::Matrix &> H_twist_i = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_j = boost::none,
      boost::optional<gtsam::Matrix &> H_accel_i = boost::none,
      boost::optional<gtsam::Matrix &> H_accel_j = boost::none,
      boost::optional<gtsam::Matrix &> H_dt = boost::none) const override {
    gtsam::Vector6 error = twist_i + 0.5 * dt * (accel_i + accel_j) - twist_j;
    if (H_twist_j) {
      *H_twist_j = -gtsam::I_6x6;
    }
    if (H_twist_i) {
      *H_twist_i = gtsam::I_6x6;
    }
    if (H_accel_i) {
      *H_accel_i = 0.5 * dt * gtsam::I_6x6;
    }
    if (H_accel_j) {
      *H_accel_j = 0.5 * dt * gtsam::I_6x6;
    }
    if (H_dt) {
      *H_dt = 0.5 * (accel_i + accel_j);
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
    std::cout << s << "trapezoidal twist collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
