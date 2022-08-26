/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CollocationFactors.h
 * @brief collocation factor on link poses and twists.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtdynamics {

/**
 * Predict link pose
 * @param pose_t0 link pose at current time step
 * @param twistdt link twist * dt
 *
 * @return pose_t1 link pose at next time step
 */
inline gtsam::Pose3 predictPose(
    const gtsam::Pose3 &pose_t0, const gtsam::Vector6 &twistdt,
    gtsam::OptionalJacobian<6, 6> H_pose_t0 = boost::none,
    gtsam::OptionalJacobian<6, 6> H_twistdt = boost::none) {
  gtsam::Matrix6 Hexp;
  gtsam::Pose3 t1Tt0 = gtsam::Pose3::Expmap(twistdt, Hexp);

  gtsam::Matrix6 pose_t1_H_t1Tt0;
  auto pose_t1 =
      pose_t0.compose(t1Tt0, H_pose_t0, H_twistdt ? &pose_t1_H_t1Tt0 : nullptr);
  if (H_twistdt) {
    *H_twistdt = pose_t1_H_t1Tt0 * (Hexp);
  }
  return pose_t1;
}

/**
 * EulerPoseCollocationFactor is a four-way nonlinear factor between link pose of
 * current and next time steps
 */
class EulerPoseCollocationFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3,
                                      gtsam::Vector6, double> {
 private:
  using This = EulerPoseCollocationFactor;
  using Base = gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3,
                                        gtsam::Vector6, double>;

 public:
  EulerPoseCollocationFactor(gtsam::Key pose_t0_key, gtsam::Key pose_t1_key,
                       gtsam::Key twist_key, gtsam::Key dt_key,
                       const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, pose_t0_key, pose_t1_key, twist_key, dt_key) {}

  virtual ~EulerPoseCollocationFactor() {}

  /**
   * Evaluate link pose errors
   * @param pose_t0 link pose of current step
   * @param pose_t1 link pose of next step
   * @param twist link twist
   * @param dt duration of time step
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose_t0, const gtsam::Pose3 &pose_t1,
      const gtsam::Vector6 &twist, const double &dt,
      boost::optional<gtsam::Matrix &> H_pose_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_pose_t1 = boost::none,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_dt = boost::none) const override {
    gtsam::Vector6 twistdt = twist * dt;
    gtsam::Matrix6 H_twistdt;
    auto pose_t1_hat = predictPose(pose_t0, twistdt, H_pose_t0, H_twistdt);
    gtsam::Vector6 error = pose_t1.logmap(pose_t1_hat);
    if (H_pose_t1) {
      *H_pose_t1 = -gtsam::I_6x6;
    }
    if (H_twist) {
      *H_twist = H_twistdt * dt;
    }
    if (H_dt) {
      *H_dt = H_twistdt * twist;
    }
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "Euler collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};

/**
 * TrapezoidalPoseCollocationFactor is a five-way nonlinear factor between link pose
 * of current and next time steps
 */
class TrapezoidalPoseCollocationFactor
    : public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3,
                                      gtsam::Vector6, gtsam::Vector6, double> {
 private:
  using This = TrapezoidalPoseCollocationFactor;
  using Base = gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3,
                                        gtsam::Vector6, gtsam::Vector6, double>;

 public:
  TrapezoidalPoseCollocationFactor(
      gtsam::Key pose_t0_key, gtsam::Key pose_t1_key, gtsam::Key twist_t0_key,
      gtsam::Key twist_t1_key, gtsam::Key dt_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, pose_t0_key, pose_t1_key, twist_t0_key, twist_t1_key,
             dt_key) {}

  virtual ~TrapezoidalPoseCollocationFactor() {}

  /**
   * Evaluate link pose errors

   * @param pose_t0 link pose of current step
   * @param pose_t1 link pose of next step
   * @param twist_t0 link twist of current step
   * @param twist_t1 link twist of next step
   * @param dt duration of time step
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose_t0, const gtsam::Pose3 &pose_t1,
      const gtsam::Vector6 &twist_t0, const gtsam::Vector6 &twist_t1,
      const double &dt,
      boost::optional<gtsam::Matrix &> H_pose_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_pose_t1 = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_t1 = boost::none,
      boost::optional<gtsam::Matrix &> H_dt = boost::none) const override {
    gtsam::Vector6 twistdt = 0.5 * dt * (twist_t0 + twist_t1);
    gtsam::Matrix6 H_twistdt;
    auto pose_t1_hat = predictPose(pose_t0, twistdt, H_pose_t0, H_twistdt);
    gtsam::Vector6 error = pose_t1.logmap(pose_t1_hat);
    if (H_pose_t1) {
      *H_pose_t1 = -gtsam::I_6x6;
    }
    if (H_twist_t0) {
      *H_twist_t0 = 0.5 * dt * H_twistdt;
    }
    if (H_twist_t1) {
      *H_twist_t1 = 0.5 * dt * H_twistdt;
    }
    if (H_dt) {
      *H_dt = H_twistdt * (0.5 * (twist_t0 + twist_t1));
    }
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "trapezoidal collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};


/**
 * TrapezoidalPoseCollocationFactor is a five-way nonlinear factor between link pose
 * of current and next time steps
 */
class FixTimeTrapezoidalPoseCollocationFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3,
                                      gtsam::Vector6, gtsam::Vector6> {
 private:
  using This = FixTimeTrapezoidalPoseCollocationFactor;
  using Base = gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3,
                                        gtsam::Vector6, gtsam::Vector6>;
  double dt_;

 public:
  FixTimeTrapezoidalPoseCollocationFactor(
      gtsam::Key pose_t0_key, gtsam::Key pose_t1_key, gtsam::Key twist_t0_key,
      gtsam::Key twist_t1_key, double dt,
      const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, pose_t0_key, pose_t1_key, twist_t0_key, twist_t1_key), dt_(dt) {}

  virtual ~FixTimeTrapezoidalPoseCollocationFactor() {}

  /**
   * Evaluate link pose errors

   * @param pose_t0 link pose of current step
   * @param pose_t1 link pose of next step
   * @param twist_t0 link twist of current step
   * @param twist_t1 link twist of next step
   * @param dt duration of time step
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose_t0, const gtsam::Pose3 &pose_t1,
      const gtsam::Vector6 &twist_t0, const gtsam::Vector6 &twist_t1,
      boost::optional<gtsam::Matrix &> H_pose_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_pose_t1 = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_t1 = boost::none) const override {
    gtsam::Vector6 twistdt = 0.5 * dt_ * (twist_t0 + twist_t1);
    gtsam::Matrix6 H_twistdt;
    auto pose_t1_hat = predictPose(pose_t0, twistdt, H_pose_t0, H_twistdt);
    gtsam::Vector6 error = pose_t1.logmap(pose_t1_hat);
    if (H_pose_t1) {
      *H_pose_t1 = -gtsam::I_6x6;
    }
    if (H_twist_t0) {
      *H_twist_t0 = 0.5 * dt_ * H_twistdt;
    }
    if (H_twist_t1) {
      *H_twist_t1 = 0.5 * dt_ * H_twistdt;
    }
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "trapezoidal collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};

/**
 * EulerTwistCollocationFactor is a four-way nonlinear factor between link twist of
 * current and next time steps
 */
class EulerTwistCollocationFactor
    : public gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, double> {
 private:
  using This = EulerTwistCollocationFactor;
  using Base = gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                        gtsam::Vector6, double>;

 public:
  EulerTwistCollocationFactor(gtsam::Key twist_t0_key, gtsam::Key twist_t1_key,
                        gtsam::Key accel_key, gtsam::Key dt_key,
                        const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, twist_t0_key, twist_t1_key, accel_key, dt_key) {}

  virtual ~EulerTwistCollocationFactor() {}

  /**
   * Evaluate link twist errors

   * @param twist_t0 link twist of current step
   * @param twist_t1 link twist of next step
   * @param accel    link twist acceleration
   * @param dt       duration of time step
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_t0, const gtsam::Vector6 &twist_t1,
      const gtsam::Vector6 &accel, const double &dt,
      boost::optional<gtsam::Matrix &> H_twist_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_t1 = boost::none,
      boost::optional<gtsam::Matrix &> H_accel = boost::none,
      boost::optional<gtsam::Matrix &> H_dt = boost::none) const override {
    gtsam::Vector6 error = twist_t0 + dt * accel - twist_t1;
    if (H_twist_t1) {
      *H_twist_t1 = -gtsam::I_6x6;
    }
    if (H_twist_t0) {
      *H_twist_t0 = gtsam::I_6x6;
    }
    if (H_accel) {
      *H_accel = gtsam::I_6x6 * dt;
    }
    if (H_dt) {
      *H_dt = accel;
    }
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "Euler twist collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};

/**
 * TrapezoidalTwistCollocationFactor is a four-way nonlinear factor between link
 * twist of current and next time steps
 */
class TrapezoidalTwistCollocationFactor
    : public gtsam::NoiseModelFactor5<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Vector6, double> {
 private:
  using This = TrapezoidalTwistCollocationFactor;
  using Base = gtsam::NoiseModelFactor5<gtsam::Vector6, gtsam::Vector6,
                                        gtsam::Vector6, gtsam::Vector6, double>;

 public:
  TrapezoidalTwistCollocationFactor(
      gtsam::Key twist_t0_key, gtsam::Key twist_t1_key, gtsam::Key accel_t0_key,
      gtsam::Key accel_t1_key, gtsam::Key dt_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, twist_t0_key, twist_t1_key, accel_t0_key, accel_t1_key,
             dt_key) {}

  virtual ~TrapezoidalTwistCollocationFactor() {}

  /**
   * Evaluate link twist errors
   *
   * @param twist_t0 link twist of current step
   * @param twist_t1 link twist of next step
   * @param accel_t0 link twist acceleration of current step
   * @param accel_t1 link twist acceleration of next step
   * @param dt duration of time step
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_t0, const gtsam::Vector6 &twist_t1,
      const gtsam::Vector6 &accel_t0, const gtsam::Vector6 &accel_t1,
      const double &dt,
      boost::optional<gtsam::Matrix &> H_twist_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_t1 = boost::none,
      boost::optional<gtsam::Matrix &> H_accel_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_accel_t1 = boost::none,
      boost::optional<gtsam::Matrix &> H_dt = boost::none) const override {
    gtsam::Vector6 error =
        twist_t0 + 0.5 * dt * (accel_t0 + accel_t1) - twist_t1;
    if (H_twist_t1) {
      *H_twist_t1 = -gtsam::I_6x6;
    }
    if (H_twist_t0) {
      *H_twist_t0 = gtsam::I_6x6;
    }
    if (H_accel_t0) {
      *H_accel_t0 = 0.5 * dt * gtsam::I_6x6;
    }
    if (H_accel_t1) {
      *H_accel_t1 = 0.5 * dt * gtsam::I_6x6;
    }
    if (H_dt) {
      *H_dt = 0.5 * (accel_t0 + accel_t1);
    }
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "trapezoidal twist collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};


/**
 * FixTimeTrapezoidalTwistCollocationFactor is a four-way nonlinear factor between link
 * twist of current and next time steps
 */
class FixTimeTrapezoidalTwistCollocationFactor
    : public gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Vector6> {
 private:
  using This = FixTimeTrapezoidalTwistCollocationFactor;
  using Base = gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                        gtsam::Vector6, gtsam::Vector6>;
  double dt_;

 public:
  FixTimeTrapezoidalTwistCollocationFactor(
      gtsam::Key twist_t0_key, gtsam::Key twist_t1_key, gtsam::Key accel_t0_key,
      gtsam::Key accel_t1_key, double dt,
      const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, twist_t0_key, twist_t1_key, accel_t0_key, accel_t1_key),
      dt_(dt) {}

  virtual ~FixTimeTrapezoidalTwistCollocationFactor() {}

  /**
   * Evaluate link twist errors
   *
   * @param twist_t0 link twist of current step
   * @param twist_t1 link twist of next step
   * @param accel_t0 link twist acceleration of current step
   * @param accel_t1 link twist acceleration of next step
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist_t0, const gtsam::Vector6 &twist_t1,
      const gtsam::Vector6 &accel_t0, const gtsam::Vector6 &accel_t1,
      boost::optional<gtsam::Matrix &> H_twist_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_twist_t1 = boost::none,
      boost::optional<gtsam::Matrix &> H_accel_t0 = boost::none,
      boost::optional<gtsam::Matrix &> H_accel_t1 = boost::none) const override {
    gtsam::Vector6 error =
        twist_t0 + 0.5 * dt_ * (accel_t0 + accel_t1) - twist_t1;
    if (H_twist_t1) {
      *H_twist_t1 = -gtsam::I_6x6;
    }
    if (H_twist_t0) {
      *H_twist_t0 = gtsam::I_6x6;
    }
    if (H_accel_t0) {
      *H_accel_t0 = 0.5 * dt_ * gtsam::I_6x6;
    }
    if (H_accel_t1) {
      *H_accel_t1 = 0.5 * dt_ * gtsam::I_6x6;
    }
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "trapezoidal twist collocation factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};


}  // namespace gtdynamics
