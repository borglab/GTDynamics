/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchFactor.h
 * @brief Wrench balance factor, common between forward and inverse dynamics.
 * @author Frank Dellaert, Mandy Xie and Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/assign/list_of.hpp>
#include <boost/optional.hpp>
#include <boost/serialization/base_object.hpp>
#include <string>
#include <vector>

#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/DynamicsSymbol.h"

namespace gtdynamics {

/**
 * WrenchFactor is an n-way nonlinear factor which enforces relation
 * between wrenches on this link
 */
class WrenchFactor : public gtsam::NoiseModelFactor {
  using This = WrenchFactor;
  using Base = gtsam::NoiseModelFactor;
  gtsam::Matrix6 inertia_;
  gtsam::Vector3 gravity_;
 public:
  /**
   * Wrench balance factor, common between forward and inverse dynamics.
   * Will create factor corresponding to Lynch & Park book:
   *  wrench balance, Equation 8.48, page 293
   * @param inertia Moment of inertia and mass for this link
   * @param gravity (optional) Create gravity wrench in link COM frame.
   */
  WrenchFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key,
               const std::vector<DynamicsSymbol> &wrench_keys,
               gtsam::Key pose_key,
               const gtsam::noiseModel::Base::shared_ptr &cost_model,
               const gtsam::Matrix6 &inertia,
               const boost::optional<gtsam::Vector3> &gravity = boost::none)
      : Base(cost_model),
        inertia_(inertia),
        gravity_(gravity ? *gravity : gtsam::Vector3::Zero()) {
    keys_.reserve(wrench_keys.size() + 3);
    keys_.push_back(twist_key);
    keys_.push_back(twistAccel_key);
    keys_.push_back(pose_key);
    keys_.insert(keys_.end(), wrench_keys.begin(), wrench_keys.end());
  }

 private:
  /// calculate jacobian of coriolis term w.r.t. joint coordinate twist
  gtsam::Matrix6 twistJacobian_(const gtsam::Vector6 &twist) const {
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
  /**
   * Evaluate wrench balance errors
   * @param values contains the twist of the link, twistAccel of the link, pose
   * of the link, and wrenches acting on the link.
   * @param H Jacobians, in the order: twist, twistAccel, pose, *wrenches twist
   * twist of this link
   */
  gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                boost::optional<std::vector<gtsam::Matrix> &>
                                    H = boost::none) const override {
    if (!this->active(x)) {
      return gtsam::Vector::Zero(this->dim());
    }

    // `keys_` order: twist, twistAccel, pose, *wrenches
    const gtsam::Vector6 twist = x.at<gtsam::Vector6>(keys_.at(0));
    const gtsam::Vector6 twistAccel = x.at<gtsam::Vector6>(keys_.at(1));
    const gtsam::Pose3 pose = x.at<gtsam::Pose3>(keys_.at(2));
    gtsam::Vector6 wrenchSum = std::accumulate(  // C++17: reduce
        keys_.cbegin() + 3, keys_.cend(),
        static_cast<gtsam::Vector6>(gtsam::Z_6x1),
        [&x](gtsam::Vector6 v1, gtsam::Key key) -> gtsam::Vector6 {
          return v1 + x.at<gtsam::Vector6>(key);
        });

    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate);
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    // Equation 8.48 (F = ma)
    gtsam::Vector6 error =
        (inertia_ * twistAccel) - wrenchSum - gravity_wrench -
        (gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist);

    if (H) {
      (*H)[0] = -twistJacobian_(twist);
      (*H)[1] = inertia_;
      (*H)[2] = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
      std::fill(H->begin()+3, H->end(), -gtsam::I_6x6);
    }

    return error;
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
    std::cout << s << "wrench factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};

/**
 * WrenchFactor0 is a three-way nonlinear factor which enforces relation
 * between wrenches on this link
 */
class WrenchFactor0
    : public gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Pose3> {
 private:
  using This = WrenchFactor0;
  using Base =
      gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Vector6, gtsam::Pose3>;

  gtsam::Matrix6 inertia_;
  gtsam::Vector3 gravity_;

 public:
  /**
   * Wrench balance factor, common between forward and inverse dynamics.
   *
   * Will create factor corresponding to Lynch & Park book:
   *  wrench balance, Equation 8.48, page 293
   *
   * @param inertia moment of inertia and mass for this link
   * @param gravity (optional) Create gravity wrench in link COM frame.
   */
  WrenchFactor0(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key pose_key,
                const gtsam::noiseModel::Base::shared_ptr &cost_model,
                const gtsam::Matrix6 &inertia,
                const boost::optional<gtsam::Vector3> &gravity = boost::none)
      : Base(cost_model, twist_key, twistAccel_key, pose_key),
        inertia_(inertia) {
    gravity_ = (gtsam::Vector(3) << 0, 0, 0).finished();
    if (gravity) {
      gravity_ = *gravity;
    }
  }
  virtual ~WrenchFactor0() {}

 private:
  /// calculate jacobian of coriolis term w.r.t. joint coordinate twist
  gtsam::Matrix6 twistJacobian_(const gtsam::Vector6 &twist) const {
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
  /**
   * Evaluate wrench balance errors
   * @param twist twist of this link
   * @param twist_accel twist acceleration of this link
   * @param pose pose of this link
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist, const gtsam::Vector6 &twistAccel,
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel = boost::none,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate);
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    gtsam::Vector6 error =
        inertia_ * twistAccel -
        gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist -
        gravity_wrench;

    if (H_twist) {
      *H_twist = -twistJacobian_(twist);
    }
    if (H_twistAccel) {
      *H_twistAccel = inertia_;
    }
    if (H_pose) {
      *H_pose = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
    }

    return error;
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
    std::cout << s << "wrench factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};

/**
 * WrenchFactor1 is a four-way nonlinear factor which enforces relation
 * between wrenches on this link
 */
class WrenchFactor1
    : public gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Pose3> {
 private:
  using This = WrenchFactor1;
  using Base = gtsam::NoiseModelFactor4<gtsam::Vector6, gtsam::Vector6,
                                        gtsam::Vector6, gtsam::Pose3>;

  gtsam::Matrix6 inertia_;
  gtsam::Vector3 gravity_;

 public:
  /**
   * Wrench balance factor, common between forward and inverse dynamics.
   * Will create factor corresponding to Lynch & Park book:
   *  wrench balance, Equation 8.48, page 293
   *
   * @param inertia moment of inertia and mass for this link
   * @param gravity (optional) Create gravity wrench in link COM frame.
   */
  WrenchFactor1(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key wrench_key_1, gtsam::Key pose_key,
                const gtsam::noiseModel::Base::shared_ptr &cost_model,
                const gtsam::Matrix6 &inertia,
                const boost::optional<gtsam::Vector3> &gravity = boost::none)
      : Base(cost_model, twist_key, twistAccel_key, wrench_key_1, pose_key),
        inertia_(inertia) {
    gravity_ = (gtsam::Vector(3) << 0, 0, 0).finished();
    if (gravity) {
      gravity_ = *gravity;
    }
  }
  virtual ~WrenchFactor1() {}

 private:
  /// calculate jacobian of coriolis term w.r.t. joint coordinate twist
  gtsam::Matrix6 twistJacobian_(const gtsam::Vector6 &twist) const {
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
  /**
   * Evaluate wrench balance errors
   * @param twist twist of this link
   * @param twist_accel twist acceleration of this link
   * @param pose pose of this link
   * @param wrench_1 1st wrench on this link
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist, const gtsam::Vector6 &twistAccel,
      const gtsam::Vector6 &wrench_1, const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_1 = boost::none,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate);
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    gtsam::Vector6 error =
        inertia_ * twistAccel - wrench_1 -
        gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist -
        gravity_wrench;

    if (H_twist) {
      *H_twist = -twistJacobian_(twist);
    }
    if (H_twistAccel) {
      *H_twistAccel = inertia_;
    }
    if (H_wrench_1) {
      *H_wrench_1 = -gtsam::I_6x6;
    }
    if (H_pose) {
      *H_pose = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
    }

    return error;
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
    std::cout << s << "wrench factor" << std::endl;
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

/**
 * WrenchFactor2 is a five-way nonlinear factor which enforces relation
 * between wrenches on this link and the next link
 */
class WrenchFactor2
    : public gtsam::NoiseModelFactor5<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Pose3> {
 private:
  using This = WrenchFactor2;
  using Base =
      gtsam::NoiseModelFactor5<gtsam::Vector6, gtsam::Vector6, gtsam::Vector6,
                               gtsam::Vector6, gtsam::Pose3>;
  gtsam::Matrix6 inertia_;
  gtsam::Vector3 gravity_;

 public:
  /**
   * Wrench balance factor, common between forward and inverse dynamics.
   * Will create factor corresponding to Lynch & Park book:
   *  wrench balance, Equation 8.48, page 293
   * @param inertia Moment of inertia and mass for this link
   * @param gravity (optional) Create gravity wrench in link COM frame.
   */
  WrenchFactor2(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                gtsam::Key pose_key,
                const gtsam::noiseModel::Base::shared_ptr &cost_model,
                const gtsam::Matrix6 &inertia,
                const boost::optional<gtsam::Vector3> &gravity = boost::none)
      : Base(cost_model, twist_key, twistAccel_key, wrench_key_1, wrench_key_2,
             pose_key),
        inertia_(inertia) {
    gravity_ = (gtsam::Vector(3) << 0, 0, 0).finished();
    if (gravity) {
      gravity_ = *gravity;
    }
  }
  virtual ~WrenchFactor2() {}

 private:
  /// calculate jacobian of coriolis term w.r.t. joint coordinate twist
  gtsam::Matrix6 twistJacobian_(const gtsam::Vector6 &twist) const {
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
  /**
   * Evaluate wrench balance errors
   * @param twist twist of this link
   * @param twist_accel twist acceleration of this link
   * @param pose pose of this link
   * @param wrench_1 1st wrench on this link
   * @param wrench_2 2nd wrench on this link
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist, const gtsam::Vector6 &twistAccel,
      const gtsam::Vector6 &wrench_1, const gtsam::Vector6 &wrench_2,
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_1 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_2 = boost::none,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate);
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    gtsam::Vector6 error =
        inertia_ * twistAccel - wrench_1 - wrench_2 -
        gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist -
        gravity_wrench;

    if (H_twist) {
      *H_twist = -twistJacobian_(twist);
    }
    if (H_twistAccel) {
      *H_twistAccel = inertia_;
    }
    if (H_wrench_1) {
      *H_wrench_1 = -gtsam::I_6x6;
    }
    if (H_wrench_2) {
      *H_wrench_2 = -gtsam::I_6x6;
    }
    if (H_pose) {
      *H_pose = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
    }

    return error;
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
    std::cout << s << "wrench factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};

/**
 * WrenchFactor3 is a six-way nonlinear factor which enforces relation
 * between wrenches on this link and the next link
 */
class WrenchFactor3
    : public gtsam::NoiseModelFactor6<gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Pose3> {
 private:
  using This = WrenchFactor3;
  using Base =
      gtsam::NoiseModelFactor6<gtsam::Vector6, gtsam::Vector6, gtsam::Vector6,
                               gtsam::Vector6, gtsam::Vector6, gtsam::Pose3>;
  gtsam::Matrix6 inertia_;
  gtsam::Vector3 gravity_;

 public:
  /**
   * Wrench balance factor, common between forward and inverse dynamics.
   * Will create factor corresponding to Lynch & Park book:
   *  wrench balance, Equation 8.48, page 293
   *
   * @param inertia Moment of inertia and mass for this link
   * @param gravity (optional) Create gravity wrench in link COM frame.
   */
  WrenchFactor3(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                gtsam::Key wrench_key_3, gtsam::Key pose_key,
                const gtsam::noiseModel::Base::shared_ptr &cost_model,
                const gtsam::Matrix6 &inertia,
                const boost::optional<gtsam::Vector3> &gravity = boost::none)
      : Base(cost_model, twist_key, twistAccel_key, wrench_key_1, wrench_key_2,
             wrench_key_3, pose_key),
        inertia_(inertia) {
    gravity_ = (gtsam::Vector(3) << 0, 0, 0).finished();
    if (gravity) {
      gravity_ = *gravity;
    }
  }
  virtual ~WrenchFactor3() {}

 private:
  /// calculate jacobian of coriolis term w.r.t. joint coordinate twist
  gtsam::Matrix6 twistJacobian_(const gtsam::Vector6 &twist) const {
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
  /**
   * Evaluate wrench balance errors
   * @param twist twist of this link
   * @param twist_accel twist acceleration of this link
   * @param pose pose of this link
   * @param wrench_1 1st wrench on this link
   * @param wrench_2 2nd wrench on this link
   * @param wrench_3 3rd wrench on this link
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist, const gtsam::Vector6 &twistAccel,
      const gtsam::Vector6 &wrench_1, const gtsam::Vector6 &wrench_2,
      const gtsam::Vector6 &wrench_3, const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_1 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_2 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_3 = boost::none,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate);
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    gtsam::Vector6 error =
        inertia_ * twistAccel - wrench_1 - wrench_2 - wrench_3 -
        gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist -
        gravity_wrench;

    if (H_twist) {
      *H_twist = -twistJacobian_(twist);
    }
    if (H_twistAccel) {
      *H_twistAccel = inertia_;
    }
    if (H_wrench_1) {
      *H_wrench_1 = -gtsam::I_6x6;
    }
    if (H_wrench_2) {
      *H_wrench_2 = -gtsam::I_6x6;
    }
    if (H_wrench_3) {
      *H_wrench_3 = -gtsam::I_6x6;
    }
    if (H_pose) {
      *H_pose = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
    }

    return error;
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
    std::cout << s << "wrench factor" << std::endl;
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

// TODO(Varun) Move this to GTSAM?
/* ************************************************************************* */
/**
 * A convenient base class for creating your own NoiseModelFactor with 7
 * variables.
 * To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5,
          class VALUE6, class VALUE7>
class NoiseModelFactor7 : public gtsam::NoiseModelFactor {
 public:
  // typedefs for value types pulled from keys
  using X1 = VALUE1;
  using X2 = VALUE2;
  using X3 = VALUE3;
  using X4 = VALUE4;
  using X5 = VALUE5;
  using X6 = VALUE6;
  using X7 = VALUE7;

 protected:
  using Base = gtsam::NoiseModelFactor;
  using This =
      NoiseModelFactor7<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6, VALUE7>;
  using Key = gtsam::Key;
  using Matrix = gtsam::Matrix;

 public:
  /**
   * Default Constructor for I/O
   */
  NoiseModelFactor7() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   * @param j5 key of the fifth variable
   * @param j6 key of the fifth variable
   */
  NoiseModelFactor7(const gtsam::SharedNoiseModel &noiseModel, Key j1, Key j2,
                    Key j3, Key j4, Key j5, Key j6, Key j7)
      : Base(noiseModel,
             boost::assign::cref_list_of<7>(j1)(j2)(j3)(j4)(j5)(j6)(j7)) {}

  virtual ~NoiseModelFactor7() {}

  /// methods to retrieve keys
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }
  inline Key key5() const { return keys_[4]; }
  inline Key key6() const { return keys_[5]; }
  inline Key key7() const { return keys_[6]; }

  /**
   * Calls the 7-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class.
   * */
  virtual gtsam::Vector unwhitenedError(
      const gtsam::Values &x,
      boost::optional<std::vector<Matrix> &> H = boost::none) const override {
    if (this->active(x)) {
      if (H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]),
                             x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                             x.at<X5>(keys_[4]), x.at<X6>(keys_[5]),
                             x.at<X7>(keys_[6]), (*H)[0], (*H)[1], (*H)[2],
                             (*H)[3], (*H)[4], (*H)[5], (*H)[6]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]),
                             x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                             x.at<X5>(keys_[4]), x.at<X6>(keys_[5]),
                             x.at<X7>(keys_[6]));
    } else {
      return gtsam::Vector::Zero(this->dim());
    }
  }

  /**
   * Override this method to finish implementing a 7-way factor.
   * If any of the optional Matrix reference arguments are specified, it should
   * compute both the function evaluation and its derivative(s) in X1 (and/or
   * X2, X3).
   */
  virtual gtsam::Vector evaluateError(
      const X1 &, const X2 &, const X3 &, const X4 &, const X5 &, const X6 &,
      const X7 &, boost::optional<Matrix &> H1 = boost::none,
      boost::optional<Matrix &> H2 = boost::none,
      boost::optional<Matrix &> H3 = boost::none,
      boost::optional<Matrix &> H4 = boost::none,
      boost::optional<Matrix &> H5 = boost::none,
      boost::optional<Matrix &> H6 = boost::none,
      boost::optional<Matrix &> H7 = boost::none) const = 0;

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};  // \class NoiseModelFactor7

/**
 * WrenchFactor4 is a six-way nonlinear factor which enforces relation
 * between wrenches on this link and the next link
 */
class WrenchFactor4
    : public NoiseModelFactor7<gtsam::Vector6, gtsam::Vector6, gtsam::Vector6,
                               gtsam::Vector6, gtsam::Vector6, gtsam::Vector6,
                               gtsam::Pose3> {
 private:
  using This = WrenchFactor4;
  using Base = NoiseModelFactor7<gtsam::Vector6, gtsam::Vector6, gtsam::Vector6,
                                 gtsam::Vector6, gtsam::Vector6, gtsam::Vector6,
                                 gtsam::Pose3>;

  gtsam::Matrix6 inertia_;
  gtsam::Vector3 gravity_;

 public:
  /**
   * Wrench balance factor, common between forward and inverse dynamics.
   * Will create factor corresponding to Lynch & Park book:
   *  wrench balance, Equation 8.48, page 293
   *
   * @param inertia Moment of inertia and mass for this link
   * @param gravity (optional) Create gravity wrench in link COM frame.
   */
  WrenchFactor4(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                gtsam::Key wrench_key_3, gtsam::Key wrench_key_4,
                gtsam::Key pose_key,
                const gtsam::noiseModel::Base::shared_ptr &cost_model,
                const gtsam::Matrix6 &inertia,
                const boost::optional<gtsam::Vector3> &gravity = boost::none)
      : Base(cost_model, twist_key, twistAccel_key, wrench_key_1, wrench_key_2,
             wrench_key_3, wrench_key_4, pose_key),
        inertia_(inertia) {
    gravity_ = (gtsam::Vector(3) << 0, 0, 0).finished();
    if (gravity) {
      gravity_ = *gravity;
    }
  }
  virtual ~WrenchFactor4() {}

 private:
  /// calculate jacobian of coriolis term w.r.t. joint coordinate twist
  gtsam::Matrix6 twistJacobian_(const gtsam::Vector6 &twist) const {
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
  /**
   * Evaluate wrench balance errors
   * @param twist twist of this link
   * @param twist_accel twist acceleration of this link
   * @param pose pose of this link
   * @param wrench_1 1st wrench on this link
   * @param wrench_2 2nd wrench on this link
   * @param wrench_3 3rd wrench on this link
   * @param wrench_4 4th wrench on this link
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist, const gtsam::Vector6 &twistAccel,
      const gtsam::Vector6 &wrench_1, const gtsam::Vector6 &wrench_2,
      const gtsam::Vector6 &wrench_3, const gtsam::Vector6 &wrench_4,
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_1 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_2 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_3 = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_4 = boost::none,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate);
    gtsam::Matrix63 intermediateMatrix;
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

    gtsam::Vector6 error =
        inertia_ * twistAccel - wrench_1 - wrench_2 - wrench_3 - wrench_4 -
        gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist -
        gravity_wrench;

    if (H_twist) {
      *H_twist = -twistJacobian_(twist);
    }
    if (H_twistAccel) {
      *H_twistAccel = inertia_;
    }
    if (H_wrench_1) {
      *H_wrench_1 = -gtsam::I_6x6;
    }
    if (H_wrench_2) {
      *H_wrench_2 = -gtsam::I_6x6;
    }
    if (H_wrench_3) {
      *H_wrench_3 = -gtsam::I_6x6;
    }
    if (H_wrench_4) {
      *H_wrench_4 = -gtsam::I_6x6;
    }
    if (H_pose) {
      *H_pose = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
    }

    return error;
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
    std::cout << s << "wrench factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor7", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
