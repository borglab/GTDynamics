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
#include <vector>

namespace manipulator {

/** ToolWrenchFactor is a four-way nonlinear factor which enforces the tool
 * wrench*/
class ToolWrenchFactor
    : public gtsam::NoiseModelFactor4<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Pose3> {
 private:
  typedef ToolWrenchFactor This;
  typedef gtsam::NoiseModelFactor4<gtsam::Vector, gtsam::Vector, gtsam::Vector,
                                   gtsam::Pose3>
      Base;
  gtsam::Pose3 tTn_;
  gtsam::Matrix inertia_;
  gtsam::Vector external_wrench_;
  gtsam::Vector gravity_;

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
                   const gtsam::Pose3 &tTn, const gtsam::Matrix &inertia,
                   const gtsam::Vector &external_wrench,
                   boost::optional<gtsam::Vector3 &> gravity = boost::none)
      : Base(cost_model, twist_key, twistAccel_key, wrench_key_j, pose_key),
        tTn_(tTn),
        inertia_(inertia),
        external_wrench_(-external_wrench) {
    gravity_ = (gtsam::Vector(3) << 0, 0, 0).finished();
    if (gravity) {
      gravity_ = *gravity;
    }
  }
  virtual ~ToolWrenchFactor() {}

 private:
  /* calculate jacobian of coriolis term w.r.t. joint coordinate twist */
  gtsam::Matrix twistJacobian_(const gtsam::Vector &twist) const {
    auto g1 = inertia_(0, 0), g2 = inertia_(1, 1), g3 = inertia_(2, 2),
         m = inertia_(3, 3);
    auto w1 = twist(0), w2 = twist(1), w3 = twist(2), v1 = twist(3),
         v2 = twist(4), v3 = twist(5);
    gtsam::Matrix H_twist =
        (gtsam::Matrix(6, 6) << 0, (g2 - g3) * w3, (g2 - g3) * w2, 0, 0, 0,
         (g3 - g1) * w3, 0, (g3 - g1) * w1, 0, 0, 0, (g1 - g2) * w2,
         (g1 - g2) * w1, 0, 0, 0, 0, 0, -m * v3, m * v2, 0, m * w3, -m * w2,
         m * v3, 0, -m * v1, -m * w3, 0, m * w1, -m * v2, m * v1, 0, m * w2,
         -m * w1, 0)
            .finished();
    return H_twist;
  }

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          twsit          -- twist on this link
          twsit_accel    -- twist acceleration on this link
          wrench_j       -- wrench on this link
          pose           -- link com pose expressed in base frame
          H_twist        -- jacobian matrix w.r.t. twist
          H_twist_accel  -- jacobian matrix w.r.t. twist acceleration
          H_wrench_j     -- jacobian matrix w.r.t. wrench on this link
          H_pose         -- jacobian matrix w.r.t. link com pose
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector &twist, const gtsam::Vector &twistAccel,
      const gtsam::Vector &wrench_j, const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_twistAccel = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench_j = boost::none,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const {
    int size = wrench_j.size();
    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate).vector();
    gtsam::Matrix63 mat;
    mat << gtsam::Matrix3::Zero(), gtsam::Matrix::Identity(3, 3);
    auto gravity_wrench = inertia_ * mat * gravity;

    gtsam::Vector error =
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
      *H_wrench_j = -gtsam::Matrix::Identity(size, size);
    }
    if (H_pose) {
      *H_pose = -inertia_ * mat * H_unrotate * H_rotation;
    }

    return error;
  }

  // overload evaluateError
  /** evaluate wrench balance errors
    Keyword argument:
        twsit          -- twist on this link
        twsit_accel    -- twist acceleration on this link
        wrench_j       -- wrench on this link
        pose           -- link com pose expressed in base frame
*/
  gtsam::Vector evaluateError(const gtsam::Vector &twist,
                              const gtsam::Vector &twistAccel,
                              const gtsam::Vector &wrench_j,
                              const gtsam::Pose3 &pose) const {
    // transform gravity from base frame to link COM frame,
    // to use unrotate function, have to convert gravity vector to a point
    gtsam::Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
    gtsam::Matrix H_rotation, H_unrotate;
    auto gravity =
        pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate).vector();
    gtsam::Matrix63 mat;
    mat << gtsam::Matrix3::Zero(), gtsam::Matrix::Identity(3, 3);
    auto gravity_wrench = inertia_ * mat * gravity;

    return inertia_ * twistAccel - wrench_j +
           tTn_.AdjointMap().transpose() * external_wrench_ -
           gtsam::Pose3::adjointMap(twist).transpose() * inertia_ * twist -
           gravity_wrench;
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
