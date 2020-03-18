/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactDynamicsFrictionConeFactor.h
 * @brief Factor to enforce that the linear contact force lies within a
 *  friction cone.
 * @Author: Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include "gtdynamics/utils/Utils.h"

namespace gtdynamics {

/** ContactDynamicsFrictionConeFactor is binary nonlinear factor which enforces
 *  that the linear contact force lies within a friction cone. */
class ContactDynamicsFrictionConeFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector6> {
 private:
  typedef ContactDynamicsFrictionConeFactor This;
  typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector6> Base;
  int up_axis_;  // Which axis is up (assuming flat ground)? {0: x, 1: y, 2: z}.
  double mu_prime_;  // static friction coefficient squared.
  const gtsam::Matrix36 H_wrench_ = (gtsam::Matrix(3, 6) << 0, 0, 0, 1, 0, 0, 0,
                                     0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1)
                                        .finished();
  const gtsam::Matrix33 H_x_ =
      (gtsam::Matrix(3, 3) << 1, 0, 0, 0, 0, 0, 0, 0, 0).finished();
  const gtsam::Matrix33 H_y_ =
      (gtsam::Matrix(3, 3) << 0, 0, 0, 0, 1, 0, 0, 0, 0).finished();
  const gtsam::Matrix33 H_z_ =
      (gtsam::Matrix(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 1).finished();

 public:
  /** Contact dynamics factor for zero moment at contact.
      Keyword argument:
          pose_key -- gtsam::Key corresponding to the link's Com pose.
          contact_wrench_key -- gtsam::Key corresponding to this link's
            contact wrench.
          cost_model -- gtsam::noiseModel for this factor.
          cTcom      -- Contact frame expressed in com frame.
   */
  ContactDynamicsFrictionConeFactor(
      gtsam::Key pose_key, gtsam::Key contact_wrench_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model, double mu,
      const gtsam::Vector3 &gravity)
      : Base(cost_model, pose_key, contact_wrench_key), mu_prime_(mu * mu) {
    if (gravity[0] != 0)
      up_axis_ = 0;  // x.
    else if (gravity[1] != 0)
      up_axis_ = 1;  // y.
    else
      up_axis_ = 2;  // z.
  }
  virtual ~ContactDynamicsFrictionConeFactor() {}

 public:
  /** Evaluate contact point moment errors.
      Keyword argument:
          contact_wrench         -- Contact wrench on this link.
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose, const gtsam::Vector6 &contact_wrench,
      boost::optional<gtsam::Matrix &> H_pose = boost::none,
      boost::optional<gtsam::Matrix &> H_contact_wrench =
          boost::none) const override {
    // Linear component of the contact wrench.
    gtsam::Matrix f_c = H_wrench_ * contact_wrench;

    // Rotate linear contact wrench force into the spatial frame.
    gtsam::Matrix36 H_p;
    gtsam::Matrix f_c_prime = pose.rotation(H_p) * f_c;

    // Compute the squared force values.
    gtsam::Matrix A = gtsam::trans(gtsam::Matrix(f_c_prime)) * H_x_;
    gtsam::Matrix B = gtsam::trans(gtsam::Matrix(f_c_prime)) * H_y_;
    gtsam::Matrix C = gtsam::trans(gtsam::Matrix(f_c_prime)) * H_z_;

    if (up_axis_ == 0)
      A *= -mu_prime_;
    else if (up_axis_ == 1)
      B *= -mu_prime_;
    else
      C *= -mu_prime_;

    // a = f_x^2, b = f_z^2, c = f_z^2.
    gtsam::Matrix a = A * gtsam::Matrix(f_c_prime);
    gtsam::Matrix b = B * gtsam::Matrix(f_c_prime);
    gtsam::Matrix c = C * gtsam::Matrix(f_c_prime);

    gtsam::Matrix resultant = a + b + c;
    gtsam::Vector error;

    // Ramp function.
    if (resultant(0, 0) > 0)
      error = resultant;
    else
      error = (gtsam::Vector(1) << 0).finished();

    // Compute the gradients based on whether or not the inequality constraint
    // is active.
    gtsam::Matrix H_f_c_prime = (2 * A + 2 * B + 2 * C);
    if (H_contact_wrench) {
      if (resultant(0, 0) > 0) {  // Active.
        gtsam::Matrix H_f_c = H_f_c_prime * pose.rotation().matrix();
        *H_contact_wrench = H_f_c * H_wrench_;
      } else {  // Inactive.
        *H_contact_wrench =
            (gtsam::Matrix(1, 6) << 0, 0, 0, 0, 0, 0).finished();
      }
    }

    if (H_pose) {
      if (resultant(0, 0) > 0) {  // Active.
        gtsam::Matrix33 H_r = pose.rotation().matrix() *
                              gtsam::skewSymmetric(-f_c(0), -f_c(1), -f_c(2));
        gtsam::Matrix H_rot = gtsam::Matrix(H_f_c_prime * H_r);
        *H_pose = H_rot * H_p;
      } else {  // Inactive.
        *H_pose = (gtsam::Matrix(1, 6) << 0, 0, 0, 0, 0, 0).finished();
      }
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
    std::cout << s << "Contact Dynamics Friction Cone Factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
