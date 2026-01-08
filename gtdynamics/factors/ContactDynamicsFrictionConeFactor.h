/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactDynamicsFrictionConeFactor.h
 * @brief Factor to enforce contact force lies within a friction cone.
 * @author Alejandro Escontrela
 */

#pragma once

#include <gtdynamics/utils/utils.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * ContactDynamicsFrictionConeFactor is binary nonlinear factor which enforces
 * that the linear contact force lies within a friction cone.
 */
class ContactDynamicsFrictionConeFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector6> {
 private:
  using This = ContactDynamicsFrictionConeFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector6>;

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
  /**
   * Contact dynamics factor for zero moment at contact.
   *
   * @param pose_key Key corresponding to the link's CoM pose.
   * @param contact_wrench_key Key corresponding to this link's contact wrench.
   * @param cost_model Noise model for this factor.
   * @param cTcom Contact frame expressed in com frame.
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

  /**
   * Evaluate contact point moment errors.
   * @param contact_wrench Contact wrench on this link.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose, const gtsam::Vector6 &contact_wrench,
      gtsam::OptionalMatrixType H_pose = nullptr,
      gtsam::OptionalMatrixType H_contact_wrench = nullptr) const override {
    // Linear component of the contact wrench.
    gtsam::Vector3 f_c = H_wrench_ * contact_wrench;

    // Rotate linear contact wrench force into the spatial frame.
    gtsam::Matrix36 H_p;
    gtsam::Vector3 f_s = pose.rotation(H_p) * f_c;

    // Compute the squared force values.
    gtsam::Matrix A = f_s.transpose() * H_x_;
    gtsam::Matrix B = f_s.transpose() * H_y_;
    gtsam::Matrix C = f_s.transpose() * H_z_;

    if (up_axis_ == 0)
      A *= -mu_prime_;
    else if (up_axis_ == 1)
      B *= -mu_prime_;
    else
      C *= -mu_prime_;

    // a = f_x^2, b = f_z^2, c = f_z^2.
    gtsam::Matrix a = A * f_s;
    gtsam::Matrix b = B * f_s;
    gtsam::Matrix c = C * f_s;

    gtsam::Matrix resultant = a + b + c;
    gtsam::Vector error;

    // Ramp function.
    if (resultant(0, 0) > 0)
      error = resultant;
    else
      error = (gtsam::Vector(1) << 0).finished();

    // Compute the gradients based on whether or not the inequality constraint
    // is active.
    gtsam::Matrix H_f_s = (2 * A + 2 * B + 2 * C);
    if (H_contact_wrench) {
      if (resultant(0, 0) > 0) {  // Active.
        gtsam::Matrix H_f_c = H_f_s * pose.rotation().matrix();
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
        gtsam::Matrix H_rot = gtsam::Matrix(H_f_s * H_r);
        *H_pose = H_rot * H_p;
      } else {  // Inactive.
        *H_pose = (gtsam::Matrix(1, 6) << 0, 0, 0, 0, 0, 0).finished();
      }
    }

    return error;
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "Contact Dynamics Friction Cone Factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtdynamics
