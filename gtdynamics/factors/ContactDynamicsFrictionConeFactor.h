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
#include <cmath>

#include <boost/optional.hpp>

#include "gtdynamics/utils/utils.h"

using std::sqrt, std::pow;

namespace gtdynamics {

/** ContactDynamicsFrictionConeFactor is binary nonlinear factor which enforces
 *  that the linear contact force lies within a friction cone. */
class ContactDynamicsFrictionConeFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector6> {
 private:
  typedef ContactDynamicsFrictionConeFactor This;
  typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector6> Base;
  int up_axis_;  // Which axis is up (assuming flat ground)? {0: x, 1: y, 2: z}.
  double mu_;
  double epsilon_ = 1e-4;  // TODO(aescontrela): Make the perturbed friction cone parameter epsilon a constructor argument.
  double delta_ = 0.3; // TODO(aescontrela): Make this a constructor argument.
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
      : Base(cost_model, pose_key, contact_wrench_key), mu_(mu) {
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
    // Linear component of the contact wrench in the link com frame.
    gtsam::Matrix Fc_l = H_wrench_ * contact_wrench;

    // Rotate linear contact wrench force into the inertial frame.
    gtsam::Matrix36 H_p;
    gtsam::Matrix Fc_i = pose.rotation(H_p) * Fc_l;

    // Gather the normal and lateral components of the linear force (Here is
    // where the flat ground assumption is applied).
    int n_idx = 2, lat1_idx = 0, lat2_idx = 1;
    if (up_axis_ == 0) {
      n_idx = 0;
      lat1_idx = 1;
      lat2_idx = 2;
    } else if (up_axis_ == 1) {
      n_idx = 1;
      lat1_idx = 0;
      lat2_idx = 2;
    }

    // Apply the perturbed friction cone equation.
    double h = mu_ * Fc_i(n_idx);
    double h_sqrt = sqrt(pow(Fc_i(lat1_idx), 2) + pow(Fc_i(lat2_idx), 2) + pow(epsilon_, 2));
    h -= h_sqrt;
    
    // Compute the gradient of h wrt Fc_i.
    gtsam::Matrix13 H_h;
    H_h[lat1_idx] = -Fc_i(lat1_idx) / h_sqrt;
    H_h[lat2_idx] = -Fc_i(lat2_idx) / h_sqrt;
    H_h[n_idx] = mu_;

    // Calculate the relaxed log barrier function value.
    double B;
    if (h >= delta_)
      B = -std::log(h);
    else
      B = 0.5 * (std::pow((h - 2 * delta_) / delta_, 2) - 1) - log(delta_);
    
    // Calculate the gradient of B wrt h.
    gtsam::Matrix11 H_B;
    if (h >= delta_)
      H_B[0] = -1 / h;
    else
      H_B[0] = (h - 2 * delta_) / std::pow(delta_, 2.0);
    
    gtsam::Vector error = (gtsam::Vector(1) << B).finished();

    if (H_contact_wrench) {
      *H_contact_wrench = H_B * H_h * pose.rotation().matrix() * H_wrench_;
    }

     if (H_pose) {
      gtsam::Matrix33 H_r = pose.rotation().matrix() *
                            gtsam::skewSymmetric(-Fc_l(0), -Fc_l(1), -Fc_l(2));
      *H_pose = H_B * H_h * H_r * H_p;
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
