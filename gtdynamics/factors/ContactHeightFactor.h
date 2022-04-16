/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactHeightFactor.h
 * @brief Factor to enforce zero height at the contact point.
 * @author: Alejandro Escontrela, Varun Agrawal
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam/slam/expressions.h>

#include <string>

namespace gtdynamics {

/**
 * ContactHeightConstraint is a constraint which enforces a known ground plane
 * height for the contact point. This factor assumes that the ground is flat and
 * level.
 */
inline gtsam::Double_ ContactHeightConstraint(gtsam::Key pose_key,
                      const gtsam::Point3 &comPc, const gtsam::Vector3 &gravity,
                      const double &ground_plane_height = 0.0) {
  gtsam::Point3_ gravity_s_unit(gravity.normalized().cwiseAbs());
  gtsam::Pose3_ sTl(pose_key);
  gtsam::Point3_ sPc = gtsam::transformFrom(sTl, gtsam::Point3_(comPc));
  gtsam::Double_ sPc_h = gtsam::dot(sPc, gravity_s_unit);
  gtsam::Double_ error = sPc_h - gtsam::Double_(ground_plane_height);
  return error;
}

/**
 * ContactHeightFactor is a one-way nonlinear factor which enforces a
 * known ground plane height for the contact point. This factor assumes that the
 * ground is flat and level.
 */
class ContactHeightFactor : public gtsam::ExpressionFactor<double> {
 private:
  using This = ContactHeightFactor;
  using Base = gtsam::ExpressionFactor<double>;

 public:
  /**
   * Factor for link end to remain in contact with the
   * ground.
   *
   * @param pose_key The key corresponding to the link's CoM pose.
   * @param cost_model Noise model associated with this factor.
   * @param comPc Static transform from point of contact to link CoM.
   * @param gravity Gravity vector in the spatial frame. Used to calculate the
   * "up" direction.
   * @param ground_plane_height Height of the ground plane in the world frame.
   */
  ContactHeightFactor(gtsam::Key pose_key,
                      const gtsam::noiseModel::Base::shared_ptr &cost_model,
                      const gtsam::Point3 &comPc, const gtsam::Vector3 &gravity,
                      const double &ground_plane_height = 0.0)
      : Base(cost_model, 0.0,
             ContactHeightConstraint(pose_key, comPc, gravity,
                                     ground_plane_height)) {}

  virtual ~ContactHeightFactor() {}

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "ContactHeightFactor"
              << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
