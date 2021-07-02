/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactKinematicsPoseFactor.h
 * @brief Factor to enforce zero height at the contact point.
 * @author: Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "gtdynamics/utils/utils.h"

namespace gtdynamics {

/**
 * ContactKinematicsPoseFactor is a one-way nonlinear factor which enforces a
 * known ground plane height for the contact point. This factor assumes that the
 * ground is flat and level.
 */
class ContactKinematicsPoseFactor
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  using This = ContactKinematicsPoseFactor;
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  gtsam::Pose3 comTc_;
  gtsam::Vector1 h_;  // Height of the ground plane in the world frame.

  gtsam::Matrix13 H_err_;

 public:
  /**
   * Contact kinematics factor for link end to remain in contact with the
   * ground.
   *
   * @param pose_key The key corresponding to the link's CoM pose.
   * @param cost_model Noise model associated with this factor.
   * @param comTc Static transform from point of contact to link CoM.
   * @param gravity Gravity vector in the spatial frame. Used to calculate the
   * "up" direction.
   * @param ground_plane_height Height of the ground plane in the world frame.
   */
  ContactKinematicsPoseFactor(
      gtsam::Key pose_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model,
      const gtsam::Pose3 &comTc, const gtsam::Vector3 &gravity,
      const double &ground_plane_height = 0.0)
      : Base(cost_model, pose_key), comTc_(comTc) {
    if (gravity[0] != 0)
      H_err_ = (gtsam::Matrix13() << 1, 0, 0).finished();  // x.
    else if (gravity[1] != 0)
      H_err_ = (gtsam::Matrix13() << 0, 1, 0).finished();  // y.
    else
      H_err_ = (gtsam::Matrix13() << 0, 0, 1).finished();  // z.

    h_ = gtsam::Vector1(ground_plane_height);
  }

  virtual ~ContactKinematicsPoseFactor() {}

 public:
  /**
   * Evaluate contact errors.
   * @param sTl This link's COM pose in the spatial frame.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &sTl,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    // Change contact reference frame from CoM to spatial.
    gtsam::Matrix6 H_sTl;
    gtsam::Pose3 sTc = sTl.transformPoseFrom(comTc_, H_sTl);

    // Obtain translation component and corresponding jacobian.
    gtsam::Matrix36 H_trans;
    gtsam::Vector3 sTc_p = sTc.translation(H_trans);

    // Compute the error.
    gtsam::Vector sTc_p_h = gtsam::Vector1(H_err_.dot(sTc_p));
    gtsam::Vector error = sTc_p_h - h_;

    if (H_pose) *H_pose = H_err_ * H_trans * H_sTl;

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
    std::cout << (s.empty() ? "" : s + " ") << "Contact kinematics pose factor"
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
