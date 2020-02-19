/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactKinematicsPoseFactor.h
 * @brief Factor to enforce zero height at the contact point.
 * @Author: Alejandro Escontrela
 */

#ifndef GTDYNAMICS_FACTORS_CONTACTKINEMATICSPOSEFACTOR_H_
#define GTDYNAMICS_FACTORS_CONTACTKINEMATICSPOSEFACTOR_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include "gtdynamics/utils/utils.h"

namespace gtdynamics {

/** ContactKinematicsPoseFactor is a one-way nonlinear factor which enforces
 * zero height at the contact point. This factor assumes that the ground is
 * flat and level. */
class ContactKinematicsPoseFactor
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  typedef ContactKinematicsPoseFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;
  gtsam::Pose3 cTcom_;

  gtsam::Matrix13 H_err_;

 public:
  /** Contact kinematics factor for link end to remain in contact with the
     ground. Keyword argument: pose_key   -- The key corresponding to the link's
     Com pose. cost_model -- Noise model associated with this factor. cTcom --
     Static transform from link com to link end (where contact is made). gravity
     -- Gravity vector in the spatial frame. Used to calculate the "up"
     direction.
   */
  ContactKinematicsPoseFactor(
      gtsam::Key pose_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model,
      const gtsam::Pose3 &cTcom, const gtsam::Vector3 &gravity)
      : Base(cost_model, pose_key), cTcom_(cTcom) {
    if (gravity[0] != 0)
      H_err_ = (gtsam::Matrix13() << 1, 0, 0).finished();  // x.
    else if (gravity[1] != 0)
      H_err_ = (gtsam::Matrix13() << 0, 1, 0).finished();  // y.
    else
      H_err_ = (gtsam::Matrix13() << 0, 0, 1).finished();  // z.
  }
  virtual ~ContactKinematicsPoseFactor() {}

 public:
  /** Evaluate contact errors.
      Keyword argument:
          pose -- This link's COM pose in the spatial frame.
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    // Change contact reference frame from com to spatial.
    gtsam::Pose3 sTc = pose.transformPoseFrom(cTcom_.inverse());

    // Obtain translation component and corresponding jacobian.
    gtsam::Matrix36 H_trans;
    gtsam::Vector3 sTc_p = gtsam::Vector3(sTc.translation(H_trans));

    // Compute the error.
    gtsam::Vector error = (gtsam::Vector(1) << H_err_.dot(sTc_p)).finished();

    // Compute the jacobian.
    if (H_pose) *H_pose = H_err_ * H_trans * cTcom_.AdjointMap();

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
    std::cout << s << "Contact kinematics pose factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_CONTACTKINEMATICSPOSEFACTOR_H_
