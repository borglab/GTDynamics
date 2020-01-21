/**
 * @file  ContactKinematicsPoseFactor.h
 * @brief Factor to enforce zero height at the contact point.
 * @Author: Alejandro Escontrela
 */

#pragma once

#include <utils.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <vector>

namespace robot {

/** ContactKinematicsPoseFactor is a one-way nonlinear factor which enforces
 * zero height at the contact point. This factor assumes that the ground is
 * flat and level. */
class ContactKinematicsPoseFactor
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  typedef ContactKinematicsPoseFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;
  gtsam::Pose3 cTcom_;

  int up_axis_;

 public:
  /** Contact kinematics factor for link end to remain in contact with the ground.
      Keyword argument:
          pose_key   -- The key corresponding to the link's Com pose.
          cost_model -- Noise model associated with this factor.
          cTcom      -- Static transform from link com to link end (where
            contact is made).
          gravity    -- Gravity vector in the spatial frame. Used to calculate
            the "up" direction.
   */
  ContactKinematicsPoseFactor(gtsam::Key pose_key,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          const gtsam::Pose3 &cTcom,
                          const gtsam::Vector3 &gravity)
      : Base(cost_model, pose_key),
        cTcom_(cTcom) {

    if (gravity[0] != 0)
      up_axis_ = 0;  // x.
    else if (gravity[1] != 0)
      up_axis_ = 1;  // y.
    else
      up_axis_ = 2;  // z.
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

      // Translate from the com frame to the link end frame, represented
      // in spatial coords.
      gtsam::Pose3 cTs =  pose * cTcom_.inverse();

      gtsam::Pose3 contactTrans(gtsam::Rot3(), cTs.translation());

      gtsam::Point3 goal_trans;
      if (up_axis_ == 0)
        goal_trans = gtsam::Point3(0, cTs.translation()[1], cTs.translation()[2]);
      else if (up_axis_ == 1)
        goal_trans = gtsam::Point3(cTs.translation()[0], 0, cTs.translation()[2]);
      else
        goal_trans = gtsam::Point3(cTs.translation()[0], cTs.translation()[1], 0);

      gtsam::Pose3 goal_pose(gtsam::Rot3(), goal_trans);

      gtsam::Vector error = goal_pose.logmap(contactTrans, H_pose);
      return error;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override{
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "wrench equivalence factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace robot
