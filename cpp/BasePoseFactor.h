/**
 * @file  BasePoseFactor.h
 * @brief Factor enforcing base Pose.
 * @Author: Mandy Xie
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

namespace manipulator {

/** BasePoseFactor is a one-way nonlinear factor which enforces the pose of the
 * base*/
class BasePoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  typedef BasePoseFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;
  gtsam::Pose3 base_pose_;

 public:
  /** Create single factor of the base pose (taken as zero-th link's pose)
      Keyword arguments:
          jMi -- base pose, expressed in the first link's COM frame, at rest
     configuration screw_axis -- screw axis expressed in link's COM frame
   */
  BasePoseFactor(gtsam::Key pose_key_0,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const gtsam::Pose3 &base_pose)
      : Base(cost_model, pose_key_0), base_pose_(base_pose) {}

  virtual ~BasePoseFactor() {}

 public:
  /** evaluate link pose errors
      Keyword argument:
          pose_0          -- the base pose
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose_0,
      boost::optional<gtsam::Matrix &> H_pose_0 = boost::none) const override {
    return pose_0.logmap(base_pose_, H_pose_0);
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
    std::cout << s << "base pose factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace manipulator
