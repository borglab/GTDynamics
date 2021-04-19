/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PosePointFactor.h
 * @brief factor to enforce the position of a fixed point on a link
 * @Author: Yetong Zhang
 */
#ifndef GTDYNAMICS_FACTORS_POSEPOINTFACTOR_H_
#define GTDYNAMICS_FACTORS_POSEPOINTFACTOR_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <boost/optional.hpp>

namespace gtdynamics {

/** PosePointFactor is a class which enforces the position of a fixed point on a rigid link*/
class PosePointFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
 private:
  typedef PosePointFactor This;
  typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> Base;
  gtsam::Point3 point_local_;

 public:
  /** Constructor
   * Keyword arguments:
      pose_key           -- link pose key
      point_key          -- point key
      point_local        -- point position in link frame
   */
  PosePointFactor(gtsam::Key pose_key, gtsam::Key point_key,
                   const gtsam::noiseModel::Base::shared_ptr &cost_model,
                   const gtsam::Point3 &point_local)
      : Base(cost_model, pose_key, point_key),
        point_local_(point_local) {}

  virtual ~PosePointFactor() {}

 public:
  /** wTp = wTl * lTp */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose, const gtsam::Point3 &point,
      boost::optional<gtsam::Matrix &> H_pose = boost::none,
      boost::optional<gtsam::Matrix &> H_point = boost::none) const override {

    gtsam::Point3 estimate = pose.transformTo(point, H_pose, H_point);
    return estimate - point_local_;
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
    std::cout << s << "PosePointFactor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_JOINTLIMITFACTOR_H_