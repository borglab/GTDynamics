/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PointGoalFactor.h
 * @brief Link point goal factor.
 * @Author: Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <string>

namespace gtdynamics {

/**
 * PointGoalFactor is a unary factor enforcing that a point on a link
 * reaches a desired goal point.
 */
class PointGoalFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  typedef PointGoalFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;

  // Transform from link CoM to point on link where this factor is enforced.
  gtsam::Pose3 comTp_;
  // Goal point in the spatial frame.
  gtsam::Point3 goalPoint_;

 public:
  /**
   * Construct from joint angle limits
   * Keyword arguments:
      key
      cost_model  -- noise model
      goalPose    -- end effector pose goal
   */
  PointGoalFactor(gtsam::Key pose_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const gtsam::Pose3 &comTp,
                 const gtsam::Point3 &goal_point)
      : Base(cost_model, pose_key), comTp_(comTp), goalPoint_(goal_point) {}

  virtual ~PointGoalFactor() {}

  /** error function
      Keyword argument:
          pose -- The link pose.
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    // Change point reference frame from com to spatial.
    gtsam::Pose3 sTp = pose.transformPoseFrom(comTp_);
    gtsam::Matrix H_point;
    gtsam::Point3 sTp_t = sTp.translation(H_point);
    gtsam::Vector error = sTp_t - goalPoint_;

    if (H_pose)
        *H_pose = H_point * comTp_.inverse().AdjointMap();

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
    std::cout << s << "point goal factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { //NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
