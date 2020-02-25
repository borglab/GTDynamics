/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PoseGoalFactor.h
 * @brief Link pose goal factor.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
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
 * PoseGoalFactor is a unary factor enforcing that a link acheive a desired
 * goal pose.
 */
class PoseGoalFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  typedef PoseGoalFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;

  gtsam::Pose3 goalPose_;

 public:
  /**
   * Construct from joint angle limits
   * Keyword arguments:
      key
      cost_model  -- noise model
      goalPose    -- end effector pose goal
   */
  PoseGoalFactor(gtsam::Key pose_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const gtsam::Pose3 &goal_pose)
      : Base(cost_model, pose_key), goalPose_(goal_pose) {}

  virtual ~PoseGoalFactor() {}

  /** error function
      Keyword argument:
          pose -- The link pose.
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    gtsam::Vector error = pose.logmap(goalPose_, H_pose);
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
    std::cout << s << "pose goal factor" << std::endl;
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
