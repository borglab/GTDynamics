/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  endEffGoalFactor.h
 * @brief End Effector pose goal factor.
 * @author Karthik Shaji
 */

#pragma once
#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

#include <memory>
#include <string>

namespace gtdynamics {

/**
 * PoseGoalConstraint is a unary constraint enforcing that a link's CoM pose
 * (wTcom) reaches a desired goal pose (wTcom_goal), both in SE(3). The error
 * lives in the tangent space (logmap of the relative pose), so it is a 6-vector
 * (rotation, translation).
 *
 * @param pose_key key for the link CoM pose in the world frame (wTcom)
 * @param wTcom_goal desired CoM pose of the link, in the world frame
 */
inline gtsam::Vector6_ PoseGoalConstraint(gtsam::Key pose_key,
                                          const gtsam::Pose3 &wTcom_goal) {
  gtsam::Pose3_ wTcom(pose_key);
  gtsam::Pose3_ wTcom_goal_(wTcom_goal);
  return gtsam::logmap(wTcom, wTcom_goal_);
}

/**
 * PoseGoalFactor is a unary factor that penalizes the deviation of a link's
 * CoM pose from a desired goal pose. It is the soft-cost counterpart of
 * PoseGoalConstraint, analogous to PointGoalFactor for point goals.
 */
class PoseGoalFactor : public gtsam::ExpressionFactor<gtsam::Vector6> {
 private:
  using This = PoseGoalFactor;
  using Base = gtsam::ExpressionFactor<gtsam::Vector6>;
  gtsam::Pose3 wTcom_goal_;

 public:
  /**
   * Constructor from goal pose.
   * @param pose_key key for the link CoM pose in the world frame (wTcom)
   * @param cost_model noise model
   * @param wTcom_goal desired CoM pose of the link, in the world frame
   */
  PoseGoalFactor(gtsam::Key pose_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const gtsam::Pose3 &wTcom_goal)
      : Base(cost_model, gtsam::Vector6::Zero(),
             PoseGoalConstraint(pose_key, wTcom_goal)),
        wTcom_goal_(wTcom_goal) {}

  /// Return goal pose (wTcom_goal).
  const gtsam::Pose3 &goalPose() const { return wTcom_goal_; }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtdynamics