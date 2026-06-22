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
 * reaches a desired goal pose. The error lives in the tangent space (logmap of
 * the relative pose), so it is a 6-vector (rotation, translation).
 *
 * @param pose_key key for the CoM pose of the link, in world coordinates
 * @param goal_pose desired CoM pose of the link, in world coordinates
 */
inline gtsam::Vector6_ PoseGoalConstraint(gtsam::Key pose_key,
                                          const gtsam::Pose3 &goal_pose) {
  gtsam::Pose3_ wTcom_expr(pose_key);
  gtsam::Pose3_ goal_pose_expr(goal_pose);
  return gtsam::logmap(wTcom_expr, goal_pose_expr);
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
  gtsam::Pose3 goal_pose_;

 public:
  /**
   * Constructor from goal pose.
   * @param pose_key key for the CoM pose of the link, in world coordinates
   * @param cost_model noise model
   * @param goal_pose desired CoM pose of the link, in world coordinates
   */
  PoseGoalFactor(gtsam::Key pose_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const gtsam::Pose3 &goal_pose)
      : Base(cost_model, gtsam::Vector6::Zero(),
             PoseGoalConstraint(pose_key, goal_pose)),
        goal_pose_(goal_pose) {}

  /// Return goal pose.
  const gtsam::Pose3 &goalPose() const { return goal_pose_; }

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