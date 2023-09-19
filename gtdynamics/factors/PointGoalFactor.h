/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PointGoalFactor.h
 * @brief Link point goal factor.
 * @author Alejandro Escontrela
 */

#pragma once

#include <gtdynamics/utils/values.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

#include <string>

namespace gtdynamics {

/**
 * PointGoalConstraint is a unary constraint enforcing that a point on a link
 * reaches a desired goal point.
 */
inline gtsam::Vector3_ PointGoalConstraint(gtsam::Key pose_key,
                                           const gtsam::Point3 &point_com,
                                           const gtsam::Point3 &goal_point) {
  gtsam::Vector3_ point_com_expr(point_com);
  gtsam::Pose3_ wTcom_expr(pose_key);
  gtsam::Vector3_ point_world_expr(wTcom_expr, &gtsam::Pose3::transformFrom,
                                   point_com_expr);

  gtsam::Vector3_ goal_point_expr(goal_point);
  gtsam::Vector3_ error = point_world_expr - goal_point_expr;
  return error;
}

class PointGoalFactor : public gtsam::ExpressionFactor<gtsam::Vector3> {
 private:
  using This = PointGoalFactor;
  using Base = gtsam::ExpressionFactor<gtsam::Vector3>;
  gtsam::Point3 goal_point_;

 public:
  /**
   * Constructor from goal point.
   * @param pose_key key for COM pose of the link
   * @param cost_model noise model
   * @param point_com point on link, in COM coordinate frame
   * @param goal_point end effector goal point, in world coordinates
   */
  PointGoalFactor(gtsam::Key pose_key,
                  const gtsam::noiseModel::Base::shared_ptr &cost_model,
                  const gtsam::Point3 &point_com,
                  const gtsam::Point3 &goal_point)
      : Base(cost_model, gtsam::Vector3::Zero(),
             PointGoalConstraint(pose_key, point_com, goal_point)),
        goal_point_(goal_point) {}

  /// Return goal point.
  const gtsam::Point3 &goalPoint() const { return goal_point_; }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
#endif
};

/**
 * Construct many PointGoalFactors from goal trajectory.
 *
 * This function willl take a key to the first pose and then increment the key
 * by 1, for each time-step in the goal trajectory. If you need more
 * sophisticated behavior then this function is not it ;-).
 *
 * @param first_key key for the COM pose.
 * @param cost_model noise model
 * @param point_com point on link, in COM coordinate frame
 * @param goal_trajectory end effector goal trajectory, in world coordinates
 */
inline gtsam::NonlinearFactorGraph PointGoalFactors(
    gtsam::Key first_key, const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const gtsam::Point3 &point_com,
    const std::vector<gtsam::Point3> &goal_trajectory) {
  gtsam::NonlinearFactorGraph factors;
  for (auto &&goal_point : goal_trajectory) {
    factors.emplace_shared<PointGoalFactor>(first_key, cost_model, point_com,
                                            goal_point);
    first_key += 1;
  }
  return factors;
}

}  // namespace gtdynamics
