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

#include <gtdynamics/factors/PointGoalFactor.h>

namespace gtdynamics {

using gtsam::Matrix;
using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Vector;

Vector PointGoalFactor::evaluateError(const Pose3 &wTcom,
                                      boost::optional<Matrix &> H_pose) const {
  // Change point reference frame from com to spatial.
  auto sTp_t = wTcom.transformFrom(point_com_, H_pose);
  return sTp_t - goal_point_;
}

/// print contents
void PointGoalFactor::print(const std::string &s,
                            const gtsam::KeyFormatter &keyFormatter) const {
  std::cout << s << "PointGoalFactor\n";
  Base::print("", keyFormatter);
  std::cout << "point on link: " << point_com_.transpose() << std::endl;
  std::cout << "goal point: " << goal_point_.transpose() << std::endl;
}

NonlinearFactorGraph PointGoalFactors(
    gtsam::Key first_key, const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const Point3 &point_com, const std::vector<Point3> &goal_trajectory) {
  NonlinearFactorGraph factors;
  for (auto &&goal_point : goal_trajectory) {
    factors.emplace_shared<PointGoalFactor>(first_key, cost_model, point_com,
                                            goal_point);
    first_key += 1;
  }
  return factors;
}

}  // namespace gtdynamics
