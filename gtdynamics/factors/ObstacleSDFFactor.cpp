/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObstacleSDFFactor.cpp
 * @brief Obstacle avoidance cost factor on a signed distance field.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#include <gtdynamics/factors/ObstacleSDFFactor.h>

#include <stdexcept>
#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
void validateObstacleSDFFactorArgs(
    const RobotQueryPoints &robot,
    const std::shared_ptr<const SignedDistanceField> &sdf, double epsilon,
    const gtsam::Vector &radii, const std::string &factorName) {
  if (!sdf) {
    throw std::invalid_argument(factorName + ": sdf must not be null.");
  }
  if (epsilon < 0.0) {
    throw std::invalid_argument(factorName + ": epsilon must be >= 0.");
  }
  validateQueryPointRadii(robot, radii, factorName);
}

/* ************************************************************************* */
gtsam::Vector ObstacleSDFFactor::evaluateError(
    const gtsam::Vector &q, gtsam::OptionalMatrixType H1) const {
  return obstacleSDFError(q, *robot_, *sdf_, epsilon_, radii_, H1);
}

}  // namespace gtdynamics
