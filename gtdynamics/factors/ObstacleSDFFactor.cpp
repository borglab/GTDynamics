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
  const size_t nrPts = robot_->nrPoints();
  gtsam::Vector err(nrPts);

  std::vector<gtsam::Point3> wPts;
  std::vector<gtsam::Matrix> ptJacobians;
  robot_->queryPoints(q, &wPts, H1 ? &ptJacobians : nullptr);
  if (H1) *H1 = gtsam::Matrix::Zero(nrPts, robot_->dof());

  for (size_t i = 0; i < nrPts; ++i) {
    const double eps = epsilon_ + radii_(i);
    if (H1) {
      gtsam::Matrix13 Hpt;
      err(i) = hingeLossObstacleCost(wPts[i], *sdf_, eps, Hpt);
      H1->row(i) = Hpt * ptJacobians[i];
    } else {
      err(i) = hingeLossObstacleCost(wPts[i], *sdf_, eps);
    }
  }
  return err;
}

}  // namespace gtdynamics
