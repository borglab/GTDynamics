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

#include <cmath>
#include <cstdint>
#include <map>
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
  if (static_cast<size_t>(radii.size()) != robot.nrPoints()) {
    throw std::invalid_argument(
        factorName + ": radii must have one entry per query point.");
  }
  if ((radii.array() < 0.0).any()) {
    throw std::invalid_argument(factorName + ": radii must be >= 0.");
  }
  // Overlapping spheres on a link are fine, but the same point registered
  // twice with different radii is a contradiction. Group by link so only
  // same-link points are compared, not every pair.
  const auto &pts = robot.points();
  std::map<uint8_t, std::vector<size_t>> ptsByLink;
  for (size_t i = 0; i < pts.size(); ++i) {
    ptsByLink[pts[i].link->id()].push_back(i);
  }
  for (const auto &group : ptsByLink) {
    const std::vector<size_t> &indices = group.second;
    for (size_t a = 0; a < indices.size(); ++a) {
      for (size_t b = a + 1; b < indices.size(); ++b) {
        if ((pts[indices[a]].point - pts[indices[b]].point).norm() < 1e-9 &&
            std::fabs(radii(indices[a]) - radii(indices[b])) > 1e-9) {
          throw std::invalid_argument(
              factorName + ": two points at the same location have "
              "conflicting radii.");
        }
      }
    }
  }
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
