/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SelfCollisionSphereFactor.cpp
 * @brief Self collision cost factor over a set of query point pairs.
 * @author Karthik Shaji - Adapted from gpmp2 by Mustafa Mukadam.
 */

#include <gtdynamics/factors/SelfCollisionSphereFactor.h>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
/// Reject inconsistent pairs or radii. factorName prefixes the errors.
static void validateSelfCollisionPairs(const RobotQueryPoints &robot,
                                       const SelfCollisionPairs &pairs,
                                       const gtsam::Vector &radii,
                                       const std::string &factorName) {
  validateQueryPointRadii(robot, radii, factorName);
  for (const auto &pair : pairs) {
    if (pair.epsilon < 0.0) {
      throw std::invalid_argument(factorName +
                                  ": a pair epsilon must be >= 0.");
    }
    if (pair.a >= robot.nrPoints() || pair.b >= robot.nrPoints()) {
      throw std::invalid_argument(factorName +
                                  ": pair point index out of range.");
    }
    if (pair.a == pair.b) {
      throw std::invalid_argument(factorName +
                                  ": a pair must use two distinct points.");
    }
    // Points on one rigid link keep a constant separation, so the hinge has
    // no gradient; reject such a pair rather than fire it permanently.
    if (robot.points()[pair.a].link->id() ==
        robot.points()[pair.b].link->id()) {
      throw std::invalid_argument(
          factorName + ": a pair must use points on different links.");
    }
  }
}

/* ************************************************************************* */
/// Restrict robot to the points pairs references, remapping *pairs and *radii
/// into the restricted model's own compact indices.
static std::shared_ptr<const RobotQueryPoints> restrictToReferencedPoints(
    const std::shared_ptr<const RobotQueryPoints> &robot,
    SelfCollisionPairs *pairs, gtsam::Vector *radii) {
  // Unique original point indices pairs references, in first-occurrence
  // order, so evaluation never queries a point no pair uses.
  std::vector<size_t> uniqueIndices;
  std::map<size_t, size_t> compactOf;
  for (const auto &pair : *pairs) {
    for (size_t idx : {pair.a, pair.b}) {
      if (compactOf.emplace(idx, uniqueIndices.size()).second) {
        uniqueIndices.push_back(idx);
      }
    }
  }

  gtsam::Vector compactRadii(uniqueIndices.size());
  for (size_t i = 0; i < uniqueIndices.size(); ++i) {
    compactRadii(i) = (*radii)(uniqueIndices[i]);
  }
  *radii = compactRadii;

  for (auto &pair : *pairs) {
    pair.a = compactOf.at(pair.a);
    pair.b = compactOf.at(pair.b);
  }

  return std::make_shared<const RobotQueryPoints>(robot, uniqueIndices);
}

/* ************************************************************************* */
std::shared_ptr<const RobotQueryPoints> validateAndRestrictSelfCollision(
    const std::shared_ptr<const RobotQueryPoints> &robot,
    SelfCollisionPairs *pairs, gtsam::Vector *radii,
    const std::string &factorName, const gtsam::Vector *sigmas) {
  if (!robot) {
    throw std::invalid_argument(factorName + ": robot must not be null.");
  }
  if (sigmas && static_cast<size_t>(sigmas->size()) != pairs->size()) {
    throw std::invalid_argument(
        factorName + ": sigmas must have one entry per pair.");
  }
  validateSelfCollisionPairs(*robot, *pairs, *radii, factorName);
  return restrictToReferencedPoints(robot, pairs, radii);
}

/* ************************************************************************* */
gtsam::Vector SelfCollisionSphereFactor::evaluateError(
    const gtsam::Vector &q, gtsam::OptionalMatrixType H1) const {
  const size_t nrPairs = pairs_.size();
  gtsam::Vector err(nrPairs);

  std::vector<gtsam::Point3> wPts;
  std::vector<gtsam::Matrix> ptJacobians;
  robot_->queryPoints(q, &wPts, H1 ? &ptJacobians : nullptr);
  if (H1) *H1 = gtsam::Matrix::Zero(nrPairs, robot_->dof());

  for (size_t r = 0; r < nrPairs; ++r) {
    const SelfCollisionPair &pair = pairs_[r];
    // The standoff folds in both spheres' radii.
    const double eps = pair.epsilon + radii_(pair.a) + radii_(pair.b);
    if (H1) {
      gtsam::Matrix13 HptA, HptB;
      err(r) = hingeLossSelfCollisionCost(wPts[pair.a], wPts[pair.b], eps,
                                          HptA, HptB);
      H1->row(r) = HptA * ptJacobians[pair.a] + HptB * ptJacobians[pair.b];
    } else {
      err(r) = hingeLossSelfCollisionCost(wPts[pair.a], wPts[pair.b], eps);
    }
  }
  return err;
}

}  // namespace gtdynamics
