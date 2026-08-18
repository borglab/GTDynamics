/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CollisionFactorUtils.h
 * @brief Constructor argument validation shared by the collision factors.
 *        Internal, unsupported implementation API.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/gpmp2/NNCableSpline.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SelfCollisionCost.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtsam/base/Vector.h>

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtdynamics {
namespace internal {

/// Reject a null model and return it, for factor initializer lists.
template <typename T>
inline const std::shared_ptr<T> &checkedNotNull(const std::shared_ptr<T> &model,
                                                const std::string &factorName,
                                                const std::string &what) {
  if (!model) {
    throw std::invalid_argument(factorName + ": " + what +
                                " must not be null.");
  }
  return model;
}

/// nrPoints of a model that must not be null, for factor initializer lists.
inline size_t checkedNrPoints(
    const std::shared_ptr<const RobotQueryPoints> &robot,
    const std::string &factorName) {
  return checkedNotNull(robot, factorName, "robot")->nrPoints();
}

/// Reject radii that are mis-sized (one entry per `what`) or negative.
inline void validateRadii(const gtsam::Vector &radii, size_t expectedSize,
                          const std::string &what,
                          const std::string &factorName) {
  if (static_cast<size_t>(radii.size()) != expectedSize) {
    throw std::invalid_argument(factorName +
                                ": radii must have one entry per " + what +
                                ".");
  }
  if ((radii.array() < 0.0).any()) {
    throw std::invalid_argument(factorName + ": radii must be >= 0.");
  }
}

/// Reject a null field or a negative standoff.
inline void validateSdfStandoff(
    const std::shared_ptr<const SignedDistanceField> &sdf, double epsilon,
    const std::string &factorName) {
  checkedNotNull(sdf, factorName, "sdf");
  if (epsilon < 0.0) {
    throw std::invalid_argument(factorName + ": epsilon must be >= 0.");
  }
}

/// Reject radii that are mis-sized, negative, or conflict at coincident
/// same-link points. factorName prefixes the error messages.
inline void validateQueryPointRadii(const RobotQueryPoints &robot,
                                    const gtsam::Vector &radii,
                                    const std::string &factorName) {
  validateRadii(radii, robot.nrPoints(), "query point", factorName);
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

/// Reject a null field, a negative standoff, or bad radii.
inline void validateObstacleSDFFactorArgs(
    const RobotQueryPoints &robot,
    const std::shared_ptr<const SignedDistanceField> &sdf, double epsilon,
    const gtsam::Vector &radii, const std::string &factorName) {
  validateSdfStandoff(sdf, epsilon, factorName);
  validateQueryPointRadii(robot, radii, factorName);
}

/// Reject inconsistent pairs or radii. factorName prefixes the errors.
inline void validateSelfCollisionPairs(const RobotQueryPoints &robot,
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

/// Restrict robot to the points pairs references, remapping *pairs and *radii
/// into the restricted model's own compact indices.
inline std::shared_ptr<const RobotQueryPoints> restrictToReferencedPoints(
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

/// numSamples of a cable model that must not be null, for factor
/// initializer lists.
inline size_t checkedNumSamples(
    const std::shared_ptr<const NNCableSpline> &cable,
    const std::string &factorName) {
  return checkedNotNull(cable, factorName, "cable")->numSamples();
}

/// Reject a null field, a negative standoff, or bad radii.
inline void validateNNCableFactorArgs(
    const NNCableSpline &cable,
    const std::shared_ptr<const SignedDistanceField> &sdf, double epsilon,
    const gtsam::Vector &radii, const std::string &factorName) {
  validateSdfStandoff(sdf, epsilon, factorName);
  validateRadii(radii, cable.numSamples(), "cable sample", factorName);
}

/// Reject a null model, inconsistent pairs/radii, or, if sigmas is given, a
/// sigma count not matching the pairs. Then return the model restricted to
/// just the points pairs references, remapping *pairs and *radii in place.
inline std::shared_ptr<const RobotQueryPoints> validateAndRestrictSelfCollision(
    const std::shared_ptr<const RobotQueryPoints> &robot,
    SelfCollisionPairs *pairs, gtsam::Vector *radii,
    const std::string &factorName, const gtsam::Vector *sigmas = nullptr) {
  checkedNotNull(robot, factorName, "robot");
  if (sigmas && static_cast<size_t>(sigmas->size()) != pairs->size()) {
    throw std::invalid_argument(
        factorName + ": sigmas must have one entry per pair.");
  }
  validateSelfCollisionPairs(*robot, *pairs, *radii, factorName);
  return restrictToReferencedPoints(robot, pairs, radii);
}

}  // namespace internal
}  // namespace gtdynamics
