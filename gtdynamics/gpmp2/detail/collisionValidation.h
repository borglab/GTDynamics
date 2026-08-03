/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  collisionValidation.h
 * @brief Constructor argument validation shared by the collision factors.
 *        Implementation detail, not part of the public API.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SelfCollisionCost.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtsam/base/Vector.h>

#include <memory>
#include <string>

namespace gtdynamics {

/// nrPoints of a model that must not be null, for factor initializer lists.
size_t checkedNrPoints(const std::shared_ptr<const RobotQueryPoints> &robot,
                       const std::string &factorName);

/// Reject radii that are mis-sized, negative, or conflict at coincident
/// same-link points. factorName prefixes the error messages.
void validateQueryPointRadii(const RobotQueryPoints &robot,
                             const gtsam::Vector &radii,
                             const std::string &factorName);

/// Reject a null field, a negative standoff, or bad radii.
void validateObstacleSDFFactorArgs(
    const RobotQueryPoints &robot,
    const std::shared_ptr<const SignedDistanceField> &sdf, double epsilon,
    const gtsam::Vector &radii, const std::string &factorName);

/// Reject a null model, inconsistent pairs/radii, or, if sigmas is given, a
/// sigma count not matching the pairs. Then return the model restricted to
/// just the points pairs references, remapping *pairs and *radii in place.
std::shared_ptr<const RobotQueryPoints> validateAndRestrictSelfCollision(
    const std::shared_ptr<const RobotQueryPoints> &robot,
    SelfCollisionPairs *pairs, gtsam::Vector *radii,
    const std::string &factorName, const gtsam::Vector *sigmas = nullptr);

}  // namespace gtdynamics
