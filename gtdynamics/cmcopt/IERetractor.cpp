/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file IERetractor.cpp
 * @brief IERetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>

#include <stdexcept>

namespace gtdynamics {
using namespace gtsam;

VectorValues IERetractorParams::priorPrecisions(
    const Values &values, const KeyVector &keys) const {
  if (useVaryingSigma && !metricSigmas)
    throw std::invalid_argument("Missing projection metric sigmas");
  double scale = 1.0;
  if (useVaryingSigma && scaleVaryingSigma) {
    // Match AddGeneralPriors' nominal-sigma convention.
    double minimum = 1e10;
    for (Key key : keys)
      if (metricSigmas->exists(key)) {
        const auto &sigma = metricSigmas->at(key);
        if (!sigma.size() || sigma.size() != values.at(key).dim() ||
            !sigma.allFinite() || (sigma.array() <= 0).any())
          throw std::invalid_argument("Invalid projection metric sigmas");
        minimum = std::min(minimum, sigma.minCoeff());
      }
    scale = priorSigma / minimum;
  }
  VectorValues precisions;
  for (Key key : keys) {
    if (useVaryingSigma && !metricSigmas->exists(key)) continue;
    const size_t dim = values.at(key).dim();
    const Vector sigma = useVaryingSigma
        ? Vector(scale * metricSigmas->at(key))
        : Vector::Constant(dim, priorSigma);
    if (sigma.size() != dim || !sigma.allFinite() || (sigma.array() <= 0).any())
      throw std::invalid_argument("Invalid projection metric sigmas");
    const Vector weights = sigma.array().square().inverse();
    if (!weights.allFinite() || (weights.array() <= 0).any())
      throw std::invalid_argument("Invalid projection metric precision");
    // Repeated prior keys have the same semantics as repeated prior factors.
    if (precisions.exists(key)) precisions.at(key) += weights;
    else precisions.insert(key, weights);
  }
  return precisions;
}


IEConstraintManifold
IERetractor::moveToBoundary(const IEConstraintManifold *manifold,
                            const IndexSet &blocking_indices,
                            IERetractionInfo *retract_info) const {
  VectorValues delta = manifold->values().zeroVectors();
  return retract(manifold, delta, blocking_indices, retract_info);
}


}  // namespace gtdynamics
