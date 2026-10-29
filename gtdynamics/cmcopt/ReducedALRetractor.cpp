/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ReducedALRetractor.cpp
 * @brief Implement reduced augmented Lagrangian retraction with augmented
 * Lagrangian.
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/AffineProjection.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/ReducedALRetractor.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace gtdynamics {

ReducedALRetractor::Settings::Settings() {
  al.maxIterations = 80;
  al.absoluteViolationTolerance = 1e-9;
  al.absoluteStationarityTolerance = 1e-7;
  // Tighten the BCL inner target gradually: a tenfold tightening per dual
  // update can request sub-machine-precision objective decreases before the
  // projection reaches its primal tolerance, even for scalar linear bounds.
  al.bclInitialPenalty = 10.0;
  al.bclBetaOmega = 0.1;
  al.lmParams.maxIterations = 200;
  al.lmParams.absoluteErrorTol = al.lmParams.relativeErrorTol = 0.0;
  al.lmParams.lambdaUpperBound = 1e12;
}

ReducedALRetractor::ReducedALRetractor(
    const IERetractorParams::shared_ptr &params, const Settings &settings,
    const std::optional<KeyVector> &priorKeys)
    : IERetractor(params), settings_(settings), priorKeys_(priorKeys) {
  if (!params_ || !(settings_.feasibilityTolerance > 0) ||
      !std::isfinite(settings_.feasibilityTolerance) ||
      settings_.activeTolerance < settings_.feasibilityTolerance ||
      !std::isfinite(settings_.activeTolerance))
    throw std::invalid_argument("Invalid ReducedALRetractor configuration");
  if (params_->useVaryingSigma && !params_->metricSigmas)
    throw std::invalid_argument("Missing reduced projection metric sigmas");
  if (settings_.al.updatePolicy != AugmentedLagrangianUpdatePolicy::BCL)
    throw std::invalid_argument(
        "Reduced AL retraction requires BCL AL updates");
}

Vector ReducedALRetractor::solveProblem(const ConstrainedOptProblem &problem,
                                        const AffineProjection &projection,
                                        IERetractionInfo *info) const {
  if (info)
    ++info->numALSolveCalls;
  auto al = std::make_shared<AugmentedLagrangianParams>(settings_.al);
  // Each reduced expression already includes the original sigma. Its noise
  // sigma is only the additional normalization, so this bound implies the
  // requested ORIGINAL whitened feasibility tolerance.
  double largestSigma = 1.0;
  for (const auto &constraint : problem.eConstraints())
    largestSigma =
        std::max(largestSigma, constraint->noiseModel()->sigmas()(0));
  for (const auto &constraint : problem.iConstraints())
    largestSigma =
        std::max(largestSigma, constraint->noiseModel()->sigmas()(0));
  al->absoluteViolationTolerance =
      std::min(al->absoluteViolationTolerance,
               0.5 * settings_.feasibilityTolerance / largestSigma);
  const Values initial = projection.initialValues(params_->initValuesAsX);
  AugmentedLagrangianOptimizer optimizer(problem, initial, al);
  // Let GTSAM retain its merit graph and multifrontal workspace across AL
  // iterations. The final state supplies diagnostics without storing history;
  // GTSAM also floors the inner target at the final stationarity tolerance.
  // Preserve rejection when the caller allows no outer iterations.
  if (al->maxIterations == 0)
    throw std::runtime_error("Reduced AL failed: no outer iterations allowed");
  const auto state = optimizer.optimizeWithState();
  if (info)
    info->numLMIterations += state.totalUnconstrainedIterations;
  if (!state.converged ||
      !std::isfinite(state.augmentedLagrangianStationarity) ||
      !std::isfinite(state.generalizedConstraintViolation) ||
      state.augmentedLagrangianStationarity >
          al->absoluteStationarityTolerance ||
      state.generalizedConstraintViolation > al->absoluteViolationTolerance) {
    std::ostringstream message;
    message << "Reduced AL failed: iterations=" << state.iteration
            << ", stationarity=" << state.augmentedLagrangianStationarity
            << ", violation=" << state.generalizedConstraintViolation
            << ", inner_converged=" << state.innerConverged
            << ", omega=" << state.bclOmega
            << ", lm_iterations=" << state.totalUnconstrainedIterations;
    throw std::runtime_error(message.str());
  }
  const Vector u = state.values.at<Vector>(AffineProjection::kReduced);
  if (!u.allFinite())
    throw std::runtime_error("Nonfinite reduced AL result");
  return u;
}

IEConstraintManifold
ReducedALRetractor::retract(const IEConstraintManifold *manifold,
                            const VectorValues &delta,
                            const std::optional<IndexSet> &blockingIndices,
                            IERetractionInfo *info) const {
  if (!manifold)
    throw std::invalid_argument("Null reduced AL retraction manifold");
  const IndexSet blocking = blockingIndices.value_or(IndexSet{});
  AffineProjection projection(*manifold, delta, *params_, priorKeys_, blocking,
                              settings_.feasibilityTolerance,
                              settings_.activeTolerance);
  if (const auto result = projection.feasibleProjection(blocking, info))
    return manifold->createWithNewValues(result->values, result->active);
  Values result = projection.recover(Vector::Zero(projection.dim()));
  if (projection.dim())
    result = projection.recover(
        solveProblem(projection.buildProblem(blocking), projection, info));
  return manifold->createWithNewValues(
      result, projection.validateResult(result, blocking));
}

} // namespace gtdynamics
