/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file CableRobotBenchmark.h
 * @brief Explicit solver settings preserving the active experiment in PR #363.
 * @author Gerry Chen
 * @author Yetong Zhang
 */
#pragma once

#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>

#include <limits>

namespace gtdynamics {

/** Build the same dense basis eagerly to avoid nested TBB linearization while
 * the lazy-basis mutex is held. Only construction timing changes; the basis and
 * unconstrained optimization retractor are the same as in the source
 * experiment.
 */
inline ManifoldOptimizerParameters cableRobotManifoldParams() {
  auto params = ConstrainedOptBenchmark::DefaultMoptParams();
  auto basisParams = std::make_shared<TangentSpaceBasisParams>();
  basisParams->alwaysConstructBasis = true;
  params.constraintManifoldParams->basisCreator =
      std::make_shared<OrthonormalBasisCreator>(basisParams);
  return params;
}

/** Preserve the source soft weight and feasible/infeasible retraction budgets.
 */
inline ConstrainedOptBenchmark::Options cableRobotBenchmarkOptions(
    ConstrainedOptBenchmark::Options options) {
  options.softMu = 100;
  options.cmFRetractorMaxIterations =
      gtsam::LevenbergMarquardtParams().maxIterations;
  options.cmIRetractorMaxIterations = 1;
  options.cmIRetractFinal = false;
  return options;
}

/** Use the source LM settings, which differed between benchmark methods. */
inline void configureCableRobotLm(ConstrainedOptBenchmark::Method method,
                                  gtsam::LevenbergMarquardtParams *params) {
  if (method == ConstrainedOptBenchmark::Method::SOFT ||
      method == ConstrainedOptBenchmark::Method::CM_F ||
      method == ConstrainedOptBenchmark::Method::CM_I) {
    params->minModelFidelity = 0.3;
    params->setlambdaUpperBound(1e10);
  }
}

/** Preserve the original 15 penalty subproblems with mu = 1, 2, ..., 2^14.
 * Zero stopping tolerances disable the new outer early-exit tests.
 */
inline void configureCableRobotPenalty(gtsam::PenaltyOptimizerParams *params) {
  params->initialMuEq = 1;
  params->muEqIncreaseRate = 2;
  params->maxIterations = 15;
  params->absoluteViolationTolerance = 0;
  params->relativeViolationTolerance = 0;
  params->absoluteCostTolerance = 0;
  params->relativeCostTolerance = 0;
}

/** Preserve the original 12 AL subproblems and uncapped lambda += mu * h
 * update. The experiment predates the BCL policy now selected by GTSAM's
 * defaults.
 */
inline void configureCableRobotAugmentedLagrangian(
    gtsam::AugmentedLagrangianParams *params) {
  configureCableRobotPenalty(params);
  params->maxIterations = 12;
  params->updatePolicy = gtsam::AugmentedLagrangianUpdatePolicy::Aggressive;
  params->maxDualStepSizeEq = std::numeric_limits<double>::infinity();
  params->dualStepSizeFactorEq = 1;
  params->muIncreaseThreshold = 0.25;
}

}  // namespace gtdynamics
