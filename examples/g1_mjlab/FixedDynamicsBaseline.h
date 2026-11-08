/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file FixedDynamicsBaseline.h
 * @brief Equality-only manifold LM on the same conditioned Dynamics app problem.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once

#include <gtdynamics/dynamics/internal/FixedDynamicsBenchmark.h>

#include <chrono>

#include "EqualityOnlyBaseline.h"

namespace g1_example {
struct ManifoldLmFixedDynamicsResult {
  gtdynamics::fixed_dynamics::Result result;
  double manifoldLmPreparationSeconds = 0;
};

inline ManifoldLmFixedDynamicsResult solveManifoldLmFixedDynamics(
    const gtdynamics::Robot& robot, const gtdynamics::Trajectory& trajectory,
    const std::vector<gtdynamics::Slice>& slices,
    const std::vector<double>& times, const gtsam::Values& fixedValues,
    const gtsam::Values& accelerationPriors,
    const std::vector<gtdynamics::fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature,
    const gtdynamics::fixed_dynamics::Options& options) {
  using namespace gtdynamics;
  using namespace gtsam;
  using Clock = std::chrono::steady_clock;
  const auto elapsed = [](Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
  };
  const auto snapshot = gtdynamics::internal::prepareFixedDynamicsBenchmark(
      robot, trajectory, slices, times, fixedValues, accelerationPriors,
      contacts, armature, options);
  ManifoldLmFixedDynamicsResult output;
  auto& result = output.result;
  static_cast<fixed_dynamics::Diagnostics&>(result) = snapshot->diagnostics;
  const auto independent = snapshot->independentKeys;
  const BasisKeyFunction select = [independent](const KeyVector& keys) {
    KeyVector selected;
    for (Key key : keys)
      if (independent.exists(key)) selected.push_back(key);
    return selected;
  };
  auto settings = std::make_shared<IEConstraintManifold::Params>();
  settings->affineEqualities = true;
  settings->equalityBasisCreator =
      std::make_shared<EliminationBasisCreator>(select);
  auto started = Clock::now();
  auto components = IEOptimizer::identifyConnectedComponents(
      snapshot->equalities, {}, snapshot->initial, settings);
  for (const auto& [key, point] : components.points)
    result.reducedDimension += point.dim();
  // Type and point conversion is measured separately from the LM loop.
  const auto prepareStart = Clock::now();
  const auto problem =
      manifoldLmProblem(snapshot->cost, components.points, {},
                            settings->equalityBasisCreator, select);
  output.manifoldLmPreparationSeconds = elapsed(prepareStart);
  result.manifoldSeconds = elapsed(started);
  result.objectiveInitial = snapshot->cost.error(snapshot->initial);
  LevenbergMarquardtParams lm;
  lm.maxIterations = options.maxIterations;
  lm.absoluteErrorTol = lm.relativeErrorTol = 1e-8;
  started = Clock::now();
  {
    LMManifoldOptimizer optimizer(ManifoldOptimizerParameters{}, lm);
    result.values = optimizer.optimize(snapshot->cost, problem);
    result.iterations = optimizer.details().back().state.iterations;
    for (const auto& iteration : optimizer.details()) {
      result.objectiveHistory.push_back(iteration.state.error);
      result.gaussianSolves += iteration.trials.size();
    }
  }
  result.optimizationSeconds = elapsed(started);
  result.objectiveFinal = snapshot->cost.error(result.values);
  result.equalityL2 = snapshot->equalities.violationNorm(result.values);
  result.originalInequalityL2 =
      snapshot->originalInequalities.violationNorm(result.values);
  result.feasible = result.equalityL2 < 1e-5;
  result.effortError = snapshot->effort.error(result.values);
  result.wrenchError = snapshot->wrench.error(result.values);
  result.torqueSmoothError = snapshot->torqueSmooth.error(result.values);
  result.wrenchSmoothError = snapshot->wrenchSmooth.error(result.values);
  result.accelerationError = snapshot->acceleration.error(result.values);
  for (const auto& factor : snapshot->rootBalance) {
    const auto native = std::dynamic_pointer_cast<NoiseModelFactor>(factor);
    const Vector6 residual = native->unwhitenedError(result.values);
    result.baseResiduals.push_back(residual);
    result.maxMomentResidual =
        std::max(result.maxMomentResidual, residual.head<3>().norm());
    result.maxForceResidual =
        std::max(result.maxForceResidual, residual.tail<3>().norm());
  }
  // The affine equality chart is constant, so its initial recovery blocks
  // also evaluate the exact reduced gradient at the final solution.
  const auto gradient =
      snapshot->cost.linearize(result.values)->gradientAtZero();
  double squaredNorm = 0;
  for (const auto& [key, point] : components.points) {
    Vector reduced = Vector::Zero(point.dim());
    for (Key ambient : point.values().keys())
      if (gradient.exists(ambient))
        reduced += point.eBasis()->recoverJacobian(ambient).transpose() *
                   gradient.at(ambient);
    squaredNorm += reduced.squaredNorm();
  }
  result.projectedGradientNorm = std::sqrt(squaredNorm);
  result.stationary = result.feasible && result.projectedGradientNorm <= 1e-5;
  return output;
}
}  // namespace g1_example
