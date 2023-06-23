/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ICLMOptimizer.cpp
 * @brief   A nonlinear optimizer that uses the Levenberg-Marquardt trust-region
 * scheme
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    Feb 26, 2012
 */

#include "manifold/IneqConstraintManifold.h"
#include "optimizer/InequalityConstraint.h"
#include <gtdynamics/manifold/ICLMOptimizer.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianBayesNet.h>

#ifdef GTSAM_USE_BOOST_FEATURES
#include <gtsam/base/timing.h>
#endif

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

using namespace std;

namespace gtsam {

typedef internal::LevenbergMarquardtState State;

/* ************************************************************************* */
Values ICLMOptimizer::optimize(
    const NonlinearFactorGraph &graph,
    const std::vector<IneqConstraintManifold::shared_ptr> &ic_manifolds,
    gtdynamics::ConstrainedOptResult* intermediate_result) const {

  ICOptimizerState state(ic_manifolds, graph, 0);
  state.lambda = params_.lambdaInitial;
  state.currentFactor = params_.lambdaFactor;
  double currentError = state.error;

  // check if we're already close enough
  if (currentError <= params_.errorTol) {
    if (params_.verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < "
           << params_.errorTol << endl;
    return state.values;
  }

  // Iterative loop
  double newError;
  size_t num_iters = 0;
  do {
    // Do next iteration
    currentError = state.error;
    state = iterate(graph, state);
    newError = state.error;
    num_iters++;

  } while (state.iterations < params_.maxIterations &&
           !checkConvergence(params_.relativeErrorTol, params_.absoluteErrorTol,
                             params_.errorTol, currentError, newError) &&
           std::isfinite(currentError));
  if (intermediate_result) {
    intermediate_result->num_iters.push_back(num_iters);
  }
  return state.values;
}

/* ************************************************************************* */
ICOptimizerState
ICLMOptimizer::iterate(const NonlinearFactorGraph &graph,
                       const ICOptimizerState &currentState) const {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = linearize(graph, currentState);

  // if(currentState.iterations==0) { // write initial error
  //   if (params_.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
  //     cout << "Initial error: " << currentState.error
  //          << ", values: " << currentState.values.size() << std::endl;
  //   }
  // }

  // Only calculate diagonal of Hessian (expensive) once per outer iteration, if
  // we need it
  VectorValues sqrtHessianDiagonal;
  if (params_.diagonalDamping) {
    sqrtHessianDiagonal = linear->hessianDiagonal();
    for (auto &[key, value] : sqrtHessianDiagonal) {
      value = value.cwiseMax(params_.minDiagonal)
                  .cwiseMin(params_.maxDiagonal)
                  .cwiseSqrt();
    }
  }

  // Keep increasing lambda until we make make progress
  ICOptimizerState state = currentState;
  while (!tryLambda(graph, *linear, sqrtHessianDiagonal, state)) {
  }

  return state;
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr
ICLMOptimizer::linearize(const NonlinearFactorGraph &graph,
                         const ICOptimizerState &state) const {
  auto linear_graph = graph.linearize(state.values);
  auto descent_dir = -1 * linear_graph->gradientAtZero();

  // identify tight constraints
  gtdynamics::InequalityConstraints tight_constraints;
  for (const auto &manifold : state.manifolds) {
    VectorValues direction_m;
    for (const Key &key : manifold->values().keys()) {
      direction_m.insert(key, descent_dir.at(key));
    }
    auto result = manifold->identifyBlockingConstraints(direction_m);
    for (const auto &tight_index : result.first) {
      tight_constraints.push_back(manifold->constraints().at(tight_index));
    }
  }

  // add tight constraints to linear graph
  auto constrained_noise = noiseModel::Constrained::All(1);
  NonlinearFactorGraph constraint_graph;
  for (const auto &constraint : tight_constraints) {
    constraint_graph.add(
        constraint->createL2Factor(1.0)->cloneWithNewNoiseModel(
            constrained_noise));
  }
  auto constraint_linear_graph = constraint_graph.linearize(state.values);
  linear_graph->push_back(*constraint_linear_graph);
  return linear_graph;
}

/* ************************************************************************* */
bool ICLMOptimizer::tryLambda(const NonlinearFactorGraph &graph,
                              const GaussianFactorGraph &linear,
                              const VectorValues &sqrtHessianDiagonal,
                              ICOptimizerState &currentState) const {

#ifdef GTSAM_USE_BOOST_FEATURES
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
  boost::timer::cpu_timer lamda_iteration_timer;
  lamda_iteration_timer.start();
#else
  boost::timer lamda_iteration_timer;
  lamda_iteration_timer.restart();
#endif
#else
  auto start = std::chrono::high_resolution_clock::now();
#endif

  // Build damped system for this lambda (adds prior factors that make it like
  // gradient descent)
  auto dampedSystem = buildDampedSystem(currentState, linear, sqrtHessianDiagonal);

  // Try solving
  double modelFidelity = 0.0;
  bool step_is_successful = false;
  bool stopSearchingLambda = false;
  double newError = numeric_limits<double>::infinity();
  double costChange = 0.0;
  Values newValues;
  VectorValues delta;
  std::vector<IneqConstraintManifold::shared_ptr> newManifolds;

  bool systemSolvedSuccessfully;
  try {
    delta = solve(dampedSystem, params_);

    // TODO: project delta into t-space

    systemSolvedSuccessfully = true;
  } catch (const IndeterminantLinearSystemException &) {
    systemSolvedSuccessfully = false;
  }

  if (systemSolvedSuccessfully) {
    double oldLinearizedError = linear.error(VectorValues::Zero(delta));
    double newlinearizedError = linear.error(delta);
    double linearizedCostChange = oldLinearizedError - newlinearizedError;

    if (linearizedCostChange >= 0) { // step is valid
      newManifolds = retract_manifolds(currentState, delta);
      newValues = CollectManifoldValues(newManifolds);
      newError = graph.error(newValues);
      costChange = currentState.error - newError;

      if (linearizedCostChange >
          std::numeric_limits<double>::epsilon() * oldLinearizedError) {
        modelFidelity = costChange / linearizedCostChange;
        step_is_successful = modelFidelity > params_.minModelFidelity;
      } // else we consider the step non successful and we either increase
        // lambda or stop if error change is small

      double minAbsoluteTolerance =
          params_.relativeErrorTol * currentState.error;
      // if the change is small we terminate
      if (std::abs(costChange) < minAbsoluteTolerance) {
        stopSearchingLambda = true;
      }
    }
  } // if (systemSolvedSuccessfully)

  if (params_.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
#ifdef GTSAM_USE_BOOST_FEATURES
// do timing
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
    double iterationTime = 1e-9 * lamda_iteration_timer.elapsed().wall;
#else
    double iterationTime = lamda_iteration_timer.elapsed();
#endif
#else
    auto end = std::chrono::high_resolution_clock::now();
    double iterationTime =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count() /
        1e6;
#endif
    if (currentState.iterations == 0) {
      cout << "iter      cost      cost_change    lambda  success iter_time"
           << endl;
    }
    cout << setw(4) << currentState.iterations << " " << setw(8) << newError
         << " " << setw(3) << setprecision(2) << costChange << " " << setw(3)
         << setprecision(2) << currentState.lambda << " " << setw(4)
         << systemSolvedSuccessfully << " " << setw(3) << setprecision(2)
         << iterationTime << endl;
  }
  if (step_is_successful) {
    // we have successfully decreased the cost and we have good modelFidelity
    // NOTE(frank): As we return immediately after this, we move the newValues
    // TODO(frank): make Values actually support move. Does not seem to happen
    // now.
    currentState.decreaseLambda(params_, modelFidelity);
    currentState.manifolds = newManifolds;
    currentState.values = newValues;
    currentState.error = newError;
    return true;
  } else if (!stopSearchingLambda) { // we failed to solved the system or had no
                                     // decrease in cost
    currentState.increaseLambda(params_);

    // check if lambda is too big
    if (currentState.lambda >= params_.lambdaUpperBound) {
      return true;
    } else {
      return false; // only case where we will keep trying
    }
  } else { // the change in the cost is very small and it is not worth trying
           // bigger lambdas
    return true;
  }
}

/* ************************************************************************* */
GaussianFactorGraph ICLMOptimizer::buildDampedSystem(
    const ICOptimizerState &state,
    const GaussianFactorGraph &linear,
    const VectorValues &sqrtHessianDiagonal) const {

  if (params_.verbosityLM >= LevenbergMarquardtParams::DAMPED)
    std::cout << "building damped system with lambda " << state.lambda
              << std::endl;

  if (params_.diagonalDamping)
    return state.buildDampedSystem(linear, sqrtHessianDiagonal);
  else
    return state.buildDampedSystem(linear);
}

/* ************************************************************************* */
std::vector<IneqConstraintManifold::shared_ptr>
ICLMOptimizer::retract_manifolds(const ICOptimizerState &state,
                                 const VectorValues &delta) const {
  std::vector<IneqConstraintManifold::shared_ptr> new_manifolds;
  for (const auto manifold : state.manifolds) {
    VectorValues delta_m;
    for (const Key& key: manifold->values().keys()) {
      delta_m.insert(key, delta.at(key));
    }
    IndexSet tight_indices; // TODO: may need to pass it in
    new_manifolds.push_back(manifold->retract(delta, tight_indices));
  }
  return new_manifolds;
}



/* ************************************************************************* */
VectorValues ICLMOptimizer::solve(const GaussianFactorGraph& gfg,
                                       const NonlinearOptimizerParams& params) const {
  // solution of linear solver is an update to the linearization point
  VectorValues delta;

  // Check which solver we are using
  if (params.isMultifrontal()) {
    // Multifrontal QR or Cholesky (decided by params.getEliminationFunction())
    if (params.ordering)
      delta = gfg.optimize(*params.ordering, params.getEliminationFunction());
    else
      delta = gfg.optimize(params.getEliminationFunction());
  } else if (params.isSequential()) {
    // Sequential QR or Cholesky (decided by params.getEliminationFunction())
    if (params.ordering)
      delta = gfg.eliminateSequential(*params.ordering,
                                      params.getEliminationFunction())
                  ->optimize();
    else
      delta = gfg.eliminateSequential(params.orderingType,
                                      params.getEliminationFunction())
                  ->optimize();
  } else if (params.isIterative()) {
    // Conjugate Gradient -> needs params.iterativeParams
    if (!params.iterativeParams)
      throw std::runtime_error(
          "NonlinearOptimizer::solve: cg parameter has to be assigned ...");

    if (auto pcg = std::dynamic_pointer_cast<PCGSolverParameters>(
            params.iterativeParams)) {
      delta = PCGSolver(*pcg).optimize(gfg);
    } else if (auto spcg =
                   std::dynamic_pointer_cast<SubgraphSolverParameters>(
                       params.iterativeParams)) {
      if (!params.ordering)
        throw std::runtime_error("SubgraphSolver needs an ordering");
      delta = SubgraphSolver(gfg, *spcg, *params.ordering).optimize();
    } else {
      throw std::runtime_error(
          "NonlinearOptimizer::solve: special cg parameter type is not handled in LM solver ...");
    }
  } else {
    throw std::runtime_error("NonlinearOptimizer::solve: Optimization parameter is invalid");
  }

  // return update
  return delta;
}


/* ************************************************************************* */
bool ICLMOptimizer::checkConvergence(double relativeErrorTreshold, double absoluteErrorTreshold,
                      double errorThreshold, double currentError, double newError) const {

  if (newError <= errorThreshold)
    return true;

  // check if diverges
  double absoluteDecrease = currentError - newError;

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / currentError;
  bool converged = (relativeErrorTreshold && (relativeDecrease <= relativeErrorTreshold)) ||
                   (absoluteDecrease <= absoluteErrorTreshold);
  return converged;
}


} /* namespace gtsam */
