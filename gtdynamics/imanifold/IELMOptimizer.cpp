/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IELMOptimizer.cpp
 * @brief   A nonlinear optimizer that uses the Levenberg-Marquardt trust-region
 * scheme
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    Feb 26, 2012
 */

#include "imanifold/IEManifoldOptimizer.h"
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>

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
Values IELMOptimizer::optimizeManifolds(
    const NonlinearFactorGraph &graph, const IEManifoldValues &manifolds,
    gtdynamics::ConstrainedOptResult *intermediate_result) const {

  std::map<Key, Key> keymap_var2manifold = Var2ManifoldKeyMap(manifolds);
  NonlinearFactorGraph manifold_graph = ManifoldOptimizer::ManifoldGraph(graph, keymap_var2manifold);

  IELMOptimizerState state(manifolds, graph, manifold_graph, 0);
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
    // std::cout << "iterate\n";
    // Do next iteration
    currentError = state.error;
    state = iterate(graph, manifold_graph, state, intermediate_result);
    // std::cout << "iterate finished\n";
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
IELMOptimizerState
IELMOptimizer::iterate(const NonlinearFactorGraph &graph,
                       const NonlinearFactorGraph &manifold_graph,
                       const IELMOptimizerState &currentState,
                       gtdynamics::ConstrainedOptResult *intermediate_result) const {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = linearize(manifold_graph, currentState);

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
  IELMOptimizerState state = currentState;
  while (!tryLambda(graph, manifold_graph, *linear, sqrtHessianDiagonal, state, intermediate_result)) {
  }
  if (intermediate_result) {
    intermediate_result->intermediate_values.emplace_back(state.values);
  }
  return state;
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr
IELMOptimizer::linearize(const NonlinearFactorGraph &manifold_graph,
                         const IELMOptimizerState &state) const {

  // linearize on e-manifolds
  auto active_linear_graph = manifold_graph.linearize(state.e_manifolds);
  return active_linear_graph;
}

/* ************************************************************************* */
bool IELMOptimizer::tryLambda(const NonlinearFactorGraph &graph,
const NonlinearFactorGraph &manifold_graph,
                              const GaussianFactorGraph &linear,
                              const VectorValues &sqrtHessianDiagonal,
                              IELMOptimizerState &currentState,
                              gtdynamics::ConstrainedOptResult *intermediate_result) const {

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
  auto dampedSystem =
      buildDampedSystem(currentState, linear, sqrtHessianDiagonal);

  // Try solving
  double modelFidelity = 0.0;
  bool step_is_successful = false;
  bool stopSearchingLambda = false;
  double newError = numeric_limits<double>::infinity();
  double costChange = 0.0;
  Values newValues;
  VectorValues delta;
  IEManifoldValues newManifolds;

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
      // std::cout << "step is valid\n";
      newManifolds = currentState.retractManifolds(delta);
      // std::cout << "retract done\n";
      newValues = CollectManifoldValues(newManifolds);
      newError = graph.error(newValues);
      costChange = currentState.error - newError;
      // std::cout << "newError: " << newError << "\n";

      if (linearizedCostChange >
          std::numeric_limits<double>::epsilon() * oldLinearizedError) {
        modelFidelity = costChange / linearizedCostChange;
        step_is_successful = modelFidelity > params_.minModelFidelity;
      } // else we consider the step non successful and we either increase
        // lambda or stop if error change is small

      if (step_is_successful && intermediate_result) {
        intermediate_result->tangent_vectors.emplace_back(currentState.computeTangentVector(delta));
      }

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
    // if (currentState.iterations == 0) {
    //   cout << "iter      cost      cost_change    lambda  success iter_time"
    //        << endl;
    // }
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
    currentState.ConstructEManifolds(manifold_graph);
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
GaussianFactorGraph IELMOptimizer::buildDampedSystem(
    const IELMOptimizerState &state, const GaussianFactorGraph &linear,
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
VectorValues
IELMOptimizer::solve(const GaussianFactorGraph &gfg,
                     const NonlinearOptimizerParams &params) const {
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
    } else if (auto spcg = std::dynamic_pointer_cast<SubgraphSolverParameters>(
                   params.iterativeParams)) {
      if (!params.ordering)
        throw std::runtime_error("SubgraphSolver needs an ordering");
      delta = SubgraphSolver(gfg, *spcg, *params.ordering).optimize();
    } else {
      throw std::runtime_error(
          "NonlinearOptimizer::solve: special cg parameter type is not handled "
          "in LM solver ...");
    }
  } else {
    throw std::runtime_error(
        "NonlinearOptimizer::solve: Optimization parameter is invalid");
  }

  // return update
  return delta;
}

/* ************************************************************************* */
bool IELMOptimizer::checkConvergence(double relativeErrorTreshold,
                                     double absoluteErrorTreshold,
                                     double errorThreshold, double currentError,
                                     double newError) const {

  if (newError <= errorThreshold)
    return true;

  // check if diverges
  double absoluteDecrease = currentError - newError;

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / currentError;
  bool converged =
      (relativeErrorTreshold && (relativeDecrease <= relativeErrorTreshold)) ||
      (absoluteDecrease <= absoluteErrorTreshold);
  return converged;
}

} /* namespace gtsam */
