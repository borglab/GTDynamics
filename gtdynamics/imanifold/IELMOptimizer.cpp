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
#include "imanifold/IERetractor.h"
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

  IELMOptimizerState state(manifolds, graph, 0);
  state.lambda = params_.lambdaInitial;
  state.currentFactor = params_.lambdaFactor;
  double currentError = state.error;

  IELMNonlinearIterDetails initial_details;
  initial_details.cost = currentError;
  initial_details.manifolds = manifolds;
  initial_details.new_manifolds = manifolds;
  initial_details.lambda =state.lambda;
  details_->emplace_back(initial_details);

  // check if we're already close enough
  if (currentError <= params_.errorTol) {
    if (params_.verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < "
           << params_.errorTol << endl;
    return IEOptimizer::CollectManifoldValues(state.manifolds);
  }

  // Iterative loop
  double newError;
  size_t num_iters = 0;
  do {
    // std::cout << "iterate\n";
    // Do next iteration
    currentError = state.error;
    state = iterate(graph, state, intermediate_result);
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
  return IEOptimizer::CollectManifoldValues(state.manifolds);
}

/* ************************************************************************* */
IELMOptimizerState
IELMOptimizer::iterate(const NonlinearFactorGraph &graph,
                       const IELMOptimizerState &currentState,
                       gtdynamics::ConstrainedOptResult *intermediate_result) const {

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
  IELMOptimizerState state = currentState;
  
  IELMNonlinearIterDetails iter_details;
  if (checkModeChange(state, graph, iter_details)) {
    iter_details.cost = currentState.error;
    iter_details.lambda = currentState.lambda;
    iter_details.manifolds = currentState.manifolds;
    details_->emplace_back(iter_details);
    return state;
  }

  while (true) {
    auto iter_details = tryLambda(graph, *linear, sqrtHessianDiagonal, state);
    if (iter_details.step_is_successful) {
      details_->emplace_back(iter_details);
      state.decreaseLambda(params_, iter_details.model_fidelity);
      state.updateManifolds(iter_details.new_manifolds, graph);
      break;
    }
    else {
      details_->back().trials.emplace_back(iter_details);
      if (iter_details.stop_searching_lambda) {
        break;
      }
      state.increaseLambda(params_);
      if (state.lambda >= params_.lambdaUpperBound) {
        break;
      }
    }
  }
  return state;
}

bool IsSameMode(const IEManifoldValues &manifolds1,
                const IEManifoldValues &manifolds2) {
  for (const auto &it : manifolds1) {
    const IndexSet &indices1 = it.second.activeIndices();
    const IndexSet &indices2 = manifolds2.at(it.first).activeIndices();
    if (indices1.size() != indices2.size()) {
      return false;
    }
    if (!std::equal(indices1.begin(), indices1.end(), indices2.begin())) {
      return false;
    }
  }
  return true;
}

std::map<Key, IndexSet> IdentifyChangeIndices(const IEManifoldValues &manifolds,
                const IEManifoldValues &new_manifolds) {
  std::map<Key, IndexSet> change_indices_map;
  for (const auto &it : manifolds) {
    const Key& key = it.first;
    const IndexSet &indices = it.second.activeIndices();
    const IndexSet &new_indices = new_manifolds.at(key).activeIndices();
    IndexSet change_indices;
    for (const auto& idx: new_indices) {
      if (indices.find(idx) == indices.end()) {
        change_indices.insert(idx);
      }
    }
    if (change_indices.size() > 0) {
      change_indices_map.insert({key, change_indices});
    }
  }
  return change_indices_map;
}


std::map<Key, IndexSet> IdentifyApproachingIndices(const IEManifoldValues& manifolds, const IEManifoldValues& new_manifolds, const std::map<Key, IndexSet>& change_indices_map) {
  std::map<Key, IndexSet> approach_indices_map;
  for (const auto& it : change_indices_map) {
    const Key& key = it.first;
    const IndexSet &change_indices = it.second;
    const IEConstraintManifold& manifold = manifolds.at(key);
    const IEConstraintManifold& new_manifold = new_manifolds.at(key);
    auto i_constraints = manifolds.at(key).iConstraints();
    IndexSet approach_indices;
    
    for (const auto& idx: change_indices) {
      const auto& constraint = i_constraints->at(idx);
      double eval = (*constraint)(manifold.values());
      double new_eval = (*constraint)(new_manifold.values());
      double approach_rate = eval / new_eval;
      std::cout << "eval: " << eval << "\n";
      std::cout << "new_eval: " << new_eval << "\n";
      std::cout << "approach_rate: " << approach_rate << "\n";
      if (approach_rate > 3) {
        approach_indices.insert(idx);
      }
    }
    if (approach_indices.size() > 0) {
      approach_indices_map.insert({key, change_indices});
    }
  }
  return approach_indices_map;
}

IEManifoldValues
MoveToBoundaries(const IEManifoldValues &manifolds,
                 const std::map<Key, IndexSet> &approach_indices_map) {
  IEManifoldValues new_manifolds;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const auto &manifold = it.second;
    if (approach_indices_map.find(key) == approach_indices_map.end()) {
      new_manifolds.insert({key, manifold});
    } else {
      new_manifolds.insert(
          {key, manifold.moveToBoundary(approach_indices_map.at(key))});
    }
  }
  return new_manifolds;
}

/* ************************************************************************* */
bool IELMOptimizer::checkModeChange(IELMOptimizerState &state,
                                    const NonlinearFactorGraph &graph,
                                    IELMNonlinearIterDetails& iter_details) const {
  // std::cout << "checkModeChange\n";
  if (details_->size() < 2) {
    return false;
  }
  const auto &last_iter_details = details_->back();
  if (last_iter_details.trials.size() > 0) {
    return false;
  }
  int n = details_->size();
  int i = n - 2;
  while (i >= 0 && IsSameMode(last_iter_details.initial.new_manifolds,
                              details_->at(i).initial.new_manifolds)) {
    i--;
  }
  i++;
  // std::cout << i << " " << n << "\n";
  // std::cout << "checkModeChange1\n";
  if (i<n) {
    const auto &init_iter_dertails = details_->at(i);
    const auto &prev_iter_details = details_->at(n - 2);
    if (IsSameMode(last_iter_details.initial.new_manifolds,
                   init_iter_dertails.initial.new_manifolds) &&
        prev_iter_details.trials.size() == 1) {
      std::map<Key, IndexSet> change_indices_map =
          IdentifyChangeIndices(last_iter_details.initial.new_manifolds,
                                prev_iter_details.trials.at(0).new_manifolds);
      if (change_indices_map.size() > 0) {
        auto approach_indices_map = IdentifyApproachingIndices(
            init_iter_dertails.initial.new_manifolds,
            last_iter_details.initial.new_manifolds, change_indices_map);
        if (approach_indices_map.size() > 0) {
          // Enforce approaching indices;
          state.updateManifolds(
              MoveToBoundaries(state.manifolds, approach_indices_map), graph);
          iter_details.forced_indices_map = approach_indices_map;
          iter_details.new_manifolds = state.manifolds;
          iter_details.new_error = state.error;
          return true;
        }
      }
    }
  }
  return false;

}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr
IELMOptimizer::linearize(const NonlinearFactorGraph &graph,
                         const IELMOptimizerState &state) const {

  // linearize on e-manifolds
  std::map<Key, Key> keymap_var2manifold = Var2ManifoldKeyMap(state.manifolds);
  NonlinearFactorGraph manifold_graph = ManifoldOptimizer::ManifoldGraph(
      graph, keymap_var2manifold, state.const_e_manifolds);
  auto active_linear_graph = manifold_graph.linearize(state.e_manifolds);
  return active_linear_graph;
}

/* ************************************************************************* */
IELMNonlinearIterDetails IELMOptimizer::tryLambda(const NonlinearFactorGraph &graph,
                              const GaussianFactorGraph &linear,
                              const VectorValues &sqrtHessianDiagonal,
                              const IELMOptimizerState &currentState) const {


  auto start = std::chrono::high_resolution_clock::now();


  // Build damped system for this lambda (adds prior factors that make it like
  // gradient descent)
  auto dampedSystem =
      buildDampedSystem(currentState, linear, sqrtHessianDiagonal);

  // Try solving
  IELMNonlinearIterDetails iter_details;
  iter_details.step_is_successful = false;
  iter_details.stop_searching_lambda = false;
  double costChange = 0.0;
  Values newValues;
  VectorValues delta;

  iter_details.lambda = currentState.lambda;
  iter_details.manifolds = currentState.manifolds;
  iter_details.blocking_indices_map = currentState.blocking_indices_map;
  iter_details.cost = currentState.error;

  try {
    delta = solve(dampedSystem, params_);

    // TODO: project delta into t-space

    iter_details.solve_successful = true;
  } catch (const IndeterminantLinearSystemException &) {
    iter_details.solve_successful = false;
  }

  if (iter_details.solve_successful) {
    double oldLinearizedError = linear.error(VectorValues::Zero(delta));
    double newlinearizedError = linear.error(delta);
    double linearizedCostChange = oldLinearizedError - newlinearizedError;

    iter_details.linear_cost_change = linearizedCostChange;
      

    if (linearizedCostChange >= 0) { // step is valid
      // std::cout << "step is valid\n";
      iter_details.new_manifolds = currentState.retractManifolds(delta, currentState.blocking_indices_map);
      // std::cout << "retract done\n";
      newValues = CollectManifoldValues(iter_details.new_manifolds);
      iter_details.new_error = graph.error(newValues);
      costChange = currentState.error - iter_details.new_error ;
      // std::cout << "newError: " << newError << "\n";

      if (linearizedCostChange >
          std::numeric_limits<double>::epsilon() * oldLinearizedError) {
        iter_details.model_fidelity = costChange / linearizedCostChange;
        iter_details.step_is_successful = iter_details.model_fidelity > params_.minModelFidelity;
      } // else we consider the step non successful and we either increase
        // lambda or stop if error change is small

      iter_details.nonlinear_cost_change = costChange;
      iter_details.delta = currentState.computeTangentVector(delta);

      double minAbsoluteTolerance =
          params_.relativeErrorTol * currentState.error;
      // if the change is small we terminate
      if (std::abs(costChange) < minAbsoluteTolerance) {
        iter_details.stop_searching_lambda = true;
      }
    }
  } // if (iter_details.solve_successful)

  if (params_.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
    auto end = std::chrono::high_resolution_clock::now();
    double iterationTime =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count() /
        1e6;
    cout << setw(4) << currentState.iterations << " " << setw(10)
         << setprecision(4) << iter_details.new_error << " " << setw(10)
         << setprecision(4) << costChange << " " << setw(5) << setprecision(2)
         << currentState.lambda << " " << setw(4)
         << iter_details.solve_successful << " " << setw(3) << setprecision(2)
         << iterationTime << endl;
  }

  return iter_details;

  // if (step_is_successful) {
  //   // we have successfully decreased the cost and we have good modelFidelity
  //   // NOTE(frank): As we return immediately after this, we move the newValues
  //   // TODO(frank): make Values actually support move. Does not seem to happen
  //   // now.
  //   currentState.decreaseLambda(params_, modelFidelity);
  //   currentState.manifolds = newManifolds;
  //   currentState.values = newValues;
  //   currentState.error = newError;
  //   currentState.ConstructEManifolds(graph);
  //   return true;
  // } else if (!stopSearchingLambda) { // we failed to solved the system or had no
  //                                    // decrease in cost
  //   currentState.increaseLambda(params_);

  //   // check if lambda is too big
  //   if (currentState.lambda >= params_.lambdaUpperBound) {
  //     return true;
  //   } else {
  //     return false; // only case where we will keep trying
  //   }
  // } else { // the change in the cost is very small and it is not worth trying
  //          // bigger lambdas
  //   return true;
  // }
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

  if (newError > currentError) {return false;}

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / currentError;
  bool converged =
      (relativeErrorTreshold && (relativeDecrease <= relativeErrorTreshold)) ||
      (absoluteDecrease <= absoluteErrorTreshold);
  return converged;
}

} /* namespace gtsam */
