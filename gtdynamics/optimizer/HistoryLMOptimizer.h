/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  HistoryLMOptimizer.h
 * @brief Modified Levenberg Marquardt optimizer that stores history states.
 * @author: Yetong Zhang, Frank Dellaert
 */

#pragma once

#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>

namespace gtsam {

/** LM optimizer that stores history states. */
class HistoryLMOptimizer : public LevenbergMarquardtOptimizer {
public:
  typedef internal::LevenbergMarquardtState State;
  typedef std::vector<State> LMStates;

protected:
  std::shared_ptr<LMStates> states_ = std::make_shared<LMStates>();

public:
  HistoryLMOptimizer(
      const NonlinearFactorGraph &graph, const Values &initialValues,
      const LevenbergMarquardtParams &params = LevenbergMarquardtParams())
      : LevenbergMarquardtOptimizer(graph, initialValues, params),
        states_(std::make_shared<LMStates>()) {
    auto currentState = static_cast<const State *>(state_.get());
    states_->emplace_back(*currentState);
  }

  HistoryLMOptimizer(
      const NonlinearFactorGraph &graph, const Values &initialValues,
      const Ordering &ordering,
      const LevenbergMarquardtParams &params = LevenbergMarquardtParams())
      : LevenbergMarquardtOptimizer(graph, initialValues, ordering, params),
        states_(std::make_shared<LMStates>()) {
    auto currentState = static_cast<const State *>(state_.get());
    states_->emplace_back(*currentState);
  }

  const LMStates &states() const { return *states_; }

  GaussianFactorGraph::shared_ptr iterate() override {
    gttic(LM_iterate);

    // Linearize graph
    GaussianFactorGraph::shared_ptr linear = linearize();

    // Only calculate diagonal of Hessian (expensive) once per outer iteration,
    // if we need it
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
    while (!tryLambda(*linear, sqrtHessianDiagonal)) {
    }
    auto newState = static_cast<const State *>(state_.get());
    states_->emplace_back(*newState);

    return linear;
  }
};

} // namespace gtsam
