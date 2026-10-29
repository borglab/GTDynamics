/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file UnconstrainedOptimizationRetractor.cpp
 * @brief UnconstrainedOptimizationRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmopt/UnconstrainedOptimizationRetractor.h>
#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "factors/ConstVarFactor.h"
#include "utils/DynamicsSymbol.h"
#include "utils/values.h"

namespace gtdynamics {
using namespace gtsam;

UnconstrainedOptimizationRetractor::UnconstrainedOptimizationRetractor(
    const NonlinearEqualityConstraints::shared_ptr &constraints,
    const RetractionParams::shared_ptr &params)
    : Retractor(params),
      optimizer_(constraints->penaltyGraph(), params->lmParams) {}

/* ************************************************************************* */
Values
UnconstrainedOptimizationRetractor::retractConstraints(const Values &values) {
  optimizer_.setValues(values);
  const Values &result = optimizer_.optimize();
  checkFeasible(optimizer_.mutableGraph(), result);
  return result;
}

/* ************************************************************************* */
Values UnconstrainedOptimizationRetractor::retractConstraints(Values &&values) {
  optimizer_.setValues(values);
  auto result = optimizer_.optimize();
  checkFeasible(optimizer_.mutableGraph(), result);
  return std::move(result);
}

} // namespace gtdynamics
