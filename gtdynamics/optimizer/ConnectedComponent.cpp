/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConnectedComponent.h
 * @brief Connected component implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/optimizer/ConnectedComponent.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace gtsam {

/* ************************************************************************* */
NonlinearFactorGraph ConnectedComponent::constructMeritGraph(
    const gtdynamics::EqualityConstraints& _constraints) {
  gtsam::NonlinearFactorGraph graph;
  for (const auto& constraint : _constraints) {
    graph.add(constraint->createFactor(1.0));
  }
  return graph;
}

}  // namespace gtsam
