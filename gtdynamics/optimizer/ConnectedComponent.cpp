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

#include <gtdynamics/optimizer/ConnectedComponent.h>

#include <gtdynamics/optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace gtsam {

/* ************************************************************************* */
ConnectedComponent::ConnectedComponent(
    const gtdynamics::EqualityConstraints& _constraints)
    : constraints(_constraints) {
  merit_graph = gtsam::NonlinearFactorGraph();
  for (const auto& constraint : constraints) {
    merit_graph.add(constraint->createFactor(1.0));
  }
  keys = merit_graph.keys();
}


}  // namespace gtsam
