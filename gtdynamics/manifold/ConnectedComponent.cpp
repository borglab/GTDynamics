/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConnectedComponent.cpp
 * @brief Connected component implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/manifold/ConnectedComponent.h>

namespace gtsam {

/* ************************************************************************* */
NonlinearFactorGraph ConnectedComponent::constructMeritGraph(
    const gtdynamics::EqualityConstraints &constraints) {
  gtsam::NonlinearFactorGraph graph;
  for (const auto &constraint : constraints) {
    graph.add(constraint->createFactor(1.0));
  }
  return graph;
}

} // namespace gtsam
