/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearDynamicsJunctionTree.cpp
 * @brief non-linear dynamics junction tree
 * @author Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearDynamicsEliminationTree.h>
#include <gtdynamics/dynamics/NonlinearDynamicsJunctionTree.h>
#include <gtsam/inference/JunctionTree-inst.h>

// Instantiate base classes
template class gtsam::EliminatableClusterTree<
    gtdynamics::NonlinearDynamicsBayesTree,
    gtdynamics::NonlinearDynamicsEliminateableGraph>;
template class gtsam::JunctionTree<
    gtdynamics::NonlinearDynamicsBayesTree,
    gtdynamics::NonlinearDynamicsEliminateableGraph>;

namespace gtdynamics {

/* ************************************************************************* */
NonlinearDynamicsJunctionTree::NonlinearDynamicsJunctionTree(
    const NonlinearDynamicsEliminationTree& eliminationTree)
    : Base(eliminationTree) {}

}  // namespace gtdynamics
