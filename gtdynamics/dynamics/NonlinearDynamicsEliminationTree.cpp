/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearDynamicsEliminationTree.cpp
 * @author Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearDynamicsEliminationTree.h>
#include <gtsam/inference/EliminationTree-inst.h>

// Instantiate base class
template class gtsam::EliminationTree<
    gtdynamics::NonlinearDynamicsBayesNet,
    gtdynamics::NonlinearDynamicsEliminateableGraph>;

namespace gtdynamics {
/* ************************************************************************* */
NonlinearDynamicsEliminationTree::NonlinearDynamicsEliminationTree(
    const NonlinearDynamicsEliminateableGraph& factorGraph,
    const VariableIndex& structure, const Ordering& order)
    : Base(factorGraph, structure, order) {}

/* ************************************************************************* */
NonlinearDynamicsEliminationTree::NonlinearDynamicsEliminationTree(
    const NonlinearDynamicsEliminateableGraph& factorGraph,
    const Ordering& order)
    : Base(factorGraph, order) {}

}  // namespace gtdynamics
