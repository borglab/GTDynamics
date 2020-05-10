/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearDynamicsJunctionTree.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
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
