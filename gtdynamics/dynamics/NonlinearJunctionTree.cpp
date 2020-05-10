/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearJunctionTree.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtdynamics/dynamics/NonlinearEliminationTree.h>
#include <gtdynamics/dynamics/NonlinearJunctionTree.h>
#include <gtsam/inference/JunctionTree-inst.h>

// Instantiate base classes
template class gtsam::EliminatableClusterTree<
    gtdynamics::NonlinearBayesTree,
    gtdynamics::NonlinearEliminateableDynamicsGraph>;
template class gtsam::JunctionTree<
    gtdynamics::NonlinearBayesTree,
    gtdynamics::NonlinearEliminateableDynamicsGraph>;

namespace gtdynamics {

/* ************************************************************************* */
NonlinearJunctionTree::NonlinearJunctionTree(
    const NonlinearEliminationTree& eliminationTree)
    : Base(eliminationTree) {}

}  // namespace gtdynamics
