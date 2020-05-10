/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearEliminationTree.cpp
 * @author Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearEliminationTree.h>
#include <gtsam/inference/EliminationTree-inst.h>

// Instantiate base class
template class gtsam::EliminationTree<gtdynamics::NonlinearBayesNet,
                               gtsam::NonlinearFactorGraph>;

namespace gtdynamics {}  // namespace gtdynamics
