/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearDynamicsBayesTree.cpp
 * @brief   Nonlinear Bayes Tree, the result of eliminating a
 *          NonlinearDynamicsJunctionTree
 * @author  Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearDynamicsBayesNet.h>
#include <gtdynamics/dynamics/NonlinearDynamicsBayesTree.h>
#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>

// Instantiate base class
template class gtsam::BayesTreeCliqueBase<
    gtdynamics::NonlinearDynamicsBayesTreeClique,
    gtdynamics::NonlinearDynamicsEliminateableGraph>;
template class gtsam::BayesTree<gtdynamics::NonlinearDynamicsBayesTreeClique>;
