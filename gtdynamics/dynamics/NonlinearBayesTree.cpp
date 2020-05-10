/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearBayesTree.cpp
 * @brief   Nonlinear Bayes Tree, the result of eliminating a
 * NonlinearJunctionTree
 * @brief   NonlinearBayesTree
 * @author  Mandy Xie
 */

#pragma once

#include <gtdynamics/dynamics/NonlinearBayesNet.h>
#include <gtdynamics/dynamics/NonlinearBayesTree.h>
#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>

// Instantiate base class
template class gtsam::BayesTreeCliqueBase<
    gtdynamics::NonlinearBayesTreeClique,
    gtdynamics::NonlinearEliminateableDynamicsGraph>;
template class gtsam::BayesTree<gtdynamics::NonlinearBayesTreeClique>;