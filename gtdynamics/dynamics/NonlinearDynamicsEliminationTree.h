/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearDynamicsEliminationTree.h
 * @author Mandy Xie
 */

#pragma once

#include <gtdynamics/dynamics/NonlinearDynamicsBayesNet.h>
#include <gtdynamics/dynamics/NonlinearDynamicsEliminateableGraph.h>
#include <gtsam/inference/EliminationTree.h>

namespace gtdynamics {
using gtsam::EliminationTree;

class NonlinearDynamicsEliminationTree
    : public EliminationTree<NonlinearDynamicsBayesNet,
                             NonlinearDynamicsEliminateableGraph> {
 public:
  typedef EliminationTree<NonlinearDynamicsBayesNet,
                          NonlinearDynamicsEliminateableGraph>
      Base;                                       ///< Base class
  typedef NonlinearDynamicsEliminationTree This;  ///< This class
  typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer to this class

  NonlinearDynamicsEliminationTree() {}
};

}  // namespace gtdynamics
