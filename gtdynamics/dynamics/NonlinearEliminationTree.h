/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearEliminationTree.h
 * @author Mandy Xie
 */

#pragma once

#include <gtdynamics/dynamics/NonlinearBayesNet.h>
#include <gtdynamics/dynamics/NonlinearEliminateableDynamicsGraph.h>
#include <gtsam/inference/EliminationTree.h>

namespace gtdynamics {
using gtsam::EliminationTree;

  class NonlinearEliminationTree :
    public EliminationTree<NonlinearBayesNet, NonlinearEliminateableDynamicsGraph>
  {
  public:
    typedef EliminationTree<NonlinearBayesNet, NonlinearEliminateableDynamicsGraph> Base; ///< Base class
    typedef NonlinearEliminationTree This; ///< This class
    typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class

  NonlinearEliminationTree() {}
  };

}
