/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearDynamicsEliminationTree.h
 * @brief non-linear dynamics elimination tree
 * @author Mandy Xie
 */

#pragma once

#include <gtdynamics/dynamics/NonlinearDynamicsBayesNet.h>
#include <gtdynamics/dynamics/NonlinearDynamicsEliminateableGraph.h>
#include <gtsam/inference/EliminationTree.h>

namespace gtdynamics {
using gtsam::EliminationTree;
using gtsam::Ordering;
using gtsam::VariableIndex;

class NonlinearDynamicsEliminationTree
    : public EliminationTree<NonlinearDynamicsBayesNet,
                             NonlinearDynamicsEliminateableGraph> {
 public:
  using Base =
      EliminationTree<NonlinearDynamicsBayesNet,
                      NonlinearDynamicsEliminateableGraph>;  ///< Base class
  using This = NonlinearDynamicsEliminationTree;             ///< This class
  using shared_ptr = boost::shared_ptr<This>;  ///< Shared pointer to this class

  /**
   * Build the elimination tree of a factor graph using pre-computed column
   * structure.
   * @param factorGraph The factor graph for which to build the elimination tree
   * @param structure The set of factors involving each variable.  If this is
   * not precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
   * named constructor instead.
   * @return The elimination tree
   */
  NonlinearDynamicsEliminationTree(
      const NonlinearDynamicsEliminateableGraph& factorGraph,
      const VariableIndex& structure, const Ordering& order);

  /** Build the elimination tree of a factor graph.  Note that this has to
   * compute the column structure as a VariableIndex, so if you already have
   * this precomputed, use the other constructor instead.
   * @param factorGraph The factor graph for which to build the elimination tree
   */
  NonlinearDynamicsEliminationTree(
      const NonlinearDynamicsEliminateableGraph& factorGraph,
      const Ordering& order);
};

}  // namespace gtdynamics
