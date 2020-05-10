/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearDynamicsJunctionTree.h
 * @author Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearDynamicsBayesTree.h>
#include <gtdynamics/dynamics/NonlinearDynamicsEliminateableGraph.h>
#include <gtsam/inference/JunctionTree.h>
namespace gtdynamics {
using gtsam::JunctionTree;

// Forward declarations
class NonlinearDynamicsEliminationTree;

/**
 * A junction tree specialized to nonlinear factors, i.e., it is a cluster tree
 * with nonlinear factors stored in each cluster. It can be eliminated into a
 * Nonlinear Bayes tree with the same structure, which is essentially doing
 * multifrontal sparse matrix factorization.
 *
 * \addtogroup Multifrontal
 * \nosubgrouping
 */
class NonlinearDynamicsJunctionTree
    : public JunctionTree<NonlinearDynamicsBayesTree, NonlinearDynamicsEliminateableGraph> {
 public:
  typedef JunctionTree<NonlinearDynamicsBayesTree, NonlinearDynamicsEliminateableGraph>
      Base;                                    ///< Base class
  typedef NonlinearDynamicsJunctionTree This;          ///< This class
  typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer to this class

  /**
   * Build the elimination tree of a factor graph using pre-computed column
   * structure.
   * @param factorGraph The factor graph for which to build the elimination tree
   * @param structure The set of factors involving each variable.  If this is
   * not precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
   * named constructor instead.
   * @return The elimination tree
   */
  NonlinearDynamicsJunctionTree(const NonlinearDynamicsEliminationTree& eliminationTree);
};

}  // namespace gtdynamics
