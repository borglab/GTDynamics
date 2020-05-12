/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearDynamicsBayesTree.h
 * @brief   Nonlinear Bayes Tree, the result of eliminating a
 *          NonlinearDynamicsJunctionTree
 * @author  Mandy Xie
 */

#pragma once

#include <gtdynamics/dynamics/NonlinearDynamicsBayesNet.h>
#include <gtdynamics/dynamics/NonlinearDynamicsEliminateableGraph.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>

namespace gtdynamics {
using gtsam::BayesTree;
using gtsam::BayesTreeCliqueBase;

// Forward declarations
class NonlinearDynamicsConditional;

/* ************************************************************************* */
/** A clique in a NonlinearDynamicsBayesTree */
class NonlinearDynamicsBayesTreeClique
    : public BayesTreeCliqueBase<NonlinearDynamicsBayesTreeClique,
                                 NonlinearDynamicsEliminateableGraph> {
 public:
  typedef NonlinearDynamicsBayesTreeClique This;
  typedef BayesTreeCliqueBase<NonlinearDynamicsBayesTreeClique,
                              NonlinearDynamicsEliminateableGraph>
      Base;
  typedef boost::shared_ptr<This> shared_ptr;
  typedef boost::weak_ptr<This> weak_ptr;
  NonlinearDynamicsBayesTreeClique() {}
  NonlinearDynamicsBayesTreeClique(
      const boost::shared_ptr<NonlinearDynamicsConditional>& conditional)
      : Base(conditional) {}
};

/* ************************************************************************* */
/** A Bayes tree representing a Nonlinear density */
class NonlinearDynamicsBayesTree : public BayesTree<NonlinearDynamicsBayesTreeClique> {
 private:
  typedef BayesTree<NonlinearDynamicsBayesTreeClique> Base;

 public:
  typedef NonlinearDynamicsBayesTree This;
  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor, creates an empty Bayes tree */
  NonlinearDynamicsBayesTree() {}
};

}  // namespace gtdynamics
