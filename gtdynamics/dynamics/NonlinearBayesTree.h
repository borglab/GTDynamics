/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearBayesTree.h
 * @brief   Nonlinear Bayes Tree, the result of eliminating a
 * NonlinearJunctionTree
 * @brief   NonlinearBayesTree
 * @author  Mandy Xie
 */

#pragma once

#include <gtdynamics/dynamics/NonlinearBayesNet.h>
#include <gtdynamics/dynamics/NonlinearEliminateableDynamicsGraph.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>

namespace gtdynamics {
using gtsam::BayesTree;
using gtsam::BayesTreeCliqueBase;

// Forward declarations
class NonlinearConditional;

/* ************************************************************************* */
/** A clique in a NonlinearBayesTree */
class NonlinearBayesTreeClique
    : public BayesTreeCliqueBase<NonlinearBayesTreeClique,
                                 NonlinearEliminateableDynamicsGraph> {
 public:
  typedef NonlinearBayesTreeClique This;
  typedef BayesTreeCliqueBase<NonlinearBayesTreeClique,
                              NonlinearEliminateableDynamicsGraph>
      Base;
  typedef boost::shared_ptr<This> shared_ptr;
  typedef boost::weak_ptr<This> weak_ptr;
  NonlinearBayesTreeClique() {}
  NonlinearBayesTreeClique(
      const boost::shared_ptr<NonlinearConditional>& conditional)
      : Base(conditional) {}
};

/* ************************************************************************* */
/** A Bayes tree representing a Nonlinear density */
class NonlinearBayesTree : public BayesTree<NonlinearBayesTreeClique> {
 private:
  typedef BayesTree<NonlinearBayesTreeClique> Base;

 public:
  typedef NonlinearBayesTree This;
  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor, creates an empty Bayes tree */
  NonlinearBayesTree() {}
};

}  // namespace gtdynamics
