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
  using This = NonlinearDynamicsBayesTreeClique;
  using Base = BayesTreeCliqueBase<NonlinearDynamicsBayesTreeClique,
                                   NonlinearDynamicsEliminateableGraph>;
  using shared_ptr = boost::shared_ptr<This>;
  using weak_ptr = boost::weak_ptr<This>;
  NonlinearDynamicsBayesTreeClique() {}
  NonlinearDynamicsBayesTreeClique(
      const boost::shared_ptr<NonlinearDynamicsConditional>& conditional)
      : Base(conditional) {}
};

/* ************************************************************************* */
/** A Bayes tree representing a Nonlinear density */
class NonlinearDynamicsBayesTree
    : public BayesTree<NonlinearDynamicsBayesTreeClique> {
 private:
  using Base = BayesTree<NonlinearDynamicsBayesTreeClique>;

 public:
  using This = NonlinearDynamicsBayesTree;
  using shared_ptr = boost::shared_ptr<This>;

  /** Default constructor, creates an empty Bayes tree */
  NonlinearDynamicsBayesTree() {}
};

}  // namespace gtdynamics
