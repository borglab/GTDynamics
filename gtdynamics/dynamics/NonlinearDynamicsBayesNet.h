/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearDynamicsBayesNet.h
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @author  Mandy Xie
 */

// \callgraph

#pragma once

#include <gtdynamics/dynamics/NonlinearDynamicsConditional.h>
#include <gtsam/inference/FactorGraph.h>

namespace gtdynamics {

using gtsam::FactorGraph;

/** A Bayes net made from non-linear Gaussian densities */
class NonlinearDynamicsBayesNet
    : public FactorGraph<NonlinearDynamicsConditional> {
 public:
  using Base = FactorGraph<NonlinearDynamicsConditional>;
  using This = NonlinearDynamicsBayesNet;
  using ConditionalType = NonlinearDynamicsConditional;
  using shared_ptr = boost::shared_ptr<This>;
  using sharedConditional = boost::shared_ptr<ConditionalType>;

  /** Construct empty factor graph */
  NonlinearDynamicsBayesNet() {}

  /** Construct from iterator over conditionals */
  template <typename ITERATOR>
  NonlinearDynamicsBayesNet(ITERATOR firstConditional, ITERATOR lastConditional)
      : Base(firstConditional, lastConditional) {}

  /** Construct from container of factors (shared_ptr or plain objects) */
  template <class CONTAINER>
  explicit NonlinearDynamicsBayesNet(const CONTAINER& conditionals)
      : Base(conditionals) {}

  /** Implicit copy/downcast constructor to override explicit template container
   * constructor */
  template <class DERIVEDCONDITIONAL>
  NonlinearDynamicsBayesNet(const FactorGraph<DERIVEDCONDITIONAL>& graph)
      : Base(graph) {}

  /// @name Testable
  /// @{
  /** Check equality */
  bool equals(const This& bn, double tol = 1e-9) const;
  /// @}
};

}  // namespace gtdynamics

/// traits
template <>
struct gtsam::traits<gtdynamics::NonlinearDynamicsBayesNet>
    : public gtsam::Testable<gtdynamics::NonlinearDynamicsBayesNet> {};
