/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearDynamicsBayesNet.h
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @brief   NonlinearDynamicsBayesNet
 * @author  Mandy Xie
 */

// \callgraph

#pragma once

#include <gtdynamics/dynamics/NonlinearDynamicsConditional.h>
#include <gtsam/global_includes.h>
#include <gtsam/inference/FactorGraph.h>

namespace gtdynamics {

using gtsam::FactorGraph;

/** A Bayes net made from non-linear Gaussian densities */
class NonlinearDynamicsBayesNet
    : public FactorGraph<NonlinearDynamicsConditional> {
 public:
  typedef FactorGraph<NonlinearDynamicsConditional> Base;
  typedef NonlinearDynamicsBayesNet This;
  typedef NonlinearDynamicsConditional ConditionalType;
  typedef boost::shared_ptr<This> shared_ptr;
  typedef boost::shared_ptr<ConditionalType> sharedConditional;

  /** Construct empty factor graph */
  NonlinearDynamicsBayesNet() {}

};

}  // namespace gtdynamics
