/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearBayesNet.h
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @brief   NonlinearBayesNet
 * @author  Mandy Xie
 */

// \callgraph

#pragma once

#include <gtdynamics/dynamics/NonlinearConditional.h>
#include <gtsam/global_includes.h>
#include <gtsam/inference/FactorGraph.h>

namespace gtdynamics {

using gtsam::FactorGraph;

/** A Bayes net made from non-linear Gaussian densities */
class NonlinearBayesNet
    : public FactorGraph<NonlinearConditional> {
 public:
  typedef FactorGraph<NonlinearConditional> Base;
  typedef NonlinearBayesNet This;
  typedef NonlinearConditional ConditionalType;
  typedef boost::shared_ptr<This> shared_ptr;
  typedef boost::shared_ptr<ConditionalType> sharedConditional;

  /** Construct empty factor graph */
  NonlinearBayesNet() {}
};

}  // namespace gtdynamics
