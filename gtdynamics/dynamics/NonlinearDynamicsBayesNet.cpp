/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearDynamicsBayesNet.cpp
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @author  Mandy Xie
 */

// \callgraph

#include <gtdynamics/dynamics/NonlinearDynamicsBayesNet.h>
#include <gtdynamics/dynamics/NonlinearDynamicsConditional.h>
#include <gtsam/inference/FactorGraph-inst.h>

// Instantiate base class
template class gtsam::FactorGraph<gtdynamics::NonlinearDynamicsConditional>;

namespace gtdynamics {
/* ************************************************************************* */
bool NonlinearDynamicsBayesNet::equals(const This& bn, double tol) const {
  return Base::equals(bn, tol);
}
}  // namespace gtdynamics