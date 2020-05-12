/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearDynamicsEliminateableGraph.cpp
 * @brief eliminateable nonlinear dynamics graph.
 * @author Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearDynamicsBayesNet.h>
#include <gtdynamics/dynamics/NonlinearDynamicsBayesTree.h>
#include <gtdynamics/dynamics/NonlinearDynamicsConditional.h>
#include <gtdynamics/dynamics/NonlinearDynamicsEliminateableGraph.h>
#include <gtdynamics/dynamics/NonlinearDynamicsEliminationTree.h>
#include <gtdynamics/dynamics/NonlinearDynamicsJunctionTree.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>

template class gtsam::FactorGraph<gtdynamics::TorqueFactor>;
template class gtsam::EliminateableFactorGraph<
    gtdynamics::NonlinearDynamicsEliminateableGraph>;

namespace gtdynamics {
std::pair<boost::shared_ptr<NonlinearDynamicsConditional>,
          boost::shared_ptr<TorqueFactor>>
EliminateNonlinear(const NonlinearDynamicsEliminateableGraph& factors,
                   const gtsam::Ordering& keys) {
  return factors[0]->EliminateNonlinear();
}

/* ************************************************************************* */
bool NonlinearDynamicsEliminateableGraph::equals(const This& fg,
                                                 double tol) const {
  return Base::equals(fg, tol);
}
}  // namespace gtdynamics
