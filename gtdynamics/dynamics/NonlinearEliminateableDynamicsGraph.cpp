/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearDynamicsEliminateableGraph.cpp
 * @brief Builds an eliminateable nonlinear dynamics graph from a Robot object.
 * @author Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearDynamicsEliminateableGraph.h>
#include <gtdynamics/dynamics/NonlinearDynamicsBayesNet.h>
#include <gtdynamics/dynamics/NonlinearDynamicsConditional.h>
#include <gtdynamics/dynamics/NonlinearDynamicsEliminationTree.h>
#include <gtdynamics/dynamics/NonlinearDynamicsBayesTree.h>
#include <gtdynamics/dynamics/NonlinearDynamicsJunctionTree.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>

std::pair<boost::shared_ptr<gtdynamics::NonlinearDynamicsConditional>,
          gtsam::NonlinearFactor::shared_ptr>
EliminateNonlinear(
    const gtdynamics::NonlinearDynamicsEliminateableGraph& factors,
    const gtsam::Ordering& keys) {
  return std::make_pair(boost::make_shared<gtdynamics::NonlinearDynamicsConditional>(),
                        boost::make_shared<gtdynamics::TorqueFactor>());
}

template class gtsam::EliminateableFactorGraph<
    gtdynamics::NonlinearDynamicsEliminateableGraph>;