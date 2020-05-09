/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearEliminateableDynamicsGraph.cpp
 * @brief Builds an eliminateable nonlinear dynamics graph from a Robot object.
 * @author Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearEliminateableDynamicsGraph.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>

std::pair<boost::shared_ptr<int>, gtsam::NonlinearFactor::shared_ptr>
EliminateNonlinear(
    const gtdynamics::NonlinearEliminateableDynamicsGraph& factors,
    const gtsam::Ordering& keys) {
  return std::make_pair(boost::make_shared<int>(),
                        boost::make_shared<gtdynamics::TorqueFactor>());
}

// template class gtsam::EliminateableFactorGraph<
//     gtdynamics::NonlinearEliminateableDynamicsGraph>;