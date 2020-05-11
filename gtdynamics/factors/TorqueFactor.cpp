/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TorqueFactor.cpp
 * @brief Torque factor, common between forward and inverse dynamics.
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearDynamicsConditional.h>
#include <gtdynamics/factors/TorqueFactor.h>

namespace gtdynamics {
/** Dense elimination function for nonlinear dynamics factors.  This is
 * usually provided as an argument to one of the factor graph elimination
 * functions (see EliminateableFactorGraph).  The factor graph elimination
 * functions do sparse variable elimination, and use this function to
 * eliminate single variables or variable cliques. */
std::pair<boost::shared_ptr<NonlinearDynamicsConditional>,
          boost::shared_ptr<TorqueFactor> >
TorqueFactor::EliminateNonlinear() {
  return std::make_pair(boost::make_shared<NonlinearDynamicsConditional>(
                            key1(), key2(), getCostModel(), getScrewAxis()),
                        boost::make_shared<TorqueFactor>());
}

}  // namespace gtdynamics
