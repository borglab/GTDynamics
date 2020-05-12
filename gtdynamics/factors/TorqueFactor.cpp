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
std::pair<boost::shared_ptr<NonlinearDynamicsConditional>,
          boost::shared_ptr<TorqueFactor> >
TorqueFactor::EliminateNonlinear() {
  return std::make_pair(boost::make_shared<NonlinearDynamicsConditional>(
                            key1(), key2(), getCostModel(), getScrewAxis()),
                        boost::make_shared<TorqueFactor>());
}

}  // namespace gtdynamics
