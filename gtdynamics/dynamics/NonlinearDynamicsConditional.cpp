/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   NonlinearDynamicsConditional.cpp
 * @brief  Nonlinear Conditional Base class
 * @author Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearDynamicsConditional.h>
#include <gtsam/inference/Conditional-inst.h>

template class gtsam::Conditional<gtsam::NoiseModelFactor,
                                  gtdynamics::NonlinearDynamicsConditional>;

namespace gtdynamics {

/** print */
void NonlinearDynamicsConditional::print(
    const std::string &s, const gtsam::KeyFormatter &formatter) const {}

/** equals function */
bool NonlinearDynamicsConditional::equals(const gtsam::NoiseModelFactor &cg,
                                          double tol) const {
  return true;
}
}  // namespace gtdynamics