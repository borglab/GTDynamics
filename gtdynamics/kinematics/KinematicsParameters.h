/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsParameters.h
 * @brief Kinematics parameter container.
 */

#pragma once

#include <gtdynamics/optimizer/Optimizer.h>
#include <gtsam/linear/NoiseModel.h>

namespace gtdynamics {

/// Noise models etc specific to Kinematics class
struct KinematicsParameters : public OptimizationParameters {
  using Isotropic = gtsam::noiseModel::Isotropic;
  gtsam::SharedNoiseModel p_cost_model,  // pose factor
      g_cost_model,                      // goal point
      prior_q_cost_model,                // joint angle prior factor
      bp_cost_model,                     // fixed-link pose prior factor
      cp_cost_model;                     // contact-height factor

  KinematicsParameters() : KinematicsParameters(1e-4, 1e-2, 0.5, 1e-4, 1e-2) {}

  KinematicsParameters(
      const gtsam::SharedNoiseModel& p_cost_model,
      const gtsam::SharedNoiseModel& g_cost_model = Isotropic::Sigma(3, 1e-2),
      const gtsam::SharedNoiseModel& prior_q_cost_model = Isotropic::Sigma(1,
                                                                           0.5),
      const gtsam::SharedNoiseModel& bp_cost_model = Isotropic::Sigma(6, 1e-4),
      const gtsam::SharedNoiseModel& cp_cost_model = Isotropic::Sigma(1, 1e-2))
      : p_cost_model(p_cost_model),
        g_cost_model(g_cost_model),
        prior_q_cost_model(prior_q_cost_model),
        bp_cost_model(bp_cost_model),
        cp_cost_model(cp_cost_model) {}

  // TODO(yetong): replace noise model with tolerance.
  KinematicsParameters(double p_cost_model_sigma,
                       double g_cost_model_sigma = 1e-2,
                       double prior_q_cost_model_sigma = 0.5,
                       double bp_cost_model_sigma = 1e-4,
                       double cp_cost_model_sigma = 1e-2)
      : KinematicsParameters(Isotropic::Sigma(6, p_cost_model_sigma),
                             Isotropic::Sigma(3, g_cost_model_sigma),
                             Isotropic::Sigma(1, prior_q_cost_model_sigma),
                             Isotropic::Sigma(6, bp_cost_model_sigma),
                             Isotropic::Sigma(1, cp_cost_model_sigma)) {}
};

}  // namespace gtdynamics
