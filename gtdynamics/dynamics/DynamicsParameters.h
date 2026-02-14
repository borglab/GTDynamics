/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  DynamicsParameters.h
 * @brief Dynamics parameter container.
 */

#pragma once

#include <gtdynamics/mechanics/MechanicsParameters.h>
#include <gtsam/linear/NoiseModel.h>

namespace gtdynamics {

/**
 * Noise models and settings used by dynamics solvers/builders.
 *
 * Includes mechanics-level parameters plus dynamic-only factor configuration.
 */
struct DynamicsParameters : public MechanicsParameters {
  gtsam::noiseModel::Base::shared_ptr ba_cost_model,  // acceleration of fixed link
      a_cost_model,              // acceleration factor
      fa_cost_model,             // wrench factor
      cfriction_cost_model,      // contact friction cone
      ca_cost_model,             // contact acceleration
      cm_cost_model;             // contact moment

  /**
   * Constructor with defaults chosen to preserve prior
   * `OptimizerSetting` behavior.
   */
  DynamicsParameters()
      : MechanicsParameters(gtsam::noiseModel::Isotropic::Sigma(6, 0.001),
            gtsam::noiseModel::Isotropic::Sigma(1, 0.001),
            gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
        ba_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.00001)),
        a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
        fa_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
        cfriction_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
        ca_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
        cm_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)) {}

  /**
   * Constructor from grouped sigma values.
   *
   * This signature is kept for compatibility with grouped-sigma construction
   * in `OptimizerSetting`. Only `sigma_dynamics` and `sigma_contact` apply to
   * `DynamicsParameters`; other groups are configured in `OptimizerSetting`.
   *
   * @param sigma_dynamics Sigma for nonlinear dynamics/mechanics factors.
   * @param sigma_linear Unused in this class (handled by OptimizerSetting).
   * @param sigma_contact Sigma for contact-related factors.
   * @param sigma_joint Unused in this class (handled by OptimizerSetting).
   * @param sigma_collocation Unused in this class (handled by OptimizerSetting).
   * @param sigma_time Unused in this class (handled by OptimizerSetting).
   */
  DynamicsParameters(double sigma_dynamics, double /*sigma_linear*/ = 0.001,
                     double sigma_contact = 0.001,
                     double /*sigma_joint*/ = 0.001,
                     double /*sigma_collocation*/ = 0.001,
                     double /*sigma_time*/ = 0.001)
      : MechanicsParameters(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics),
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics),
            gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics)),
        ba_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        fa_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        cfriction_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_contact)),
        ca_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_contact)),
        cm_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_contact)) {}
};

}  // namespace gtdynamics
