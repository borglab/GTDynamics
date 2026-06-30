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

#include <map>
#include <string>
#include <utility>

namespace gtdynamics {

/// Noise models etc specific to Kinematics class
struct KinematicsParameters : public OptimizationParameters {
  using Isotropic = gtsam::noiseModel::Isotropic;
  gtsam::SharedNoiseModel p_cost_model,  // pose factor
      g_cost_model,                      // goal point
      prior_q_cost_model,                // joint angle prior factor (default)
      bp_cost_model,                     // fixed-link pose prior factor
      cp_cost_model,                     // contact-height factor
      bv_cost_model,                     // fixed-link twist prior factor
      v_cost_model,                      // twist factor
      cv_cost_model;                     // contact-twist factor

  /// Per-joint joint-angle prior models keyed by joint name; overrides
  /// prior_q_cost_model for listed joints (used by kinematics IK only).
  std::map<std::string, gtsam::SharedNoiseModel> joint_prior_overrides;

  /// Per-joint {lower, upper} limit overrides by joint name (IK only).
  std::map<std::string, std::pair<double, double>> joint_limit_overrides;

  KinematicsParameters()
      : KinematicsParameters(1e-4, 1e-2, 0.5, 1e-4, 1e-2, 1e-4, 1e-4, 1e-2) {}

  KinematicsParameters(
      const gtsam::SharedNoiseModel& p_cost_model,
      const gtsam::SharedNoiseModel& g_cost_model = Isotropic::Sigma(3, 1e-2),
      const gtsam::SharedNoiseModel& prior_q_cost_model = Isotropic::Sigma(1,
                                                                           0.5),
      const gtsam::SharedNoiseModel& bp_cost_model = Isotropic::Sigma(6, 1e-4),
      const gtsam::SharedNoiseModel& cp_cost_model = Isotropic::Sigma(1, 1e-2),
      const gtsam::SharedNoiseModel& bv_cost_model = Isotropic::Sigma(6, 1e-4),
      const gtsam::SharedNoiseModel& v_cost_model = Isotropic::Sigma(6, 1e-4),
      const gtsam::SharedNoiseModel& cv_cost_model = Isotropic::Sigma(3, 1e-2))
      : p_cost_model(p_cost_model),
        g_cost_model(g_cost_model),
        prior_q_cost_model(prior_q_cost_model),
        bp_cost_model(bp_cost_model),
        cp_cost_model(cp_cost_model),
        bv_cost_model(bv_cost_model),
        v_cost_model(v_cost_model),
        cv_cost_model(cv_cost_model) {}

  // TODO(yetong): replace noise model with tolerance.
  KinematicsParameters(double p_cost_model_sigma,
                       double g_cost_model_sigma = 1e-2,
                       double prior_q_cost_model_sigma = 0.5,
                       double bp_cost_model_sigma = 1e-4,
                       double cp_cost_model_sigma = 1e-2,
                       double bv_cost_model_sigma = 1e-4,
                       double v_cost_model_sigma = 1e-4,
                       double cv_cost_model_sigma = 1e-2)
      : KinematicsParameters(Isotropic::Sigma(6, p_cost_model_sigma),
                             Isotropic::Sigma(3, g_cost_model_sigma),
                             Isotropic::Sigma(1, prior_q_cost_model_sigma),
                             Isotropic::Sigma(6, bp_cost_model_sigma),
                             Isotropic::Sigma(1, cp_cost_model_sigma),
                             Isotropic::Sigma(6, bv_cost_model_sigma),
                             Isotropic::Sigma(6, v_cost_model_sigma),
                             Isotropic::Sigma(3, cv_cost_model_sigma)) {}

  /// Override the joint-angle prior sigma for one joint by name.
  void setJointPriorSigma(const std::string& joint_name, double sigma) {
    joint_prior_overrides[joint_name] = Isotropic::Sigma(1, sigma);
  }

  /// Override the joint-angle limits for one joint by name.
  void setJointLimit(const std::string& joint_name, double lower, double upper) {
    joint_limit_overrides[joint_name] = {lower, upper};
  }
};

}  // namespace gtdynamics
