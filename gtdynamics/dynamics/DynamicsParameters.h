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

namespace gtdynamics {

/**
 * Noise models and settings used by dynamics solvers/builders.
 *
 * Includes mechanics-level parameters plus dynamic-only factor configuration.
 */
struct DynamicsParameters : public MechanicsParameters {
  gtsam::noiseModel::Base::shared_ptr ba_cost_model,  // acceleration of fixed link
      a_cost_model,              // acceleration factor
      linear_a_cost_model,       // linear acceleration factor
      linear_f_cost_model,       // linear wrench equivalence factor
      fa_cost_model,             // wrench factor
      linear_t_cost_model,       // linear torque factor
      cfriction_cost_model,      // contact friction cone
      ca_cost_model,             // contact acceleration
      cm_cost_model,             // contact moment
      linear_planar_cost_model,  // linear planar factor
      prior_qv_cost_model,       // joint velocity prior factor
      prior_qa_cost_model,       // joint acceleration prior factor
      prior_t_cost_model,        // joint torque prior factor
      q_col_cost_model,          // joint collocation factor
      v_col_cost_model,          // joint vel collocation factor
      pose_col_cost_model,       // pose collocation factor
      twist_col_cost_model,      // twist collocation factor
      time_cost_model,           // time prior
      jl_cost_model;             // joint limit factor

  /**
   * Constructor with defaults chosen to preserve prior
   * `OptimizerSetting` behavior.
   */
  DynamicsParameters()
      : MechanicsParameters(
            KinematicsParameters(gtsam::noiseModel::Isotropic::Sigma(6, 0.001),
                                 gtsam::noiseModel::Isotropic::Sigma(3, 0.001),
                                 gtsam::noiseModel::Isotropic::Sigma(1, 0.001),
                                 gtsam::noiseModel::Isotropic::Sigma(6, 0.00001),
                                 gtsam::noiseModel::Isotropic::Sigma(1, 0.001),
                                 gtsam::noiseModel::Isotropic::Sigma(6, 0.00001),
                                 gtsam::noiseModel::Isotropic::Sigma(6, 0.001),
                                 gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
            gtsam::noiseModel::Isotropic::Sigma(6, 0.001),
            gtsam::noiseModel::Isotropic::Sigma(1, 0.001),
            gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
        ba_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.00001)),
        a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
        linear_a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
        linear_f_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
        fa_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
        linear_t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
        cfriction_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
        ca_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
        cm_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
        linear_planar_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
        prior_qv_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
        prior_qa_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
        prior_t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
        q_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
        v_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
        pose_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
        twist_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
        time_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
        jl_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)) {}

  /**
   * Constructor from grouped sigma values.
   * @param sigma_dynamics Sigma for nonlinear dynamics/mechanics factors.
   * @param sigma_linear Sigma for linearized dynamics factors.
   * @param sigma_contact Sigma for contact-related factors.
   * @param sigma_joint Sigma for joint priors/limits.
   * @param sigma_collocation Sigma for collocation factors.
   * @param sigma_time Sigma for time-duration priors.
   */
  DynamicsParameters(double sigma_dynamics, double sigma_linear = 0.001,
                     double sigma_contact = 0.001, double sigma_joint = 0.001,
                     double sigma_collocation = 0.001,
                     double sigma_time = 0.001)
      : MechanicsParameters(
            KinematicsParameters(sigma_dynamics, sigma_contact, sigma_joint,
                                 sigma_dynamics, sigma_contact, sigma_dynamics,
                                 sigma_dynamics, sigma_contact),
            gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics),
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics),
            gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics)),
        ba_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        linear_a_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(6, sigma_linear)),
        linear_f_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(6, sigma_linear)),
        fa_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        linear_t_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_linear)),
        cfriction_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_contact)),
        ca_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_contact)),
        cm_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_contact)),
        linear_planar_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(3, sigma_linear)),
        prior_qv_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_joint)),
        prior_qa_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_joint)),
        prior_t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_joint)),
        q_col_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_collocation)),
        v_col_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_collocation)),
        time_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_time)),
        jl_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_joint)) {}
};

}  // namespace gtdynamics
