/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  OptimizerSetting.h
 * @brief Factor graph optimizer settings.
 * @author Mandy Xie
 */

#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace gtdynamics {

/// OptimizerSetting is a class used to set parameters for motion planner
class OptimizerSetting {
 public:
  /// optimization iteration types
  enum IterationType { GaussNewton, LM, Dogleg };
  enum VerbosityLevel { None, Error };

  // factor cost models
  gtsam::noiseModel::Base::shared_ptr bp_cost_model,  // pose of fixed link
      bv_cost_model,                                  // velocity of fixed link
      ba_cost_model,             // acceleration of fixed link
      p_cost_model,              // pose factor
      v_cost_model,              // twist factor
      a_cost_model,              // acceleration factor
      linear_a_cost_model,       // linear acceleration factor
      f_cost_model,              // wrench equivalence factor
      linear_f_cost_model,       // linear wrench equivalence factor
      fa_cost_model,             // wrench factor
      t_cost_model,              // torque factor
      linear_t_cost_model,       // linear torque factor
      cp_cost_model,             // contact pose
      cfriction_cost_model,      // contact friction cone
      cv_cost_model,             // contact twist
      ca_cost_model,             // contact acceleration
      cm_cost_model,             // contact moment
      planar_cost_model,         // planar factor
      linear_planar_cost_model,  // linear planar factor
      prior_q_cost_model,        // joint angle prior factor
      prior_qv_cost_model,       // joint velocity prior factor
      prior_qa_cost_model,       // joint acceleration prior factor
      prior_t_cost_model,        // joint torque prior factor
      q_col_cost_model,          // joint collocation factor
      v_col_cost_model,          // joint vel collocation factor
      pose_col_cost_model,       // pose collocation factor
      twist_col_cost_model,      // twist collocation factor
      time_cost_model,           // time prior
      jl_cost_model;             // joint limit factor

  /// optimization settings
  IterationType opt_type = GaussNewton;  // optimizer type
  VerbosityLevel opt_verbosity = None;   // optimizer print out
  double rel_thresh = 1e-2;  // relative error decrease threshold for stopping
                             // optimization
  int max_iter = 50;         // max iteration for stopping optimization

  /// collision checking setting
  double epsilon = 0.0;     // obstacle clearance
  double obsSigma = 0.001;  // obstacle cost model covariance

  /// default constructor
  OptimizerSetting();

  // TODO: Make this accept an object like OptimizerParams.
  /**
   * Constructor which accepts various noise sigma values.
   *
   * @param sigma_dynamics
   * @param sigma_linear
   * @param sigma_contact
   * @param sigma_joint
   * @param sigma_collocation
   * @param sigma_time
   */
  OptimizerSetting(double sigma_dynamics, double sigma_linear = 0.001,
                   double sigma_contact = 0.001, double sigma_joint = 0.001,
                   double sigma_collocation = 0.001, double sigma_time = 0.001)
      : bp_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        bv_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        ba_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        p_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        v_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        linear_a_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(6, sigma_linear)),
        f_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        linear_f_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(6, sigma_linear)),
        fa_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
        linear_t_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_linear)),
        cp_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_contact)),
        cfriction_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_contact)),
        cv_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_contact)),
        ca_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_contact)),
        cm_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_contact)),
        planar_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics)),
        linear_planar_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(3, sigma_linear)),
        prior_q_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_joint)),
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
        jl_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_joint)),
        rel_thresh(1e-2),
        max_iter(50) {}

  // default destructor
  ~OptimizerSetting() {}

  // Gauss-Newton optimizer
  void setGaussNewton() { opt_type = GaussNewton; }

  // Levenberg-Marquardt optimizer
  void setLM() { opt_type = LM; }

  // Dog-Leg optimizer
  void setDogleg() { opt_type = Dogleg; }

  // set relative error threshold
  void setRelativeThreshold(double thresh) { rel_thresh = thresh; }

  // set maximum iteration number
  void setMaxIteration(size_t iter) { max_iter = iter; }
};

}  // namespace gtdynamics
