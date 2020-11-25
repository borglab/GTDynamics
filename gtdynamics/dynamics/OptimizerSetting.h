/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  OptimizerSetting.h
 * @brief Factor graph optimizer settings.
 * @Author: Mandy Xie
 */

#ifndef GTDYNAMICS_DYNAMICS_OPTIMIZERSETTING_H_
#define GTDYNAMICS_DYNAMICS_OPTIMIZERSETTING_H_

#include <gtsam/linear/NoiseModel.h>

namespace gtdynamics {

/**
 * OptimizerSetting is a class used to set parameters for motion planner
 */
class OptimizerSetting {
 public:
  /// optimization iteration types
  enum IterationType { GaussNewton, LM, Dogleg };
  enum VerbosityLevel { None, Error };

  // factor cost models
  gtsam::noiseModel::Base::shared_ptr bp_cost_model,  // pose of fixed link
      bv_cost_model,                                  // velocity of fixed link
      ba_cost_model,  // cceleration of fixed link
      p_cost_model,   // pose factor
      v_cost_model,   // twist factor
      a_cost_model,   // acceleration factor
      linear_a_cost_model,
      f_cost_model,  // wrench equivalence factor
      linear_f_cost_model,
      fa_cost_model,  // wrench factor
      t_cost_model,   // torque factor
      linear_t_cost_model,
      cp_cost_model,         // contact pose
      cfriction_cost_model,  // contact friction cone
      cv_cost_model,         // contact twist
      ca_cost_model,         // contact acceleration
      cm_cost_model,         // contact moment
      planar_cost_model,     // planar factor
      linear_planar_cost_model,
      prior_q_cost_model,   // joint angle prior factor
      prior_qv_cost_model,  // joint velocity prior factor
      prior_qa_cost_model,  // joint acceleration prior factor
      prior_t_cost_model,   // joint torque prior factor
      q_col_cost_model,     // joint collocation factor
      v_col_cost_model,     // joint vel collocation factor
      time_cost_model,      // time prior
      jl_cost_model;        // joint limit factor

  /// optimization settings
  IterationType opt_type;        // optimizer type
  VerbosityLevel opt_verbosity;  // optimizer print out
  double rel_thresh;  // relative error decrease threshold for stopping
                      // optimization
  int max_iter;       // max iteration for stopping optimization

  /// collision checking setting
  double epsilon;   // obstacle clearance
  double obsSigma;  // obstacle cost model covariance

  // default constructor
  OptimizerSetting();

  //Constructor
  OptimizerSetting(const double& sigma_dynamics)
    : bp_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
      bv_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
      ba_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
      p_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
      v_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
      a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
      linear_a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
      f_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
      linear_f_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
      fa_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
      t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      linear_t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      cp_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      cfriction_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      cv_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics)),
      ca_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics)),
      cm_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
      planar_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics)),
      linear_planar_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
      prior_q_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      prior_qv_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      prior_qa_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      prior_t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      q_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      v_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      time_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)),
      jl_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      rel_thresh(1e-2),
      max_iter(50) {}

  // default destructor
  ~OptimizerSetting() {}

  // Gauss-Newton optimizer
  void setGaussNewton() { opt_type = GaussNewton; }

  // Levenberg-Marquart optimizer
  void setLM() { opt_type = LM; }

  // Dog-Leg optimizer
  void setDogleg() { opt_type = Dogleg; }

  // set relative error threshold
  void setRelativeThreshold(double thresh) { rel_thresh = thresh; }

  // set maximum iteration number
  void setMaxIteration(size_t iter) { max_iter = iter; }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_DYNAMICS_OPTIMIZERSETTING_H_
