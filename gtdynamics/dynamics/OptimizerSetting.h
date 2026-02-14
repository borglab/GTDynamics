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

#include <gtdynamics/dynamics/DynamicsParameters.h>
#include <gtdynamics/kinematics/KinematicsParameters.h>

namespace gtdynamics {

/// OptimizerSetting is a class used to set parameters for motion planner
class OptimizerSetting : public KinematicsParameters, public DynamicsParameters {
 public:
  gtsam::noiseModel::Base::shared_ptr
      linear_a_cost_model,       // linear acceleration factor
      linear_f_cost_model,       // linear wrench equivalence factor
      linear_t_cost_model,       // linear torque factor
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

  /// optimization iteration types
  enum IterationType { GaussNewton, LM, Dogleg };
  enum VerbosityLevel { None, Error };

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
      : KinematicsParameters(sigma_dynamics, sigma_contact, sigma_joint,
                             sigma_dynamics, sigma_contact, sigma_dynamics,
                             sigma_dynamics, sigma_contact),
        DynamicsParameters(sigma_dynamics, sigma_linear, sigma_contact,
                           sigma_joint, sigma_collocation, sigma_time),
        linear_a_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(6, sigma_linear)),
        linear_f_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(6, sigma_linear)),
        linear_t_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_linear)),
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
        pose_col_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(6, sigma_collocation)),
        twist_col_cost_model(
            gtsam::noiseModel::Isotropic::Sigma(6, sigma_collocation)),
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
