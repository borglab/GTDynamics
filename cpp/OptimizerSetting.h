/**
 * @file  OptimizerSetting.h
 * @brief factor graph optimizer setting
 * @Author: Mandy Xie
 */

#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace manipulator {

/**
 * OptimizerSetting is a class used to set parameters for motion planner
 */
class OptimizerSetting {
 public:
  /// optimization iteration types
  enum IterationType { GaussNewton, LM, Dogleg };
  enum VerbosityLevel {None, Error};

  /// time settings
  int total_step;     // number of steps optimized the whole trajectory
  double total_time;  // time duration (second) of the whole trajectory

  // factor cost models
  gtsam::noiseModel::Base::shared_ptr bp_cost_model, bv_cost_model,
      ba_cost_model, p_cost_model, v_cost_model, a_cost_model, f_cost_model,
      t_cost_model, q_cost_model, qv_cost_model, qa_cost_model, tf_cost_model,
      tp_cost_model, jl_cost_model;
  // Gaussian process settings
  gtsam::noiseModel::Gaussian::shared_ptr Qc_model, Qc_model_pose3;  // Qc for GP

  // optimization settings
  IterationType opt_type;  // optimizer type
  VerbosityLevel opt_verbosity;  // optimizer print out 
  double rel_thresh;       // relative error decrease threshold for stopping
                           // optimization
  int max_iter;            // max iteration for stopping optimization

  // collision checking setting
  double radius;    // sphere link radius
  double epsilon;   // obstacle clearance
  double obs_sigma;     // obstacle cost model covariance

  // default constructor
  OptimizerSetting();

  // default deconstructor
  ~OptimizerSetting() {}

  // settings for time steps
  void set_total_step(size_t step) { total_step = step; }
  void set_total_time(double time) { total_time = time; }

  // settings for factor cost model
  void setToolPoseCostModel(const double sigma);
  void setJointLimitCostModel(const double sigma);
  void setQcModel(const gtsam::Matrix &Qc);
  void setQcModelPose3(const gtsam::Matrix &Qc);
  void setObstacleCostModel(const double sigma) {obs_sigma = sigma; };

  // optimization settings
  void setGaussNewton() { opt_type = GaussNewton; }
  void setLM() { opt_type = LM; }
  void setDogleg() { opt_type = Dogleg; }
  void set_rel_thresh(double thresh) { rel_thresh = thresh; }
  void set_max_iter(size_t iter) { max_iter = iter; }
  void setSphereRadius(double r) {radius = r;}
  void setCollisionEpsilon(double e) {epsilon = e;}
};
}  // namespace manipulator
