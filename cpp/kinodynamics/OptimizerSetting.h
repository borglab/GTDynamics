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
  enum VerbosityLevel { None, Error };

  /// time settings
  int total_step;     // number of steps optimized the whole trajectory
  double total_time;  // time duration (second) of the whole trajectory

  // factor cost models
  gtsam::noiseModel::Base::shared_ptr bp_cost_model,  // base pose factor
      bv_cost_model,                                  // base velocity
      ba_cost_model,                                  // base acceleration
      p_cost_model,                                   // pose factor
      v_cost_model,                                   // twist factor
      a_cost_model,                                   // acceleration factor
      f_cost_model,                                   // wrench factor
      t_cost_model,                                   // torque factor
      q_cost_model,   // joint angle prior factor
      qv_cost_model,  // joint velocity prior factor
      qa_cost_model,  // joint acceleration prior factor
      tf_cost_model,  // tool wrench factor
      tp_cost_model,  // tool pose factor
      jl_cost_model;  // joint limit factor
  // Gaussian process settings
  gtsam::noiseModel::Gaussian::shared_ptr Qc_model,  // Qc for GP linear
      Qc_model_pose3;                                // Qc for GP pose3

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

  // default destructor
  ~OptimizerSetting() {}

  /// settings for time steps
  void setTotalStep(size_t step) { total_step = step; }
  void setTotalTime(double time) { total_time = time; }

  /// settings for factor cost model
  void setToolPoseCostModel(const double sigma);
  void setJointLimitCostModel(const double sigma);
  void setQcModel(const gtsam::Matrix &Qc);
  void setQcModelPose3(const gtsam::Matrix &Qc);
  void setObstacleCostModel(const double sigma) { obsSigma = sigma; };

  /// optimization settings
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

  /// collision check setting
  // set safty distance threshold
  void setCollisionEpsilon(double e) { epsilon = e; }
};
}  // namespace manipulator
