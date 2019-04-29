/**
 * @file  OptimizerSetting.cpp
 * @brief factor graph optimizer setting
 * @Author: Mandy Xie
 */

#include <OptimizerSetting.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

OptimizerSetting::OptimizerSetting()
    : total_step(100),
      total_time(10),
      bp_cost_model(noiseModel::Isotropic::Sigma(6, 0.00001)),
      bv_cost_model(noiseModel::Isotropic::Sigma(6, 0.00001)),
      ba_cost_model(noiseModel::Isotropic::Sigma(6, 0.00001)),
      p_cost_model(noiseModel::Isotropic::Sigma(6, 0.001)),
      v_cost_model(noiseModel::Isotropic::Sigma(6, 0.1)),
      a_cost_model(noiseModel::Isotropic::Sigma(6, 0.1)),
      f_cost_model(noiseModel::Isotropic::Sigma(6, 0.1)),
      t_cost_model(noiseModel::Isotropic::Sigma(1, 0.1)),
      q_cost_model(noiseModel::Isotropic::Sigma(1, 0.001)),
      qv_cost_model(noiseModel::Isotropic::Sigma(1, 0.001)),
      qa_cost_model(noiseModel::Isotropic::Sigma(1, 0.001)),
      tf_cost_model(noiseModel::Isotropic::Sigma(6, 0.00001)),
      opt_type(LM),
      opt_verbosity(None),
      rel_thresh(1e-2),
      max_iter(50),
      radius(0.05),
      epsilon(0.2) {}

void OptimizerSetting::setToolPoseCostModel(const double sigma) {
  tp_cost_model = noiseModel::Isotropic::Sigma(6, sigma);
}

void OptimizerSetting::setJointLimitCostModel(const double sigma) {
  jl_cost_model = noiseModel::Isotropic::Sigma(1, sigma);
}

void OptimizerSetting::setQcModel(const Matrix &Qc) {
  Qc_model = noiseModel::Gaussian::Covariance(Qc);
}

void OptimizerSetting::setQcModelPose3(const Matrix &Qc) {
  Qc_model_pose3 = noiseModel::Gaussian::Covariance(Qc);
}
