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

#include <gtdynamics/dynamics/OptimizerSetting.h>

namespace gtdynamics {

OptimizerSetting::OptimizerSetting()
    : KinematicsParameters(gtsam::noiseModel::Isotropic::Sigma(6, 0.001),
                           gtsam::noiseModel::Isotropic::Sigma(3, 0.001),
                           gtsam::noiseModel::Isotropic::Sigma(1, 0.001),
                           gtsam::noiseModel::Isotropic::Sigma(6, 0.00001),
                           gtsam::noiseModel::Isotropic::Sigma(1, 0.001),
                           gtsam::noiseModel::Isotropic::Sigma(6, 0.00001),
                           gtsam::noiseModel::Isotropic::Sigma(6, 0.001),
                           gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
      ba_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.00001)),
      a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
      linear_a_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
      f_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
      linear_f_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
      fa_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
      t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      linear_t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      cfriction_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      ca_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
      cm_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
      planar_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
      linear_planar_cost_model(gtsam::noiseModel::Isotropic::Sigma(3, 0.001)),
      prior_qv_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      prior_qa_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      prior_t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      q_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      v_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      pose_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
      twist_col_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.001)),
      time_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      jl_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, 0.001)),
      rel_thresh(1e-2),
      max_iter(50) {}

// void OptimizerSetting::setQcModelPose3(const gtsam::Matrix &Qc) {
//   Qc_model_pose3 = gtsam::noiseModel::Gaussian::Covariance(Qc);
// }

}  // namespace gtdynamics
