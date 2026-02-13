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
    : DynamicsParameters(),
      rel_thresh(1e-2),
      max_iter(50) {}

// void OptimizerSetting::setQcModelPose3(const gtsam::Matrix &Qc) {
//   Qc_model_pose3 = gtsam::noiseModel::Gaussian::Covariance(Qc);
// }

}  // namespace gtdynamics
