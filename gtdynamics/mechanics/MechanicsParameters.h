/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MechanicsParameters.h
 * @brief Shared parameter container for mechanics-level factors.
 */

#pragma once

#include <gtdynamics/kinematics/KinematicsParameters.h>
#include <gtsam/base/Vector.h>

#include <optional>

namespace gtdynamics {

/// Shared parameters for mechanics-level factors common to statics and dynamics.
struct MechanicsParameters : public KinematicsParameters {
  using Isotropic = gtsam::noiseModel::Isotropic;

  std::optional<gtsam::Vector3> gravity, planar_axis;
  gtsam::SharedNoiseModel f_cost_model,      // wrench equivalence factor
      t_cost_model,                          // torque factor
      planar_cost_model;                     // planar factor

  /// Constructor from explicit noise models.
  MechanicsParameters(
      const KinematicsParameters& kinematics_parameters,
      const gtsam::SharedNoiseModel& f_cost_model,
      const gtsam::SharedNoiseModel& t_cost_model,
      const gtsam::SharedNoiseModel& planar_cost_model,
      const std::optional<gtsam::Vector3>& gravity = {},
      const std::optional<gtsam::Vector3>& planar_axis = {})
      : KinematicsParameters(kinematics_parameters),
        gravity(gravity),
        planar_axis(planar_axis),
        f_cost_model(f_cost_model),
        t_cost_model(t_cost_model),
        planar_cost_model(planar_cost_model) {}

  /// Constructor from scalar sigmas.
  MechanicsParameters(double sigma_dynamics = 1e-5,
                      const std::optional<gtsam::Vector3>& gravity = {},
                      const std::optional<gtsam::Vector3>& planar_axis = {})
      : KinematicsParameters(),
        gravity(gravity),
        planar_axis(planar_axis),
        f_cost_model(Isotropic::Sigma(6, sigma_dynamics)),
        t_cost_model(Isotropic::Sigma(1, sigma_dynamics)),
        planar_cost_model(Isotropic::Sigma(3, sigma_dynamics)) {}
};

}  // namespace gtdynamics
