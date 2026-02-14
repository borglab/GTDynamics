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

#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

#include <optional>

namespace gtdynamics {

/**
 * Shared parameters for mechanics-level factors common to statics and dynamics.
 *
 * These parameters configure only the shared wrench-equivalence, torque, and
 * planar wrench factor groups.
 */
struct MechanicsParameters {
  using Isotropic = gtsam::noiseModel::Isotropic;

  std::optional<gtsam::Vector3> gravity, planar_axis;
  gtsam::SharedNoiseModel f_cost_model,      // wrench equivalence factor
      t_cost_model,                          // torque factor
      planar_cost_model;                     // planar factor

  /**
   * Constructor from explicit noise models.
   * @param f_cost_model Noise model for wrench-equivalence factors.
   * @param t_cost_model Noise model for torque factors.
   * @param planar_cost_model Noise model for planar wrench factors.
   * @param gravity Optional gravity vector used by mechanics-related factors.
   * @param planar_axis Optional axis used to enforce planar wrench terms.
   */
  MechanicsParameters(const gtsam::SharedNoiseModel& f_cost_model,
                      const gtsam::SharedNoiseModel& t_cost_model,
                      const gtsam::SharedNoiseModel& planar_cost_model,
                      const std::optional<gtsam::Vector3>& gravity = {},
                      const std::optional<gtsam::Vector3>& planar_axis = {})
      : gravity(gravity),
        planar_axis(planar_axis),
        f_cost_model(f_cost_model),
        t_cost_model(t_cost_model),
        planar_cost_model(planar_cost_model) {}

  /**
   * Constructor from scalar sigmas.
   * @param sigma_dynamics Shared sigma for mechanics factors.
   * @param gravity Optional gravity vector used by mechanics-related factors.
   * @param planar_axis Optional axis used to enforce planar wrench terms.
   */
  MechanicsParameters(double sigma_dynamics = 1e-5,
                      const std::optional<gtsam::Vector3>& gravity = {},
                      const std::optional<gtsam::Vector3>& planar_axis = {})
      : gravity(gravity),
        planar_axis(planar_axis),
        f_cost_model(Isotropic::Sigma(6, sigma_dynamics)),
        t_cost_model(Isotropic::Sigma(1, sigma_dynamics)),
        planar_cost_model(Isotropic::Sigma(3, sigma_dynamics)) {}
};

}  // namespace gtdynamics
