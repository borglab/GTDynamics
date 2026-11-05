/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ChainInitializer.h
 * @brief Utility methods for initializing trajectory optimization solutions for
 * chain graph.
 * @author: Dan Barladeanu
 */

#pragma once

#include <gtdynamics/dynamics/ZeroInitializer.h>

namespace gtdynamics {

class ChainInitializer : public ZeroSliceInitializer {
 public:
  /// Copy the Gaussian perturbation scale used by the chain seed.
  explicit ChainInitializer(const Robot& robot, double gaussianNoise = 0.0);

  /**
   * @fn Return zero values for all variables for initial value of optimization.
   *
   * @param[in] t              Timestep to return zero initial values for.
   * @param[in] contactPoints Contact points for timestep t.
   */
  gtsam::Values run(
      const int t,
      const std::optional<PointOnLinks>& contactPoints = {}) const override;
};

}  // namespace gtdynamics
