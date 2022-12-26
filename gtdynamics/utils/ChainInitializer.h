/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ChainInitializer.h
 * @brief Utility methods for initializing trajectory optimization solutions for chain graph.
 * @author: Dan Barladeanu
 */

#pragma once

#include <gtdynamics/utils/Initializer.h>

namespace gtdynamics {

class ChainInitializer : public Initializer {

  public:
      /**
     * @fn Return zero values for all variables for initial value of optimization.
     *
     * @param[in] robot          A Robot object.
     * @param[in] t              Timestep to return zero initial values for.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points Contact points for timestep t.
     */
    gtsam::Values ZeroValues(
        const Robot& robot, const int t, double gaussian_noise = 0.0,
        const boost::optional<PointOnLinks>& contact_points = boost::none) const override;
};

} // namespace gtdynamics
