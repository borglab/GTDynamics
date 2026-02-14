/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Mechanics.h
 * @brief Mechanics factors common to statics and dynamics.
 */

#pragma once

#include <gtdynamics/mechanics/MechanicsParameters.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtdynamics {

/**
 * Mechanics-level factor builder shared by statics and dynamics.
 *
 * This class only contains factor groups that are identical in static and
 * dynamic graph assembly. For the templated API below, `CONTEXT` is typically
 * `Slice`, `Interval`, or `Phase`, and `robot` is always the same model.
 */
class Mechanics {
 protected:
  const MechanicsParameters p_;

 public:
  /**
   * Constructor.
   * @param parameters Mechanics parameter bundle with noise models and optional
   * gravity/planar settings.
   */
  explicit Mechanics(const MechanicsParameters& parameters = MechanicsParameters())
      : p_(parameters) {}

  /**
   * Build wrench-equivalence factors for all joints.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph wrenchEquivalenceFactors(
      const CONTEXT& context, const Robot& robot) const;

  /**
   * Build torque factors for all joints.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph torqueFactors(const CONTEXT& context,
                                            const Robot& robot) const;

  /**
   * Build planar wrench factors for all joints.
   * No factors are added when no planar axis is configured.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph wrenchPlanarFactors(const CONTEXT& context,
                                                  const Robot& robot) const;
};

}  // namespace gtdynamics
