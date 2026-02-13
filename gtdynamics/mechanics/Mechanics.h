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

/// Common mechanics factors shared between statics and dynamics.
class Mechanics {
 protected:
  const MechanicsParameters p_;

 public:
  /// Constructor.
  explicit Mechanics(const MechanicsParameters& parameters = MechanicsParameters())
      : p_(parameters) {}

  /// Graph with a WrenchEquivalenceFactor for each joint.
  gtsam::NonlinearFactorGraph wrenchEquivalenceFactors(
      const Slice& slice, const Robot& robot) const;

  /// Graph with a TorqueFactor for each joint.
  gtsam::NonlinearFactorGraph torqueFactors(const Slice& slice,
                                            const Robot& robot) const;

  /// Graph with a WrenchPlanarFactor for each joint.
  gtsam::NonlinearFactorGraph wrenchPlanarFactors(const Slice& slice,
                                                  const Robot& robot) const;
};

}  // namespace gtdynamics
