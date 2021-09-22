/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WalkCycle.h
 * @brief Class to store walk cycle.
 * @author: Disha Das, Varun Agrawal
 */

#pragma once

#include <gtdynamics/utils/Phase.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <map>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * @class WalkCycle class stores the sequence of phases
 * in a walk cycle.
 */
class WalkCycle {
 protected:
  std::vector<Phase> phases_;    ///< Phases in walk cycle

 public:
  /// Default Constructor
  WalkCycle() {}

  /**
   * @fn Return phase for given phase number p.
   * @param[in]p    Phase number \in [0..numPhases()[.
   * @return Phase instance.
   */
  const Phase& phase(size_t p) const;

  /// Returns vector of phases in the walk cycle
  const std::vector<Phase>& phases() const { return phases_; }

  /// Returns count of phases in the walk cycle
  size_t numPhases() const { return phases_.size(); }

  /// Returns the number of time steps, summing over all phases.
  size_t numTimeSteps() const;

  /// Print to stream.
  friend std::ostream& operator<<(std::ostream& os,
                                  const WalkCycle& walk_cycle);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string& s = "") const;
  
}  // namespace gtdynamics
