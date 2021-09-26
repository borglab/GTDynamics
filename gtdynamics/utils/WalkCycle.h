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
#include <gtdynamics/utils/FootContactState.h>

#include <map>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * @class WalkCycle class stores the sequence of phases
 * in a walk cycle. A WalkCycle is built from FootContactStates
 * and phase lengths and can spawn phases.
 */
class WalkCycle {
 protected:
  std::vector<Phase> phases_;    ///< Phases in walk cycle

 public:
  /// Default Constructor
  WalkCycle() {}

  /**
   * @fn Construct a new Walk Cycle object from a vector of 
   * FootContactState pointers and phase lengths. 
   * for each element in the vectors, a Phase will be generated with a 
   * corresponding FootContactState and phase length.
   * 
   * @param states ........... a vector of FootContactState shared pointers.
   * @param phase_lengths .... a vector of phase lengths corresponding to the states vector.
   */
  WalkCycle(const std::vector<boost::shared_ptr<FootContactState>> &states, const std::vector<size_t> &phase_lengths) {
    if (states.size() != phase_lengths.size()){
      throw std::runtime_error("states vector and phase_lengths vector have different sizes");
    }
    size_t k = 0;
    for (size_t i = 0; i < states.size(); i++)  {
      Phase ph = Phase(k, k + phase_lengths[i], states[i]);
      phases_.push_back(ph);
      k += phase_lengths[i];
    }
  }

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
  
};
}  // namespace gtdynamics
