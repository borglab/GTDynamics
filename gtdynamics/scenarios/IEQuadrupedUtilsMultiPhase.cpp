/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.cpp
 * @brief Quadruped utilities implementations.
 * @author: Yetong Zhang
 */

#include "optimizer/InequalityConstraint.h"
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
IEVision60RobotMultiPhase::IEVision60RobotMultiPhase(
    const std::vector<IEVision60Robot> &phase_robots,
    const std::vector<IEVision60Robot> &boundary_robots,
    const std::vector<size_t> &phase_num_steps)
    : phase_robots_(phase_robots), boundary_robots_(boundary_robots),
      phase_num_steps_(phase_num_steps) {
  size_t k = 0;
  for (size_t i = 0; i < phase_num_steps.size() - 1; i++) {
    k += phase_num_steps[i];
    boundary_ks_.push_back(k);
  }
}

/* ************************************************************************* */
const IEVision60Robot &
IEVision60RobotMultiPhase::robotAtStep(const size_t k) const {
  for (size_t i = 0; i < boundary_ks_.size(); i++) {
    size_t boundary_k = boundary_ks_.at(i);
    if (k == boundary_k) {
      return boundary_robots_.at(i);
    } else if (k < boundary_k) {
      return phase_robots_.at(i);
    }
  }
  return phase_robots_.back();
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::collocationCosts() const {
  NonlinearFactorGraph graph;
  size_t start_k = 0;
  for (size_t phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
    size_t end_k = start_k + phase_num_steps_[phase_idx];
    graph.add(phase_robots_.at(phase_idx).multiPhaseCollocationCosts(
        start_k, end_k, phase_idx));
    start_k = end_k;
  }
  return graph;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60RobotMultiPhase::phaseMinDurationConstraints(
    const std::vector<double> &phases_min_dt) const {
  InequalityConstraints constraints;
  for (size_t phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
    Key phase_key = PhaseKey(phase_idx);
    double phase_min_dt = phases_min_dt.at(phase_idx);
    Double_ phase_expr(phase_key);
    Double_ phase_min_expr = phase_expr - Double_(phase_min_dt);
    constraints.emplace_shared<DoubleExpressionInequality>(
        phase_min_expr, phase_robots_.at(0).params.tol_phase_dt);
  }
  return constraints;
}

} // namespace gtsam
