/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file build_scate_graph.h
 * @author Harrison Jenkins
 *
 * Assembles the manipulator SCATE factor graph (paper Algorithm 1, "Generate
 * Initial Plan") from a GraphSpec, and provides the solve + evaluation helpers.
 * No robot model is needed (only the GTDynamics factors), so it can be built
 * and tested with a synthetic robot. scate_init.h fills the GraphSpec from the
 * real robot.
 *
 * Factors (n = number of joints, N+1 states x_i = [q_i; v_i], N controls u_i):
 *   start prior   q_0 = q_start, v_0 = 0                          (paper eq. 9)
 *   goal prior    q_N = q_goal,  v_N = 0                          (paper eq. 10)
 *   dynamics      x_{i+1} = Fx x_i + Fu u_i,  i = 0..N-1          (paper eq. 15)
 *   control limit hinge on u_i,                i = 0..N-1          (paper eq. 17)
 *   joint limit   hinge on q_i,                i = 0..N            (kinematic validity)
 *
 * The dynamics factor is gtdynamics::DoubleIntegratorFactor and both hinges
 * are gtdynamics::JointLimitFactorVector (a vector hinge with a standoff
 * `threshold` inside each limit, the paper's eqs. 16-17).
 *
 * Obstacle and self-collision factors and the online loop are in scate_online.h.
 */

#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>

#include <gtdynamics/factors/DoubleIntegratorFactor.h>
#include <gtdynamics/factors/JointLimitFactorVector.h>

#include "scate_keys.h"
#include "trajectory_init.h"

namespace scate {

/// Everything needed to build the offline SCATE graph.
struct GraphSpec {
  Vector q_start;  ///< joint-space start (arm at rest)
  Vector q_goal;   ///< joint-space goal  (arm at rest)
  Vector q_lower;  ///< joint position lower limits
  Vector q_upper;  ///< joint position upper limits
  Vector u_limit;  ///< symmetric acceleration limit per joint: |u_j| <= u_limit_j
  int num_poses = 21;  ///< N+1
  double dt = 1.0;     ///< time between states

  // Noise sigmas: smaller = higher priority.
  double sigma_endpoint = 1e-3;  ///< start / goal priors
  double sigma_dynamics = 1e-2;  ///< dynamics
  double sigma_limit = 1e-1;     ///< limit hinges

  // Hinge margins: cost starts this far INSIDE each limit (paper's threshold).
  double u_margin = 0.1;   ///< rad/s^2
  double q_margin = 0.05;  ///< rad

  // Pass/fail tolerances for evaluateSolution (all constraints are soft).
  double tolerance_endpoint = 1e-3;  ///< start and goal state error
  double tolerance_dynamics = 1e-3;  ///< dynamics residual
  double tolerance_limit = 1e-3;     ///< control and joint limit excess
};

/// Check that a GraphSpec is consistent; throws std::invalid_argument if not.
inline void validateSpec(const GraphSpec& spec) {
  const long n = spec.q_start.size();
  if (n == 0) throw std::invalid_argument("GraphSpec: q_start is empty");
  if (spec.q_goal.size() != n || spec.q_lower.size() != n || spec.q_upper.size() != n ||
      spec.u_limit.size() != n) {
    throw std::invalid_argument("GraphSpec: q_start, q_goal, q_lower, q_upper, u_limit must all have the same size");
  }
  if (spec.num_poses < 3) throw std::invalid_argument("GraphSpec: need at least 3 poses");
  if (!(spec.dt > 0.0)) throw std::invalid_argument("GraphSpec: dt must be positive");
}

/**
 * Number of factors buildScateGraph adds for a given horizon:
 * 4 priors + (N dynamics + N control limits) + (N+1 joint limits).
 */
inline size_t expectedFactorCount(int num_poses) {
  const size_t num_intervals = static_cast<size_t>(num_poses - 1);
  return 4 + 2 * num_intervals + (num_intervals + 1);
}

/// Isotropic noise model of dimension `dimension`.
inline gtsam::SharedNoiseModel isotropicNoise(int dimension, double sigma) {
  return gtsam::noiseModel::Isotropic::Sigma(dimension, sigma);
}

/// Dynamics factor on the step from state i to i+1 under control u_i.
inline gtsam::NonlinearFactor::shared_ptr dynamicsFactor(const GraphSpec& spec, int i) {
  const int n = static_cast<int>(spec.q_start.size());
  return std::make_shared<gtdynamics::DoubleIntegratorFactor>(
      QKey(i), VKey(i), QKey(i + 1), VKey(i + 1), UKey(i), spec.dt,
      isotropicNoise(2 * n, spec.sigma_dynamics));
}

/// Hinge keeping the control u_i within +-u_limit, u_margin inside it.
inline gtsam::NonlinearFactor::shared_ptr controlLimitFactor(const GraphSpec& spec, int i) {
  const int n = static_cast<int>(spec.q_start.size());
  return std::make_shared<gtdynamics::JointLimitFactorVector>(
      UKey(i), isotropicNoise(n, spec.sigma_limit), Vector(-spec.u_limit), spec.u_limit,
      Vector::Constant(n, spec.u_margin));
}

/// Hinge keeping the joint angles q_i within their limits, q_margin inside them.
inline gtsam::NonlinearFactor::shared_ptr jointLimitFactor(const GraphSpec& spec, int i) {
  const int n = static_cast<int>(spec.q_start.size());
  return std::make_shared<gtdynamics::JointLimitFactorVector>(
      QKey(i), isotropicNoise(n, spec.sigma_limit), spec.q_lower, spec.q_upper,
      Vector::Constant(n, spec.q_margin));
}

/// Build the offline factor graph: endpoint priors, dynamics and limit hinges.
inline gtsam::NonlinearFactorGraph buildScateGraph(const GraphSpec& spec) {
  validateSpec(spec);
  const int n = static_cast<int>(spec.q_start.size());
  const int last = spec.num_poses - 1;
  const Vector zero = Vector::Zero(n);

  gtsam::NonlinearFactorGraph graph;

  // Endpoints: full-state priors, at rest.
  graph.add(gtsam::PriorFactor<Vector>(QKey(0), spec.q_start, isotropicNoise(n, spec.sigma_endpoint)));
  graph.add(gtsam::PriorFactor<Vector>(VKey(0), zero, isotropicNoise(n, spec.sigma_endpoint)));
  graph.add(gtsam::PriorFactor<Vector>(QKey(last), spec.q_goal, isotropicNoise(n, spec.sigma_endpoint)));
  graph.add(gtsam::PriorFactor<Vector>(VKey(last), zero, isotropicNoise(n, spec.sigma_endpoint)));

  for (int i = 0; i < last; ++i) {
    graph.add(dynamicsFactor(spec, i));
    graph.add(controlLimitFactor(spec, i));
  }
  for (int i = 0; i <= last; ++i) graph.add(jointLimitFactor(spec, i));
  return graph;
}

/// Solve with Levenberg-Marquardt, as in the paper.
inline gtsam::Values optimizeGraph(const gtsam::NonlinearFactorGraph& graph,
                                   const gtsam::Values& initial,
                                   bool verbose = false) {
  gtsam::LevenbergMarquardtParams params;
  if (verbose) params.setVerbosityLM("SUMMARY");
  return gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
}

/// Pull q, v, u out of a Values into the plain-vector form used elsewhere.
inline JointTrajectory extractTrajectory(const gtsam::Values& values, int num_poses) {
  JointTrajectory trajectory;
  for (int i = 0; i < num_poses; ++i) {
    trajectory.q.push_back(values.at<Vector>(QKey(i)));
    trajectory.v.push_back(values.at<Vector>(VKey(i)));
  }
  for (int i = 0; i + 1 < num_poses; ++i) {
    trajectory.u.push_back(values.at<Vector>(UKey(i)));
  }
  return trajectory;
}

/// How well a solution meets the requirements of a GraphSpec.
struct SolutionMetrics {
  double start_error = 0.0;        ///< max |q_0 - q_start|, |v_0|
  double goal_error = 0.0;         ///< max |q_N - q_goal|, |v_N|
  double dynamics_residual = 0.0;  ///< max dynamics residual over all steps
  double control_violation = 0.0;  ///< how far any |u| exceeds its limit
  double joint_violation = 0.0;    ///< how far any q is outside [lower, upper]
  double max_control = 0.0;        ///< peak |u|
  bool passed = false;             ///< every metric is within its tolerance
};

/// Measure a trajectory against the requirements of a GraphSpec.
inline SolutionMetrics computeSolutionMetrics(const JointTrajectory& trajectory,
                                              const GraphSpec& spec) {
  const int n = static_cast<int>(spec.q_start.size());
  const int last = spec.num_poses - 1;

  SolutionMetrics metrics;
  metrics.start_error = std::max((trajectory.q.front() - spec.q_start).cwiseAbs().maxCoeff(),
                                 trajectory.v.front().cwiseAbs().maxCoeff());
  metrics.goal_error = std::max((trajectory.q.back() - spec.q_goal).cwiseAbs().maxCoeff(),
                                trajectory.v.back().cwiseAbs().maxCoeff());
  metrics.dynamics_residual = maxDynamicsResidual(trajectory, makeDoubleIntegrator(n, spec.dt));

  for (int i = 0; i < last; ++i) {
    metrics.max_control = std::max(metrics.max_control, trajectory.u[i].cwiseAbs().maxCoeff());
    metrics.control_violation = std::max(
        metrics.control_violation, (trajectory.u[i].cwiseAbs() - spec.u_limit).maxCoeff());
  }
  metrics.control_violation = std::max(0.0, metrics.control_violation);
  for (int i = 0; i <= last; ++i) {
    metrics.joint_violation = std::max(
        {metrics.joint_violation, (spec.q_lower - trajectory.q[i]).maxCoeff(),
         (trajectory.q[i] - spec.q_upper).maxCoeff()});
  }

  metrics.passed = metrics.start_error <= spec.tolerance_endpoint &&
                   metrics.goal_error <= spec.tolerance_endpoint &&
                   metrics.dynamics_residual <= spec.tolerance_dynamics &&
                   metrics.control_violation <= spec.tolerance_limit &&
                   metrics.joint_violation <= spec.tolerance_limit;
  return metrics;
}

/// Print one PASS/FAIL line per metric, then the overall verdict.
inline void printSolutionMetrics(const SolutionMetrics& metrics, const GraphSpec& spec) {
  const auto line = [](const char* name, double value, double tolerance) {
    std::cout << (value <= tolerance ? "[PASS] " : "[FAIL] ") << name << ": " << value
              << " (tol " << tolerance << ")\n";
  };
  std::cout << "\n--- Graph solution evaluation ---\n";
  line("start state error     ", metrics.start_error, spec.tolerance_endpoint);
  line("goal state error      ", metrics.goal_error, spec.tolerance_endpoint);
  line("dynamics residual     ", metrics.dynamics_residual, spec.tolerance_dynamics);
  line("control limit excess  ", metrics.control_violation, spec.tolerance_limit);
  line("joint limit excess    ", metrics.joint_violation, spec.tolerance_limit);
  std::cout << "peak |u| = " << metrics.max_control << ", overall: "
            << (metrics.passed ? "PASS" : "FAIL") << "\n";
}

/**
 * Check a solution directly against the requirements (the constraints are
 * soft, so the total graph error alone can hide a small violation).
 */
inline SolutionMetrics evaluateSolution(const gtsam::Values& values,
                                        const GraphSpec& spec,
                                        bool print = true) {
  validateSpec(spec);
  const SolutionMetrics metrics =
      computeSolutionMetrics(extractTrajectory(values, spec.num_poses), spec);
  if (print) printSolutionMetrics(metrics, spec);
  return metrics;
}

}  // namespace scate
