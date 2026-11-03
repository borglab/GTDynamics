/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testScateGraph.cpp
 * @author Harrison Jenkins
 *
 * Tests for build_scate_graph.h on a synthetic 6-joint problem (no robot
 * model). Builds the graph, solves it with Levenberg-
 * Marquardt, and checks the solution against the requirements.
 */

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <stdexcept>

#include "build_scate_graph.h"

using namespace scate;

namespace {

Vector makeVector(std::initializer_list<double> entries) {
  Vector vector(static_cast<long>(entries.size()));
  long i = 0;
  for (double x : entries) vector(i++) = x;
  return vector;
}

// UR5-like home -> goal, generous limits.
GraphSpec makeSpec() {
  GraphSpec spec;
  spec.q_start = makeVector({0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0});
  spec.q_goal = makeVector({0.6, -1.2, 1.3, -1.9, -1.3, 0.4});
  spec.q_lower = Vector::Constant(6, -kPi);
  spec.q_upper = Vector::Constant(6, kPi);
  spec.u_limit = Vector::Constant(6, 1.0);
  spec.v_limit = Vector::Constant(6, 1.0);
  return spec;
}

JointTrajectory guess(const GraphSpec& spec, double bump = 0.0, const Vector& direction = Vector()) {
  TrajectoryConfig config;
  config.num_poses = spec.num_poses;
  config.dt = spec.dt;
  config.bump_amplitude = bump;
  config.bump_direction = direction;
  return makeInitialTrajectory(spec.q_start, spec.q_goal, config);
}

gtsam::Values toValues(const JointTrajectory& trajectory) {
  gtsam::Values values;
  for (size_t i = 0; i < trajectory.q.size(); ++i) {
    values.insert(QKey(i), trajectory.q[i]);
    values.insert(VKey(i), trajectory.v[i]);
  }
  for (size_t i = 0; i < trajectory.u.size(); ++i) values.insert(UKey(i), trajectory.u[i]);
  return values;
}

}  // namespace

TEST(ScateGraph, hasTheExpectedNumberOfFactors) {
  const GraphSpec spec = makeSpec();
  const auto graph = buildScateGraph(spec);
  // 4 priors + 20 dynamics + 20 control limits + 21 joint limits + 21 velocity limits
  EXPECT_LONGS_EQUAL(86, graph.size());
  EXPECT_LONGS_EQUAL(expectedFactorCount(spec.num_poses), graph.size());
}

TEST(ScateGraph, everyVariableOfTheInitialGuessIsInTheGraph) {
  const GraphSpec spec = makeSpec();
  const auto graph = buildScateGraph(spec);
  const auto values = toValues(guess(spec));
  EXPECT_LONGS_EQUAL(62, values.size());  // 21 q + 21 v + 20 u
  const auto graph_keys = graph.keys();
  for (gtsam::Key k : values.keys()) EXPECT(graph_keys.exists(k));
  EXPECT_LONGS_EQUAL(values.size(), graph_keys.size());  // and nothing extra
}

TEST(ScateGraph, extractTrajectoryRoundTrips) {
  const GraphSpec spec = makeSpec();
  const JointTrajectory trajectory = guess(spec);
  const JointTrajectory round_trip = extractTrajectory(toValues(trajectory), spec.num_poses);
  double worst = 0.0;
  for (size_t i = 0; i < trajectory.q.size(); ++i) {
    worst = std::max({worst, (trajectory.q[i] - round_trip.q[i]).norm(), (trajectory.v[i] - round_trip.v[i]).norm()});
  }
  for (size_t i = 0; i < trajectory.u.size(); ++i) worst = std::max(worst, (trajectory.u[i] - round_trip.u[i]).norm());
  EXPECT_DOUBLES_EQUAL(0.0, worst, 0.0);
}

TEST(ScateGraph, solveReducesTheErrorAndMeetsTheRequirements) {
  const GraphSpec spec = makeSpec();
  const auto graph = buildScateGraph(spec);
  const auto initial = toValues(guess(spec));
  const auto result = optimizeGraph(graph, initial);

  const double before = graph.error(initial), after = graph.error(result);
  std::cout << "  [diag] graph error " << before << " -> " << after << "\n";
  EXPECT(after < before);
  EXPECT(after < 1e-6);
  EXPECT(evaluateSolution(result, spec, /*print=*/false).passed);
}

TEST(ScateGraph, solutionStaysCloseToAGoodInitialGuess) {
  // No factor prefers a different path (no effort cost, as in the paper), so a
  // feasible guess should barely move.
  const GraphSpec spec = makeSpec();
  const auto graph = buildScateGraph(spec);
  const JointTrajectory trajectory = guess(spec);
  const JointTrajectory solved = extractTrajectory(optimizeGraph(graph, toValues(trajectory)), spec.num_poses);
  double worst = 0.0;
  for (size_t i = 0; i < trajectory.q.size(); ++i) worst = std::max(worst, (trajectory.q[i] - solved.q[i]).cwiseAbs().maxCoeff());
  EXPECT(worst < 1e-2);
}

TEST(ScateGraph, jointLimitHingePullsAnOverreachingGuessBackInside) {
  GraphSpec spec = makeSpec();
  // Joint 0 goes 0 -> 0.6; with a 0.5 rad bump its guess peaks at ~0.84 mid-path,
  // above this 0.7 limit (the goal, 0.6, is inside it, so a feasible path exists).
  spec.q_upper(0) = 0.7;
  const Vector direction = makeVector({1, 0, 0, 0, 0, 0});
  const JointTrajectory trajectory = guess(spec, 0.5, direction);
  const auto graph = buildScateGraph(spec);
  const auto initial = toValues(trajectory);
  const SolutionMetrics before = evaluateSolution(initial, spec, false);
  const SolutionMetrics after = evaluateSolution(optimizeGraph(graph, initial), spec, false);
  std::cout << "  [diag] joint-limit excess " << before.joint_violation << " -> "
            << after.joint_violation << "\n";
  EXPECT(before.joint_violation > 0.05);  // the guess really does violate the limit
  EXPECT(after.joint_violation <= spec.tolerance_limit);
  EXPECT(after.passed);
}

TEST(ScateGraph, controlLimitHingeReducesAnOverlyAggressiveGuess) {
  GraphSpec spec = makeSpec();
  spec.num_poses = 11;                 // short horizon => larger accelerations
  spec.q_goal(0) = 2.5;                // big move on joint 0
  // Smootherstep peaks at ~5.77 D/T^2 = 0.144; the minimum possible peak is
  // ~4 D/T^2 = 0.100, so a limit of 0.12 is feasible but the guess violates it.
  spec.u_limit = Vector::Constant(6, 0.12);
  spec.u_margin = 0.005;
  const auto graph = buildScateGraph(spec);
  const auto initial = toValues(guess(spec));
  const SolutionMetrics before = evaluateSolution(initial, spec, false);
  const SolutionMetrics after = evaluateSolution(optimizeGraph(graph, initial), spec, false);
  std::cout << "  [diag] control-limit excess " << before.control_violation << " -> "
            << after.control_violation << ", dynamics residual "
            << after.dynamics_residual << "\n";
  EXPECT(before.control_violation > 0.01);
  EXPECT(after.control_violation < 0.1 * before.control_violation);
  EXPECT(after.control_violation <= 2e-3);
  EXPECT(after.dynamics_residual <= spec.tolerance_dynamics);
}

TEST(ScateGraph, evaluateSolutionDetectsAWrongGoal) {
  GraphSpec spec = makeSpec();
  const auto result = optimizeGraph(buildScateGraph(spec), toValues(guess(spec)));
  spec.q_goal(2) += 0.05;  // evaluate against a goal the solution does not reach
  const SolutionMetrics metrics = evaluateSolution(result, spec, false);
  EXPECT_DOUBLES_EQUAL(0.05, metrics.goal_error, 1e-3);
  EXPECT(!metrics.passed);
}

TEST(ScateGraph, rejectsInconsistentSpecs) {
  GraphSpec spec = makeSpec();
  spec.q_goal = makeVector({0.0, 0.0});  // wrong size
  bool threw = false;
  try { buildScateGraph(spec); } catch (const std::invalid_argument&) { threw = true; }
  EXPECT(threw);

  spec = makeSpec();
  spec.num_poses = 2;
  threw = false;
  try { buildScateGraph(spec); } catch (const std::invalid_argument&) { threw = true; }
  EXPECT(threw);

  spec = makeSpec();
  spec.q_lower(1) = spec.q_upper(1);  // empty joint range => hinge factor rejects it
  threw = false;
  try { buildScateGraph(spec); } catch (const std::invalid_argument&) { threw = true; }
  EXPECT(threw);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
