/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testTrajectoryInit.cpp
 * @author Harrison Jenkins
 *
 * Unit tests for trajectory_init.h (Eigen only, no GTDynamics needed).
 */

#include <CppUnitLite/TestHarness.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "trajectory_init.h"

using namespace scate;

namespace {

double maxAbsDiff(const Vector& a, const Vector& b) { return (a - b).cwiseAbs().maxCoeff(); }
double maxAbsDiff(const Matrix& a, const Matrix& b) { return (a - b).cwiseAbs().maxCoeff(); }

Vector makeVector(std::initializer_list<double> entries) {
  Vector vector(static_cast<long>(entries.size()));
  long i = 0;
  for (double x : entries) vector(i++) = x;
  return vector;
}

// Forward-simulate x_{i+1} = Fx x_i + Fu u_i, so the result is dynamically
// feasible by construction.
JointTrajectory simulate(const Vector& q0, const Vector& v0, const std::vector<Vector>& u,
                         const DoubleIntegrator& dynamics) {
  JointTrajectory trajectory;
  trajectory.u = u;
  Vector x = stack(q0, v0);
  const long n = q0.size();
  trajectory.q.push_back(x.head(n));
  trajectory.v.push_back(x.tail(n));
  for (const Vector& ui : u) {
    x = dynamics.Fx * x + dynamics.Fu * ui;
    trajectory.q.push_back(x.head(n));
    trajectory.v.push_back(x.tail(n));
  }
  return trajectory;
}

}  // namespace

// ------------------------------------------------------------------
// Dynamics model
// ------------------------------------------------------------------

TEST(DoubleIntegrator, blockStructure) {
  const int n = 3;
  const double dt = 0.7;
  const DoubleIntegrator dynamics = makeDoubleIntegrator(n, dt);
  const Matrix I = Matrix::Identity(n, n);
  EXPECT_LONGS_EQUAL(2 * n, dynamics.Fx.rows());
  EXPECT_LONGS_EQUAL(2 * n, dynamics.Fx.cols());
  EXPECT_LONGS_EQUAL(2 * n, dynamics.Fu.rows());
  EXPECT_LONGS_EQUAL(n, dynamics.Fu.cols());
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(Matrix(dynamics.Fx.topLeftCorner(n, n)), I), 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(Matrix(dynamics.Fx.topRightCorner(n, n)), dt * I), 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, dynamics.Fx.bottomLeftCorner(n, n).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(Matrix(dynamics.Fx.bottomRightCorner(n, n)), I), 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(Matrix(dynamics.Fu.topRows(n)), 0.5 * dt * dt * I), 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(Matrix(dynamics.Fu.bottomRows(n)), dt * I), 1e-15);
}

TEST(DoubleIntegrator, matchesConstantAccelerationKinematics) {
  const double dt = 0.4;
  const DoubleIntegrator dynamics = makeDoubleIntegrator(2, dt);
  const Vector q = makeVector({0.3, -1.1}), v = makeVector({0.5, 0.2}), u = makeVector({-0.7, 1.3});
  const Vector x_next = dynamics.Fx * stack(q, v) + dynamics.Fu * u;
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(Vector(x_next.head(2)), Vector(q + dt * v + 0.5 * dt * dt * u)), 1e-14);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(Vector(x_next.tail(2)), Vector(v + dt * u)), 1e-14);
}

TEST(DoubleIntegrator, twoHalfStepsEqualOneFullStep) {
  // For constant u, stepping dt/2 twice must equal stepping dt once.
  const double dt = 0.9;
  const DoubleIntegrator full = makeDoubleIntegrator(3, dt);
  const DoubleIntegrator half = makeDoubleIntegrator(3, dt / 2);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(Matrix(half.Fx * half.Fx), full.Fx), 1e-14);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(Matrix(half.Fx * half.Fu + half.Fu), full.Fu), 1e-14);
}

TEST(Stack, putsQAboveV) {
  const Vector x = stack(makeVector({1, 2}), makeVector({3, 4, 5}));
  EXPECT_LONGS_EQUAL(5, x.size());
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(x, makeVector({1, 2, 3, 4, 5})), 0.0);
}

// ------------------------------------------------------------------
// smootherstep
// ------------------------------------------------------------------

TEST(Smootherstep, endpointsAndSymmetry) {
  EXPECT_DOUBLES_EQUAL(0.0, smootherstep(0.0), 1e-15);
  EXPECT_DOUBLES_EQUAL(1.0, smootherstep(1.0), 1e-15);
  EXPECT_DOUBLES_EQUAL(0.5, smootherstep(0.5), 1e-15);
  for (double t = 0.0; t <= 1.0; t += 0.05) {
    EXPECT_DOUBLES_EQUAL(1.0, smootherstep(t) + smootherstep(1.0 - t), 1e-14);
  }
}

TEST(Smootherstep, monotoneNondecreasing) {
  double prev = smootherstep(0.0);
  for (int i = 1; i <= 1000; ++i) {
    const double s = smootherstep(i / 1000.0);
    EXPECT(s >= prev);
    prev = s;
  }
}

TEST(Smootherstep, zeroVelocityAtBothEnds) {
  const double h = 1e-4;  // slope ~ 30 h^2 ~ 3e-7 if the ends are flat
  EXPECT_DOUBLES_EQUAL(0.0, (smootherstep(h) - smootherstep(0.0)) / h, 1e-6);
  EXPECT_DOUBLES_EQUAL(0.0, (smootherstep(1.0) - smootherstep(1.0 - h)) / h, 1e-6);
}

// ------------------------------------------------------------------
// fitControl
// ------------------------------------------------------------------

TEST(FitControl, recoversTheControlThatGeneratedTheStep) {
  const DoubleIntegrator dynamics = makeDoubleIntegrator(3, 0.8);
  const Vector x = stack(makeVector({0.1, 0.2, 0.3}), makeVector({-0.4, 0.5, 0.0}));
  const Vector u_true = makeVector({0.7, -1.2, 0.05});
  const Vector x_next = dynamics.Fx * x + dynamics.Fu * u_true;
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(fitControl(dynamics, x, x_next), u_true), 1e-12);
}

TEST(FitControl, matchesClosedFormAndNormalEquations) {
  const double dt = 0.7;
  const int n = 3;
  const DoubleIntegrator dynamics = makeDoubleIntegrator(n, dt);
  // An inconsistent step: no control explains it exactly.
  const Vector x = stack(makeVector({0.0, 0.1, -0.2}), makeVector({0.3, 0.0, 0.1}));
  const Vector x_next = stack(makeVector({0.5, 0.2, -0.9}), makeVector({-0.2, 0.4, 0.8}));
  const Vector u = fitControl(dynamics, x, x_next);

  const Vector r = x_next - dynamics.Fx * x;
  const Vector expected = (0.5 * dt * dt * Vector(r.head(n)) + dt * Vector(r.tail(n))) /
                       (0.25 * dt * dt * dt * dt + dt * dt);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(u, expected), 1e-12);

  // Least-squares optimality: the leftover residual is orthogonal to range(Fu).
  const Vector leftover = r - dynamics.Fu * u;
  EXPECT_DOUBLES_EQUAL(0.0, (dynamics.Fu.transpose() * leftover).cwiseAbs().maxCoeff(), 1e-12);
}

// ------------------------------------------------------------------
// makeInitialTrajectory
// ------------------------------------------------------------------

TEST(InitialTrajectory, sizesAndEndpoints) {
  const Vector q_start = makeVector({0.0, -0.5, 0.2}), q_goal = makeVector({1.0, 0.5, -0.4});
  TrajectoryConfig config;
  config.num_poses = 21;
  const JointTrajectory trajectory = makeInitialTrajectory(q_start, q_goal, config);

  EXPECT_LONGS_EQUAL(21, trajectory.q.size());
  EXPECT_LONGS_EQUAL(21, trajectory.v.size());
  EXPECT_LONGS_EQUAL(20, trajectory.u.size());
  for (const Vector& x : trajectory.q) EXPECT_LONGS_EQUAL(3, x.size());
  for (const Vector& x : trajectory.v) EXPECT_LONGS_EQUAL(3, x.size());
  for (const Vector& x : trajectory.u) EXPECT_LONGS_EQUAL(3, x.size());

  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(trajectory.q.front(), q_start), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(trajectory.q.back(), q_goal), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, trajectory.v.front().cwiseAbs().maxCoeff(), 0.0);  // rest at start
  EXPECT_DOUBLES_EQUAL(0.0, trajectory.v.back().cwiseAbs().maxCoeff(), 0.0);   // rest at goal
}

TEST(InitialTrajectory, noBumpIsStraightLineWithSmootherstepTiming) {
  // Also confirms the C++ time grid i/(N-1) equals np.linspace(0, 1, N) in the
  // Python version: s_i must equal smootherstep(i/(N-1)).
  const Vector q_start = makeVector({0.0, 0.5}), q_goal = makeVector({1.0, -0.5});
  TrajectoryConfig config;
  config.num_poses = 21;
  const JointTrajectory trajectory = makeInitialTrajectory(q_start, q_goal, config);
  const Vector displacement = q_goal - q_start;
  double prev = -1.0;
  for (int i = 0; i < 21; ++i) {
    const double s = (trajectory.q[i] - q_start).dot(displacement) / displacement.dot(displacement);
    EXPECT_DOUBLES_EQUAL(smootherstep(i / 20.0), s, 1e-12);
    EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(trajectory.q[i], Vector(q_start + s * displacement)), 1e-12);  // on the line
    EXPECT(s >= prev);
    prev = s;
  }
}

TEST(InitialTrajectory, velocitiesAreCentralDifferences) {
  const Vector q_start = makeVector({0.0, 0.5}), q_goal = makeVector({1.0, -0.5});
  TrajectoryConfig config;
  config.num_poses = 11;
  config.dt = 0.5;
  const JointTrajectory trajectory = makeInitialTrajectory(q_start, q_goal, config);
  for (int i = 1; i < 10; ++i) {
    EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(trajectory.v[i], Vector((trajectory.q[i + 1] - trajectory.q[i - 1]) / (2 * config.dt))), 1e-14);
  }
}

TEST(InitialTrajectory, bumpMovesMidpointAndKeepsEndpoints) {
  const Vector q_start = makeVector({0.0, 0.5}), q_goal = makeVector({1.0, -0.5});
  TrajectoryConfig config;
  config.num_poses = 21;
  config.bump_amplitude = 0.3;
  config.bump_direction = makeVector({1.0, -2.0});
  const JointTrajectory trajectory = makeInitialTrajectory(q_start, q_goal, config);

  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(trajectory.q.front(), q_start), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(trajectory.q.back(), q_goal), 1e-12);
  // At the midpoint s = 0.5 and sin(pi * s) = 1, so the offset is exactly A * dir.
  const Vector expected_mid = q_start + 0.5 * (q_goal - q_start) + 0.3 * config.bump_direction;
  EXPECT_DOUBLES_EQUAL(0.0, maxAbsDiff(trajectory.q[10], expected_mid), 1e-12);
}

TEST(InitialTrajectory, sparseBumpOnlyMovesTheChosenJoint) {
  const Vector q_start = makeVector({0.0, 0.5, 0.1}), q_goal = makeVector({1.0, -0.5, 0.4});
  TrajectoryConfig plain;
  plain.num_poses = 21;
  TrajectoryConfig bumped = plain;
  bumped.bump_amplitude = 0.4;
  bumped.bump_direction = makeVector({0.0, 1.0, 0.0});  // bump joint 1 only
  const JointTrajectory a = makeInitialTrajectory(q_start, q_goal, plain);
  const JointTrajectory b = makeInitialTrajectory(q_start, q_goal, bumped);
  double moved = 0.0;
  for (int i = 0; i < 21; ++i) {
    EXPECT_DOUBLES_EQUAL(a.q[i](0), b.q[i](0), 1e-15);
    EXPECT_DOUBLES_EQUAL(a.q[i](2), b.q[i](2), 1e-15);
    moved = std::max(moved, std::fabs(a.q[i](1) - b.q[i](1)));
  }
  EXPECT_DOUBLES_EQUAL(0.4, moved, 1e-12);
}

TEST(InitialTrajectory, rejectsBadInput) {
  const Vector q_start = makeVector({0.0, 0.5}), q_goal = makeVector({1.0, -0.5});
  TrajectoryConfig config;

  bool threw = false;
  try { makeInitialTrajectory(q_start, makeVector({1.0}), config); } catch (const std::invalid_argument&) { threw = true; }
  EXPECT(threw);  // q_start / q_goal size mismatch

  threw = false;
  config.num_poses = 2;
  try { makeInitialTrajectory(q_start, q_goal, config); } catch (const std::invalid_argument&) { threw = true; }
  EXPECT(threw);  // too few poses

  threw = false;
  config.num_poses = 21;
  config.bump_direction = makeVector({1.0, 0.0, 0.0});
  try { makeInitialTrajectory(q_start, q_goal, config); } catch (const std::invalid_argument&) { threw = true; }
  EXPECT(threw);  // bump direction of the wrong size
}

TEST(InitialTrajectory, goldenValuesFromIndependentNumpyImplementation) {
  // q: (0, 0.5) -> (1, -0.5), N = 21, dt = 1, no bump.
  const Vector q_start = makeVector({0.0, 0.5}), q_goal = makeVector({1.0, -0.5});
  TrajectoryConfig config;
  config.num_poses = 21;
  const JointTrajectory trajectory = makeInitialTrajectory(q_start, q_goal, config);
  EXPECT_DOUBLES_EQUAL(0.093126875, trajectory.v[10](0), 1e-9);
  EXPECT_DOUBLES_EQUAL(-0.093126875, trajectory.v[10](1), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.01339125, trajectory.u[5](0), 1e-9);
  double max_control = 0.0;
  for (const Vector& u : trajectory.u) max_control = std::max(max_control, u.cwiseAbs().maxCoeff());
  EXPECT_DOUBLES_EQUAL(0.01416, max_control, 1e-9);
  EXPECT_DOUBLES_EQUAL(8.8125e-4, maxDynamicsResidual(trajectory, makeDoubleIntegrator(2, config.dt)), 1e-9);
}

// ------------------------------------------------------------------
// maxDynamicsResidual
// ------------------------------------------------------------------

TEST(MaxDynamicsResidual, zeroForADynamicallyFeasibleTrajectory) {
  const DoubleIntegrator dynamics = makeDoubleIntegrator(2, 0.5);
  std::vector<Vector> u;
  for (int i = 0; i < 12; ++i) u.push_back(makeVector({std::sin(0.7 * i), 0.3 * i - 1.0}));
  const JointTrajectory trajectory = simulate(makeVector({0.1, -0.2}), makeVector({0.0, 0.4}), u, dynamics);
  EXPECT_DOUBLES_EQUAL(0.0, maxDynamicsResidual(trajectory, dynamics), 1e-12);
}

TEST(MaxDynamicsResidual, reportsTheSizeOfADeliberateViolation) {
  const double dt = 0.5;
  const DoubleIntegrator dynamics = makeDoubleIntegrator(2, dt);
  std::vector<Vector> u(8, makeVector({0.2, -0.1}));
  JointTrajectory trajectory = simulate(makeVector({0.0, 0.0}), makeVector({0.0, 0.0}), u, dynamics);
  const double delta = 0.25;
  trajectory.u[3](1) += delta;  // control no longer matches the states
  // Error vector is Fu * delta = [0.5 dt^2 delta; dt delta]; worst entry is dt * delta (dt < 2).
  EXPECT_DOUBLES_EQUAL(dt * delta, maxDynamicsResidual(trajectory, dynamics), 1e-12);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
