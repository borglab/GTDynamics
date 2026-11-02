/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testScateInit.cpp
 * @author Harrison Jenkins
 *
 * Tests for scate_init.h: robot loading, FK/IK helpers, and the layout and
 * content of the initial gtsam::Values, on the UR5 model that ships with
 * GTDynamics.
 */

#include <CppUnitLite/TestHarness.h>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <set>
#include <string>

#include "scate_init.h"

using namespace scate;
using gtdynamics::JointAngleKey;
using gtdynamics::Kinematics;
using gtdynamics::Robot;

namespace {

// Built once and shared by all tests (loading + IK is the slow part).
const Robot& robot() {
  static const Robot loaded = loadRobot(ur5UrdfPath());
  return loaded;
}
const Kinematics& kinematics() {
  static const Kinematics solver = makeKinematics();
  return solver;
}
const InitialProblem& problem() {
  static const InitialProblem initialized = initializeProblem(kinematics(), robot());
  return initialized;
}

double poseError(const gtsam::Pose3& a, const gtsam::Pose3& b) {
  return gtsam::Pose3::Logmap(a.between(b)).norm();
}

size_t numJoints() { return robot().joints().size(); }

}  // namespace

// ------------------------------------------------------------------
// Robot loading
// ------------------------------------------------------------------

TEST(ScateInit, robotIsAFixedBaseTreeWithOneBaseAndOneTip) {
  EXPECT_LONGS_EQUAL(startJoints().size(), numJoints());  // 6 for the UR5
  EXPECT_LONGS_EQUAL(numJoints() + 1, robot().links().size());  // connected tree
  EXPECT_LONGS_EQUAL(1, rootLinks(robot()).size());   // one base
  EXPECT_LONGS_EQUAL(1, leafLinks(robot()).size());   // one tip (no gripper branch)
  const std::string base = baseLinkName(robot());
  const std::string end_effector = endEffectorName(robot());
  std::cout << "  [diag] base link '" << base << "', end-effector link '" << end_effector << "'\n";
  EXPECT(base != end_effector);
  EXPECT(robot().link(base)->isFixed());   // base anchored
  EXPECT(!robot().link(end_effector)->isFixed());    // arm free to move
}

TEST(ScateInit, jointChainRunsFromTheBaseToTheTipWithUniqueIds) {
  const auto order = jointIdsInChainOrder(robot());
  EXPECT_LONGS_EQUAL(numJoints(), order.size());
  EXPECT_LONGS_EQUAL(numJoints(), std::set<size_t>(order.begin(), order.end()).size());  // unique
  for (auto&& joint : robot().joints()) {
    const size_t p = chainIndex(order, joint->id());
    // The p-th joint's parent is the previous joint's child (or the base for p = 0).
    if (p == 0) {
      EXPECT(joint->parent()->name() == baseLinkName(robot()));
    }
    if (p + 1 == order.size()) {
      EXPECT(joint->child()->name() == endEffectorName(robot()));
    }
  }
}

// ------------------------------------------------------------------
// Kinematics helpers
// ------------------------------------------------------------------

TEST(ScateInit, jointValuesFollowsChainOrder) {
  Vector q(numJoints());
  for (size_t j = 0; j < numJoints(); ++j) q(j) = 0.1 * (j + 1);
  const gtsam::Values values = jointValues(robot(), q, 3);
  const auto order = jointIdsInChainOrder(robot());
  EXPECT_LONGS_EQUAL(numJoints(), values.size());
  for (size_t p = 0; p < order.size(); ++p) {
    EXPECT_DOUBLES_EQUAL(q(p), values.at<double>(JointAngleKey(order[p], 3)), 0.0);
  }
}

TEST(ScateInit, forwardKinematicsDoesNotDependOnSliceIndex) {
  const Vector q = startJoints();
  const gtsam::Pose3 a = endEffectorPose(kinematics(), robot(), q, 0);
  const gtsam::Pose3 b = endEffectorPose(kinematics(), robot(), q, 5);
  EXPECT_DOUBLES_EQUAL(0.0, poseError(a, b), 1e-9);
}

TEST(ScateInit, forwardKinematicsRespondsToJointAngles) {
  const gtsam::Pose3 a = endEffectorPose(kinematics(), robot(), startJoints());
  const gtsam::Pose3 b = endEffectorPose(kinematics(), robot(), hiddenGoalJoints());
  EXPECT((a.translation() - b.translation()).norm() > 0.05);
}

// ------------------------------------------------------------------
// Start, goal, IK
// ------------------------------------------------------------------

// Diagnostics that separate WHY IK might fail. Read them together:
//   true-seed fails            -> frame/convention mismatch (goal vs FK), not the solver
//   true-seed ok, near-seed ok -> solver fine locally; far seed (start) is the issue
//   near-seed fails            -> solver/constraint settings are too weak
TEST(ScateInit, ikSeededAtTheTrueConfigurationReachesTheGoal) {
  const Vector q_true = hiddenGoalJoints();
  const gtsam::Pose3 goal = endEffectorPose(kinematics(), robot(), q_true);
  for (bool hard : {true, false}) {
    const IkResult attempt = tryIK(kinematics(), robot(), goal, q_true, hard);
    std::cout << "  [diag] true seed, " << (hard ? "hard" : "soft")
              << ": trans " << attempt.translation_error << " m, rot " << attempt.rotation_error << " rad\n";
    EXPECT_DOUBLES_EQUAL(0.0, attempt.translation_error, kIkTolerance);
    EXPECT_DOUBLES_EQUAL(0.0, attempt.rotation_error, kIkTolerance);
  }
}

TEST(ScateInit, ikSeededNearTheTrueConfigurationConverges) {
  const Vector q_true = hiddenGoalJoints();
  const gtsam::Pose3 goal = endEffectorPose(kinematics(), robot(), q_true);
  const Vector seed = q_true + Vector::Constant(q_true.size(), 0.1);  // 0.1 rad off each joint
  for (bool hard : {true, false}) {
    const IkResult attempt = tryIK(kinematics(), robot(), goal, seed, hard);
    std::cout << "  [diag] near seed, " << (hard ? "hard" : "soft")
              << ": trans " << attempt.translation_error << " m, rot " << attempt.rotation_error << " rad\n";
    EXPECT_DOUBLES_EQUAL(0.0, attempt.translation_error, kIkTolerance);
    EXPECT_DOUBLES_EQUAL(0.0, attempt.rotation_error, kIkTolerance);
  }
}

TEST(ScateInit, interpolatePoseHitsEndpointsAndSplitsTheTwistEvenly) {
  const gtsam::Pose3 a(gtsam::Rot3::RzRyRx(0.3, -0.2, 0.5), gtsam::Point3(0.1, -0.4, 0.7));
  const gtsam::Pose3 b(gtsam::Rot3::RzRyRx(-0.4, 0.6, 0.1), gtsam::Point3(-0.5, 0.2, 0.3));
  EXPECT(a.equals(interpolatePose(a, b, 0.0), 1e-9));
  EXPECT(b.equals(interpolatePose(a, b, 1.0), 1e-9));
  const gtsam::Vector6 full = gtsam::Pose3::Logmap(a.between(b));
  const gtsam::Vector6 half = gtsam::Pose3::Logmap(a.between(interpolatePose(a, b, 0.5)));
  EXPECT_DOUBLES_EQUAL(0.0, (half - 0.5 * full).cwiseAbs().maxCoeff(), 1e-9);
}

TEST(ScateInit, ikContinuationReachesTheGoalFromTheStartSeed) {
  const gtsam::Pose3 goal = endEffectorPose(kinematics(), robot(), hiddenGoalJoints());
  const Vector q = solveGoalIKContinuation(kinematics(), robot(), goal, startJoints(), kIkContinuationSteps);
  EXPECT_DOUBLES_EQUAL(0.0, poseError(endEffectorPose(kinematics(), robot(), q), goal), kIkTolerance);
}

TEST(ScateInit, startAndIkGoalAreWithinJointLimits) {
  EXPECT_LONGS_EQUAL(0, checkJointLimits(robot(), "start", problem().q_start));
  EXPECT_LONGS_EQUAL(0, checkJointLimits(robot(), "goal", problem().q_goal));
}

TEST(ScateInit, ikSolutionReachesTheTaskSpaceGoal) {
  // Independent of the check inside solveGoalIK: push q_goal back through FK.
  const gtsam::Pose3 reached = endEffectorPose(kinematics(), robot(), problem().q_goal);
  EXPECT_DOUBLES_EQUAL(0.0, poseError(reached, problem().wTcom_goal), 1e-3);
}

TEST(ScateInit, goalIsReachableByAGenuinelyDifferentMotion) {
  const gtsam::Pose3 start = endEffectorPose(kinematics(), robot(), problem().q_start);
  EXPECT((start.translation() - problem().wTcom_goal.translation()).norm() > 0.05);
  EXPECT((problem().q_goal - problem().q_start).norm() > 0.1);  // not a no-op trajectory
}

// ------------------------------------------------------------------
// Initial Values
// ------------------------------------------------------------------

TEST(ScateInit, keysEncodeVariableTypeAndTimeIndex) {
  std::set<gtsam::Key> keys;
  for (int i = 0; i < kNumPoses; ++i) {
    EXPECT_LONGS_EQUAL('q', gtsam::Symbol(QKey(i)).chr());
    EXPECT_LONGS_EQUAL('v', gtsam::Symbol(VKey(i)).chr());
    EXPECT_LONGS_EQUAL(i, gtsam::Symbol(QKey(i)).index());
    keys.insert(QKey(i));
    keys.insert(VKey(i));
    if (i < kNumControls) {
      EXPECT_LONGS_EQUAL('u', gtsam::Symbol(UKey(i)).chr());
      keys.insert(UKey(i));
    }
  }
  EXPECT_LONGS_EQUAL(2 * kNumPoses + kNumControls, keys.size());  // all distinct
}

TEST(ScateInit, valuesHaveTheExpectedLayout) {
  const gtsam::Values& values = problem().values;
  EXPECT_LONGS_EQUAL(2 * kNumPoses + kNumControls, values.size());  // 21 + 21 + 20 = 62
  for (int i = 0; i < kNumPoses; ++i) {
    EXPECT(values.exists(QKey(i)));
    EXPECT(values.exists(VKey(i)));
    EXPECT_LONGS_EQUAL(numJoints(), values.at<gtsam::Vector>(QKey(i)).size());
    EXPECT_LONGS_EQUAL(numJoints(), values.at<gtsam::Vector>(VKey(i)).size());
  }
  for (int i = 0; i < kNumControls; ++i) {
    EXPECT(values.exists(UKey(i)));
    EXPECT_LONGS_EQUAL(numJoints(), values.at<gtsam::Vector>(UKey(i)).size());
  }
  EXPECT(!values.exists(UKey(kNumControls)));  // there is no u_N
}

TEST(ScateInit, valuesStartAtRestAtStartAndEndAtRestAtGoal) {
  const gtsam::Values& values = problem().values;
  EXPECT_DOUBLES_EQUAL(0.0, (Vector(values.at<gtsam::Vector>(QKey(0))) - problem().q_start).norm(), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, (Vector(values.at<gtsam::Vector>(QKey(kNumPoses - 1))) - problem().q_goal).norm(), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, Vector(values.at<gtsam::Vector>(VKey(0))).norm(), 0.0);
  EXPECT_DOUBLES_EQUAL(0.0, Vector(values.at<gtsam::Vector>(VKey(kNumPoses - 1))).norm(), 0.0);
}

TEST(ScateInit, valuesAreNearlyConsistentWithTheDynamicsFactor) {
  // Recomputed from the Values (not the trajectory struct), so it checks what the
  // optimizer will actually start from. ~1e-3 expected from finite-difference
  // velocities; this is a loose guard against gross errors.
  const gtsam::Values& values = problem().values;
  const auto dynamics = makeDoubleIntegrator(static_cast<int>(numJoints()), kDt);
  double worst = 0.0;
  for (int i = 0; i < kNumControls; ++i) {
    const Vector x = stack(values.at<gtsam::Vector>(QKey(i)), values.at<gtsam::Vector>(VKey(i)));
    const Vector x_next = stack(values.at<gtsam::Vector>(QKey(i + 1)), values.at<gtsam::Vector>(VKey(i + 1)));
    const Vector residual = x_next - dynamics.Fx * x - dynamics.Fu * Vector(values.at<gtsam::Vector>(UKey(i)));
    worst = std::max(worst, residual.cwiseAbs().maxCoeff());
  }
  EXPECT(worst < 1e-2);
}

TEST(ScateInit, wholeInitialPathStaysWithinJointLimits) {
  for (int i = 0; i < kNumPoses; ++i) {
    EXPECT_LONGS_EQUAL(
        0, checkJointLimits(robot(), "q" + std::to_string(i),
                            problem().values.at<gtsam::Vector>(QKey(i))));
  }
}

TEST(ScateInit, endEffectorEndsAtTheGoal) {
  const Vector q_end = problem().values.at<gtsam::Vector>(QKey(kNumPoses - 1));
  EXPECT_DOUBLES_EQUAL(0.0, poseError(endEffectorPose(kinematics(), robot(), q_end), problem().wTcom_goal), 1e-3);
}

// ------------------------------------------------------------------
// Report / CSV
// ------------------------------------------------------------------

TEST(ScateInit, reportWritesOneCsvRowPerState) {
  const std::string path = "scate_test_trajectory.csv";
  report(kinematics(), robot(), problem(), path);

  std::ifstream in(path);
  EXPECT(in.good());
  std::string header, line;
  std::getline(in, header);
  const long columns = std::count(header.begin(), header.end(), ',') + 1;
  EXPECT_LONGS_EQUAL(1 + 2 * numJoints() + 3, columns);  // i, q*, v*, ee_xyz
  long rows = 0;
  while (std::getline(in, line)) {
    if (!line.empty()) ++rows;
  }
  EXPECT_LONGS_EQUAL(kNumPoses, rows);
  std::remove(path.c_str());
}

// ------------------------------------------------------------------
// Bridge to the factor graph: real robot limits, then build and solve
// ------------------------------------------------------------------

TEST(ScateInit, graphSpecUsesTheRobotsJointLimitsInChainOrder) {
  const GraphSpec spec = makeGraphSpec(robot(), problem());
  EXPECT_LONGS_EQUAL(numJoints(), spec.q_lower.size());
  EXPECT_LONGS_EQUAL(numJoints(), spec.q_upper.size());
  EXPECT_LONGS_EQUAL(numJoints(), spec.u_limit.size());
  const auto order = jointIdsInChainOrder(robot());
  for (auto&& joint : robot().joints()) {
    const size_t p = chainIndex(order, joint->id());
    const auto& limits = joint->parameters().scalar_limits;
    EXPECT_DOUBLES_EQUAL(limits.value_lower_limit, spec.q_lower(p), 0.0);
    EXPECT_DOUBLES_EQUAL(limits.value_upper_limit, spec.q_upper(p), 0.0);
    EXPECT(spec.q_lower(p) < spec.q_upper(p));
    EXPECT(spec.q_start(p) >= spec.q_lower(p) && spec.q_start(p) <= spec.q_upper(p));
    EXPECT(spec.q_goal(p) >= spec.q_lower(p) && spec.q_goal(p) <= spec.q_upper(p));
  }
}

TEST(ScateInit, factorGraphBuildsAndSolvesOnTheRealInitialGuess) {
  const GraphSpec spec = makeGraphSpec(robot(), problem());
  const auto graph = buildScateGraph(spec);
  EXPECT_LONGS_EQUAL(expectedFactorCount(kNumPoses), graph.size());

  const auto result = optimizeGraph(graph, problem().values);
  std::cout << "  [diag] graph error " << graph.error(problem().values) << " -> "
            << graph.error(result) << "\n";
  EXPECT(graph.error(result) <= graph.error(problem().values));
  EXPECT(evaluateSolution(result, spec, /*print=*/true).passed);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
