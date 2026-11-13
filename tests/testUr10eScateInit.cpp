/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testUr10eScateInit.cpp
 * @author Harrison Jenkins
 *
 * scate_init.h on the UR10e model that ships with GTDynamics: loading (the
 * URDF has massless frames and fixed joints the UR5 file lacks), chain order,
 * FK against an independent numpy FK of the URDF, IK for the scenario goals,
 * and the offline graph on the real initial guess. The robot-independent
 * parts of scate_init.h are covered by testScateInit on the UR5.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "scate_init.h"
#include "scate_scenario.h"
#include "ur10e_collision.h"

using namespace scate;
using gtdynamics::Kinematics;
using gtdynamics::Robot;

namespace {

// Built once and shared by all tests (loading + IK is the slow part).
const Robot& robot() {
  static const Robot loaded = loadRobot(ur10eUrdfPath());
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

}  // namespace

TEST(Ur10eScateInit, robotIsASixJointSerialChain) {
  EXPECT_LONGS_EQUAL(6, robot().joints().size());
  EXPECT_LONGS_EQUAL(7, robot().links().size());  // massless frames are not links
  EXPECT_LONGS_EQUAL(1, rootLinks(robot()).size());
  EXPECT_LONGS_EQUAL(1, leafLinks(robot()).size());
  std::cout << "  [diag] base link '" << baseLinkName(robot()) << "', end-effector link '"
            << endEffectorName(robot()) << "'\n";
  EXPECT(endEffectorName(robot()) == "wrist_3_link");
  EXPECT(robot().link(baseLinkName(robot()))->isFixed());
}

TEST(Ur10eScateInit, chainOrderFollowsTheUrdfJointNames) {
  const std::vector<std::string> expected = {"shoulder_pan_joint", "shoulder_lift_joint",
                                             "elbow_joint",        "wrist_1_joint",
                                             "wrist_2_joint",      "wrist_3_joint"};
  const auto order = jointIdsInChainOrder(robot());
  for (auto&& joint : robot().joints()) {
    EXPECT(joint->name() == expected[chainIndex(order, joint->id())]);
  }
}

// FK of the end-effector link's CoM frame (0.026 m behind the flange) at the
// home pose, against the numpy FK of the URDF.
TEST(Ur10eScateInit, forwardKinematicsMatchesReferenceAtHome) {
  const gtsam::Pose3 wTcom = endEffectorPose(kinematics(), robot(), startJoints());
  const gtsam::Pose3 expected(
      gtsam::Rot3((gtsam::Matrix3() << 0, -1, 0, -1, 0, 0, 0, 0, -1).finished()),
      gtsam::Point3(0.691398, 0.174150, 0.702850));
  std::cout << "  [diag] home end-effector " << wTcom.translation().transpose() << "\n";
  EXPECT(assert_equal(expected, wTcom, 1e-4));
}

TEST(Ur10eScateInit, startAndScenarioGoalsAreWithinJointLimits) {
  EXPECT_LONGS_EQUAL(0, checkJointLimits(robot(), "start", startJoints()));
  EXPECT_LONGS_EQUAL(0, checkJointLimits(robot(), "hidden goal", hiddenGoalJoints()));
  EXPECT_LONGS_EQUAL(0, checkJointLimits(robot(), "sweep goal", sweepGoalJoints()));
  EXPECT_LONGS_EQUAL(0, checkJointLimits(robot(), "IK goal", problem().q_goal));
}

TEST(Ur10eScateInit, ikSeededNearTheTrueConfigurationConverges) {
  const Vector q_true = hiddenGoalJoints();
  const gtsam::Pose3 goal = endEffectorPose(kinematics(), robot(), q_true);
  const Vector seed = q_true + Vector::Constant(q_true.size(), 0.1);
  const IkResult attempt = tryIK(kinematics(), robot(), goal, seed, true);
  EXPECT(attempt.ok());
}

TEST(Ur10eScateInit, ikReachesBothScenarioGoalsFromTheStart) {
  for (const Vector& hidden : {hiddenGoalJoints(), sweepGoalJoints()}) {
    const gtsam::Pose3 goal = endEffectorPose(kinematics(), robot(), hidden);
    const Vector q = solveGoalIK(kinematics(), robot(), goal, startJoints());
    EXPECT_DOUBLES_EQUAL(0.0, poseError(endEffectorPose(kinematics(), robot(), q), goal),
                         kIkTolerance);
    EXPECT((q - startJoints()).norm() > 0.1);  // a genuine motion
  }
}

TEST(Ur10eScateInit, graphSpecUsesTheUr10eLimits) {
  const GraphSpec spec = makeGraphSpec(robot(), problem());
  const auto order = jointIdsInChainOrder(robot());
  for (auto&& joint : robot().joints()) {
    const size_t p = chainIndex(order, joint->id());
    const auto& limits = joint->parameters().scalar_limits;
    EXPECT_DOUBLES_EQUAL(limits.value_lower_limit, spec.q_lower(p), 0.0);
    EXPECT_DOUBLES_EQUAL(limits.value_upper_limit, spec.q_upper(p), 0.0);
    // The planning velocity limit, capped by the model's (2.09 or 3.14 rad/s).
    EXPECT_DOUBLES_EQUAL(std::min(kVelocityLimit, joint->parameters().velocity_limit),
                         spec.v_limit(p), 0.0);
  }
  EXPECT_DOUBLES_EQUAL(-M_PI, spec.q_lower(2), 1e-9);  // the elbow is the tightest joint
  EXPECT_DOUBLES_EQUAL(M_PI, spec.q_upper(2), 1e-9);
}

TEST(Ur10eScateInit, factorGraphBuildsAndSolvesOnTheRealInitialGuess) {
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
