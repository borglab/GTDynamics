/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSelfCollisionSphereFactor.cpp
 * @brief test the self collision hinge primitive and factor on bar_lab.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/config.h>
#include <gtdynamics/factors/SelfCollisionSphereFactor.h>
#include <gtdynamics/gpmp2/ObstacleCost.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Matrix13;
using gtsam::Point3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::noiseModel::Isotropic;
using gtsam::symbol_shorthand::X;

static const Robot kRobot =
    CreateRobotFromFile(kUrdfPath + std::string("bar_lab.urdf"));

/* ************************ point-to-point primitive ******************** */

// Beyond epsilon the cost is zero; within it, epsilon - dist with Jacobians
// pointing along the separation. Probed at a fixed distance away from the knee.
TEST(SelfCollisionCost, hingeAndJacobian) {
  const Point3 pA(1.0, 0.5, 0.2), pB(1.3, 0.5, 0.2);  // 0.3 apart
  const double eps = 0.5;

  EXPECT_DOUBLES_EQUAL(eps - 0.3, hingeLossSelfCollisionCost(pA, pB, eps),
                       1e-9);
  // Beyond epsilon: no cost, zero Jacobians.
  Matrix13 H_far;
  EXPECT_DOUBLES_EQUAL(
      0.0, hingeLossSelfCollisionCost(pA, Point3(3.0, 0.5, 0.2), eps, H_far),
      1e-9);
  EXPECT(assert_equal(Matrix(Matrix13::Zero()), Matrix(H_far), 1e-9));

  Matrix13 H_pA, H_pB;
  hingeLossSelfCollisionCost(pA, pB, eps, H_pA, H_pB);
  std::function<double(const Point3 &)> fa = [&](const Point3 &p) {
    return hingeLossSelfCollisionCost(p, pB, eps);
  };
  std::function<double(const Point3 &)> fb = [&](const Point3 &p) {
    return hingeLossSelfCollisionCost(pA, p, eps);
  };
  EXPECT(assert_equal(
      Matrix(gtsam::numericalDerivative11<double, Point3>(fa, pA)),
      Matrix(H_pA), 1e-5));
  EXPECT(assert_equal(
      Matrix(gtsam::numericalDerivative11<double, Point3>(fb, pB)),
      Matrix(H_pB), 1e-5));
}

/* ************************ bar_lab 18-DOF model ************************ */

// All eighteen joints, robot1's nine then robot2's nine.
static std::vector<JointSharedPtr> bothArmJoints() {
  const std::vector<std::string> names = {
      "bridge1_joint_EA_X", "robot1_joint_EA_Y", "robot1_joint_EA_Z",
      "robot1_joint_1",     "robot1_joint_2",    "robot1_joint_3",
      "robot1_joint_4",     "robot1_joint_5",    "robot1_joint_6",
      "bridge2_joint_EA_X", "robot2_joint_EA_Y", "robot2_joint_EA_Z",
      "robot2_joint_1",     "robot2_joint_2",    "robot2_joint_3",
      "robot2_joint_4",     "robot2_joint_5",    "robot2_joint_6"};
  std::vector<JointSharedPtr> joints;
  for (auto &&name : names) joints.push_back(kRobot.joint(name));
  return joints;
}

// Query points: two wrists (cross-arm pairs), robot1's forearm (same-arm
// pairs), and the two arm bases (reliably close for the push-apart test).
static std::vector<PointOnLink> queryPoints() {
  return {PointOnLink(kRobot.link("robot1_link_6"), Point3(0, 0, 0)),
          PointOnLink(kRobot.link("robot2_link_6"), Point3(0, 0, 0)),
          PointOnLink(kRobot.link("robot1_link_4"), Point3(0, 0, 0)),
          PointOnLink(kRobot.link("robot1_base"), Point3(0, 0, 0)),
          PointOnLink(kRobot.link("robot2_base"), Point3(0, 0, 0))};
}

static const Vector kArm =
    (Vector(6) << 0.2, -0.5, -1.0, 0.3, 0.5, 0.2).finished();

// Bridges far apart on the rail.
static Vector configApart() {
  Vector q(18);
  q << 2.0, 3.0, 1.0, kArm, 9.0, 3.0, 1.0, kArm;
  return q;
}
// Bridges 0.2 m apart, so the two arm bases nearly coincide.
static Vector configClose() {
  Vector q(18);
  q << 5.0, 3.0, 1.0, kArm, 5.2, 3.0, 1.0, kArm;
  return q;
}

/* ************************ factor Jacobians *************************** */

// Point-to-point across the two arms. Epsilon is set a metre past the measured
// wrist distance so the hinge is active and the numerical check exercises it.
TEST(SelfCollisionSphereFactor, pointPointJacobians) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), queryPoints());
  const Vector q = configApart();
  std::vector<Point3> wPs;
  model.queryPoints(q, &wPs);
  const double eps = (wPs[0] - wPs[1]).norm() + 1.0;

  const std::vector<SelfCollisionPair> pairs = {
      SelfCollisionPair(0, 1, eps)};  // wrist1 vs wrist2
  SelfCollisionSphereFactor factor(X(0), model, pairs, Vector::Zero(5), 0.1);

  EXPECT(factor.evaluateError(q)(0) > 0.0);  // active branch

  Values values;
  values.insert(X(0), q);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Several pairs in one factor, one cross-arm and one within robot1's arm, so a
// single row couples both arms' DOFs and another only robot1's.
TEST(SelfCollisionSphereFactor, multiplePairs) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), queryPoints());
  const Vector q = configApart();
  std::vector<Point3> wPs;
  model.queryPoints(q, &wPs);

  const std::vector<SelfCollisionPair> pairs = {
      SelfCollisionPair(0, 1, (wPs[0] - wPs[1]).norm() + 1.0),   // cross-arm
      SelfCollisionPair(2, 0, (wPs[2] - wPs[0]).norm() + 1.0)};  // same arm
  SelfCollisionSphereFactor factor(X(0), model, pairs, Vector::Zero(5), 0.1);

  const Vector err = factor.evaluateError(q);
  EXPECT_LONGS_EQUAL(2, err.size());
  EXPECT(err(0) > 0.0);  // both branches active
  EXPECT(err(1) > 0.0);

  Values values;
  values.insert(X(0), q);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************ behavioral push-apart ********************** */

// The two arm bases start within epsilon; the factor drives them apart. Epsilon
// is 0.3 m past the measured start distance so the pair starts in collision.
TEST(SelfCollisionSphereFactor, pushesPointsApart) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), queryPoints());

  const Vector q0 = configClose();
  std::vector<Point3> ps0;
  model.queryPoints(q0, &ps0);
  const double start_dist = (ps0[3] - ps0[4]).norm();
  const double eps = start_dist + 0.3;

  const std::vector<SelfCollisionPair> pairs = {
      SelfCollisionPair(3, 4, eps)};  // base1 vs base2
  SelfCollisionSphereFactor factor(X(0), model, pairs, Vector::Zero(5), 0.01);

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  // A weak prior keeps the rest of the configuration near its start.
  graph.addPrior<Vector>(X(0), q0, Isotropic::Sigma(18, 0.5));

  Values init;
  init.insert(X(0), q0);
  const Values result =
      gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();

  std::vector<Point3> ps;
  model.queryPoints(result.at<Vector>(X(0)), &ps);
  // The weak prior pulls back slightly, so equilibrium sits just under eps.
  EXPECT((ps[3] - ps[4]).norm() > eps - 0.05);
  EXPECT((ps[3] - ps[4]).norm() > start_dist + 0.15);  // clearly separated
}

/* ************************ per-point radii *************************** */

// eps is set 0.05 below the measured distance so the points are clear, then two
// 0.1 radii push eps + rA + rB past d, turning the pair into a collision.
TEST(SelfCollisionSphereFactor, radiiPointPoint) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), queryPoints());
  const Vector q = configApart();
  std::vector<Point3> wPs;
  model.queryPoints(q, &wPs);
  const double d = (wPs[0] - wPs[1]).norm();

  const double eps = d - 0.05;
  const std::vector<SelfCollisionPair> pairs = {
      SelfCollisionPair(0, 1, eps)};

  SelfCollisionSphereFactor clear(X(0), model, pairs, Vector::Zero(5), 0.1);
  EXPECT_DOUBLES_EQUAL(0.0, clear.evaluateError(q)(0), 1e-9);

  Vector radii = Vector::Zero(5);
  radii(0) = 0.1;
  radii(1) = 0.1;
  SelfCollisionSphereFactor inflated(X(0), model, pairs, radii, 0.1);
  EXPECT_DOUBLES_EQUAL(eps + 0.2 - d, inflated.evaluateError(q)(0), 1e-9);

  // The same total radius on either point gives the same standoff.
  Vector on_a = Vector::Zero(5), on_b = Vector::Zero(5);
  on_a(0) = 0.2;
  on_b(1) = 0.2;
  SelfCollisionSphereFactor f_a(X(0), model, pairs, on_a, 0.1);
  SelfCollisionSphereFactor f_b(X(0), model, pairs, on_b, 0.1);
  EXPECT_DOUBLES_EQUAL(f_a.evaluateError(q)(0), f_b.evaluateError(q)(0), 1e-9);

  // A radius on an uninvolved point is ignored.
  Vector other = Vector::Zero(5);
  other(2) = 0.2;
  SelfCollisionSphereFactor f_other(X(0), model, pairs, other, 0.1);
  EXPECT_DOUBLES_EQUAL(0.0, f_other.evaluateError(q)(0), 1e-9);
}

// In the active branch a radius adds directly to the cost, once per sphere.
TEST(SelfCollisionSphereFactor, radiiAddToTheStandoff) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), queryPoints());
  const Vector q = configApart();
  std::vector<Point3> wPs;
  model.queryPoints(q, &wPs);

  const double eps = (wPs[0] - wPs[1]).norm() + 1.0;  // active by construction
  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, eps)};

  SelfCollisionSphereFactor base(X(0), model, pairs, Vector::Zero(5), 0.1);
  const double e0 = base.evaluateError(q)(0);
  EXPECT(e0 > 0.0);

  // Each sphere's radius adds to the standoff, so the cost rises by their sum.
  Vector radii = Vector::Zero(5);
  radii(0) = 0.1;
  radii(1) = 0.2;
  SelfCollisionSphereFactor inflated(X(0), model, pairs, radii, 0.1);
  EXPECT_DOUBLES_EQUAL(e0 + 0.3, inflated.evaluateError(q)(0), 1e-9);

  // A radius on an uninvolved point is ignored.
  Vector other = Vector::Zero(5);
  other(2) = 0.2;
  SelfCollisionSphereFactor f_other(X(0), model, pairs, other, 0.1);
  EXPECT_DOUBLES_EQUAL(e0, f_other.evaluateError(q)(0), 1e-9);
}

/* ************************ input validation *************************** */

TEST(SelfCollisionSphereFactor, rejectsBadInput) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), queryPoints());
  const std::vector<SelfCollisionPair> pairs = {
      SelfCollisionPair(0, 1, 1.0)};

  // radii length must equal the number of query points.
  CHECK_EXCEPTION(
      SelfCollisionSphereFactor(X(0), model, pairs, Vector::Zero(3), 0.1),
      std::invalid_argument);
  // a point index out of range.
  const std::vector<SelfCollisionPair> bad = {
      SelfCollisionPair(0, 99, 1.0)};
  CHECK_EXCEPTION(
      SelfCollisionSphereFactor(X(0), model, bad, Vector::Zero(5), 0.1),
      std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
