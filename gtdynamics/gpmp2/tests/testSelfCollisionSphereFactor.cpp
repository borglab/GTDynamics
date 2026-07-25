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
  Matrix13 Hfar;
  EXPECT_DOUBLES_EQUAL(
      0.0, hingeLossSelfCollisionCost(pA, Point3(3.0, 0.5, 0.2), eps, Hfar),
      1e-9);
  EXPECT(assert_equal(Matrix(Matrix13::Zero()), Matrix(Hfar), 1e-9));

  Matrix13 HptA, HptB;
  hingeLossSelfCollisionCost(pA, pB, eps, HptA, HptB);
  std::function<double(const Point3 &)> fa = [&](const Point3 &p) {
    return hingeLossSelfCollisionCost(p, pB, eps);
  };
  std::function<double(const Point3 &)> fb = [&](const Point3 &p) {
    return hingeLossSelfCollisionCost(pA, p, eps);
  };
  EXPECT(assert_equal(
      Matrix(gtsam::numericalDerivative11<double, Point3>(fa, pA)),
      Matrix(HptA), 1e-5));
  EXPECT(assert_equal(
      Matrix(gtsam::numericalDerivative11<double, Point3>(fb, pB)),
      Matrix(HptB), 1e-5));

  // Coincident points: full standoff cost, finite fixed-direction Jacobian.
  Matrix13 HcA, HcB;
  EXPECT_DOUBLES_EQUAL(eps, hingeLossSelfCollisionCost(pA, pA, eps, HcA, HcB),
                       1e-9);
  EXPECT(HcA.allFinite());
  EXPECT(HcB.allFinite());
  EXPECT(assert_equal(Matrix(-HcB), Matrix(HcA), 1e-9));
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
  std::vector<Point3> wPts;
  model.queryPoints(q, &wPts);
  const double eps = (wPts[0] - wPts[1]).norm() + 1.0;

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
  std::vector<Point3> wPts;
  model.queryPoints(q, &wPts);

  const std::vector<SelfCollisionPair> pairs = {
      SelfCollisionPair(0, 1, (wPts[0] - wPts[1]).norm() + 1.0),   // cross-arm
      SelfCollisionPair(2, 0, (wPts[2] - wPts[0]).norm() + 1.0)};  // same arm
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
  std::vector<Point3> startPts;
  model.queryPoints(q0, &startPts);
  const double startDistance = (startPts[3] - startPts[4]).norm();
  const double eps = startDistance + 0.3;

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

  std::vector<Point3> pts;
  model.queryPoints(result.at<Vector>(X(0)), &pts);
  // The weak prior pulls back slightly, so equilibrium sits just under eps.
  EXPECT((pts[3] - pts[4]).norm() > eps - 0.05);
  EXPECT((pts[3] - pts[4]).norm() > startDistance + 0.15);  // clearly separated
}

/* ************************ per-point radii *************************** */

// eps is set 0.05 below the measured distance so the points are clear, then two
// 0.1 radii push eps + rA + rB past dist, turning the pair into a collision.
TEST(SelfCollisionSphereFactor, radiiPointPoint) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), queryPoints());
  const Vector q = configApart();
  std::vector<Point3> wPts;
  model.queryPoints(q, &wPts);
  const double dist = (wPts[0] - wPts[1]).norm();

  const double eps = dist - 0.05;
  const std::vector<SelfCollisionPair> pairs = {
      SelfCollisionPair(0, 1, eps)};

  SelfCollisionSphereFactor clear(X(0), model, pairs, Vector::Zero(5), 0.1);
  EXPECT_DOUBLES_EQUAL(0.0, clear.evaluateError(q)(0), 1e-9);

  Vector radii = Vector::Zero(5);
  radii(0) = 0.1;
  radii(1) = 0.1;
  SelfCollisionSphereFactor inflated(X(0), model, pairs, radii, 0.1);
  EXPECT_DOUBLES_EQUAL(eps + 0.2 - dist, inflated.evaluateError(q)(0), 1e-9);

  // The same total radius on either point gives the same standoff.
  Vector radiiOnA = Vector::Zero(5), radiiOnB = Vector::Zero(5);
  radiiOnA(0) = 0.2;
  radiiOnB(1) = 0.2;
  SelfCollisionSphereFactor factorA(X(0), model, pairs, radiiOnA, 0.1);
  SelfCollisionSphereFactor factorB(X(0), model, pairs, radiiOnB, 0.1);
  EXPECT_DOUBLES_EQUAL(factorA.evaluateError(q)(0), factorB.evaluateError(q)(0),
                       1e-9);

  // A radius on an uninvolved point is ignored.
  Vector other = Vector::Zero(5);
  other(2) = 0.2;
  SelfCollisionSphereFactor factorOther(X(0), model, pairs, other, 0.1);
  EXPECT_DOUBLES_EQUAL(0.0, factorOther.evaluateError(q)(0), 1e-9);
}

// In the active branch a radius adds directly to the cost, once per sphere.
TEST(SelfCollisionSphereFactor, radiiAddToTheStandoff) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), queryPoints());
  const Vector q = configApart();
  std::vector<Point3> wPts;
  model.queryPoints(q, &wPts);

  // Active by construction: eps sits a metre past the measured distance.
  const double eps = (wPts[0] - wPts[1]).norm() + 1.0;
  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, eps)};

  SelfCollisionSphereFactor base(X(0), model, pairs, Vector::Zero(5), 0.1);
  const double err0 = base.evaluateError(q)(0);
  EXPECT(err0 > 0.0);

  // Each sphere's radius adds to the standoff, so the cost rises by their sum.
  Vector radii = Vector::Zero(5);
  radii(0) = 0.1;
  radii(1) = 0.2;
  SelfCollisionSphereFactor inflated(X(0), model, pairs, radii, 0.1);
  EXPECT_DOUBLES_EQUAL(err0 + 0.3, inflated.evaluateError(q)(0), 1e-9);

  // A radius on an uninvolved point is ignored.
  Vector other = Vector::Zero(5);
  other(2) = 0.2;
  SelfCollisionSphereFactor factorOther(X(0), model, pairs, other, 0.1);
  EXPECT_DOUBLES_EQUAL(err0, factorOther.evaluateError(q)(0), 1e-9);
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

// A pair whose two spheres sit on the same rigid link has a constant separation
// and no gradient, so it is rejected.
TEST(SelfCollisionSphereFactor, rejectsSameLinkPair) {
  const LinkSharedPtr link = kRobot.link("robot1_link_6");
  const std::vector<PointOnLink> points = {
      PointOnLink(link, Point3(0.0, 0.0, 0.0)),
      PointOnLink(link, Point3(0.1, 0.0, 0.0))};  // both on link_6
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), points);
  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, 0.1)};

  CHECK_EXCEPTION(
      SelfCollisionSphereFactor(X(0), model, pairs, Vector::Zero(2), 0.1),
      std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
