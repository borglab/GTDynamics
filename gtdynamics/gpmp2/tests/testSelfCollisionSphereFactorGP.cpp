/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSelfCollisionSphereFactorGP.cpp
 * @brief test the self collision factor at a GP interpolated state on bar_lab.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/config.h>
#include <gtdynamics/factors/SelfCollisionSphereFactor.h>
#include <gtdynamics/factors/SelfCollisionSphereFactorGP.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <memory>
#include <string>
#include <vector>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Point3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::noiseModel::Isotropic;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

static const Robot kRobot =
    CreateRobotFromFile(kUrdfPath + std::string("bar_lab.urdf"));

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

// Query points: the two wrists, for a cross-arm pair.
static std::vector<PointOnLink> wristPoints() {
  return {PointOnLink(kRobot.link("robot1_link_6"), Point3(0, 0, 0)),
          PointOnLink(kRobot.link("robot2_link_6"), Point3(0, 0, 0))};
}

static const Vector kArm =
    (Vector(6) << 0.2, -0.5, -1.0, 0.3, 0.5, 0.2).finished();

// Bridges far apart on the rail.
static Vector supportConfig1() {
  Vector q(18);
  q << 2.0, 3.0, 1.0, kArm, 9.0, 3.0, 1.0, kArm;
  return q;
}
// The bridges have moved toward each other.
static Vector supportConfig2() {
  Vector q(18);
  q << 3.0, 3.0, 1.0, kArm, 8.0, 3.0, 1.0, kArm;
  return q;
}
// Constant velocity consistent with the two support states over deltaT = 0.5.
static Vector supportVelocity() {
  Vector v = Vector::Zero(18);
  v(0) = 2.0;
  v(9) = -2.0;
  return v;
}

/* ************************ endpoint agreement ************************** */

// At tau = 0 and tau = deltaT the GP factor must reproduce the unary error
// and Jacobian at q1 and q2 respectively.
TEST(SelfCollisionSphereFactorGP, agreesWithUnaryFactorAtEndpoints) {
  const auto model = std::make_shared<const RobotQueryPoints>(
      kRobot, "columns", bothArmJoints(), wristPoints());
  const size_t dof = model->dof();

  const Vector q1 = supportConfig1(), q2 = supportConfig2();
  const Vector v1 = supportVelocity(), v2 = supportVelocity();
  const double deltaT = 0.5, costSigma = 0.1;

  std::vector<Point3> wPts;
  model->queryPoints(q1, &wPts);
  // Active by construction: eps sits a metre past the measured distance.
  const double eps = (wPts[0] - wPts[1]).norm() + 1.0;

  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, eps)};
  // Nonzero radii so the agreement also covers the radii in the standoff.
  const Vector radii = (Vector(2) << 0.05, 0.08).finished();
  SelfCollisionSphereFactor unary(X(0), model, pairs, radii, costSigma);

  // q1 passes through with unit weight at tau = 0, so H1 is the unary
  // Jacobian and the other support states get zero.
  SelfCollisionSphereFactorGP atStart(X(0), V(0), X(1), V(1), model, pairs,
                                       radii, costSigma,
                                       Isotropic::Sigma(dof, 1.0), deltaT,
                                       0.0);
  Matrix Hu, H1, H2, H3, H4;
  EXPECT(assert_equal(unary.evaluateError(q1, &Hu),
                      atStart.evaluateError(q1, v1, q2, v2, &H1, &H2, &H3,
                                            &H4),
                      1e-9));
  EXPECT(assert_equal(Hu, H1, 1e-9));
  const Matrix zero = Matrix::Zero(Hu.rows(), Hu.cols());
  EXPECT(assert_equal(zero, H2, 1e-9));
  EXPECT(assert_equal(zero, H3, 1e-9));
  EXPECT(assert_equal(zero, H4, 1e-9));

  // Likewise q2 at tau = deltaT, whose Jacobian lands in H3.
  SelfCollisionSphereFactorGP atEnd(X(0), V(0), X(1), V(1), model, pairs,
                                     radii, costSigma,
                                     Isotropic::Sigma(dof, 1.0), deltaT,
                                     deltaT);
  Matrix HuEnd, E1, E2, E3, E4;
  EXPECT(assert_equal(unary.evaluateError(q2, &HuEnd),
                      atEnd.evaluateError(q1, v1, q2, v2, &E1, &E2, &E3, &E4),
                      1e-9));
  EXPECT(assert_equal(HuEnd, E3, 1e-9));
  EXPECT(assert_equal(zero, E1, 1e-9));
  EXPECT(assert_equal(zero, E2, 1e-9));
  EXPECT(assert_equal(zero, E4, 1e-9));
}

/* ************************ interpolated Jacobians ********************** */

// At a tau strictly between the support states, with the hinge active, the
// analytic Jacobians w.r.t. all four support state variables must match the
// numerical ones.
TEST(SelfCollisionSphereFactorGP, interpolatedJacobians) {
  const auto model = std::make_shared<const RobotQueryPoints>(
      kRobot, "columns", bothArmJoints(), wristPoints());
  const size_t dof = model->dof();

  const Vector q1 = supportConfig1(), q2 = supportConfig2();
  const Vector v1 = supportVelocity(), v2 = supportVelocity();
  const double deltaT = 0.5, tau = 0.3 * deltaT;

  // Standoff a metre past the wrist distance at the interpolated state, so the
  // active branch is the one the numerical check exercises.
  GPLinearInterpolator interp(Isotropic::Sigma(dof, 1.0), deltaT, tau);
  std::vector<Point3> wPts;
  model->queryPoints(interp.interpolatePose(q1, v1, q2, v2), &wPts);
  const double eps = (wPts[0] - wPts[1]).norm() + 1.0;

  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, eps)};
  SelfCollisionSphereFactorGP factor(X(0), V(0), X(1), V(1), model, pairs,
                                     Vector::Zero(2), 0.1,
                                     Isotropic::Sigma(dof, 1.0), deltaT, tau);

  EXPECT(factor.evaluateError(q1, v1, q2, v2)(0) > 0.0);  // active branch

  Values values;
  values.insert(X(0), q1);
  values.insert(V(0), v1);
  values.insert(X(1), q2);
  values.insert(V(1), v2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************ per-point radii ***************************** */

// The radii fold into the standoff at the interpolated state exactly as they
// do in the unary factor: each sphere's radius adds to the active-branch cost.
TEST(SelfCollisionSphereFactorGP, radiiAddToTheStandoff) {
  const auto model = std::make_shared<const RobotQueryPoints>(
      kRobot, "columns", bothArmJoints(), wristPoints());
  const size_t dof = model->dof();

  const Vector q1 = supportConfig1(), q2 = supportConfig2();
  const Vector v1 = supportVelocity(), v2 = supportVelocity();
  const double deltaT = 0.5, tau = 0.3 * deltaT;

  GPLinearInterpolator interp(Isotropic::Sigma(dof, 1.0), deltaT, tau);
  std::vector<Point3> wPts;
  model->queryPoints(interp.interpolatePose(q1, v1, q2, v2), &wPts);
  // Active by construction: eps sits a metre past the measured distance.
  const double eps = (wPts[0] - wPts[1]).norm() + 1.0;

  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, eps)};
  SelfCollisionSphereFactorGP base(X(0), V(0), X(1), V(1), model, pairs,
                                   Vector::Zero(2), 0.1,
                                   Isotropic::Sigma(dof, 1.0), deltaT, tau);
  const double err0 = base.evaluateError(q1, v1, q2, v2)(0);
  EXPECT(err0 > 0.0);

  Vector radii(2);
  radii << 0.1, 0.2;
  SelfCollisionSphereFactorGP inflated(X(0), V(0), X(1), V(1), model, pairs,
                                       radii, 0.1, Isotropic::Sigma(dof, 1.0),
                                       deltaT, tau);
  EXPECT_DOUBLES_EQUAL(err0 + 0.3, inflated.evaluateError(q1, v1, q2, v2)(0),
                       1e-9);
}

/* ************************ input validation **************************** */

TEST(SelfCollisionSphereFactorGP, rejectsBadInput) {
  const auto model = std::make_shared<const RobotQueryPoints>(
      kRobot, "columns", bothArmJoints(), wristPoints());
  const size_t dof = model->dof();
  const auto QcModel = Isotropic::Sigma(dof, 1.0);
  const double deltaT = 0.5, tau = 0.1;

  // radii length must equal the number of query points.
  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, 1.0)};
  CHECK_EXCEPTION(
      SelfCollisionSphereFactorGP(X(0), V(0), X(1), V(1), model, pairs,
                                  Vector::Zero(3), 0.1, QcModel, deltaT, tau),
      std::invalid_argument);
  // a point index out of range.
  const std::vector<SelfCollisionPair> bad = {SelfCollisionPair(0, 99, 1.0)};
  CHECK_EXCEPTION(
      SelfCollisionSphereFactorGP(X(0), V(0), X(1), V(1), model, bad,
                                  Vector::Zero(2), 0.1, QcModel, deltaT, tau),
      std::invalid_argument);
  // sigmas must have one entry per pair.
  CHECK_EXCEPTION(
      SelfCollisionSphereFactorGP(X(0), V(0), X(1), V(1), model, pairs,
                                  Vector::Zero(2), Vector::Ones(3), QcModel,
                                  deltaT, tau),
      std::invalid_argument);
}

// A pair whose two spheres sit on the same rigid link has a constant separation
// and no gradient, so it is rejected here just as in the unary factor.
TEST(SelfCollisionSphereFactorGP, rejectsSameLinkPair) {
  const LinkSharedPtr link = kRobot.link("robot1_link_6");
  const std::vector<PointOnLink> points = {
      PointOnLink(link, Point3(0.0, 0.0, 0.0)),
      PointOnLink(link, Point3(0.1, 0.0, 0.0))};  // both on link_6
  const auto model = std::make_shared<const RobotQueryPoints>(
      kRobot, "columns", bothArmJoints(), points);
  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, 0.1)};

  CHECK_EXCEPTION(
      SelfCollisionSphereFactorGP(X(0), V(0), X(1), V(1), model, pairs,
                                  Vector::Zero(2), 0.1,
                                  Isotropic::Sigma(model->dof(), 1.0), 0.5, 0.1),
      std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
