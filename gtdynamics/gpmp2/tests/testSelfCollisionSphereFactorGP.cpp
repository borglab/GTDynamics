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
// Constant velocity consistent with the two support states over delta_t = 0.5.
static Vector supportVelocity() {
  Vector v = Vector::Zero(18);
  v(0) = 2.0;
  v(9) = -2.0;
  return v;
}

/* ************************ endpoint agreement ************************** */

// At tau = 0 the interpolation reproduces the first support state exactly, so
// the interpolated factor must agree with the unary factor at q1; likewise at
// tau = delta_t with q2.
TEST(SelfCollisionSphereFactorGP, agreesWithUnaryFactorAtEndpoints) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), wristPoints());
  const size_t dof = model.dof();

  const Vector q1 = supportConfig1(), q2 = supportConfig2();
  const Vector v1 = supportVelocity(), v2 = supportVelocity();
  const double delta_t = 0.5, cost_sigma = 0.1;

  std::vector<Point3> wPs;
  model.queryPoints(q1, &wPs);
  const double eps = (wPs[0] - wPs[1]).norm() + 1.0;  // active by construction

  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, eps)};
  const Vector radii = Vector::Zero(2);
  SelfCollisionSphereFactor unary(X(0), model, pairs, radii, cost_sigma);

  SelfCollisionSphereFactorGP at_start(X(0), V(0), X(1), V(1), model, pairs,
                                       radii, cost_sigma,
                                       Isotropic::Sigma(dof, 1.0), delta_t,
                                       0.0);
  EXPECT(assert_equal(unary.evaluateError(q1),
                      at_start.evaluateError(q1, v1, q2, v2), 1e-9));

  SelfCollisionSphereFactorGP at_end(X(0), V(0), X(1), V(1), model, pairs,
                                     radii, cost_sigma,
                                     Isotropic::Sigma(dof, 1.0), delta_t,
                                     delta_t);
  EXPECT(assert_equal(unary.evaluateError(q2),
                      at_end.evaluateError(q1, v1, q2, v2), 1e-9));
}

/* ************************ interpolated Jacobians ********************** */

// At a tau strictly between the support states, with the hinge active, the
// analytic Jacobians w.r.t. all four support state variables must match the
// numerical ones.
TEST(SelfCollisionSphereFactorGP, interpolatedJacobians) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), wristPoints());
  const size_t dof = model.dof();

  const Vector q1 = supportConfig1(), q2 = supportConfig2();
  const Vector v1 = supportVelocity(), v2 = supportVelocity();
  const double delta_t = 0.5, tau = 0.3 * delta_t;

  // Standoff a metre past the wrist distance at the interpolated state, so the
  // active branch is the one the numerical check exercises.
  GPLinearInterpolator interp(Isotropic::Sigma(dof, 1.0), delta_t, tau);
  std::vector<Point3> wPs;
  model.queryPoints(interp.interpolatePose(q1, v1, q2, v2), &wPs);
  const double eps = (wPs[0] - wPs[1]).norm() + 1.0;

  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, eps)};
  SelfCollisionSphereFactorGP factor(X(0), V(0), X(1), V(1), model, pairs,
                                     Vector::Zero(2), 0.1,
                                     Isotropic::Sigma(dof, 1.0), delta_t, tau);

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
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), wristPoints());
  const size_t dof = model.dof();

  const Vector q1 = supportConfig1(), q2 = supportConfig2();
  const Vector v1 = supportVelocity(), v2 = supportVelocity();
  const double delta_t = 0.5, tau = 0.3 * delta_t;

  GPLinearInterpolator interp(Isotropic::Sigma(dof, 1.0), delta_t, tau);
  std::vector<Point3> wPs;
  model.queryPoints(interp.interpolatePose(q1, v1, q2, v2), &wPs);
  const double eps = (wPs[0] - wPs[1]).norm() + 1.0;  // active by construction

  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, eps)};
  SelfCollisionSphereFactorGP base(X(0), V(0), X(1), V(1), model, pairs,
                                   Vector::Zero(2), 0.1,
                                   Isotropic::Sigma(dof, 1.0), delta_t, tau);
  const double e0 = base.evaluateError(q1, v1, q2, v2)(0);
  EXPECT(e0 > 0.0);

  Vector radii(2);
  radii << 0.1, 0.2;
  SelfCollisionSphereFactorGP inflated(X(0), V(0), X(1), V(1), model, pairs,
                                       radii, 0.1, Isotropic::Sigma(dof, 1.0),
                                       delta_t, tau);
  EXPECT_DOUBLES_EQUAL(e0 + 0.3, inflated.evaluateError(q1, v1, q2, v2)(0),
                       1e-9);
}

/* ************************ input validation **************************** */

TEST(SelfCollisionSphereFactorGP, rejectsBadInput) {
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), wristPoints());
  const size_t dof = model.dof();
  const auto Qc_model = Isotropic::Sigma(dof, 1.0);
  const double delta_t = 0.5, tau = 0.1;

  // radii length must equal the number of query points.
  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, 1.0)};
  CHECK_EXCEPTION(
      SelfCollisionSphereFactorGP(X(0), V(0), X(1), V(1), model, pairs,
                                  Vector::Zero(3), 0.1, Qc_model, delta_t, tau),
      std::invalid_argument);
  // a point index out of range.
  const std::vector<SelfCollisionPair> bad = {SelfCollisionPair(0, 99, 1.0)};
  CHECK_EXCEPTION(
      SelfCollisionSphereFactorGP(X(0), V(0), X(1), V(1), model, bad,
                                  Vector::Zero(2), 0.1, Qc_model, delta_t, tau),
      std::invalid_argument);
  // sigmas must have one entry per pair.
  CHECK_EXCEPTION(
      SelfCollisionSphereFactorGP(X(0), V(0), X(1), V(1), model, pairs,
                                  Vector::Zero(2), Vector::Ones(3), Qc_model,
                                  delta_t, tau),
      std::invalid_argument);
}

// A pair whose two spheres sit on the same rigid link has a constant separation
// and no gradient, so it is rejected here just as in the unary factor.
TEST(SelfCollisionSphereFactorGP, rejectsSameLinkPair) {
  const LinkSharedPtr link = kRobot.link("robot1_link_6");
  const std::vector<PointOnLink> points = {
      PointOnLink(link, Point3(0.0, 0.0, 0.0)),
      PointOnLink(link, Point3(0.1, 0.0, 0.0))};  // both on link_6
  RobotQueryPoints model(kRobot, "columns", bothArmJoints(), points);
  const std::vector<SelfCollisionPair> pairs = {SelfCollisionPair(0, 1, 0.1)};

  CHECK_EXCEPTION(
      SelfCollisionSphereFactorGP(X(0), V(0), X(1), V(1), model, pairs,
                                  Vector::Zero(2), 0.1,
                                  Isotropic::Sigma(model.dof(), 1.0), 0.5, 0.1),
      std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
