/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/ContactWrenchFactor.h>
#include <gtdynamics/utils/ContactWrench.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <cmath>
#include <limits>
#include <random>

using namespace gtdynamics;
using namespace gtsam;

/* ************************************************************************* */
namespace contact_wrench {
const RectangularContactWrench cone(.1, .04, .7);
constexpr Key key = 12;

bool feasible(const Vector6& wrench, double tolerance = 1e-10) {
  return (cone.matrix() * wrench).maxCoeff() <= tolerance;
}

// A wrench frame change includes its lever-arm moment and preserves power.
TEST(ContactWrench, FramesAndVirtualPower) {
  const Pose3 comTcontact(Rot3::RzRyRx(.3, -.2, .7), Point3(.1, -.05, -.8));
  const Vector6 wrench = (Vector6() << 2, -3, 1, 10, 20, 100).finished();
  const Vector3 force = comTcontact.rotation().rotate(wrench.tail<3>());
  Vector6 expected;
  expected << comTcontact.rotation().rotate(wrench.head<3>()) +
                  comTcontact.translation().cross(force),
      force;
  EXPECT(
      assert_equal(expected, contactWrenchToCom(comTcontact, wrench), 1e-12));

  const Vector6 comTwist = (Vector6() << .2, -.3, .1, 1, 2, 3).finished();
  const Vector6 contactTwist = comTcontact.inverse().AdjointMap() * comTwist;
  DOUBLES_EQUAL(wrench.dot(contactTwist), expected.dot(comTwist), 1e-10);
  EXPECT(assert_equal(
      wrench, contactWrenchToCom(comTcontact.inverse(), expected), 1e-10));
}

// A transformed sole wrench balances a COM gravity load and has exact
// Jacobians.
TEST(ContactWrench, GravityAndFactorJacobians) {
  const Pose3 comTcontact(Rot3(), Point3(0, 0, -.8));
  const Vector6 soleWrench = (Vector6() << 0, 0, 0, 0, 0, 98.1).finished();
  const Vector6 gravityWrench = -soleWrench;
  const Vector6 netWrench =
      contactWrenchToCom(comTcontact, soleWrench) + gravityWrench;
  EXPECT(netWrench.isZero(1e-12));
  EXPECT(feasible(soleWrench));

  const Pose3 offset(Rot3::RzRyRx(.2, -.4, .8), Point3(.1, .2, -.7));
  ContactWrenchFactor factor(1, 2, offset, noiseModel::Isotropic::Sigma(6, 1));
  const Vector6 comWrench = contactWrenchToCom(offset, soleWrench);
  Matrix HContact, HCom;
  EXPECT(assert_equal(
      Vector6::Zero(),
      factor.evaluateError(soleWrench, comWrench, &HContact, &HCom), 1e-12));
  const std::function<Vector6(const Vector6&, const Vector6&)> error =
      [&](const Vector6& first, const Vector6& second) {
        return factor.evaluateError(first, second);
      };
  EXPECT(assert_equal(numericalDerivative21(error, soleWrench, comWrench),
                      HContact, 1e-8));
  EXPECT(assert_equal(numericalDerivative22(error, soleWrench, comWrench), HCom,
                      1e-8));
}

// Normal, friction, CoP, and zero-load signs reject every nonphysical
// direction.
TEST(ContactWrench, NormalFrictionAndCenterOfPressure) {
  EXPECT(feasible(Vector6::Zero()));
  EXPECT(!feasible((Vector6() << 0, 0, 0, 0, 0, -1).finished()));
  Vector6 wrench = (Vector6() << 0, 0, 0, 0, 0, 100).finished();
  const double mu = cone.pyramidCoefficient();
  for (int axis = 0; axis < 6; ++axis) {
    if (axis == 5) continue;
    Vector6 unloaded = Vector6::Zero();
    unloaded(axis) = 1;
    EXPECT(!feasible(unloaded));
    EXPECT(!feasible(-unloaded));
  }
  for (double sign : {-1., 1.}) {
    for (int axis : {3, 4}) {
      Vector6 tangent = wrench;
      tangent(axis) = sign * mu * wrench(5);
      EXPECT(feasible(tangent));
      EXPECT(std::hypot(tangent(3), tangent(4)) <= .7 * tangent(5));
      tangent(axis) *= 1.01;
      EXPECT(!feasible(tangent));
    }
    Vector6 roll = wrench, pitch = wrench;
    roll(0) = sign * cone.halfWidth() * wrench(5);
    pitch(1) = sign * cone.halfLength() * wrench(5);
    EXPECT(feasible(roll));
    EXPECT(feasible(pitch));
    roll(0) *= 1.01;
    pitch(1) *= 1.01;
    EXPECT(!feasible(roll));
    EXPECT(!feasible(pitch));
  }
}

// Coupled yaw limits collapse at saturated tangential force or a CoP corner.
TEST(ContactWrench, CoupledYawBounds) {
  const double mu = cone.pyramidCoefficient();
  Vector6 wrench = (Vector6() << 0, 0, 0, 0, 0, 100).finished();
  const double yawLimit = mu * (cone.halfLength() + cone.halfWidth()) * 100;
  for (const double sign : {-1., 1.}) {
    wrench(2) = sign * yawLimit;
    EXPECT(feasible(wrench));
    wrench(2) *= 1.01;
    EXPECT(!feasible(wrench));
  }
  wrench << 0, 0, 0, mu * 100, mu * 100, 100;
  EXPECT(feasible(wrench));
  for (const double sign : {-1., 1.}) {
    wrench(2) = sign * .01;
    EXPECT(!feasible(wrench));
  }
  wrench << cone.halfWidth() * 100, cone.halfLength() * 100, 0, 0, 0, 100;
  EXPECT(feasible(wrench));
  wrench(2) = .01;
  EXPECT(!feasible(wrench));
}

// Random corner forces and direct absolute-value bounds verify all eight yaw
// faces.
TEST(ContactWrench, CornerForcesAndYawFormula) {
  const double x = cone.halfLength(), y = cone.halfWidth();
  const double mu = cone.pyramidCoefficient();
  std::mt19937 generator(12);
  std::uniform_real_distribution<double> uniform(-1, 1);
  for (int trial = 0; trial < 200; ++trial) {
    Vector6 wrench = Vector6::Zero();
    for (const double signX : {-1., 1.}) {
      for (const double signY : {-1., 1.}) {
        const double normal = 50 * (uniform(generator) + 1);
        const Vector3 force(mu * normal * uniform(generator),
                            mu * normal * uniform(generator), normal);
        wrench.head<3>() += Vector3(signX * x, signY * y, 0).cross(force);
        wrench.tail<3>() += force;
      }
    }
    EXPECT(feasible(wrench));
    const double lower = -mu * (x + y) * wrench(5) +
                         std::abs(y * wrench(3) - mu * wrench(0)) +
                         std::abs(x * wrench(4) - mu * wrench(1));
    const double upper = mu * (x + y) * wrench(5) -
                         std::abs(y * wrench(3) + mu * wrench(0)) -
                         std::abs(x * wrench(4) + mu * wrench(1));
    const Vector faces = cone.matrix() * wrench;
    DOUBLES_EQUAL(wrench(2) - upper, faces.segment(9, 4).maxCoeff(), 1e-10);
    DOUBLES_EQUAL(lower - wrench(2), faces.segment(13, 4).maxCoeff(), 1e-10);
    wrench(2) = upper + 1e-5;
    EXPECT(!feasible(wrench));
    wrench(2) = lower - 1e-5;
    EXPECT(!feasible(wrench));
  }
}

// Every scalar inequality and blocking equality retains its signed linear row.
TEST(ContactWrench, ScalarFacesAndJacobians) {
  const auto constraints = cone.constraints(key);
  const Matrix faces = cone.matrix();
  EXPECT_LONGS_EQUAL(17, constraints.size());
  const Vector6 wrench = (Vector6() << 1, -2, .3, 5, 10, 100).finished();
  Values values;
  values.insert(key, Vector(wrench));
  for (size_t row = 0; row < constraints.size(); ++row) {
    const Matrix16 expected = faces.row(row) / faces.row(row).norm();
    std::vector<Matrix> jacobians(1), boundaryJacobians(1);
    const auto& constraint = constraints[row];
    const double value = (expected * wrench)(0);
    DOUBLES_EQUAL(value, constraint->unwhitenedExpr(values, &jacobians)(0),
                  1e-12);
    EXPECT(assert_equal(expected, jacobians[0], 1e-12));
    const auto boundary = constraint->createEqualityConstraint();
    DOUBLES_EQUAL(
        value, boundary->unwhitenedError(values, &boundaryJacobians)(0), 1e-12);
    EXPECT(assert_equal(expected, boundaryJacobians[0], 1e-12));
    Values violated;
    violated.insert(key, Vector(expected.transpose()));
    EXPECT(constraint->unwhitenedExpr(violated)(0) > 0);
    Values feasible;
    feasible.insert(key, Vector(-expected.transpose()));
    EXPECT(constraint->unwhitenedExpr(feasible)(0) < 0);
  }
}

// Invalid dimensions, friction, and six-dimensional factor noise are rejected.
TEST(ContactWrench, InvalidConfiguration) {
  const double invalid[] = {0., -1., std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()};
  for (const double value : invalid) {
    for (int axis = 0; axis < 3; ++axis) {
      bool threw = false;
      try {
        RectangularContactWrench candidate(axis == 0 ? value : .1,
                                           axis == 1 ? value : .04,
                                           axis == 2 ? value : .7);
      } catch (const std::invalid_argument&) {
        threw = true;
      }
      EXPECT(threw);
    }
  }
  bool threw = false;
  try {
    ContactWrenchFactor factor(1, 2, Pose3(),
                               noiseModel::Isotropic::Sigma(3, 1));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  EXPECT(threw);
}
}  // namespace contact_wrench
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
