/**
 * @file  testCableVelocityFactor.cpp
 * @brief test cable vel factor
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include "factors/CableVelocityFactor.h"

#include <gtdynamics/utils/values.h>

#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gtdynamics;
using namespace gtdynamics::internal;

/**
 * Test cable factor
 */
TEST(CableVelocityFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);

  int jid = 0;
  int lid = 0;
  Point3 frameLoc = Point3(0.1, 0.2, 0.3);
  Point3 eeLoc = Point3(-0.15, 0, -0.15);

  CableVelocityFactor factor(JointVelKey(jid), PoseKey(lid), TwistKey(lid),
                             cost_model, frameLoc, eeLoc);

  Values values;
  InsertJointVel(&values, jid, 1.0);
  InsertPose(&values, lid, Pose3(Rot3(), Point3(1.5, 0, 1.5)));
  InsertTwist(&values, lid, (Vector6() << 0, 1, 0, 1.1, 0, 1.2).finished());

  // "expected" calculation
  Point3 dir = normalize(Point3(1.25, -0.2, 1.05));
  Vector3 expected_v = Point3(1.1 - 0.15, 0, 1.2 + 0.15);
  double expected_ldot = dot(dir, expected_v);
  Vector1 expected_errors{JointVel(values, jid) - expected_ldot};

  // evaluateError
  Vector1 actual_errors = factor.evaluateError(
      JointVel(values, jid), Pose(values, lid), Twist(values, lid));

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-4));

  // jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);

  values.update(PoseKey(lid),
                Pose3(Rot3::Rz(0.4), Point3(0.2, 0.3, 0.4)));
  values.update(TwistKey(lid),
                (Vector6() << 0.12, 0.13, 0.14, 0.18, 0.19, 0.21).finished());
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
