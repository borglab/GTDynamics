/**
 * @file  testCableAccelerationFactor.cpp
 * @brief test cable accel factor
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include <gtdynamics/cablerobot/factors/CableAccelerationFactor.h>

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
TEST(CableAccelerationFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);

  int jid = 0;
  int lid = 0;
  Point3 frameLoc = Point3(0.1, 0.2, 0.3);
  Point3 eeLoc = Point3(-0.15, 0, -0.15);

  CableAccelerationFactor factor(JointAccelKey(jid), PoseKey(lid),
                                 TwistKey(lid), TwistAccelKey(lid), cost_model,
                                 frameLoc, eeLoc);

  // Test configuration
  Values values;
  InsertJointAccel(&values, jid, 1.0);
  InsertPose(&values, lid, Pose3(Rot3(), Point3(1.5, 0, 1.5)));
  InsertTwist(&values, lid, (Vector6() << 0, 1, 0, 1.1, 0, 1.2).finished());
  InsertTwistAccel(&values, lid,
                   (Vector6() << 0, 1.3, 0, 1.4, 0, 1.5).finished());

  // "expected" calculation
  Vector3 dir = normalize(Point3(1.25, -.2, 1.05));
  Vector3 linaccel = Vector3(1.4, 0, 1.5);  // translational part of VA
  Vector3 rotaccel = 1.3 * normalize(Vector3(-0.15, 0, 0.15));  // rot of VA
  Vector3 coraccel =
      0.9 * 0.9 * 0.15 * sqrt(2) * normalize(-eeLoc);  // centripet
  double expected_lddot = dot(dir, linaccel + rotaccel + coraccel);
  Vector1 expected_errors{JointAccel(values, jid) - expected_lddot};

  // evaluateError
  Vector1 actual_errors =
      factor.evaluateError(JointAccel(values, jid), Pose(values, lid),
                           Twist(values, lid), TwistAccel(values, lid));

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-4));

  // jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
  values.update(PoseKey(lid), Pose3(Rot3::Rz(0.4), Point3(0.2, 0.3, 0.4)));
  values.update(TwistKey(lid),
                (Vector6() << 0.12, 0.13, 0.14, 0.18, 0.19, 0.21).finished());
  values.update(TwistAccelKey(lid),
                (Vector6() << 0.21, 0.22, 0.23, 0.24, 0.25, 0.26).finished());
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
