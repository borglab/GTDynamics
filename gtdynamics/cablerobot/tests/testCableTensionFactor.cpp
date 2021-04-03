/**
 * @file  testCableTensionFactor.cpp
 * @brief test cable tension factor
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include "factors/CableTensionFactor.h"

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
TEST(CableTensionFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(6, 1.0);

  int jid = 0;
  int lid = 0;
  Point3 frameLoc = Point3(0, 0, 0);
  Point3 eeLoc = Point3(-0.15, 0, -0.15);
  CableTensionFactor factor(TorqueKey(jid), PoseKey(lid), WrenchKey(lid, jid),
                            cost_model, frameLoc, eeLoc);

  Values values;
  InsertTorque(&values, jid, 1.0);
  InsertPose(&values, lid, Pose3(Rot3(), Point3(1.5, 0, 1.5)));
  InsertWrench(&values, lid, jid,
               (Vector6() << 0, 1, 0, 1.1, 0, 1.2).finished());

  Vector6 expected_errors =
      (Vector6() << 0, 1, 0, 1.1 + 1 / sqrt(2), 0, 1.2 + 1 / sqrt(2))
          .finished();

  Vector6 actual_errors = factor.evaluateError(
      Torque(values, jid), Pose(values, lid), Wrench(values, lid, jid));

  // jacobians
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-4));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);

  values.update(PoseKey(lid), Pose3(Rot3::Rz(0.4), Point3(0.2, 0.3, 0.4)));
  values.update(WrenchKey(lid, jid),
                (Vector6() << 0.12, 0.13, 0.14, 0.18, 0.19, 0.21).finished());
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}