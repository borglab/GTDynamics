/**
 * @file  testCableLengthFactor.cpp
 * @brief test cable vel factor
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include "factors/CableLengthFactor.h"
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
TEST(CableLengthFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);

  int jid = 0;
  int lid = 0;
  Point3 frameLoc = Point3(0.1, 0.2, 0.3);
  Point3 eeLoc = Point3(-0.15, 0, -0.15);
  auto eeKey = PoseKey(lid);
  auto lKey = JointAngleKey(jid);

  CableLengthFactor factor(lKey, eeKey, cost_model, frameLoc, eeLoc);

  Values values;
  InsertJointAngle(&values, jid, 1.0);
  InsertPose(&values, lid, Pose3(Rot3(), Point3(1.5, 0, 1.5)));
  Vector1 expected_errors { 1.0 - sqrt(1.25*1.25 + 0.2*0.2 + 1.05*1.05) };

  Vector1 actual_errors =
      factor.evaluateError(JointAngle(values, jid), Pose(values, lid));

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-4));

  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);

  values.update(PoseKey(lid), Pose3(Rot3::Ry(1), Point3(1.2, 0, 2.1)));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
