/**
 * @file   testGeneralPriorFactor.cpp
 * @brief  Test GeneralPriorFactor
 * @author Frank Dellaert
 * @date   Nov 4, 2014
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/geometry/Pose3.h>
#include <gtdynamics/factors/GeneralPriorFactor.h>

using namespace std;
using namespace std::placeholders;
using namespace gtsam;
using namespace gtdynamics;


// /* ************************************************************************* */
// TEST(GeneralPriorFactor, ConstructorPose3) {
//   Key x_key = 1;
//   Values values;
//   values.insert(x_key, Pose3(Rot3::Rx(0.5), Point3(1, 2, 3)));
  
//   SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 1.0);
//   GeneralPriorFactor factor(x_key, values.at(x_key), model);

//   // Check Error and Jacobians
//   Vector6 expected_error = Vector::Zero(6);
//   EXPECT(assert_equal(expected_error,
//                     factor.unwhitenedError(values), 1e-6));
//   EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

//   Values values1;
//   values1.insert(x_key, Pose3(Rot3::Rx(0.6), Point3(1, 3, 3)));
//   EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values1, 1e-7, 1e-5);
// }

/* ************************************************************************* */
TEST(GeneralPriorFactor, ConstructorPoint3) {
  Key x_key = 1;
  Values values;
  values.insert(x_key, Point3(1, 2, 3));
  
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  GeneralPriorFactor factor(x_key, values.at(x_key), model);

  // Check Error and Jacobians
  Vector3 expected_error = Vector::Zero(3);
  EXPECT(assert_equal(expected_error,
                    factor.unwhitenedError(values), 1e-6));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

  Values values1;
  values1.insert(x_key, Point3(1, 3, 3));
  Vector3 expected_error1 = (Vector(3) << 0, 1, 0).finished();
  EXPECT(assert_equal(expected_error1,
                    factor.unwhitenedError(values1), 1e-6));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values1, 1e-7, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
