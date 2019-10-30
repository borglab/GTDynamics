/**
 * @file  test_link.cpp
 * @brief test link
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <CppUnitLite/TestHarness.h>
#include <DhLink.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

static const Vector1 ZERO1 = Vector1::Zero();
static const Vector6 ZERO6 = Vector6::Zero();
DhLink dhLink = DhLink(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0),
                          (Matrix(3, 3) << 0, 0, 0, 0, 1 / 6.0, 0, 0, 0, 1 / 6.0).finished(), -5, 10, 2);

// Test factors for forward dynamics, middle link of stationary RRR example
TEST(Link, forward_factors) {
  // Create stationary state
  double v2 = 0;
  Vector6 twist_2 = ZERO6;
  double torque_2 = 0.0;

  // Create all factors
  Pose3 jTi = Pose3(Rot3(), Point3(-2, 0, 0));
  Pose3 kTj = Pose3(Rot3(), Point3(-2, 0, 0));
  GaussianFactorGraph factors =
      dhLink.forwardFactors(2, jTi, v2, twist_2, torque_2, kTj);
  EXPECT(assert_equal(factors.size(), 3));

  // Create ground truth values
  VectorValues ground_truth;
  ground_truth.insert(a(2), ZERO1);
  ground_truth.insert(T(1), ZERO6);
  ground_truth.insert(T(2), ZERO6);
  ground_truth.insert(F(2), ZERO6);
  ground_truth.insert(F(3), ZERO6);

  // // Assert that error is zero for ground-truth
  EXPECT(assert_equal(factors.error(ground_truth), 0));
}

// Test factors for forward dynamics, middle link of stationary RRR example
TEST(Link, inverse_factors) {
  // Create stationary state
  double v2 = 0;
  Vector6 twist_2 = ZERO6;
  double acceleration_2 = 0.0;

  // Create all factors
  Pose3 jTi = Pose3(Rot3(), Point3(-2, 0, 0));
  Pose3 kTj = Pose3(Rot3(), Point3(-2, 0, 0));
  GaussianFactorGraph factors =
      dhLink.inverseFactors(2, jTi, v2, twist_2, acceleration_2, kTj);
  EXPECT(assert_equal(factors.size(), 3));

  // // Create ground truth values
  // VectorValues ground_truth;
  // ground_truth.insert(t(2), ZERO1);
  // ground_truth.insert(T(1), ZERO6);
  // ground_truth.insert(T(2), ZERO6);
  // ground_truth.insert(F(2), ZERO6);
  // ground_truth.insert(F(3), ZERO6);

  // // // Assert that error is zero for ground-truth
  // EXPECT(assert_equal(factors.error(ground_truth), 0));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
