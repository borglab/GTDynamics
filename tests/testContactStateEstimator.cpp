/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactStateEstimator.cpp
 * @brief Tests for contact based state estimation.
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include "gtdynamics/universal_robot/Robot.h"

using namespace gtdynamics;
using gtsam::assert_equal;

class ContactStateEstimator {
 private:
 public:
  ContactStateEstimator() {}
};

// Test one invocation of the estimator
TEST(ContactStateEstimator, Constructor) {
  Robot robot = CreateRobotFromFile();
  ContactStateEstimator estimator(robot);

  // Create an initial estimate given the joint angles, assuming all feet on the
  // ground. The constructor below will create the continuous and the discrete
  // states for the initial timestep 0.
  ContactStateEstimator::Estimate estimate_0 =
      estimator.initialize(joint_angles_0);

  // Measurement object containing IMU measurement between state 0 and 1, and
  // joint angles at timestep 1.
  ContactStateEstimator::Measurement measurements_01(imu_measurements,
                                                     joint_angles_1);

  // Invoke the fixed lag smoother
  ContactStateEstimator::Estimate estimate_1 =
      estimator.run(estimate_0, measurements_01);

  // State sequence for fixed lag smoothing from previous invocation
  Values expected_continuous_states_0_to_1;

  // Sequence of discrete contact states
  DiscreteFactor::Values expected_contact_sequence;
  expected_contact_sequence[C(0)] =
      15;  // All feet on the ground at initial time (1111)
  expected_contact_sequence[C(1)] = 11;  // Foot 2 lifted off the ground (1011)

  // Compare expected sequence with the actual sequence.
  EXPECT(assert_equal(expected_continuous_states_0_to_1,
                      estimate_1.continuousStates()));
  EXPECT(assert_equal(expected_contact_sequence, estimate_1.contactSequence()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
