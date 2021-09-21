/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testKinematicsSlice.cpp
 * @brief Test Kinematics in single time slice.
 * @author: Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Slice.h>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::noiseModel::Isotropic;
using gtsam::noiseModel::Constrained;
using std::map;
using std::string;

TEST(Phase, InverseKinematics) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  // Create a slice.
  const size_t k = 777;
  const Slice slice(k);

  // Instantiate kinematics algorithms
  auto parameters = boost::make_shared<KinematicsParameters>();
  parameters->p_cost_model = Constrained::All(6, 1e-4);
  parameters->g_cost_model = Constrained::All(3, 0.01);
  parameters->prior_q_cost_model = Isotropic::Sigma(1, 0.5);
  Kinematics kinematics(parameters);

  // Create initial values
  auto values = kinematics.initialValues(slice, robot, 0.0);
  EXPECT_LONGS_EQUAL(13 + 12, values.size());

  // Set twists to zero for FK. TODO(frank): separate kinematics from velocity?
  for (auto&& link : robot.links()) {
    InsertTwist(&values, link->id(), k, gtsam::Z_6x1);
  }

  // Do forward kinematics
  const std::string root_link_name("body");
  const auto root_link_id = robot.link(root_link_name)->id();
  EXPECT(values.exists(internal::PoseKey(root_link_id, k)));
  auto fk = robot.forwardKinematics(values, k, std::string(root_link_name));

  // Check goals with FK solution
  for (const ContactGoal& goal : contact_goals) {
    EXPECT(goal.satisfied(fk, k, 0.05));
  }

  auto result = kinematics.inverseConstrained(slice, robot, contact_goals);


  // Check that goals are achieved
  constexpr double tol = 1e-5;
  for (const ContactGoal& goal : contact_goals) {
    EXPECT(goal.satisfied(result, k, tol));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
