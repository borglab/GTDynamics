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
#include <gtsam/base/Vector.h>
#include <gtsam/linear/Sampler.h>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

// Create a slice.
static const size_t k = 777;
static const Slice kSlice(k);

TEST(Slice, InverseKinematics) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  // Create initial values
  auto values = kinematics.initialValues(kSlice, robot, 0.0);
  EXPECT_LONGS_EQUAL(13 + 12, values.size());

  // Set twists to zero for FK. TODO(frank): separate kinematics from velocity?
  for (auto&& link : robot.links()) {
    InsertTwist(&values, link->id(), k, gtsam::Z_6x1);
  }

  // Do forward kinematics
  const std::string root_link_name("body");
  const auto root_link_id = robot.link(root_link_name)->id();
  EXPECT(values.exists(PoseKey(root_link_id, k)));
  auto fk = robot.forwardKinematics(values, k, std::string(root_link_name));

  // Check goals with FK solution
  for (const ContactGoal& goal : contact_goals) {
    EXPECT(goal.satisfied(fk, k, 0.05));
  }

  auto graph = kinematics.graph(kSlice, robot);
  EXPECT_LONGS_EQUAL(12, graph.size());

  auto objectives = kinematics.pointGoalObjectives(kSlice, contact_goals);
  EXPECT_LONGS_EQUAL(4, objectives.size());

  auto objectives2 = kinematics.jointAngleObjectives(kSlice, robot);
  EXPECT_LONGS_EQUAL(12, objectives2.size());

  auto result = kinematics.inverse(kSlice, robot, contact_goals);

  // Check that well-determined
  graph.add(objectives);
  graph.add(objectives2);
  EXPECT_LONGS_EQUAL(12 + 12 + 4, graph.size());
  // auto factor = graph.linearizeToHessianFactor(result);
  // GTD_PRINT(*factor);

  // Check that goals are achieved
  constexpr double tol = 1e-5;
  for (const ContactGoal& goal : contact_goals) {
    EXPECT(goal.satisfied(result, k, tol));
  }
}

gtsam::Values jointVectorToValues(const Robot& robot,
                                  const gtsam::Vector7& joints) {
  gtsam::Values joint_values;
  for (auto&& j : robot.joints()) {
    size_t joint_id = j->id();
    joint_values.insert(JointAngleKey(joint_id, k), joints(joint_id));
  }

  return joint_values;
}

TEST(Slice, initialValues) {
  // Load robot from urdf file
  const Robot panda =
      CreateRobotFromFile(kUrdfPath + std::string("panda/panda.urdf"));
  const Robot robot = panda.fixLink("link0");

  Kinematics kinematics;

  // Create Values where initial joint angles are stored
  gtsam::Vector7 initial;
  initial << 0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7;
  gtsam::Values initial_joints = jointVectorToValues(robot, initial);

  gtsam::Values initial_values =
      kinematics.initialValues(kSlice, robot, 0.0, initial_joints, true);

  // We should only have 7 values for joints and 8 for link poses
  EXPECT_LONGS_EQUAL(15, initial_values.size())

  // check that joint angles are the same
  gtsam::Vector7 actual_initial;
  for (size_t j = 0; j < 7; j++)
    actual_initial[j] = initial_values.at<double>(JointAngleKey(j, k));
  double tol = 1e-5;
  EXPECT(assert_equal(initial, actual_initial, tol))

  // check that the last fk is the same
  gtsam::Rot3 sR7{{0.98161623, 0.13223102, -0.13763916},
                  {0.08587125, -0.9499891, -0.30024463},
                  {-0.17045735, 0.28290575, -0.94387956}};
  gtsam::Pose3 sT7(sR7, Point3(0.323914, 0.167266, 0.905973));
  EXPECT(assert_equal(sT7, initial_values.at<gtsam::Pose3>(PoseKey(7, k)), tol))
}

TEST(Slice, JointAngleObjectives) {
  const Robot panda =
      CreateRobotFromFile(kUrdfPath + std::string("panda/panda.urdf"));
  const Robot robot = panda.fixLink("link0");

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  // Priors with means at 0
  auto joint_priors = kinematics.jointAngleObjectives(kSlice, robot);
  EXPECT_LONGS_EQUAL(7, joint_priors.size())

  // Check that error from priors evaluated at 0 is 0
  gtsam::Vector7 means_vector;
  means_vector << 0, 0, 0, 0, 0, 0, 0;
  gtsam::Values expected_means = jointVectorToValues(robot, means_vector);
  gtsam::Values initial =
      kinematics.initialValues(kSlice, robot, 0.0, expected_means, true);
  double tol = 1e-5;
  EXPECT_DOUBLES_EQUAL(0.0, joint_priors.error(initial), tol)

  // Define some prior means different than 0
  gtsam::Values means;
  means.insert(JointAngleKey(0, k), 1.0);
  means.insert(JointAngleKey(2, k), 1.0);
  means.insert(JointAngleKey(4, k), 1.0);
  means.insert(JointAngleKey(6, k), 1.0);
  joint_priors = kinematics.jointAngleObjectives(kSlice, robot, means);
  EXPECT_LONGS_EQUAL(7, joint_priors.size())

  // check that error at 0 is now not 0
  initial = kinematics.initialValues(kSlice, robot, 0.0, expected_means, true);
  EXPECT(tol < joint_priors.error(initial))

  // Check that the evaluated error at the expected means is 0
  means_vector << 1, 0, 1, 0, 1, 0, 1;
  expected_means = jointVectorToValues(robot, means_vector);
  initial = kinematics.initialValues(kSlice, robot, 0.0, expected_means, true);
  EXPECT_DOUBLES_EQUAL(0.0, joint_priors.error(initial), tol)

  // Define means of all joints different than 0
  means.insert(JointAngleKey(1, k), 0.5);
  means.insert(JointAngleKey(3, k), -1.0);
  means.insert(JointAngleKey(5, k), 0.5);
  joint_priors = kinematics.jointAngleObjectives(kSlice, robot, means);
  EXPECT_LONGS_EQUAL(7, joint_priors.size())

  // Check that the evaluated error at the expected means is 0
  means_vector << 1, 0.5, 1, -1, 1, 0.5, 1;
  expected_means = jointVectorToValues(robot, means_vector);
  initial = kinematics.initialValues(kSlice, robot, 0.0, expected_means, true);
  EXPECT_DOUBLES_EQUAL(0.0, joint_priors.error(initial), tol)
}

TEST(Slice, jointAngleLimits) {
  const Robot panda =
      CreateRobotFromFile(kUrdfPath + std::string("panda/panda.urdf"));
  const Robot robot = panda.fixLink("link0");

  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  auto jointLimits = kinematics.jointAngleLimits(kSlice, robot);

  // get joint limits
  gtsam::Vector7 lower_limits, upper_limits;
  for (auto&& joint : robot.joints()) {
    auto scalar_values = joint->parameters().scalar_limits;
    lower_limits(joint->id()) = scalar_values.value_lower_limit;
    upper_limits(joint->id()) = scalar_values.value_upper_limit;
  }
  auto ones = gtsam::Vector7::Ones();

  const double tol = 1e-5;
  // if lower than lower_limit, error must be greater than 0
  auto values = jointVectorToValues(robot, lower_limits - 0.1 * ones);
  EXPECT(tol < jointLimits.error(values))

  // if inside the limits, the error must be 0
  values = jointVectorToValues(robot, (lower_limits + upper_limits) / 2);
  EXPECT_DOUBLES_EQUAL(0.0, jointLimits.error(values), tol)

  // if upper than upper_limit, error must be greater than 0
  values = jointVectorToValues(robot, upper_limits + 0.1 * ones);
  EXPECT(tol < jointLimits.error(values))
}

TEST(Slice, PoseGoalObjectives) {
  const Robot panda =
      CreateRobotFromFile(kUrdfPath + std::string("panda/panda.urdf"));
  const Robot robot = panda.fixLink("link0");

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  // Add prior to pose
  gtsam::Rot3 sR7{{0.98161623, 0.13223102, -0.13763916},
                  {0.08587125, -0.9499891, -0.30024463},
                  {-0.17045735, 0.28290575, -0.94387956}};
  gtsam::Pose3 sT7(sR7, Point3(0.323914, 0.167266, 0.905973));
  PoseGoals pose_goals = {
      {k, PoseGoal(robot.links()[7], gtsam::Pose3(), sT7)}};
  auto pose_priors = kinematics.poseGoalObjectives(kSlice, pose_goals);

  gtsam::Vector7 initial;
  initial << 0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7;
  gtsam::Values initial_joints = jointVectorToValues(robot, initial);
  auto initial_values =
      kinematics.initialValues(kSlice, robot, 0.0, initial_joints, true);
  double tol = 1e-4;
  GTSAM_PRINT(initial_values.at<gtsam::Pose3>(PoseKey(7, k)));
  EXPECT(assert_equal(sT7, initial_values.at<gtsam::Pose3>(PoseKey(7, k)), tol))
  EXPECT_DOUBLES_EQUAL(0, pose_priors.error(initial_values), tol)
}

TEST(Slice, panda_constraints) {
  const Robot panda =
      CreateRobotFromFile(kUrdfPath + std::string("panda/panda.urdf"));
  const Robot robot = panda.fixLink("link0");

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  // We should only have 7 constraints plus one for the fixed link
  auto constraints = kinematics.constraints(kSlice, robot);
  EXPECT_LONGS_EQUAL(8, constraints.size());
}

TEST(Slice, PandaIK) {
  using gtsam::Pose3;
  using gtsam::Rot3;
  using gtsam::Values;
  using gtsam::Vector7;

  const Robot panda =
      CreateRobotFromFile(kUrdfPath + std::string("panda/panda.urdf"));
  const Robot robot = panda.fixLink("link0");

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  // We should only have 7 unknown joint angles and That is still 7 factors.
  auto graph = kinematics.graph(kSlice, robot);
  EXPECT_LONGS_EQUAL(7, graph.size());

  // Define the goal pose and add it to a values container
  // This is the FK solution of {0.1,0.2,0.3,-0.4,0.5,0.6,0.7}
  Rot3 sR7{{0.98161623, 0.13223102, -0.13763916},
           {0.08587125, -0.9499891, -0.30024463},
           {-0.17045735, 0.28290575, -0.94387956}};
  Pose3 sT7(sR7, Point3(0.323914, 0.167266, 0.905973));
  PoseGoals pose_goals = {
      {k, PoseGoal(robot.links()[7], gtsam::Pose3(), sT7)}};
  
  // Give a noisy estimate of the original point
  Vector7 initial, noise;
  initial << 0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7;
  noise << 0.04, -0.1, 0.07, 0.14, -0.05, 0.02, 0.1;
  gtsam::Values initial_joints = jointVectorToValues(robot, noise + initial);

  // Call IK solver
  auto values = kinematics.inverse(kSlice, robot, pose_goals, initial_joints);

  // Check that base link did not budge (much)
  auto base_link = robot.link("link0");
  const Pose3 sM0 = base_link->bMcom();
  double tol = 1e-5;
  EXPECT(assert_equal(sM0, values.at<Pose3>(PoseKey(0, k)), tol));

  // Check that desired pose was achieved
  EXPECT(assert_equal(sT7, values.at<Pose3>(PoseKey(7, k)), tol));

  // Check that joint angles are not too different
  Vector7 optimal_q;
  for (size_t j = 0; j < 7; j++)
    optimal_q[j] = values.at<double>(JointAngleKey(j, k));
  tol = 0.1;
  EXPECT(assert_equal(initial, optimal_q, tol));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
