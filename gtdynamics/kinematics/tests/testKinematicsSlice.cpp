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
  // TODO: cleanup this test
  constexpr double tol = 1e-4;
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

// A feasible robot1 (gantry + arm) configuration for the bar_lab pose-goal
// tests. Forward kinematics of this configuration yields a reachable
// end-effector goal pose.
static gtsam::Values barLabConfig(const Robot& robot) {
  gtsam::Values q;
  auto set_q = [&](const std::string& name, double value) {
    q.insert(JointAngleKey(robot.joint(name)->id(), k), value);
  };
  set_q("bridge1_joint_EA_X", 2.0);
  set_q("robot1_joint_EA_Y", 1.5);
  set_q("robot1_joint_EA_Z", 0.5);
  set_q("robot1_joint_1", 0.3);
  set_q("robot1_joint_2", 0.4);
  set_q("robot1_joint_3", -0.5);
  set_q("robot1_joint_4", 0.2);
  set_q("robot1_joint_5", 0.6);
  set_q("robot1_joint_6", -0.3);
  return q;
}

// IK on bar_lab driving a single end-effector pose as a hard constraint, where
// we only care that the end-effector reaches the goal pose (any feasible joint
// configuration is acceptable).
TEST(Slice, BarLabEndEffectorPoseIK) {
  using gtsam::Pose3;
  using gtsam::Values;

  // Load the bar_lab gantry and anchor it at the base link.
  const Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("bar_lab.urdf"))
          .fixLink("columns");

  // Use augmented Lagrangian so the pose goal is enforced as a hard constraint.
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  auto ee_link = robot.link("robot1_link_6");
  const auto ee_id = ee_link->id();

  // Reachable goal: the end-effector CoM pose in world (wTcom) from forward
  // kinematics of a feasible configuration.
  Values fk = kinematics.initialValues(kSlice, robot, 0.0, barLabConfig(robot),
                                       true);
  const Pose3 wTcom_goal = fk.at<Pose3>(PoseKey(ee_id, k));

  // The pose goal is the only goal constraint (the rest is robot kinematics).
  PoseGoals pose_goals = {{k, PoseGoal(ee_link, Pose3(), wTcom_goal)}};

  // Solve with no warm start: we only care that the end-effector pose is met,
  // not which joint configuration achieves it.
  auto result = kinematics.inverse(kSlice, robot, pose_goals, gtsam::Values(),
                                   /*pose_goals_as_constraints=*/true);

  // The end-effector reaches the goal pose.
  EXPECT(assert_equal(wTcom_goal, result.at<Pose3>(PoseKey(ee_id, k)), 1e-3));
}

// Same hard pose constraint, but warm-started near a known configuration and
// also checking that the anchored base link does not move.
TEST(Slice, BarLabPoseConstraintIK) {
  using gtsam::Pose3;
  using gtsam::Values;

  const Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("bar_lab.urdf"))
          .fixLink("columns");

  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  auto ee_link = robot.link("robot1_link_6");
  const auto ee_id = ee_link->id();

  // Reachable goal: the end-effector CoM pose in world (wTcom) from forward
  // kinematics of a feasible configuration.
  Values q_true = barLabConfig(robot);
  Values fk = kinematics.initialValues(kSlice, robot, 0.0, q_true, true);
  const Pose3 wTcom_goal = fk.at<Pose3>(PoseKey(ee_id, k));

  PoseGoals pose_goals = {{k, PoseGoal(ee_link, Pose3(), wTcom_goal)}};

  // Warm start the solver with a noisy version of the true joint angles.
  Values initial_joints;
  gtsam::Sampler sampler(gtsam::noiseModel::Isotropic::Sigma(1, 0.05), 42u);
  for (const auto key : q_true.keys()) {
    initial_joints.insert(key, q_true.at<double>(key) + sampler.sample()(0));
  }

  // Solve IK with the pose goal as a hard constraint.
  auto result = kinematics.inverse(kSlice, robot, pose_goals, initial_joints,
                                   /*pose_goals_as_constraints=*/true);

  // The end-effector reaches the goal pose.
  double tol = 1e-3;
  EXPECT(assert_equal(wTcom_goal, result.at<Pose3>(PoseKey(ee_id, k)), tol));

  // The anchored base link does not move (bMcom is its rest CoM pose in base).
  auto base_link = robot.link("columns");
  EXPECT(assert_equal(base_link->bMcom(),
                      result.at<Pose3>(PoseKey(base_link->id(), k)), tol));
}

// Higher gantry prior sigmas let the gantry absorb more of the motion to goal.
TEST(Slice, BarLabGantryPriorSigma) {
  using gtsam::Pose3;
  using gtsam::Values;

  const Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("bar_lab.urdf"))
          .fixLink("columns");
  auto ee_link = robot.link("robot1_link_6");
  const auto ee_id = ee_link->id();
  const std::vector<std::string> gantry_joints = {
      "bridge1_joint_EA_X", "robot1_joint_EA_Y", "robot1_joint_EA_Z"};

  // Reachable goal from a config that uses the gantry, so there is redundancy.
  Kinematics goal_kinematics;
  Values fk = goal_kinematics.initialValues(kSlice, robot, 0.0,
                                            barLabConfig(robot), true);
  const Pose3 wTcom_goal = fk.at<Pose3>(PoseKey(ee_id, k));
  PoseGoals pose_goals = {{k, PoseGoal(ee_link, Pose3(), wTcom_goal)}};

  // Solve IK with the given gantry prior sigma; pose goal is a hard constraint.
  auto solve = [&](double gantry_sigma) {
    KinematicsParameters params;
    params.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
    for (const auto& name : gantry_joints)
      params.setJointPriorSigma(robot.joint(name)->id(), gantry_sigma);
    return Kinematics(params).inverse(kSlice, robot, pose_goals, gtsam::Values(),
                                      /*pose_goals_as_constraints=*/true);
  };

  // Total absolute travel of the gantry joints from their zero prior mean.
  auto gantry_travel = [&](const Values& result) {
    double travel = 0.0;
    for (const auto& name : gantry_joints)
      travel += std::fabs(
          result.at<double>(JointAngleKey(robot.joint(name)->id(), k)));
    return travel;
  };

  const Values tight = solve(0.1);
  const Values loose = solve(10.0);

  // Both reach the goal pose; the looser gantry priors move the gantry more.
  EXPECT(assert_equal(wTcom_goal, tight.at<Pose3>(PoseKey(ee_id, k)), 1e-3));
  EXPECT(assert_equal(wTcom_goal, loose.at<Pose3>(PoseKey(ee_id, k)), 1e-3));
  EXPECT(gantry_travel(tight) < gantry_travel(loose));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
