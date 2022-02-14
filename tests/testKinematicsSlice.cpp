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

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

TEST(Slice, InverseKinematics) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  // Create a slice.
  const size_t k = 777;
  const Slice slice(k);

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
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
  EXPECT(values.exists(PoseKey(root_link_id, k)));
  auto fk = robot.forwardKinematics(values, k, std::string(root_link_name));

  // Check goals with FK solution
  for (const ContactGoal& goal : contact_goals) {
    EXPECT(goal.satisfied(fk, k, 0.05));
  }

  auto graph = kinematics.graph(slice, robot);
  EXPECT_LONGS_EQUAL(12, graph.size());

  auto objectives = kinematics.pointGoalObjectives(slice, contact_goals);
  EXPECT_LONGS_EQUAL(4, objectives.size());

  auto objectives2 = kinematics.jointAngleObjectives(slice, robot);
  EXPECT_LONGS_EQUAL(12, objectives2.size());

  auto result = kinematics.inverse(slice, robot, contact_goals);

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

// Values tonisFunction(const Slice& slice, const Robot& robot,
//                                   const ContactGoals& contact_goals) const {
//   auto graph = this->graph(slice, robot);

//   // Add prior.
//   graph.add(jointAngleObjectives(slice, robot));

//   graph.addPrior<gtsam::Pose3>(PoseKey(0, slice.k),
//   gtsam::Pose3(), nullptr);

//   auto initial_values = initialValues(slice, robot);

//   return optimize(graph, initial_values);
// }

gtsam::Values ikWithPose(const Kinematics* self, const Slice& slice,
                         const Robot& robot,
                         const std::map<size_t, gtsam::Pose3> constrained_poses,
                         const std::map<size_t, gtsam::Pose3> desired_poses,
                         const gtsam::Vector7& initial) {
  auto graph = self->graph(slice, robot);

  // Add prior on joint angles to constrain the solution
  graph.add(self->jointAngleObjectives(slice, robot));

  // Add priors on link poses with constrained poses from argument
  // TODO: do real pose constraints - ask Yetong
  for (auto&& kv : constrained_poses) {
    const size_t link_index = kv.first;
    const gtsam::Pose3& constrained_pose = kv.second;
    graph.addPrior<gtsam::Pose3>(PoseKey(link_index, slice.k), constrained_pose,
                                 gtsam::noiseModel::Constrained::All(6));
  }

  // Add priors on link poses with desired poses from argument
  for (auto&& kv : desired_poses) {
    const size_t link_index = kv.first;
    const gtsam::Pose3& desired_pose = kv.second;
    graph.addPrior<gtsam::Pose3>(PoseKey(link_index, slice.k), desired_pose,
                                 nullptr);
  }

  // TODO: make use of initial?
  auto initial_values = self->initialValues(slice, robot);

  return self->optimize(graph, {}, initial_values);
}

TEST(Slice, PandaIK) {
  using gtsam::Pose3;
  using gtsam::Rot3;
  using gtsam::Values;
  using gtsam::Vector7;

  const Robot panda =
      CreateRobotFromFile(kUrdfPath + std::string("panda/panda.urdf"));
  const Robot robot = panda.fixLink("link0");

  // Create a slice.
  const size_t k = 777;
  const Slice slice(k);

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  Vector7 initial = Vector7::Zero();
  Rot3 sR7({{1, 0, 0}, {0, -1, 0}, {0, 0, -1}});
  Pose3 sM7(sR7, Point3(0.0882972, 0.00213401, 0.933844));
  auto base_link = robot.link("link0");
  const Pose3 sM0 = base_link->bMcom();
  auto values =
      ikWithPose(&kinematics, slice, robot, {{0, sM0}}, {{7, sM7}}, initial);
  Vector7 optimal_q;
  for (size_t j = 0; j < 7; j++)
    optimal_q[j] = values.at<double>(JointAngleKey(j, slice.k));
  EXPECT(assert_equal(initial, optimal_q));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
