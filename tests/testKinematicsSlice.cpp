/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testKinematicsSliceInitialValues.cpp
 * @brief Test Kinematics in single time slice.
 * @author: Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/utils/ContactPoint.h>

// #includ<">gtdynamics/kinematics/KinematicsSliceInitialValues.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <vector>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

struct ContactGoal {
  PointOnLink point_on_link;  ///< In COM.
  gtsam::Point3 goal_point;   ///< In world frame.

  /// Return link associated with contact point.
  const LinkSharedPtr& link() const { return point_on_link.link; }

  /// Return contact point in link COM frame.
  const Point3& contact_in_com() const { return point_on_link.point; }

  /**
   * @fn For given values, predict where point_on_link is in world frame.
   * @param values a GTSAM Values instance that should contain link pose.
   * @param k time step to check (default 0).
   */
  gtsam::Point3 predict(const gtsam::Values& values, size_t k = 0) const {
    const gtsam::Pose3 wTcom = Pose(values, link()->id(), k);
    return wTcom.transformFrom(point_on_link.point);
  }

  /**
   * @fn Check that the contact goal has been achived for given values.
   * @param values a GTSAM Values instance that should contain link pose.
   * @param k time step to check (default 0).
   * @param tol tolerance in 3D (default 1e-9).
   */
  bool satisfied(const gtsam::Values& values, size_t k = 0,
                 double tol = 1e-9) const {
    return gtsam::distance3(predict(values, k), goal_point) < tol;
  }
};

///< Map of link name to ContactGoal
using ContactGoals = std::vector<ContactGoal>;

struct KinematicsSettings {
  using Isotropic = gtsam::noiseModel::Isotropic;
  const gtsam::SharedNoiseModel p_cost_model,     // pose factor
      g_cost_model,                               // goal point
      prior_q_cost_model;                         // joint angle prior factor
  gtsam::LevenbergMarquardtParams lm_parameters;  // LM parameters

  KinematicsSettings()
      : p_cost_model(Isotropic::Sigma(6, 1e-4)),
        g_cost_model(Isotropic::Sigma(3, 0.01)),
        prior_q_cost_model(Isotropic::Sigma(1, 0.5)) {}
};

/**
 * @fn Slice with kinematics constraints.
 * @param robot robot configuration
 * @param opt KinematicsSettings
 * @param k time step (default 0).
 * @returns factor graph..
 */
gtsam::NonlinearFactorGraph KinematicsSlice(
    const Robot& robot, const KinematicsSettings& opt = KinematicsSettings(),
    size_t k = 0) {
  gtsam::NonlinearFactorGraph graph;

  // Constrain kinematics at joints.
  for (auto&& joint : robot.joints()) {
    const auto j = joint->id();
    graph.emplace_shared<PoseFactor>(
        internal::PoseKey(joint->parent()->id(), k),
        internal::PoseKey(joint->child()->id(), k),
        internal::JointAngleKey(j, k), opt.p_cost_model, joint);
  }

  return graph;
}

/**
 * @fn Create point goal objectives.
 * @param robot robot configuration
 * @param contact_goals goals for contact points
 * @param opt KinematicsSettings
 * @param k time step to check (default 0).
 * @returns graph with point goal factors.
 */
gtsam::NonlinearFactorGraph PointGoalObjectives(
    const Robot& robot, const ContactGoals& contact_goals,
    const KinematicsSettings& opt = KinematicsSettings(), size_t k = 0) {
  gtsam::NonlinearFactorGraph graph;

  // Add objectives.
  for (const ContactGoal& goal : contact_goals) {
    const gtsam::Key pose_key = internal::PoseKey(goal.link()->id(), k);
    graph.emplace_shared<PointGoalFactor>(
        pose_key, opt.g_cost_model, goal.contact_in_com(), goal.goal_point);
  }

  return graph;
}

/**
 * @fn Factors that minimize joint angles.
 * @param robot robot configuration
 * @param opt KinematicsSettings
 * @param k time step to check (default 0).
 * @returns graph with prior factors on joint angles.
 */
gtsam::NonlinearFactorGraph MinimumJointAngleSlice(
    const Robot& robot, const KinematicsSettings& opt = KinematicsSettings(),
    size_t k = 0) {
  gtsam::NonlinearFactorGraph graph;

  // Minimize the joint angles.
  for (auto&& joint : robot.joints()) {
    const gtsam::Key key = internal::JointAngleKey(joint->id(), k);
    graph.addPrior<double>(key, 0.0, opt.prior_q_cost_model);
  }

  return graph;
}
/**
 * @fn Initialize kinematics.
 *
 * Use wTcom for poses and zero-mean noise for joint angles.
 *
 * @param robot robot configuration
 * @param k time step to check (default 0).
 * @param gaussian_noise time step to check (default 0.1).
 * @returns values with identity poses and zero joint angles.
 */
gtsam::Values KinematicsSliceInitialValues(const Robot& robot, size_t k = 0,
                                           double gaussian_noise = 0.1) {
  gtsam::Values values;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
  gtsam::Sampler sampler(sampler_noise_model);

  // Initialize all joint angles.
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&values, joint->id(), k, sampler.sample()[0]);
  }

  // Initialize all poses.
  for (auto&& link : robot.links()) {
    InsertPose(&values, link->id(), k, link->wTcom());
  }

  return values;
}

/**
 * @fn Inverse kinematics given a set of contact goals.
 * @param robot robot configuration
 * @param contact_goals goals for contact points
 * @param opt KinematicsSettings
 * @param k time step to check (default 0).
 * @returns values with poses and joint angles.
 */
gtsam::Values InverseKinematics(
    const Robot& robot, const ContactGoals& contact_goals,
    const KinematicsSettings& opt = KinematicsSettings(), size_t k = 0) {
  auto graph = KinematicsSlice(robot, opt, k);

  // Add objectives.
  graph.add(PointGoalObjectives(robot, contact_goals, opt, k));
  graph.add(MinimumJointAngleSlice(robot, opt, k));

  // This should not be needed:
  // graph.addPrior<gtsam::Pose3>(internal::PoseKey(0, k), gtsam::Pose3(),
  // nullptr);

  auto values = KinematicsSliceInitialValues(robot, k);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values,
                                               opt.lm_parameters);
  gtsam::Values results = optimizer.optimize();
  return results;
}

TEST(Phase, inverse_kinematics) {
  Robot robot = CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"));

  const size_t k = 777;  // time step for slice

  // Create initial values
  auto values = KinematicsSliceInitialValues(robot, k, 0.0);
  EXPECT_LONGS_EQUAL(13 + 12, values.size());

  // establish contact/goal pairs
  const Point3 contact_in_com(0.14, 0, 0);
  const ContactGoals contact_goals = {
      {{robot.link("lower1"), contact_in_com}, {-0.4, 0.16, -0.2}},    // LH
      {{robot.link("lower0"), contact_in_com}, {0.3, 0.16, -0.2}},     // LF
      {{robot.link("lower2"), contact_in_com}, {0.3, -0.16, -0.2}},    // RF
      {{robot.link("lower3"), contact_in_com}, {-0.4, -0.16, -0.2}}};  // RH

  // Set twists to zero fro FK. TODO(frank): separate kinematics from velocity?
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
    // EXPECT(assert_equal(goal.goal_point, goal.predict(fk, k)));
    EXPECT(goal.satisfied(fk, k, 0.05));
  }

  KinematicsSettings opt;
  // opt.lm_parameters.setVerbosityLM("SUMMARY");
  opt.lm_parameters.setlambdaInitial(1e7);
  opt.lm_parameters.setAbsoluteErrorTol(1e-3);

  auto graph = KinematicsSlice(robot, opt, k);
  EXPECT_LONGS_EQUAL(12, graph.size());

  auto objectives = PointGoalObjectives(robot, contact_goals, opt, k);
  EXPECT_LONGS_EQUAL(4, objectives.size());

  auto objectives2 = MinimumJointAngleSlice(robot, opt, k);
  EXPECT_LONGS_EQUAL(12, objectives2.size());

  constexpr size_t redundancy = 6;
  EXPECT_LONGS_EQUAL(13 * 6 + redundancy, 12 * 6 + 4 * 3);

  // TODO(frank): consider renaming ContactPoint to PointOnLink
  auto result = InverseKinematics(robot, contact_goals, opt, k);

  // Check that well-determined
  graph.add(objectives);
  graph.add(objectives2);
  EXPECT_LONGS_EQUAL(12 + 12 + 4, graph.size());
  // auto factor = graph.linearizeToHessianFactor(result);
  // GTD_PRINT(*factor);

  // Check that goals are achieved
  constexpr double tol = 0.01;
  for (const ContactGoal& goal : contact_goals) {
    EXPECT(assert_equal(goal.goal_point, goal.predict(result, k), tol));
  }

  // Check that goals are achieved
  for (const ContactGoal& goal : contact_goals) {
    EXPECT(goal.satisfied(result, k, tol));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
