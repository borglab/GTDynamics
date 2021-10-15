/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPoseFactor.cpp
 * @brief Test forward kinematics factor.
 * @author Frank Dellaert and Mandy Xie
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "make_joint.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector6;
using gtsam::Z_6x1;
using gtsam::noiseModel::Gaussian;

namespace example {
// noise model
Gaussian::shared_ptr cost_model = Gaussian::Covariance(gtsam::I_6x6);
size_t k = 777;
}  // namespace example

// Test twist factor for stationary case
TEST(PoseFactor, error) {
  // create functor
  Pose3 cMp = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(cMp, screw_axis);

  // Create factor
  PoseFactor factor(example::cost_model, joint, example::k);

  // call unwhitenedError
  Values values;
  InsertPose(&values, 1, example::k, Pose3(Rot3(), Point3(1, 0, 0)));
  InsertPose(&values, 2, example::k, Pose3(Rot3(), Point3(3, 0, 0)));
  InsertJointAngle(&values, 1, example::k, 0.0);
  auto actual_errors = factor.unwhitenedError(values);

  // check value
  auto expected_errors = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test breaking case
TEST(PoseFactor, breaking) {
  // create functor
  Pose3 cMp = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(cMp, screw_axis);
  PoseFactor factor(example::cost_model, joint, example::k);

  // check prediction at zero joint angle
  {
    Values values;
    InsertPose(&values, 1, example::k, Pose3(Rot3(), Point3(1, 0, 0)));
    InsertPose(&values, 2, example::k, Pose3(Rot3(), Point3(3, 0, 0)));
    InsertJointAngle(&values, 1, example::k, 0.0);
    EXPECT(assert_equal(Z_6x1, factor.unwhitenedError(values), 1e-6));
  }

  // check prediction at half PI
  {
    Values values;
    InsertPose(&values, 1, example::k, Pose3(Rot3(), Point3(1, 0, 0)));
    InsertPose(&values, 2, example::k,
               Pose3(Rot3::Rz(M_PI / 2), Point3(2, 1, 0)));
    InsertJointAngle(&values, 1, example::k, M_PI / 2);
    EXPECT(assert_equal(Z_6x1, factor.unwhitenedError(values), 1e-6));
  }
}

// Test breaking case for rr link
TEST(PoseFactor, breaking_rr) {
  // Evaluate PoseFunctor on an RR link.
  auto robot = simple_urdf_zero_inertia::getRobot();

  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");
  auto j1 = boost::dynamic_pointer_cast<gtdynamics::ScrewJointBase>(
      robot.joint("j1"));

  Vector6 screw_axis = (Vector6() << 1, 0, 0, 0, -1, 0).finished();
  Pose3 cMp = j1->relativePoseOf(l1, 0.0);
  auto joint = make_joint(cMp, screw_axis);
  PoseFactor factor(example::cost_model, joint, example::k);

  // unwhitenedError
  Values values;
  InsertPose(&values, 1, example::k, Pose3());
  InsertPose(&values, 2, example::k, j1->relativePoseOf(l2, M_PI / 4));
  InsertJointAngle(&values, 1, example::k, M_PI / 4);
  EXPECT(assert_equal(Z_6x1, factor.unwhitenedError(values), 1e-6));
}

// Test non-zero cMp rotation case
TEST(PoseFactor, nonzero_rest) {
  // Create factor
  Pose3 cMp = Pose3(Rot3::Rx(1), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(cMp, screw_axis);
  PoseFactor factor(example::cost_model, joint, example::k);

  double jointAngle;
  Pose3 pose_p, pose_c;
  // zero joint angle
  jointAngle = 0;
  pose_p = Pose3(Rot3(), Point3(1, 0, 0));
  pose_c = Pose3(Rot3(), Point3(3, 0, 0));
  // Make sure linearization is correct
  {
    Values values;
    InsertPose(&values, 1, example::k, pose_p);
    InsertPose(&values, 2, example::k, pose_c);
    InsertJointAngle(&values, 1, example::k, jointAngle);
    double diffDelta = 1e-7;
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
  }

  // half PI
  {
    Values values;
    jointAngle = M_PI / 2;
    pose_p = Pose3(Rot3(), Point3(1, 0, 0));
    pose_c = Pose3(Rot3::Rz(jointAngle), Point3(2, 1, 0));
    InsertPose(&values, 1, example::k, pose_p);
    InsertPose(&values, 2, example::k, pose_c);
    InsertJointAngle(&values, 1, example::k, jointAngle);
    double diffDelta = 1e-7;
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
  }
}

TEST(PoseFactor, ForwardKinematics) {
  auto robot = simple_rr::getRobot();

  gtsam::NonlinearFactorGraph graph;
  Values initial;
  size_t k = example::k;
  double angle = M_PI_2;

  auto link0_key = internal::PoseKey(0, k);

  auto pose_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  auto joint_angle_model = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);

  // Set prior for base link aka fix the base link
  auto base_link = robot.link("link_0");
  graph.addPrior<gtsam::Pose3>(link0_key, base_link->bMcom());

  for (auto&& joint : robot.joints()) {
    graph.emplace_shared<PoseFactor>(pose_model, joint, k);
    graph.addPrior<double>(internal::JointAngleKey(joint->id(), k), angle,
                           joint_angle_model);
  }

  InsertPose(&initial, 0, k, robot.links()[0]->bMcom());
  InsertPose(&initial, 1, k, robot.links()[1]->bMcom());
  InsertPose(&initial, 2, k, robot.links()[2]->bMcom());
  InsertJointAngle(&initial, 0, k, angle);
  InsertJointAngle(&initial, 1, k, angle);

  gtsam::GaussNewtonOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  Values known_values;
  InsertJointAngle(&known_values, 0, k, angle);
  InsertJointAngle(&known_values, 1, k, angle);
  InsertPose(&known_values, 0, k, robot.links()[0]->bMcom());

  Values expected =
      robot.forwardKinematics(known_values, k, base_link->name());

  EXPECT(assert_equal(Pose(result, 0, k), Pose(expected, 0, k)));
  EXPECT(assert_equal(Pose(result, 1, k), Pose(expected, 1, k)));
  EXPECT(assert_equal(Pose(result, 2, k), Pose(expected, 2, k)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
