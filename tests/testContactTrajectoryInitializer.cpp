/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/TestableAssertions.h>

using namespace gtdynamics;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::assert_equal;

TEST(ContactTrajectoryInitializer, DenseSeedsPreserveMeasurementsAndCOMFrames) {
  const auto robot = CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto root = robot.link("l1");
  const auto foot = robot.link("l2");
  const auto joint = robot.joint("j1");
  const std::vector<size_t> samples{4, 5, 6, 7, 8};
  Values measurements;
  for (size_t sample : samples) {
    InsertJointAngle(&measurements, joint->id(), sample, 0.25 + 0.07 * (sample - 4));
    InsertJointVel(&measurements, joint->id(), sample, 0.3 + 0.01 * sample);
    // Pose values supplied alongside measurements are deliberately ignored.
    InsertPose(&measurements, root->id(), sample,
               Pose3(Rot3(), Point3(1000, 1000, 1000)));
  }
  const PointOnLinks patch{{foot, Point3(-0.1, -0.05, -0.2)},
                           {foot, Point3(0.1, -0.05, -0.2)},
                           {foot, Point3(0.1, 0.05, -0.2)},
                           {foot, Point3(-0.1, 0.05, -0.2)}};
  const ContactEpisode stance{"foot-stance-0", 4, 8, patch};
  const Pose3 initial_root(Rot3::RzRyRx(0.12, -0.23, 0.34), Point3(1.2, -0.8, 1.5));
  contact_init::Options options;
  options.model = contact_init::Model::Patch;
  options.perturb = true;
  contact_init::Result diagnostics;
  Initializer initializer;
  const auto result = initializer.InitializeSolutionContacts(
      robot, "l1", initial_root, measurements, samples, {stance}, 0.01,
      options, &diagnostics);

  EXPECT_LONGS_EQUAL(2, diagnostics.graph.frames.size());
  EXPECT_LONGS_EQUAL(4, diagnostics.graph.frames.front().sampleIndex);
  EXPECT_LONGS_EQUAL(8, diagnostics.graph.frames.back().sampleIndex);
  EXPECT(assert_equal(0.04, diagnostics.graph.frames.front().time, 1e-12));
  EXPECT(diagnostics.values.exists(PoseKey(root->id(), 4)));
  EXPECT(diagnostics.values.exists(PoseKey(root->id(), 8)));
  EXPECT(assert_equal(initial_root, Pose(result, root->id(), 4), 1e-10));
  EXPECT(diagnostics.residualRms < 1e-7);

  const auto stationary_foot = Pose(result, foot->id(), 4);
  for (size_t sample : samples) {
    EXPECT(assert_equal(stationary_foot, Pose(result, foot->id(), sample), 1e-6));
    DOUBLES_EQUAL(JointAngle(measurements, joint->id(), sample),
                  JointAngle(result, joint->id(), sample), 0);
    DOUBLES_EQUAL(JointVel(measurements, joint->id(), sample),
                  JointVel(result, joint->id(), sample), 0);
    EXPECT(result.exists(TorqueKey(joint->id(), sample)));
    EXPECT(result.exists(JointAccelKey(joint->id(), sample)));
    EXPECT(result.exists(ContactWrenchKey(foot->id(), 0, sample)));
    EXPECT(Twist(result, root->id(), sample).norm() < 1e-12);
  }
}

TEST(ContactTrajectoryInitializer, ExplicitContactScheduleAndMeasuredAnglesRequired) {
  const auto robot = CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto foot = robot.link("l2");
  const auto joint = robot.joint("j1");
  const ContactEpisode stance{"foot-stance-0", 4, 8,
                              {{foot, Point3(0, 0, 0)}}};
  Initializer initializer;
  Values measurements;
  InsertJointAngle(&measurements, joint->id(), 4, 0.0);
  bool missing_angle_rejected = false;
  try {
    initializer.InitializeSolutionContacts(robot, "l1", Pose3(), measurements,
                                           {4, 8}, {stance});
  } catch (const std::invalid_argument&) { missing_angle_rejected = true; }
  EXPECT(missing_angle_rejected);

  bool missing_boundary_rejected = false;
  try {
    initializer.InitializeSolutionContacts(robot, "l1", Pose3(), measurements,
                                           {4, 9}, {stance});
  } catch (const std::invalid_argument&) { missing_boundary_rejected = true; }
  EXPECT(missing_boundary_rejected);

  bool unordered_samples_rejected = false;
  try {
    initializer.InitializeSolutionContacts(robot, "l1", Pose3(), measurements,
                                           {8, 4}, {stance});
  } catch (const std::invalid_argument&) { unordered_samples_rejected = true; }
  EXPECT(unordered_samples_rejected);
}

TEST(ContactTrajectoryInitializer, DisconnectedContactGaugesRejected) {
  const auto robot = CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto foot = robot.link("l2");
  const auto joint = robot.joint("j1");
  const PointOnLinks point{{foot, Point3(0, 0, 0)}};
  Values measurements;
  for (size_t sample : {4, 5, 8, 9}) {
    InsertJointAngle(&measurements, joint->id(), sample, 0.0);
  }
  Initializer initializer;
  contact_init::Options options;
  options.model = contact_init::Model::Point;
  bool disconnected_rejected = false;
  try {
    initializer.InitializeSolutionContacts(
        robot, "l1", Pose3(), measurements, {4, 5, 8, 9},
        {{"stance-before-flight", 4, 5, point},
         {"stance-after-flight", 8, 9, point}}, 0.01, options);
  } catch (const std::invalid_argument& error) {
    disconnected_rejected = std::string(error.what()).find("disconnected") != std::string::npos;
  }
  EXPECT(disconnected_rejected);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
