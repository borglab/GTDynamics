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

#include <cmath>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;

// Dense expansion preserves measured joints and the chosen root COM gauge.
TEST(ContactTrajectoryInitializer, DenseSeedsPreserveMeasurementsAndCOMFrames) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto root = robot.link("l1");
  const auto foot = robot.link("l2");
  const auto joint = robot.joint("j1");
  const std::vector<size_t> samples{4, 5, 6, 7, 8};
  Values measurements;
  for (size_t sample : samples) {
    InsertJointAngle(&measurements, joint->id(), sample,
                     0.25 + 0.07 * (sample - 4));
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
  const Pose3 initial_root(Rot3::RzRyRx(0.12, -0.23, 0.34),
                           Point3(1.2, -0.8, 1.5));
  contact_init::Options options;
  options.model = contact_init::Model::Patch;
  options.perturb = true;
  contact_init::Result diagnostics;
  Initializer initializer;
  const auto result = initializer.InitializeSolutionContacts(
      robot, "l1", initial_root, measurements, samples, {stance}, 0.01, options,
      &diagnostics);

  EXPECT_LONGS_EQUAL(2, diagnostics.graph.measurements.size());
  EXPECT_LONGS_EQUAL(4, diagnostics.graph.measurements.front().slice.k);
  EXPECT_LONGS_EQUAL(8, diagnostics.graph.measurements.back().slice.k);
  EXPECT(
      assert_equal(0.04, diagnostics.graph.measurements.front().time, 1e-12));
  EXPECT(diagnostics.values.exists(PoseKey(root->id(), 4)));
  EXPECT(diagnostics.values.exists(PoseKey(root->id(), 8)));
  EXPECT(assert_equal(initial_root, Pose(result, root->id(), 4), 1e-10));
  EXPECT(diagnostics.residualRms < 1e-7);

  const auto stationary_foot = Pose(result, foot->id(), 4);
  for (size_t sample : samples) {
    EXPECT(
        assert_equal(stationary_foot, Pose(result, foot->id(), sample), 1e-6));
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

// Missing joint angles, missing boundaries, and unordered sample keys are
// rejected.
TEST(ContactTrajectoryInitializer,
     ExplicitContactScheduleAndMeasuredAnglesRequired) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto foot = robot.link("l2");
  const auto joint = robot.joint("j1");
  const ContactEpisode stance{"foot-stance-0", 4, 8, {{foot, Point3(0, 0, 0)}}};
  Initializer initializer;
  Values measurements;
  InsertJointAngle(&measurements, joint->id(), 4, 0.0);
  bool missing_angle_rejected = false;
  try {
    initializer.InitializeSolutionContacts(robot, "l1", Pose3(), measurements,
                                           {4, 8}, {stance});
  } catch (const std::invalid_argument&) {
    missing_angle_rejected = true;
  }
  EXPECT(missing_angle_rejected);

  bool missing_boundary_rejected = false;
  try {
    initializer.InitializeSolutionContacts(robot, "l1", Pose3(), measurements,
                                           {4, 9}, {stance});
  } catch (const std::invalid_argument&) {
    missing_boundary_rejected = true;
  }
  EXPECT(missing_boundary_rejected);

  bool unordered_samples_rejected = false;
  try {
    initializer.InitializeSolutionContacts(robot, "l1", Pose3(), measurements,
                                           {8, 4}, {stance});
  } catch (const std::invalid_argument&) {
    unordered_samples_rejected = true;
  }
  EXPECT(unordered_samples_rejected);
}

// Long stances retain all boundaries and add original samples at most 1 s
// apart.
TEST(ContactTrajectoryInitializer,
     LongStancesRetainBoundariesAndAddMeasuredFrames) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto root = robot.link("l1");
  const auto foot = robot.link("l2");
  const auto joint = robot.joint("j1");
  std::vector<size_t> samples;
  Values measurements;
  for (size_t sample = 4; sample <= 38; ++sample) {
    samples.push_back(sample);
    InsertJointAngle(&measurements, joint->id(), sample,
                     0.2 + 0.1 * std::sin(sample));
  }
  const PointOnLinks patch{{foot, Point3(-0.1, -0.05, 0)},
                           {foot, Point3(0.1, -0.05, 0)},
                           {foot, Point3(0.1, 0.05, 0)}};
  contact_init::Options options;
  options.model = contact_init::Model::Patch;
  contact_init::Result diagnostics;
  const auto values = Initializer().InitializeSolutionContacts(
      robot, "l1", Pose3(), measurements, samples,
      {{"stance-0", 4, 18, patch}, {"stance-1", 17, 38, patch}}, 0.1, options,
      &diagnostics);
  const std::vector<size_t> expected{4, 14, 17, 18, 28, 38};
  EXPECT_LONGS_EQUAL(expected.size(), diagnostics.graph.measurements.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    const auto& frame = diagnostics.graph.measurements.at(i);
    EXPECT_LONGS_EQUAL(expected[i], frame.slice.k);
    EXPECT(diagnostics.values.exists(PoseKey(root->id(), expected[i])));
    EXPECT(assert_equal(Pose(values, foot->id(), samples.front()),
                        Pose(values, foot->id(), expected[i]), 1e-7));
    if (i)
      EXPECT(frame.time - diagnostics.graph.measurements[i - 1].time <=
             1.0 + 1e-8);
  }
  for (const size_t sample : samples) {
    DOUBLES_EQUAL(JointAngle(measurements, joint->id(), sample),
                  JointAngle(values, joint->id(), sample), 0);
  }
}

// The adapter rejects source samples too sparse to meet its frame-gap bound.
TEST(ContactTrajectoryInitializer,
     SparseSamplesCannotMeetOneSecondFrameInterval) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto foot = robot.link("l2");
  Values measurements;
  for (size_t sample : {4, 25}) {
    InsertJointAngle(&measurements, robot.joint("j1")->id(), sample, 0.0);
  }
  bool rejected = false;
  try {
    Initializer().InitializeSolutionContacts(
        robot, "l1", Pose3(), measurements, {4, 25},
        {{"stance", 4, 25, {{foot, Point3(0, 0, 0)}}}}, 0.1);
  } catch (const std::invalid_argument& error) {
    rejected =
        std::string(error.what()).find("too sparse") != std::string::npos;
  }
  EXPECT(rejected);
}

// A soft height prior derives its target from anchored FK and keeps Point3
// states.
TEST(ContactTrajectoryInitializer, LandmarkGroundPriorUsesAnchoredFK) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto root = robot.link("l1");
  const auto foot = robot.link("l2");
  Values measurements;
  const std::vector<size_t> samples{4, 5, 6};
  for (const size_t sample : samples) {
    InsertJointAngle(&measurements, robot.joint("j1")->id(), sample,
                     0.1 * sample);
  }
  const PointOnLinks patch{{foot, Point3(-0.1, -0.05, 0)},
                           {foot, Point3(0.1, -0.05, 0)},
                           {foot, Point3(0.1, 0.05, 0)}};
  contact_init::Options options;
  options.model = contact_init::Model::Patch;
  options.addGroundFactors = true;
  const Pose3 anchor(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1, 2, 3));
  contact_init::Result diagnostics;
  const auto values = Initializer().InitializeSolutionContacts(
      robot, "l1", anchor, measurements, samples, {{"stance", 4, 6, patch}},
      0.01, options, &diagnostics);
  double height = 0;
  for (const auto& observation :
       diagnostics.graph.measurements.front().observations) {
    height += anchor.transformFrom(observation.localPoint).z() / patch.size();
  }
  EXPECT_LONGS_EQUAL(patch.size(), diagnostics.graph.groundFactors.size());
  EXPECT(!diagnostics.graph.groundHeight.has_value());
  EXPECT(assert_equal(height, *diagnostics.graph.groundPriorHeight, 1e-12));
  EXPECT(
      assert_equal(anchor, Pose(values, root->id(), samples.front()), 1e-12));
  EXPECT(assert_equal(
      diagnostics.objectiveFinal,
      diagnostics.contactObjective + diagnostics.groundObjective, 1e-10));
}

// Contact-only components cannot be interpolated into one common world
// trajectory.
TEST(ContactTrajectoryInitializer, DisconnectedContactGaugesRejected) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
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
         {"stance-after-flight", 8, 9, point}},
        0.01, options);
  } catch (const std::invalid_argument& error) {
    disconnected_rejected =
        std::string(error.what()).find("disconnected") != std::string::npos;
  }
  EXPECT(disconnected_rejected);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
