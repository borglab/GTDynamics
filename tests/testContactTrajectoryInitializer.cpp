/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/TestableAssertions.h>

#include <cmath>
#include <limits>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;

/* ************************************************************************* */
namespace contact_trajectory {
class GeneralContactSpec : public ConstraintSpec {
 public:
  PointOnLinks points;
  const PointOnLinks& contactPoints() const override { return points; }
  void print(const std::string&) const override {}
};
Phase ContactPhase(size_t first, size_t last, const PointOnLinks& points) {
  return Phase(first, last,
               std::make_shared<FootContactConstraintSpec>(points));
}
Trajectory Phases(const std::vector<Phase>& phases) {
  return Trajectory(WalkCycle(phases), 1);
}
template <class F>
bool Rejects(F&& call, const std::string& message) {
  try {
    call();
  } catch (const std::invalid_argument& error) {
    return std::string(error.what()).find(message) != std::string::npos;
  }
  return false;
}
// General specifications retain ground and non-ground contact observations.
TEST(ContactTrajectoryInitializer, GeneralSpecificationKeepsNonGroundContacts) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto link = robot.link("l2");
  auto spec = std::make_shared<GeneralContactSpec>();
  spec->points = {{link, Point3(-.1, -.1, 0), true},
                  {link, Point3(.1, -.1, .3), false},
                  {link, Point3(.1, .1, .3), false}};
  Values measurements;
  for (size_t k = 4; k <= 5; ++k)
    InsertJointAngle(&measurements, robot.joint("j1")->id(), k, 0.0);
  contact_init::Options options;
  options.model = contact_init::Model::Patch;
  contact_init::Result diagnostics;
  const auto result = Initializer().InitializeSolutionContacts(
      robot, "l1", Pose3(Rot3::Rx(-std::acos(-1.0) / 2), Point3(0, 0, 1)),
      measurements, Phases({Phase(4, 5, spec)}), .01, options, &diagnostics);
  EXPECT_LONGS_EQUAL(3, diagnostics.landmarks.size());
  EXPECT_LONGS_EQUAL(
      3, diagnostics.graph.measurements.front().observations.size());
  EXPECT(diagnostics.residualRms < 1e-8);
  EXPECT(assert_equal(spec->points[1].predict(result, 4),
                      spec->points[1].predict(result, 5), 1e-8));
  // The non-ground contacts remain above the ground contact.
  DOUBLES_EQUAL(.3,
                spec->points[1].predict(result, 4).z() -
                    spec->points[0].predict(result, 4).z(),
                1e-8);
}

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
  const auto trajectory = Phases({ContactPhase(4, 8, patch)});
  const Pose3 initial_root(Rot3::RzRyRx(0.12, -0.23, 0.34),
                           Point3(1.2, -0.8, 1.5));
  contact_init::Options options;
  options.model = contact_init::Model::Patch;
  options.perturb = true;
  contact_init::Result diagnostics;
  Initializer initializer;
  const auto result = initializer.InitializeSolutionContacts(
      robot, "l1", initial_root, measurements, trajectory, 0.01, options,
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

// Reject malformed schedules and missing or invalid joint/contact inputs.
TEST(ContactTrajectoryInitializer, InvalidPhasesAndMeasurementsRejected) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto foot = robot.link("l2");
  const PointOnLinks patch{{foot, Point3(-.1, -.1, 0)},
                           {foot, Point3(.1, -.1, 0)},
                           {foot, Point3(.1, .1, 0)}};
  Values measurements;
  InsertJointAngle(&measurements, robot.joint("j1")->id(), 4, 0.0);
  auto initialize = [&](const Trajectory& trajectory) {
    return Initializer().InitializeSolutionContacts(
        robot, "l1", Pose3(), measurements, trajectory, .01);
  };
  EXPECT(Rejects([&] { initialize(Phases({ContactPhase(4, 8, patch)})); },
                 "missing joint angle"));
  EXPECT(Rejects([&] { initialize(Trajectory()); }, "contain phases"));
  EXPECT(Rejects([&] { initialize(Phases({ContactPhase(8, 4, patch)})); },
                 "reversed"));
  EXPECT(Rejects(
      [&] {
        initialize(
            Phases({ContactPhase(4, 5, patch), ContactPhase(7, 8, patch)}));
      },
      "contiguous"));
  EXPECT(Rejects(
      [&] {
        initialize(
            Phases({ContactPhase(4, 7, patch), ContactPhase(6, 8, patch)}));
      },
      "contiguous"));
  EXPECT(Rejects([&] { initialize(Phases({Phase(4, 8, nullptr)})); },
                 "ConstraintSpec"));
  EXPECT(Rejects([&] { initialize(Phases({ContactPhase(4, 8, {})})); },
                 "active contact"));
  auto duplicate = patch;
  duplicate.push_back(patch.front());
  EXPECT(Rejects([&] { initialize(Phases({ContactPhase(4, 8, duplicate)})); },
                 "duplicate"));
  const auto other_robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  EXPECT(Rejects(
      [&] {
        initialize(Phases(
            {ContactPhase(4, 8, {{other_robot.link("l2"), Point3::Zero()}})}));
      },
      "another robot"));
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
      robot, "l1", Pose3(), measurements, Phases({ContactPhase(4, 38, patch)}),
      0.1, options, &diagnostics);
  const std::vector<size_t> expected{4, 14, 24, 34, 38};
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
      robot, "l1", anchor, measurements, Phases({ContactPhase(4, 6, patch)}),
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
  for (size_t sample : {4, 5, 6, 7, 8, 9}) {
    InsertJointAngle(&measurements, joint->id(), sample, 0.0);
  }
  Initializer initializer;
  contact_init::Options options;
  options.model = contact_init::Model::Point;
  bool disconnected_rejected = false;
  try {
    initializer.InitializeSolutionContacts(
        robot, "l1", Pose3(), measurements,
        Phases({ContactPhase(4, 5, point), ContactPhase(6, 7, {}),
                ContactPhase(8, 9, point)}),
        0.01, options);
  } catch (const std::invalid_argument& error) {
    disconnected_rejected =
        std::string(error.what()).find("disconnected") != std::string::npos;
  }
  EXPECT(disconnected_rejected);
}

// Changing the other foot or reordering contact points must not split a
// stance.
TEST(ContactTrajectoryInitializer, PhaseContinuityAndPointReentry) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto foot = robot.link("l2"), root = robot.link("l1");
  const PointOnLinks patch{{foot, Point3(-.1, -.1, 0)},
                           {foot, Point3(.1, -.1, 0)},
                           {foot, Point3(.1, .1, 0)},
                           {foot, Point3(-.1, .1, 0)}};
  PointOnLinks double_support(patch.rbegin(), patch.rend());
  double_support.insert(double_support.end(), {{root, Point3(-.1, -.1, 0)},
                                               {root, Point3(.1, -.1, 0)},
                                               {root, Point3(.1, .1, 0)}});
  Values measured;
  for (size_t k = 4; k <= 9; ++k)
    InsertJointAngle(&measured, robot.joint("j1")->id(), k, 0.0);
  contact_init::Options options;
  options.model = contact_init::Model::Patch;
  contact_init::Result result;
  Initializer initializer;
  initializer.InitializeSolutionContacts(
      robot, "l1", Pose3(), measured,
      Phases({ContactPhase(4, 5, patch), ContactPhase(6, 7, double_support),
              ContactPhase(8, 9, patch)}),
      .01, options, &result);
  EXPECT_LONGS_EQUAL(7, result.landmarks.size());
  EXPECT_LONGS_EQUAL(1, result.graph.contactComponents.size());
  EXPECT(result.residualRms < 1e-8);
  // A point absent for a phase gets a fresh landmark even when its link stays.
  double_support.erase(double_support.begin());
  initializer.InitializeSolutionContacts(
      robot, "l1", Pose3(), measured,
      Phases({ContactPhase(4, 5, patch), ContactPhase(6, 7, double_support),
              ContactPhase(8, 9, patch)}),
      .01, options, &result);
  EXPECT_LONGS_EQUAL(8, result.landmarks.size());
}

// Shared endpoints are deduplicated and flight prefixes/suffixes are seeded.
TEST(ContactTrajectoryInitializer, SharedEndpointsAndFlightSeeds) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto foot = robot.link("l2"), root = robot.link("l1");
  const PointOnLinks patch{{foot, Point3(-.1, -.1, 0)},
                           {foot, Point3(.1, -.1, 0)},
                           {foot, Point3(.1, .1, 0)}};
  Values measured;
  for (size_t k = 4; k <= 9; ++k)
    InsertJointAngle(&measured, robot.joint("j1")->id(), k, .02 * k);
  contact_init::Options options;
  options.model = contact_init::Model::Patch;
  contact_init::Result diagnostics;
  const Pose3 anchor(Rot3(), Point3(1, 2, 3));
  const auto result = Initializer().InitializeSolutionContacts(
      robot, "l1", anchor, measured,
      Phases({ContactPhase(4, 4, {}), ContactPhase(5, 6, patch),
              ContactPhase(6, 7, patch), ContactPhase(8, 9, {})}),
      .01, options, &diagnostics);
  EXPECT_LONGS_EQUAL(3, diagnostics.graph.measurements.size());
  EXPECT_LONGS_EQUAL(3, diagnostics.graph.measurements[1].observations.size());
  EXPECT_LONGS_EQUAL(3, diagnostics.landmarks.size());
  EXPECT(assert_equal(anchor, Pose(result, root->id(), 4), 1e-10));
  EXPECT(assert_equal(anchor, Pose(result, root->id(), 5), 1e-10));
  EXPECT(assert_equal(Pose(result, root->id(), 7), Pose(result, root->id(), 9),
                      1e-10));
  for (size_t k = 4; k <= 9; ++k)
    DOUBLES_EQUAL(JointAngle(measured, robot.joint("j1")->id(), k),
                  JointAngle(result, robot.joint("j1")->id(), k), 0.0);
}

// A new stance after flight gets new landmarks, even with motion
// regularization.
TEST(ContactTrajectoryInitializer, FlightBreaksLandmarksEvenWithMotionPrior) {
  const auto robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  const auto foot = robot.link("l2");
  const PointOnLinks patch{{foot, Point3(-.1, -.1, 0)},
                           {foot, Point3(.1, -.1, 0)},
                           {foot, Point3(.1, .1, 0)}};
  Values measured;
  for (size_t k = 4; k <= 9; ++k)
    InsertJointAngle(&measured, robot.joint("j1")->id(), k, 0.0);
  contact_init::Options options;
  options.model = contact_init::Model::Patch;
  options.addBetweenFactors = true;
  contact_init::Result diagnostics;
  Initializer().InitializeSolutionContacts(
      robot, "l1", Pose3(), measured,
      Phases({ContactPhase(4, 5, patch), ContactPhase(6, 7, {}),
              ContactPhase(8, 9, patch)}),
      .01, options, &diagnostics);
  EXPECT_LONGS_EQUAL(6, diagnostics.landmarks.size());
  EXPECT_LONGS_EQUAL(2, diagnostics.graph.contactComponents.size());
  EXPECT_LONGS_EQUAL(1, diagnostics.graph.components.size());
}

}  // namespace contact_trajectory
/* ************************************************************************* */

/* ************************************************************************* */
namespace buffered_contacts {
using contact_trajectory::ContactPhase;
using contact_trajectory::Phases;
using contact_trajectory::Rejects;

struct Fixture {
  Robot robot = CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  PointOnLinks foot{{robot.link("l2"), Point3(-.1, -.1, 0)},
                    {robot.link("l2"), Point3(.1, -.1, 0)},
                    {robot.link("l2"), Point3(.1, .1, 0)}};
  PointOnLinks other{{robot.link("l1"), Point3(-.1, -.1, 0)},
                     {robot.link("l1"), Point3(.1, -.1, 0)},
                     {robot.link("l1"), Point3(.1, .1, 0)}};
  Pose3 anchor{Rot3::RzRyRx(.1, -.2, .3), Point3(1, 2, 3)};
  Values measured;
  contact_init::Options options;

  Fixture() {
    options.model = contact_init::Model::Patch;
    for (size_t k = 4; k <= 38; ++k)
      InsertJointAngle(&measured, robot.joint("j1")->id(), k, 0.0);
  }

  Values initialize(const Trajectory& trajectory, size_t buffer,
                    contact_init::Result* result, double dt = .01) const {
    return Initializer().InitializeSolutionContacts(
        robot, "l1", anchor, measured, trajectory, dt, options, result, buffer);
  }
};

// Zero preserves the legacy selection exactly; buffers 1/2/3 move both ends.
TEST(ContactTrajectoryInitializer, OptionalSliceBufferAndDensePrefix) {
  Fixture f;
  const auto trajectory = Phases({ContactPhase(4, 14, f.foot)});
  contact_init::Result legacy, zero;
  const auto expected = Initializer().InitializeSolutionContacts(
      f.robot, "l1", f.anchor, f.measured, trajectory, .01, f.options, &legacy);
  const auto unchanged = f.initialize(trajectory, 0, &zero);
  EXPECT(assert_equal(expected, unchanged, 1e-12));
  EXPECT_LONGS_EQUAL(legacy.graph.measurements.size(),
                     zero.graph.measurements.size());
  for (size_t i = 0; i < legacy.graph.measurements.size(); ++i)
    EXPECT_LONGS_EQUAL(legacy.graph.measurements[i].slice.k,
                       zero.graph.measurements[i].slice.k);
  for (size_t buffer : {1, 2, 3}) {
    contact_init::Result result;
    const auto values = f.initialize(trajectory, buffer, &result);
    EXPECT_LONGS_EQUAL(2, result.graph.measurements.size());
    EXPECT_LONGS_EQUAL(4 + buffer, result.graph.measurements.front().slice.k);
    EXPECT_LONGS_EQUAL(14 - buffer, result.graph.measurements.back().slice.k);
    for (size_t k = 4; k <= 14; ++k) {
      EXPECT(assert_equal(f.anchor, Pose(values, f.robot.link("l1")->id(), k),
                          1e-8));
      DOUBLES_EQUAL(0.0, JointAngle(values, f.robot.joint("j1")->id(), k), 0);
      EXPECT(values.exists(ContactWrenchKey(f.robot.link("l2")->id(), 0, k)));
    }
  }
}

// A continuous foot stance spans phases; the other foot cannot leak transients.
TEST(ContactTrajectoryInitializer, BufferFiltersEachFootIndependently) {
  Fixture f;
  PointOnLinks both = f.foot;
  both.insert(both.end(), f.other.begin(), f.other.end());
  contact_init::Result result;
  f.initialize(Phases({ContactPhase(4, 7, f.foot), ContactPhase(8, 16, both),
                       ContactPhase(17, 20, f.foot)}),
               2, &result);
  const std::vector<size_t> expected{6, 7, 8, 10, 14, 16, 17, 18};
  EXPECT_LONGS_EQUAL(expected.size(), result.graph.measurements.size());
  EXPECT_LONGS_EQUAL(6, result.landmarks.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    const auto& measurement = result.graph.measurements[i];
    EXPECT_LONGS_EQUAL(expected[i], measurement.slice.k);
    const bool otherTrusted =
        measurement.slice.k >= 10 && measurement.slice.k <= 14;
    EXPECT_LONGS_EQUAL(otherTrusted ? 6 : 3, measurement.observations.size());
    for (const auto& observation : measurement.observations)
      EXPECT(observation.foot == "l2" || otherTrusted);
  }
}

// Keep the one-second supplemental samples in addition to trimmed endpoints.
TEST(ContactTrajectoryInitializer, BufferedLongStanceSupplementalSlices) {
  Fixture f;
  contact_init::Result result;
  f.initialize(Phases({ContactPhase(4, 38, f.foot)}), 2, &result, .1);
  const std::vector<size_t> expected{6, 14, 24, 34, 36};
  EXPECT_LONGS_EQUAL(expected.size(), result.graph.measurements.size());
  for (size_t i = 0; i < expected.size(); ++i)
    EXPECT_LONGS_EQUAL(expected[i], result.graph.measurements[i].slice.k);
}

// Never shrink the requested margin to retain a short stance; zero is legacy.
TEST(ContactTrajectoryInitializer, BufferRequiresTwoInteriorSlices) {
  Fixture f;
  contact_init::Result result;
  f.initialize(Phases({ContactPhase(4, 7, f.foot)}), 1, &result);
  EXPECT_LONGS_EQUAL(2, result.graph.measurements.size());
  for (size_t last : {4, 5, 6, 7}) {
    EXPECT(Rejects(
        [&] {
          f.initialize(Phases({ContactPhase(4, last, f.foot)}), 2, &result);
        },
        "fewer than two slices"));
  }
  EXPECT(Rejects(
      [&] {
        f.initialize(Phases({ContactPhase(4, 38, f.foot)}),
                     std::numeric_limits<size_t>::max(), &result);
      },
      "fewer than two slices"));
  f.initialize(Phases({ContactPhase(4, 4, f.foot)}), 0, &result);
  EXPECT_LONGS_EQUAL(1, result.graph.measurements.size());
}

// Trimming a shared endpoint must expose lost connectivity, not restore it.
TEST(ContactTrajectoryInitializer, BufferCanDisconnectSharedEndpoint) {
  Fixture f;
  contact_init::Result result;
  const auto trajectory =
      Phases({ContactPhase(4, 10, f.foot), ContactPhase(10, 16, f.other)});
  f.initialize(trajectory, 0, &result);
  EXPECT_LONGS_EQUAL(1, result.graph.contactComponents.size());
  EXPECT(
      Rejects([&] { f.initialize(trajectory, 1, &result); }, "disconnected"));
  f.options.addBetweenFactors = true;
  f.initialize(trajectory, 1, &result);
  EXPECT_LONGS_EQUAL(2, result.graph.contactComponents.size());
  EXPECT_LONGS_EQUAL(1, result.graph.components.size());
  for (const auto& measurement : result.graph.measurements)
    EXPECT(measurement.slice.k != 10);
}
}  // namespace buffered_contacts
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
