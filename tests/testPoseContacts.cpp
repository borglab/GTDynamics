/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/kinematics/ContactHeightFactor.h>
#include <gtdynamics/kinematics/ContactPoseEqualityFactor.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/ContactInitializationFactors.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/PoseContactConstraintSpec.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
using namespace gtdynamics;
using namespace gtsam;
/* ************************************************************************* */
namespace pose_contacts {
Robot robot() {
  return CreateRobotFromFile(kUrdfPath +
                             std::string("test/simple_urdf_prismatic.urdf"));
}
Phase phase(size_t first, size_t last,
            const std::vector<PoseOnLink>& contacts) {
  return Phase(first, last,
               std::make_shared<PoseContactConstraintSpec>(contacts));
}
// Nonidentity COM offsets and rotations are included in both analytic
// Jacobians.
TEST(PoseContacts, JacobiansAndFrames) {
  const auto r = robot();
  const PoseOnLink contact(
      r.link("l2"), Pose3(Rot3::RzRyRx(.2, -.1, .3), Point3(.2, .1, -.2)));
  ContactPoseEqualityFactor factor(contact, noiseModel::Isotropic::Sigma(6, 1),
                                   0, 1);
  const Pose3 a(Rot3::RzRyRx(.4, .2, -.1), Point3(1, 2, 3)),
      b(Rot3::RzRyRx(.1, -.3, .7), Point3(2, 1, 3));
  Matrix H1, H2;
  factor.evaluateError(a, b, &H1, &H2);
  std::function<Vector6(const Pose3&, const Pose3&)> f =
      [&](const auto& x, const auto& y) { return factor.evaluateError(x, y); };
  EXPECT(assert_equal(numericalDerivative21<Vector6, Pose3, Pose3>(f, a, b), H1,
                      1e-6));
  EXPECT(assert_equal(numericalDerivative22<Vector6, Pose3, Pose3>(f, a, b), H2,
                      1e-6));
  ContactNormalFactor normal(1, contact.comTcontact, Vector3::UnitZ(),
                             noiseModel::Isotropic::Sigma(3, 1));
  Matrix H;
  normal.evaluateError(a, &H);
  std::function<Vector3(const Pose3&)> n = [&](const auto& x) {
    return normal.evaluateError(x);
  };
  EXPECT(assert_equal(numericalDerivative11<Vector3, Pose3>(n, a), H, 1e-6));
  EXPECT(!contact.equals(PoseOnLink(contact.link, contact.comTcontact, false)));
}
// Ground alignment leaves world XY and yaw free; non-ground contacts are
// skipped.
TEST(PoseContacts, GroundAndPhaseContinuity) {
  const auto r = robot();
  const PoseOnLink c(r.link("l2"), Pose3());
  const Trajectory t(WalkCycle({phase(0, 2, {c}), phase(3, 4, {c}),
                                phase(5, 5, {}), phase(6, 7, {c})}),
                     1);
  const Kinematics kinematics;
  const auto graph = kinematics.contactPoseEqualityFactors(
      t, noiseModel::Isotropic::Sigma(6, 1));
  EXPECT_LONGS_EQUAL(5, graph.size());
  const auto ground =
      kinematics.contactPoseGroundObjectives(Slice(0), {c}, Vector3(0, 0, -1));
  Values values;
  values.insert(PoseKey(c.link->id(), 0),
                Pose3(Rot3::Rz(1.3), Point3(2, 3, 0)));
  DOUBLES_EQUAL(0, ground.error(values), 1e-12);
  EXPECT_LONGS_EQUAL(0, kinematics
                            .contactPoseGroundObjectives(
                                Slice(0), {PoseOnLink(c.link, Pose3(), false)},
                                Vector3(0, 0, -1))
                            .size());
}
// Height and sole normal share the signed upward axis for tilted gravity.
TEST(PoseContacts, TiltedGroundHeight) {
  const auto r = robot();
  const PoseOnLink contact(r.link("l2"), Pose3(Rot3(), Point3(.2, -.1, .3)));
  const Vector3 gravity(1, 0, -1), up = -gravity.normalized();
  const Rot3 rotation = Rot3::Ry(-M_PI / 4);
  const double height = .4, heightSigma = .2;
  const auto graph = Kinematics().contactPoseGroundObjectives(
      Slice(0), {contact}, gravity, height, heightSigma, .1);
  const Point3 worldContact = Point3(1, 0, 1) + height * up;
  const Key key = PoseKey(contact.link->id(), 0);
  Values values;
  values.insert(
      key,
      Pose3(rotation,
            worldContact - rotation.rotate(contact.comTcontact.translation())));
  DOUBLES_EQUAL(0, graph.error(values), 1e-12);
  const auto pose = values.at<Pose3>(key);
  values.update(key, Pose3(rotation, pose.translation() + .3 * up));
  DOUBLES_EQUAL(.5 * std::pow(.3 / heightSigma, 2), graph.error(values), 1e-12);
}
// A role change must not carry a ground landmark into a non-ground stance.
TEST(PoseContacts, GroundRoleBreaksContinuity) {
  const auto r = robot();
  const PoseOnLink ground(r.link("l2"), Pose3(), true);
  const PoseOnLink other(r.link("l2"), Pose3(), false);
  const Trajectory trajectory(
      WalkCycle(
          {phase(0, 1, {ground}), phase(2, 3, {other}), phase(4, 5, {ground})}),
      1);
  const auto frames = contactPoseFrames(trajectory);
  EXPECT(frames.at(0).front().stance != frames.at(2).front().stance);
  EXPECT(frames.at(2).front().stance != frames.at(4).front().stance);
  EXPECT_LONGS_EQUAL(3, Kinematics()
                            .contactPoseEqualityFactors(
                                trajectory, noiseModel::Isotropic::Sigma(6, 1))
                            .size());
}
// Buffered pose initialization returns actual source slices and preserves
// FK/gauge.
TEST(PoseContacts, BufferedInitializationAndRecontact) {
  const auto r = robot();
  const PoseOnLink c(r.link("l2"), Pose3(Rot3(), Point3(.1, 0, 0)), false);
  Values measured;
  for (size_t k = 4; k <= 12; ++k)
    InsertJointAngle(&measured, r.joint("j1")->id(), k, .1 * (k - 4));
  const Trajectory t(WalkCycle({phase(4, 8, {c}), phase(9, 12, {c})}), 1);
  pose_contact_init::Options options;
  options.addGroundFactors = false;
  pose_contact_init::Result result;
  const Pose3 anchor(Rot3::Rz(.2), Point3(1, 2, 3));
  const auto values = Initializer().InitializeSolutionContactPoses(
      r, "l1", anchor, measured, t, .02, options, &result, 1);
  EXPECT_LONGS_EQUAL(5, result.slices().front().k);
  EXPECT_LONGS_EQUAL(11, result.slices().back().k);
  EXPECT_LONGS_EQUAL(1, result.contactPoses.size());
  EXPECT_LONGS_EQUAL(1, result.contactFrames.size());
  EXPECT(result.contactFrames.front().equals(c));
  EXPECT_LONGS_EQUAL(result.slices().size(), result.observations.size());
  DOUBLES_EQUAL(result.objectiveFinal, 2 * result.graph.error(result.values),
                1e-10);
  for (const auto& observation : result.observations) {
    EXPECT_LONGS_EQUAL(0, observation.contactIndex);
    EXPECT(assert_equal(
        result.poses.at(observation.poseIndex) * observation.rootTcontact,
        result.contactPoses.at(observation.contactIndex), 1e-7));
  }
  EXPECT(assert_equal(anchor, Pose(values, r.link("l1")->id(), 5), 1e-10));
  EXPECT(result.residualRms < 1e-8);
  EXPECT(result.rotationRms < 1e-8);
  for (size_t k = 5; k <= 11; ++k)
    EXPECT(assert_equal(c.predict(values, 5), c.predict(values, k), 1e-7));
  const Trajectory disconnected(
      WalkCycle({phase(4, 6, {c}), phase(7, 8, {}), phase(9, 12, {c})}), 1);
  bool rejected = false;
  try {
    Initializer().InitializeSolutionContactPoses(r, "l1", anchor, measured,
                                                 disconnected, .02, options);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
  options.addBetweenFactors = true;
  Initializer().InitializeSolutionContactPoses(
      r, "l1", anchor, measured, disconnected, .02, options, &result);
  EXPECT_LONGS_EQUAL(2, result.contactPoses.size());
}
// Short, one-slice, and long double stance retain central or buffered
// endpoints; single stance is skipped and short phases survive any buffer.
TEST(PoseContacts, DoubleSupportSelection) {
  const auto r = robot();
  const PoseOnLink left(r.link("l1"), Pose3()), right(r.link("l2"), Pose3());
  const Trajectory t(
      WalkCycle({phase(0, 0, {left, right}), phase(1, 4, {left}),
                 phase(5, 9, {left, right}), phase(10, 14, {right}),
                 phase(15, 34, {left, right})}),
      1);
  const auto selected = doubleSupportSlices(t, .02, .2, 1);
  EXPECT_LONGS_EQUAL(4, selected.size());
  for (size_t i = 0; i < 4; ++i)
    EXPECT_LONGS_EQUAL((std::vector<size_t>{0, 7, 16, 33})[i], selected[i].k);
  CHECK_EXCEPTION(doubleSupportSlices(t, 0), std::invalid_argument);
  CHECK_EXCEPTION(doubleSupportSlices(t, .02, -1), std::invalid_argument);
}
// Sparse output contains only requested slices; footholds retain identities
// through skipped single-stance phases and reset after lift-off/recontact.
TEST(PoseContacts, ExplicitSparseInitialization) {
  const auto r = robot();
  const PoseOnLink left(r.link("l1"), Pose3()), right(r.link("l2"), Pose3());
  const Trajectory t(
      WalkCycle({phase(0, 2, {left, right}), phase(3, 5, {left}),
                 phase(6, 8, {left, right}), phase(9, 11, {right}),
                 phase(12, 14, {left, right})}),
      1);
  Values measured;
  for (size_t k = 0; k < 15; ++k)
    InsertJointAngle(&measured, r.joint("j1")->id(), k, 0);
  pose_contact_init::Options options;
  options.selectedSlices = doubleSupportSlices(t, .02);
  options.interpolateOutput = false;
  options.addGroundFactors = false;
  pose_contact_init::Result result;
  const auto values = Initializer().InitializeSolutionContactPoses(
      r, "l1", Pose3(), measured, t, .02, options, &result);
  EXPECT_LONGS_EQUAL(3, result.poses.size());
  EXPECT_LONGS_EQUAL(4, result.contactPoses.size());
  EXPECT_LONGS_EQUAL(6, result.observations.size());
  for (size_t k = 0; k < 15; ++k)
    EXPECT(values.exists(PoseKey(r.link("l1")->id(), k)) ==
           (k == 1 || k == 7 || k == 13));
  EXPECT_LONGS_EQUAL(result.observations[0].contactIndex,
                     result.observations[2].contactIndex);
  EXPECT(result.observations[1].contactIndex !=
         result.observations[3].contactIndex);
  EXPECT_LONGS_EQUAL(result.observations[3].contactIndex,
                     result.observations[5].contactIndex);
  // Omitting an entire old foothold leaves compact, valid diagnostic indices.
  options.selectedSlices = {Slice(7), Slice(13)};
  Initializer().InitializeSolutionContactPoses(r, "l1", Pose3(), measured, t,
                                               .02, options, &result);
  EXPECT_LONGS_EQUAL(3, result.contactFrames.size());
  EXPECT_LONGS_EQUAL(3, result.contactPoses.size());
  for (const auto& observation : result.observations)
    EXPECT(assert_equal(
        result.poses.at(observation.poseIndex) * observation.rootTcontact,
        result.contactPoses.at(observation.contactIndex), 1e-8));
  options.selectedSlices = {Slice(7), Slice(1)};
  CHECK_EXCEPTION(Initializer().InitializeSolutionContactPoses(
                      r, "l1", Pose3(), measured, t, .02, options),
                  std::invalid_argument);
}
// Optional root upright and height-only priors reuse existing factors and keep
// the exact gauge fixed. Default sole-normal behavior remains available.
TEST(PoseContacts, SharedUprightAndHeightPriors) {
  const auto r = robot();
  const PoseOnLink c(r.link("l2"), Pose3(), true);
  Values measured;
  for (size_t k = 0; k < 4; ++k)
    InsertJointAngle(&measured, r.joint("j1")->id(), k, 0.0);
  const Trajectory t(WalkCycle({phase(0, 3, {c})}), 1);
  pose_contact_init::Options options;
  options.selectedSlices = {Slice(0), Slice(3)};
  options.addUprightFactors = true;
  options.addNormalFactors = false;
  options.addBetweenFactors = true;
  options.groundHeight = 0.0;
  options.groundSigma = 0.1;
  pose_contact_init::Result result;
  const Pose3 anchor(Rot3::Rz(0.3), Point3(1, 2, 0));
  Initializer().InitializeSolutionContactPoses(r, "l1", anchor, measured, t,
                                               .02, options, &result);
  size_t upright = 0, height = 0, normal = 0;
  for (const auto& factor : result.graph) {
    upright +=
        dynamic_cast<contact_init::UprightFactor*>(factor.get()) != nullptr;
    height += dynamic_cast<ContactHeightFactor*>(factor.get()) != nullptr;
    normal += dynamic_cast<ContactNormalFactor*>(factor.get()) != nullptr;
  }
  EXPECT_LONGS_EQUAL(1, upright);
  EXPECT_LONGS_EQUAL(1, height);
  EXPECT_LONGS_EQUAL(0, normal);
  EXPECT(assert_equal(anchor, result.poses.front(), 1e-12));
  EXPECT(std::isfinite(result.objectiveFinal));
}

// Sole rocking must not accumulate into root tilt in upright starting values.
// This option changes only the start, independently of the upright prior.
TEST(PoseContacts, UprightStartRejectsAccumulatedRocking) {
  const auto r =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));
  const PoseOnLink contact(r.link("l2"), Pose3(), false);
  const Trajectory trajectory(WalkCycle({phase(0, 2, {contact})}), 1);
  Values measured;
  for (size_t k = 0; k < 3; ++k)
    InsertJointAngle(&measured, r.joint("j1")->id(), k,
                     k * 30.0 * 3.141592653589793 / 180.0);
  pose_contact_init::Options options;
  options.selectedSlices = {Slice(0), Slice(1), Slice(2)};
  options.addGroundFactors = false;
  options.interpolateOutput = false;
  options.maxIterations = 1;
  const Pose3 anchor(Rot3::Rz(0.3), Point3(1, 2, 3));
  pose_contact_init::Result original, upright;
  Initializer().InitializeSolutionContactPoses(
      r, "l1", anchor, measured, trajectory, .02, options, &original);
  options.initializeUpright = true;
  Initializer().InitializeSolutionContactPoses(
      r, "l1", anchor, measured, trajectory, .02, options, &upright);
  EXPECT_LONGS_EQUAL(original.graph.size(), upright.graph.size());
  EXPECT(Rot3::Logmap(anchor.rotation().between(
                          original.history.front().poses.back().rotation()))
             .norm() > 0.5);
  const auto& start = upright.history.front();
  for (const auto& pose : start.poses)
    EXPECT(assert_equal(anchor.rotation(), pose.rotation(), 1e-9));
  for (const auto& observation : upright.observations)
    EXPECT(assert_equal(
        (start.poses[observation.poseIndex] * observation.rootTcontact)
            .translation(),
        start.contactPoses[observation.contactIndex].translation(), 1e-9));
  EXPECT(assert_equal(anchor, start.poses.front(), 1e-12));
}

}  // namespace pose_contacts
/* ************************************************************************* */
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
