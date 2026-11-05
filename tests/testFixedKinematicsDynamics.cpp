/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file testFixedKinematicsDynamics.cpp
 * @brief Test fixed-configuration trajectory dynamics and acceleration priors.
 * @author GTDynamics contributors
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/FixedKinematicsDynamics.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/utils/PoseContactConstraintSpec.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/base/TestableAssertions.h>

#include <cmath>

using namespace gtdynamics;
using namespace gtsam;

/* ************************************************************************* */
namespace static_support {
struct Fixture {
  LinkSharedPtr body = std::make_shared<Link>(
      0, "floating", 8.0, Matrix3::Identity(), Pose3(), Pose3());
  LinkSharedPtr leftLink = std::make_shared<Link>(
      1, "left", 1.0, Matrix3::Identity(), Pose3(), Pose3());
  LinkSharedPtr rightLink = std::make_shared<Link>(
      2, "right", 1.0, Matrix3::Identity(), Pose3(), Pose3());
  JointSharedPtr leftJoint = std::make_shared<RevoluteJoint>(
      0, "left_joint", Pose3(), body, leftLink, Vector3::UnitZ());
  JointSharedPtr rightJoint = std::make_shared<RevoluteJoint>(
      1, "right_joint", Pose3(), body, rightLink, Vector3::UnitZ());
  Robot robot{
      LinkMap{{"floating", body}, {"left", leftLink}, {"right", rightLink}},
      JointMap{{"left_joint", leftJoint}, {"right_joint", rightJoint}}};
  PoseOnLink left{leftLink, Pose3(Rot3(), Point3(0, 0.1, -0.5))};
  PoseOnLink right{rightLink, Pose3(Rot3(), Point3(0, -0.1, -0.5))};
  Trajectory trajectory{
      WalkCycle({Phase(0, 2,
                       std::make_shared<PoseContactConstraintSpec>(
                           std::vector<PoseOnLink>{left, right}))}),
      1};
  Values fixed, priors;
  Fixture() {
    body->addJoint(leftJoint);
    body->addJoint(rightJoint);
    leftLink->addJoint(leftJoint);
    rightLink->addJoint(rightJoint);
    for (size_t k = 0; k < 3; ++k) {
      for (const auto& link : robot.links()) {
        fixed.insert(PoseKey(link->id(), k), Pose3(Rot3(), Point3(0, 0, 0.5)));
        fixed.insert(TwistKey(link->id(), k), Vector(Vector6::Zero()));
        if (link == body)
          priors.insert(TwistAccelKey(link->id(), k), Vector(Vector6::Zero()));
      }
      for (const auto& joint : robot.joints()) {
        fixed.insert(JointAngleKey(joint->id(), k), 0.0);
        fixed.insert(JointVelKey(joint->id(), k), 0.0);
        priors.insert(JointAccelKey(joint->id(), k), 0.0);
      }
    }
  }
  fixed_dynamics::Options tightPriors() const {
    fixed_dynamics::Options options;
    options.baseAngularAccelerationSigmaDegrees = .01;
    options.baseLinearAccelerationSigma = .001;
    options.jointAccelerationSigmaDegrees = .01;
    return options;
  }
  std::vector<fixed_dynamics::Contact> contacts() const {
    return {{left, .1, .04, .6}, {right, .1, .04, .6}};
  }
};

// Global CMC-Opt with affine ReducedAL retractions preserves static support,
// uses actual unequal sample intervals, and never changes fixed poses/twists.
TEST(FixedKinematicsDynamics, StaticSupportTrajectory) {
  Fixture fixture;
  const auto before = fixture.fixed;
  const auto result = solveFixedKinematicsDynamics(
      fixture.robot, fixture.trajectory, {Slice(0), Slice(1), Slice(2)},
      {0.0, .08, .3}, fixture.fixed, fixture.priors, fixture.contacts(),
      Vector(), fixture.tightPriors());
  EXPECT(result.feasible);
  EXPECT(!result.softDynamics);
  EXPECT_LONGS_EQUAL(42, result.reducedDimension);
  EXPECT_LONGS_EQUAL(6, result.contactKeys.size());
  EXPECT(result.numFeasibleTrials + result.numFeasibleProjections >= 3);
  EXPECT(result.equalityL2 < 1e-8);
  EXPECT(result.inequalityL2 < 1e-8);
  EXPECT_LONGS_EQUAL(6, result.torqueKeys.size());
  EXPECT(result.maxForceResidual < 1e-8);
  EXPECT(result.maxMomentResidual < 1e-8);
  EXPECT(assert_equal(before, fixture.fixed));
  for (size_t k = 0; k < 3; ++k) {
    const Vector6 left = result.values.at<Vector6>(result.contactKeys[2 * k]);
    const Vector6 right =
        result.values.at<Vector6>(result.contactKeys[2 * k + 1]);
    EXPECT_DOUBLES_EQUAL(49.05, left(5), 1e-5);
    EXPECT_DOUBLES_EQUAL(49.05, right(5), 1e-5);
    EXPECT(left.head<5>().norm() < 1e-5);
    EXPECT(right.head<5>().norm() < 1e-5);
  }
}

// A single QR seed respects changing support, exact mechanics and cone faces.
// It rejects unsupported soft dynamics instead of silently relaxing mechanics.
TEST(FixedKinematicsDynamics, LinearSeedAndConstraintChecks) {
  Fixture f;
  const auto phase = [](size_t k, const std::vector<PoseOnLink>& contacts) {
    return Phase(k, k, std::make_shared<PoseContactConstraintSpec>(contacts));
  };
  const Trajectory trajectory(
      WalkCycle({phase(0, {f.left}), phase(1, {f.left, f.right}),
                 phase(2, {f.right})}),
      1);
  const auto result = fixedKinematicsDynamicsSeed(
      f.robot, trajectory, {Slice(0), Slice(1), Slice(2)}, {0., .08, .3},
      f.fixed, f.priors, f.contacts(), Vector(), {});
  EXPECT(result.feasible);
  EXPECT(result.maxForceResidual < 1e-8);
  EXPECT(result.maxMomentResidual < 1e-8);
  EXPECT_LONGS_EQUAL(0, result.iterations);
  EXPECT_LONGS_EQUAL(0, result.numALSolves);
  EXPECT_DOUBLES_EQUAL(
      98.1, result.values.at<Vector6>(result.contactKeys[0])(5), 1e-8);
  EXPECT_DOUBLES_EQUAL(
      49.05, result.values.at<Vector6>(result.contactKeys[2])(5), 1e-8);
  EXPECT(result.values.at<Vector6>(result.contactKeys[1]).norm() < 1e-8);
  EXPECT(result.values.at<Vector6>(result.contactKeys[4]).norm() < 1e-8);
  EXPECT_DOUBLES_EQUAL(result.accelerationError, result.objectiveFinal, 1e-12);
  EXPECT_DOUBLES_EQUAL(0, result.effortError, 0);
  EXPECT_DOUBLES_EQUAL(0, result.wrenchSmoothError, 0);
  EXPECT(result.linearSolveSeconds > 0);
  EXPECT(result.linearizationSeconds + result.linearSolveSeconds +
             result.valueAssemblySeconds <=
         result.initializationSeconds);
  // Full-optimizer parameters do not affect seed construction or validation.
  fixed_dynamics::Options unusedObjectives;
  unusedObjectives.maxIterations = 0;
  unusedObjectives.effortSigma = 0;
  unusedObjectives.wrenchSmoothSigma = 0;
  const auto sameSeed = fixedKinematicsDynamicsSeed(
      f.robot, trajectory, {Slice(0), Slice(1), Slice(2)}, {0., .08, .3},
      f.fixed, f.priors, f.contacts(), Vector(), unusedObjectives);
  EXPECT(assert_equal(result.values, sameSeed.values, 1e-12));
  CHECK_EXCEPTION(
      solveFixedKinematicsDynamics(
          f.robot, trajectory, {Slice(0), Slice(1), Slice(2)}, {0., .08, .3},
          f.fixed, f.priors, f.contacts(), Vector(), unusedObjectives),
      std::invalid_argument);
  fixed_dynamics::Options soft;
  soft.softDynamics = true;
  CHECK_EXCEPTION(fixedKinematicsDynamicsSeed(f.robot, trajectory,
                                              {Slice(0), Slice(1), Slice(2)},
                                              {0., .08, .3}, f.fixed, f.priors,
                                              f.contacts(), Vector(), soft),
                  std::invalid_argument);
}

// Acceleration observations are soft: exact balance and unilateral support can
// correct a downward acceleration observation that is faster than free fall.
TEST(FixedKinematicsDynamics, AccelerationIsEstimated) {
  Fixture fixture;
  for (size_t k = 0; k < 3; ++k) {
    Vector6 mean = Vector6::Zero();
    mean(5) = -20;
    fixture.priors.update(TwistAccelKey(fixture.body->id(), k), Vector(mean));
  }
  const auto before = fixture.fixed;
  const auto result = solveFixedKinematicsDynamics(
      fixture.robot, fixture.trajectory, {Slice(0), Slice(1), Slice(2)},
      {0.0, .08, .3}, fixture.fixed, fixture.priors, fixture.contacts(),
      Vector(), {});
  EXPECT(result.feasible);
  EXPECT(!result.softDynamics);
  EXPECT(result.accelerationError > 0);
  EXPECT(result.maxForceResidual < 1e-7);
  EXPECT(result.maxMomentResidual < 1e-7);
  EXPECT(assert_equal(before, fixture.fixed));
  Values complete = fixture.fixed;
  complete.insert(result.values);
  for (size_t k = 0; k < 3; ++k) {
    EXPECT(result.values.at<Vector6>(TwistAccelKey(fixture.body->id(), k))(5) >=
           -9.81 - 1e-7);
    for (const auto& joint : fixture.robot.joints())
      EXPECT(joint->twistAccelConstraint(k).value(complete).norm() < 1e-7);
  }
}

// Lift-off imposes exactly zero wrench; contact changes do not split the global
// objective or prevent affine manifolds with zero independent coordinates.
TEST(FixedKinematicsDynamics, SwingAndSingleSupport) {
  Fixture fixture;
  const auto phase = [](size_t k, const std::vector<PoseOnLink>& contacts) {
    return Phase(k, k, std::make_shared<PoseContactConstraintSpec>(contacts));
  };
  const Trajectory trajectory(
      WalkCycle({phase(0, {fixture.left}),
                 phase(1, {fixture.left, fixture.right}),
                 phase(2, {fixture.right})}),
      1);
  auto contacts = fixture.contacts();
  // The synthetic feet are wider than their COM offsets, so either single
  // support patch can physically contain the robot COM projection.
  for (auto& contact : contacts) contact.halfWidth = .2;
  const auto result = solveFixedKinematicsDynamics(
      fixture.robot, trajectory, {Slice(0), Slice(1), Slice(2)}, {0.0, .08, .3},
      fixture.fixed, fixture.priors, contacts, Vector(), fixture.tightPriors());
  EXPECT(result.feasible);
  EXPECT_LONGS_EQUAL(30, result.reducedDimension);
  EXPECT_LONGS_EQUAL(2, result.wrenchSmoothFactors);
  EXPECT(result.values.at<Vector6>(result.contactKeys[1]).norm() < 1e-8);
  EXPECT(result.values.at<Vector6>(result.contactKeys[4]).norm() < 1e-8);
  EXPECT_DOUBLES_EQUAL(
      98.1, result.values.at<Vector6>(result.contactKeys[0])(5), 1e-6);
  EXPECT_DOUBLES_EQUAL(
      98.1, result.values.at<Vector6>(result.contactKeys[5])(5), 1e-6);

  // In soft-base mode the arm/swing branches are separate exactly determined
  // components. Substituting their zero-dimensional values must retain the
  // nonzero contact coordinates in the global LM system.
  fixed_dynamics::Options soft;
  soft.softDynamics = true;
  const auto relaxed = solveFixedKinematicsDynamics(
      fixture.robot, trajectory, {Slice(0), Slice(1), Slice(2)}, {0.0, .08, .3},
      fixture.fixed, fixture.priors, contacts, Vector(), soft);
  EXPECT(relaxed.feasible);
  EXPECT(relaxed.values.at<Vector6>(relaxed.contactKeys[1]).norm() < 1e-8);
  EXPECT(relaxed.values.at<Vector6>(relaxed.contactKeys[4]).norm() < 1e-8);

  const Trajectory singleSupport(
      WalkCycle({Phase(0, 2,
                       std::make_shared<PoseContactConstraintSpec>(
                           std::vector<PoseOnLink>{fixture.left}))}),
      1);
  const auto determined = solveFixedKinematicsDynamics(
      fixture.robot, singleSupport, {Slice(0), Slice(1), Slice(2)},
      {0.0, .08, .3}, fixture.fixed, fixture.priors, contacts, Vector(),
      fixture.tightPriors());
  EXPECT(determined.feasible);
  EXPECT(determined.stationary);
  EXPECT_LONGS_EQUAL(24, determined.reducedDimension);
}

// A downward acceleration observation beyond free fall would imply tensile
// ground force. Start at the feasible cone apex and keep forces unilateral.
TEST(FixedKinematicsDynamics, DownwardPriorAtConeApex) {
  Fixture fixture;
  for (size_t k = 0; k < 3; ++k) {
    Vector6 mean = Vector6::Zero();
    mean(5) = -30;
    fixture.priors.update(TwistAccelKey(0, k), Vector(mean));
  }
  const auto result = solveFixedKinematicsDynamics(
      fixture.robot, fixture.trajectory, {Slice(0), Slice(1), Slice(2)},
      {0.0, .08, .3}, fixture.fixed, fixture.priors, fixture.contacts(),
      Vector(), fixed_dynamics::Options());
  EXPECT(result.feasible);
  EXPECT(result.equalityL2 < 1e-8);
  EXPECT(result.inequalityL2 < 1e-8);
  for (size_t k = 0; k < 3; ++k) {
    EXPECT_DOUBLES_EQUAL(-9.81, TwistAccel(result.values, 0, k)(5), 1e-5);
    EXPECT(result.values.at<Vector6>(result.contactKeys[2 * k])(5) > -1e-8);
    EXPECT(result.values.at<Vector6>(result.contactKeys[2 * k + 1])(5) > -1e-8);
  }
}

// Motor effort includes rotor inertia times the estimated joint acceleration,
// and optional effort limits are genuine inequalities in the same CMC solve.
TEST(FixedKinematicsDynamics, RotorInertiaAndTorqueLimits) {
  Fixture fixture;
  for (size_t k = 0; k < 3; ++k)
    for (const auto& joint : fixture.robot.joints())
      fixture.priors.update(JointAccelKey(joint->id(), k), 1.0);
  auto options = fixture.tightPriors();
  options.torqueLimits = Vector2::Constant(3.5);
  const auto result = solveFixedKinematicsDynamics(
      fixture.robot, fixture.trajectory, {Slice(0), Slice(1), Slice(2)},
      {0.0, .08, .3}, fixture.fixed, fixture.priors, fixture.contacts(),
      Vector2::Constant(2.0), options);
  EXPECT(result.feasible);
  for (Key key : result.torqueKeys)
    EXPECT_DOUBLES_EQUAL(2.0, result.values.at<double>(key), 1e-5);
  options.torqueLimits = Vector2::Constant(1.5);
  const auto limited = solveFixedKinematicsDynamics(
      fixture.robot, fixture.trajectory, {Slice(0), Slice(1), Slice(2)},
      {0.0, .08, .3}, fixture.fixed, fixture.priors, fixture.contacts(),
      Vector2::Constant(2.0), options);
  EXPECT(limited.feasible);
  for (size_t k = 0; k < 3; ++k)
    for (const auto& joint : fixture.robot.joints()) {
      const double torque =
          limited.values.at<double>(TorqueKey(joint->id(), k));
      const double qdd =
          limited.values.at<double>(JointAccelKey(joint->id(), k));
      const Vector6 reaction = limited.values.at<Vector6>(
          WrenchKey(joint->child()->id(), joint->id(), k));
      EXPECT(std::abs(torque) <= 1.5 + 1e-7);
      EXPECT_DOUBLES_EQUAL(torque,
                           joint->cScrewAxis().dot(reaction) + 2.0 * qdd, 1e-7);
    }
}
// An omitted swing phase still breaks a foot's wrench-rate prior on recontact.
TEST(FixedKinematicsDynamics, UnsampledLiftOffBreaksWrenchSmoothing) {
  Fixture fixture;
  const auto phase = [](size_t k, const std::vector<PoseOnLink>& contacts) {
    return Phase(k, k, std::make_shared<PoseContactConstraintSpec>(contacts));
  };
  const Trajectory trajectory(
      WalkCycle({phase(0, {fixture.left, fixture.right}),
                 phase(1, {fixture.right}),
                 phase(2, {fixture.left, fixture.right})}),
      1);
  const auto result = solveFixedKinematicsDynamics(
      fixture.robot, trajectory, {Slice(0), Slice(2)}, {0.0, .3}, fixture.fixed,
      fixture.priors, fixture.contacts(), Vector(), fixture.tightPriors());
  EXPECT(result.feasible);
  EXPECT_LONGS_EQUAL(1, result.wrenchSmoothFactors);
}

// Invalid default or foreign-link contacts fail before any link dereference.
TEST(FixedKinematicsDynamics, InvalidContactFrames) {
  Fixture fixture;
  const PoseOnLink foreign(
      std::make_shared<Link>(99, "foreign", 1, Matrix3::Identity(), Pose3(),
                             Pose3()),
      Pose3());
  for (const auto& invalid : {fixed_dynamics::Contact(),
                              fixed_dynamics::Contact(foreign, .1, .04, .6)}) {
    auto contacts = fixture.contacts();
    contacts[0] = invalid;
    bool rejected = false;
    try {
      solveFixedKinematicsDynamics(
          fixture.robot, fixture.trajectory, {Slice(0), Slice(1), Slice(2)},
          {0.0, .08, .3}, fixture.fixed, fixture.priors, contacts, Vector(),
          fixture.tightPriors());
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    EXPECT(rejected);
  }
}
}  // namespace static_support
/* ************************************************************************* */

namespace acceleration_prior {

// The acceleration/force tradeoff matches a closed-form quadratic solution;
// this detects accidental fixing of acceleration or incorrect prior whitening.
TEST(FixedKinematicsDynamics, AccelerationPriorWeight) {
  const auto body = std::make_shared<Link>(0, "base", 10.0, Matrix3::Identity(),
                                           Pose3(), Pose3());
  const Robot robot(LinkMap{{"base", body}}, JointMap{});
  const PoseOnLink sole(body, Pose3(Rot3(), Point3(0, 0, -.5)));
  const Trajectory trajectory(
      WalkCycle({Phase(0, 1,
                       std::make_shared<PoseContactConstraintSpec>(
                           std::vector<PoseOnLink>{sole}))}),
      1);
  Values fixed, priors;
  Vector6 mean = Vector6::Zero();
  mean(5) = 1.0;
  for (size_t k = 0; k < 2; ++k) {
    fixed.insert(PoseKey(0, k), Pose3());
    fixed.insert(TwistKey(0, k), Vector(Vector6::Zero()));
    priors.insert(TwistAccelKey(0, k), mean);
  }
  fixed_dynamics::Options options;
  options.baseLinearAccelerationSigma = 2.0;
  options.wrenchSigma = 100.0;
  const auto result = solveFixedKinematicsDynamics(
      robot, trajectory, {Slice(0), Slice(1)}, {0.0, .2}, fixed, priors,
      {{sole, .1, .1, .6}}, Vector(), options);
  const double ratio = std::pow(10.0 * 2.0 / 100.0, 2);
  const double expected = (1.0 - ratio * 9.81) / (1.0 + ratio);
  EXPECT(result.feasible);
  for (size_t k = 0; k < 2; ++k) {
    EXPECT_DOUBLES_EQUAL(
        expected, result.values.at<Vector6>(TwistAccelKey(0, k))(5), 1e-7);
    EXPECT_DOUBLES_EQUAL(10.0 * (9.81 + expected),
                         result.values.at<Vector6>(result.contactKeys[k])(5),
                         1e-7);
  }
  EXPECT(result.accelerationError > 0);
}

}  // namespace acceleration_prior
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
