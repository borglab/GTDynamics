/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testKinematicsTrajectory.cpp
 * @brief Contact and IMU factors over changing trajectory phases.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/ContactEqualityFactor.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Trajectory.h>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::Point3;

/* ************************************************************************* */
namespace galilean_imu {

// Connects unique slices through flight using world velocities and one bias.
TEST(Trajectory, GalileanImuAcrossSharedBoundaryAndFlight) {
  const auto spec = std::make_shared<FootContactConstraintSpec>();
  const Trajectory trajectory(WalkCycle({Phase(5, 6, spec), Phase(6, 7, spec)}),
                              1);
  auto params = gtsam::PreintegrationParams::MakeSharedU(9.81);
  const gtsam::Rot3 wRc = gtsam::Rot3::RzRyRx(.2, -.3, .5);
  const gtsam::Rot3 cRs = gtsam::Rot3::RzRyRx(-.4, .1, .3);
  params->body_P_sensor = gtsam::Pose3(cRs, Point3(.1, -.05, .03));
  const gtsam::imuBias::ConstantBias bias(Point3(.1, -.2, .3),
                                          Point3(.01, -.02, .03));
  const gtsam::Vector3 acceleration =
      cRs.unrotate(wRc.unrotate(Point3(0, 0, 9.81))) + bias.accelerometer();
  gtsam::PreintegratedImuMeasurementsG pim(params, bias);
  for (int i = 0; i < 10; ++i)
    pim.integrateMeasurement(acceleration, bias.gyroscope(), .01);
  const gtsam::Key bias_key = gtsam::Symbol('b', 0);
  const auto graph =
      Kinematics().imuFactors(trajectory, 2, {pim, pim}, bias_key);
  EXPECT_LONGS_EQUAL(2, graph.size());
  gtsam::Values values;
  values.insert(bias_key, bias);
  // Nonzero WORLD velocity, deliberately different from body velocity.
  const gtsam::Vector3 velocity(1, .2, -.1);
  for (int k = 5; k <= 7; ++k) {
    values.insert(PoseKey(2, k), gtsam::Pose3(wRc, velocity * (.1 * (k - 5))));
    values.insert(LinearVelocityKey(2, k), velocity);
  }
  DOUBLES_EQUAL(0, graph.error(values), 1e-12);
  EXPECT(graph.at(0)->keys() ==
         gtsam::KeyVector({PoseKey(2, 5), LinearVelocityKey(2, 5),
                           PoseKey(2, 6), LinearVelocityKey(2, 6), bias_key}));
  EXPECT(std::dynamic_pointer_cast<gtsam::GalileanImuFactor>(graph.at(0)));
  EXPECT(LinearVelocityKey(2, 5) != TwistKey(2, 5));
  EXPECT(LinearVelocityKey(2, 5) != JointVelKey(2, 5));
  EXPECT_LONGS_EQUAL(
      0, Kinematics().imuFactors(Trajectory(), 2, {}, bias_key).size());
  bool rejected = false;
  try {
    Kinematics().imuFactors(trajectory, 2, {pim}, bias_key);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
  rejected = false;
  try {
    Kinematics().imuFactors(trajectory, 2,
                            {gtsam::PreintegratedImuMeasurementsG(params), pim},
                            bias_key);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
}

// Corrects sensor rotation and centripetal acceleration at an offset IMU.
TEST(Trajectory, GalileanImuSensorLeverArm) {
  const auto spec = std::make_shared<FootContactConstraintSpec>();
  const Trajectory trajectory(WalkCycle({Phase(0, 1, spec)}), 1);
  auto params = gtsam::PreintegrationParams::MakeSharedU(9.81);
  const gtsam::Rot3 cRs = gtsam::Rot3::RzRyRx(.3, -.2, .1);
  const Point3 lever(.2, .1, -.1), omega(0, 0, 2);
  params->body_P_sensor = gtsam::Pose3(cRs, lever);
  const gtsam::Vector3 acceleration =
      cRs.unrotate(Point3(0, 0, 9.81) + omega.cross(omega.cross(lever)));
  gtsam::PreintegratedImuMeasurementsG pim(params);
  for (int i = 0; i < 100; ++i)
    pim.integrateMeasurement(acceleration, cRs.unrotate(omega), .001);
  const auto graph =
      Kinematics().imuFactors(trajectory, 2, {pim}, gtsam::Symbol('b', 0));
  const auto factor =
      std::dynamic_pointer_cast<gtsam::GalileanImuFactor>(graph.at(0));
  const gtsam::Vector error = factor->evaluateError(
      gtsam::Pose3(), Point3::Zero(),
      gtsam::Pose3(gtsam::Rot3::Expmap(.1 * omega), Point3::Zero()),
      Point3::Zero(), gtsam::imuBias::ConstantBias());
  DOUBLES_EQUAL(0, error.norm(), 1e-10);
}

}  // namespace galilean_imu
/* ************************************************************************* */

/* ************************************************************************* */
namespace contact_height {

// Accumulates each phase's ground-height objectives, including shared
// endpoints.
TEST(Trajectory, ContactHeightPhaseAccumulation) {
  using namespace contact_goals_example;
  const PointOnLink a(LH, Point3::Zero()), b(LH, Point3(0.2, 0, 0)),
      c(RF, Point3::Zero());
  const auto first =
      std::make_shared<FootContactConstraintSpec>(PointOnLinks{a, b});
  const auto second =
      std::make_shared<FootContactConstraintSpec>(PointOnLinks{c, b});
  const auto flight = std::make_shared<FootContactConstraintSpec>();
  const auto landing =
      std::make_shared<FootContactConstraintSpec>(PointOnLinks{a});
  const Trajectory trajectory(
      WalkCycle({Phase(5, 6, first), Phase(6, 7, second), Phase(8, 9, flight),
                 Phase(10, 10, landing)}),
      1);
  KinematicsParameters parameters;
  parameters.cp_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.2);
  const Kinematics kinematics(parameters);
  const auto graph = kinematics.contactHeightObjectives(
      trajectory, gtsam::Vector3(0, 0, -9.81), 0.2);

  // Each phase contributes its own objectives: 4 + 4 + 0 + 1.
  EXPECT_LONGS_EQUAL(9, graph.size());
  EXPECT_LONGS_EQUAL(6, graph.keys().size());
  gtsam::Values values;
  for (const auto key : graph.keys()) {
    values.insert(key, gtsam::Pose3(gtsam::Rot3(), Point3(0, 0, 0.4)));
  }
  DOUBLES_EQUAL(4.5, graph.error(values), 1e-10);
  EXPECT(graph.keys().count(PoseKey(LH->id(), 8)) == 0);
  EXPECT(graph.keys().count(PoseKey(LH->id(), 9)) == 0);
}

// An empty trajectory contributes no height objectives.
TEST(Trajectory, ContactHeightEmpty) {
  const Kinematics kinematics;
  const gtsam::Vector3 gravity(0, 0, -9.81);
  EXPECT_LONGS_EQUAL(
      0, kinematics.contactHeightObjectives(Trajectory(), gravity).size());
}

}  // namespace contact_height
/* ************************************************************************* */

/* ************************************************************************* */
namespace contact_equality {

// Maintains persistent ground contacts but breaks edges across flight and gaps.
TEST(Trajectory, ContactEqualityContinuity) {
  using namespace contact_goals_example;
  const PointOnLink a(LH, Point3::Zero()), b(LH, Point3(0.2, 0, 0)),
      c(RF, Point3::Zero()), nonground(RF, Point3(0.1, 0, 0), false);
  const auto phase = [](size_t first, size_t last, const PointOnLinks& points) {
    return Phase(first, last,
                 std::make_shared<FootContactConstraintSpec>(points));
  };
  const Trajectory trajectory(
      WalkCycle({phase(5, 6, {a, b, a, nonground}),
                 phase(6, 7, {c, b, a, nonground}), phase(8, 8, {a}),
                 phase(9, 9, {}), phase(10, 11, {a}), phase(13, 14, {a})}),
      1);
  const auto graph = Kinematics().contactEqualityFactors(
      trajectory, gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
  // Two points on LH survive the shared endpoint; RF starts at that endpoint.
  // Non-ground contacts, flight, and the gap at sample 12 add no temporal
  // edges.
  const std::vector<gtsam::KeyVector> expected = {
      {PoseKey(LH->id(), 5), PoseKey(LH->id(), 6)},
      {PoseKey(LH->id(), 5), PoseKey(LH->id(), 6)},
      {PoseKey(RF->id(), 6), PoseKey(RF->id(), 7)},
      {PoseKey(LH->id(), 6), PoseKey(LH->id(), 7)},
      {PoseKey(LH->id(), 6), PoseKey(LH->id(), 7)},
      {PoseKey(LH->id(), 7), PoseKey(LH->id(), 8)},
      {PoseKey(LH->id(), 10), PoseKey(LH->id(), 11)},
      {PoseKey(LH->id(), 13), PoseKey(LH->id(), 14)}};
  EXPECT_LONGS_EQUAL(expected.size(), graph.size());
  for (size_t i = 0; i < graph.size(); ++i) {
    EXPECT(std::dynamic_pointer_cast<ContactEqualityFactor>(graph.at(i)));
    EXPECT(graph.at(i)->keys() == expected.at(i));
  }
}

// Rotation about a fixed contact is allowed; translating that point is
// penalized.
TEST(Trajectory, ContactEqualityAllowsRotationAboutPoint) {
  using namespace contact_goals_example;
  const Point3 p(0.2, 0.1, 0);
  const auto spec = std::make_shared<FootContactConstraintSpec>(
      PointOnLinks{PointOnLink(LH, p)});
  const Trajectory trajectory(WalkCycle({Phase(0, 1, spec)}), 1);
  const auto graph = Kinematics().contactEqualityFactors(
      trajectory, gtsam::noiseModel::Isotropic::Sigma(3, 0.005));
  EXPECT_LONGS_EQUAL(1, graph.size());
  gtsam::Values values;
  const auto rotation = gtsam::Rot3::Rz(0.4);
  values.insert(PoseKey(LH->id(), 0), gtsam::Pose3());
  values.insert(PoseKey(LH->id(), 1),
                gtsam::Pose3(rotation, p - rotation.rotate(p)));
  DOUBLES_EQUAL(0.0, graph.error(values), 1e-12);
  values.update(
      PoseKey(LH->id(), 1),
      gtsam::Pose3(rotation, p - rotation.rotate(p) + Point3(0.005, 0, 0)));
  DOUBLES_EQUAL(0.5, graph.error(values), 1e-10);
}

// Empty and single-slice trajectories have no temporal contact factors.
TEST(Trajectory, ContactEqualityEmptyAndSingleton) {
  using namespace contact_goals_example;
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  const auto spec = std::make_shared<FootContactConstraintSpec>(
      PointOnLinks{PointOnLink(LH, Point3::Zero())});
  const Kinematics kinematics;
  EXPECT_LONGS_EQUAL(
      0, kinematics.contactEqualityFactors(Trajectory(), noise).size());
  EXPECT_LONGS_EQUAL(
      0, kinematics
             .contactEqualityFactors(
                 Trajectory(WalkCycle({Phase(4, 4, spec)}), 1), noise)
             .size());
}

// Only intervals where a point has a ground role receive contact factors.
TEST(Trajectory, ContactEqualityGroundRoleChanges) {
  using namespace contact_goals_example;
  const auto ground = std::make_shared<FootContactConstraintSpec>(
      PointOnLinks{PointOnLink(LH, Point3::Zero(), true)});
  const auto nonground = std::make_shared<FootContactConstraintSpec>(
      PointOnLinks{PointOnLink(LH, Point3::Zero(), false)});
  const Trajectory trajectory(
      WalkCycle({Phase(0, 1, nonground), Phase(2, 3, ground),
                 Phase(4, 5, nonground), Phase(6, 7, ground)}),
      1);
  const auto graph = Kinematics().contactEqualityFactors(
      trajectory, gtsam::noiseModel::Isotropic::Sigma(3, 0.005));
  EXPECT_LONGS_EQUAL(2, graph.size());
  EXPECT(graph.at(0)->keys() ==
         gtsam::KeyVector({PoseKey(LH->id(), 2), PoseKey(LH->id(), 3)}));
  EXPECT(graph.at(1)->keys() ==
         gtsam::KeyVector({PoseKey(LH->id(), 6), PoseKey(LH->id(), 7)}));
}

}  // namespace contact_equality
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
