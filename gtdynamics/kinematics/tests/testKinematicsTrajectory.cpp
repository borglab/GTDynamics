/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testKinematicsTrajectory.cpp
 * @brief Contact-height and temporal equality factors over changing phases.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/ContactEqualityFactor.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Trajectory.h>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::Point3;

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

TEST(Trajectory, ContactHeightEmpty) {
  const Kinematics kinematics;
  const gtsam::Vector3 gravity(0, 0, -9.81);
  EXPECT_LONGS_EQUAL(
      0, kinematics.contactHeightObjectives(Trajectory(), gravity).size());
}

TEST(Trajectory, ContactEqualityContinuity) {
  using namespace contact_goals_example;
  const PointOnLink a(LH, Point3::Zero()), b(LH, Point3(0.2, 0, 0)),
      c(RF, Point3::Zero()), nonground(RF, Point3(0.1, 0, 0), false);
  const auto phase = [](size_t first, size_t last, const PointOnLinks& points) {
    return Phase(first, last, std::make_shared<FootContactConstraintSpec>(points));
  };
  const Trajectory trajectory(
      WalkCycle({phase(5, 6, {a, b, a, nonground}),
                 phase(6, 7, {c, b, a, nonground}),
                 phase(8, 8, {a}), phase(9, 9, {}), phase(10, 11, {a}),
                 phase(13, 14, {a})}),
      1);
  const auto graph = Kinematics().contactEqualityFactors(
      trajectory, gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
  // Two points on LH survive the shared endpoint; RF starts at that endpoint.
  // Non-ground contacts, flight, and the gap at sample 12 add no temporal edges.
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

TEST(Trajectory, ContactEqualityEmptyAndSingleton) {
  using namespace contact_goals_example;
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  const auto spec = std::make_shared<FootContactConstraintSpec>(
      PointOnLinks{PointOnLink(LH, Point3::Zero())});
  const Kinematics kinematics;
  EXPECT_LONGS_EQUAL(
      0, kinematics.contactEqualityFactors(Trajectory(), noise).size());
  EXPECT_LONGS_EQUAL(0, kinematics.contactEqualityFactors(
                           Trajectory(WalkCycle({Phase(4, 4, spec)}), 1), noise)
                           .size());
}

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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
