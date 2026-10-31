/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testKinematicsTrajectory.cpp
 * @brief Contact-height objectives over changing contact phases.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Trajectory.h>

#include <stdexcept>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::Point3;

TEST(Trajectory, ContactHeightSharedBoundaryAndFlight) {
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

  // 2 at slice 5, union {a,b,c} at 6, 2 at 7, and 1 at 10. In particular,
  // b is not counted twice at 6, and a/b remain distinct despite sharing LH.
  EXPECT_LONGS_EQUAL(8, graph.size());
  EXPECT_LONGS_EQUAL(6, graph.keys().size());
  gtsam::Values values;
  for (const auto key : graph.keys()) {
    values.insert(key, gtsam::Pose3(gtsam::Rot3(), Point3(0, 0, 0.4)));
  }
  DOUBLES_EQUAL(4.0, graph.error(values), 1e-10);
  EXPECT(graph.keys().count(PoseKey(LH->id(), 8)) == 0);
  EXPECT(graph.keys().count(PoseKey(LH->id(), 9)) == 0);
}

TEST(Trajectory, ContactHeightEmptyAndMissingSpecification) {
  const Kinematics kinematics;
  const gtsam::Vector3 gravity(0, 0, -9.81);
  EXPECT_LONGS_EQUAL(
      0, kinematics.contactHeightObjectives(Trajectory(), gravity).size());
  bool rejected = false;
  try {
    kinematics.contactHeightObjectives(
        Trajectory(WalkCycle({Phase(5, 7, nullptr)}), 1), gravity);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
