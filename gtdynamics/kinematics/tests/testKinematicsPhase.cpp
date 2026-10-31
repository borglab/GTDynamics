/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testKinematicsPhase.cpp
 * @brief Test Kinematics for a phase with fixed contacts.
 * @author: Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Phase.h>
#include <gtdynamics/utils/Trajectory.h>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

#include "contactGoalsExample.h"

namespace {
// A specification unrelated to FootContactConstraintSpec exercises dispatch
// through the base interface, with one contact on the ground and one elsewhere.
class MixedContactSpec : public ConstraintSpec {
 public:
  PointOnLinks points;
  const PointOnLinks& contactPoints() const override { return points; }
  void print(const std::string&) const override {}
};
}  // namespace

TEST(Phase, GroundContactsFromGeneralSpecification) {
  using namespace contact_goals_example;
  auto spec = std::make_shared<MixedContactSpec>();
  spec->points = {{LH, Point3::Zero(), true}, {RF, Point3::Zero(), false}};
  const Phase phase(5, 7, spec);
  EXPECT_LONGS_EQUAL(2, phase.constraintSpec()->contactPoints().size());
  EXPECT_LONGS_EQUAL(1, phase.constraintSpec()->groundContactPoints().size());
  const auto& wall = spec->points[1];
  const PointOnLink same_material_point(wall.link, wall.point, true);
  EXPECT(wall == same_material_point);
  EXPECT(!wall.equals(same_material_point));
  EXPECT(wall.equals(PointOnLink(wall.link, wall.point, false)));

  KinematicsParameters parameters;
  parameters.cp_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.2);
  const Kinematics kinematics(parameters);
  const gtsam::Vector3 gravity(0, 0, -9.81);
  const auto graph = kinematics.contactHeightObjectives(phase, gravity, 0.2);
  EXPECT_LONGS_EQUAL(3, graph.size());
  gtsam::Values values;
  for (size_t k = 5; k <= 7; ++k) {
    values.insert(PoseKey(LH->id(), k),
                  gtsam::Pose3(gtsam::Rot3(), Point3(0, 0, 0.4)));
    EXPECT(graph.keys().count(PoseKey(RF->id(), k)) == 0);
  }
  DOUBLES_EQUAL(1.5, graph.error(values), 1e-10);
  EXPECT_LONGS_EQUAL(1, kinematics.contactHeightObjectives(
                           Slice(5), spec->points, gravity).size());
  const Trajectory trajectory(WalkCycle({phase}), 1);
  EXPECT(graph.equals(
      kinematics.contactHeightObjectives(trajectory, gravity, 0.2)));
  spec->points.front().isGround = false;
  EXPECT_LONGS_EQUAL(0, kinematics.contactHeightObjectives(phase, gravity).size());
}

TEST(Phase, ContactHeightObjectives) {
  using namespace contact_goals_example;
  const auto contacts = std::make_shared<FootContactConstraintSpec>(
      std::vector<LinkSharedPtr>{LH, RF}, contact_in_com);
  // A nonzero start checks that the builder retains source slice indices.
  const Phase phase(5, 7, contacts);
  const gtsam::Vector3 gravity(0, -9.81, 0);
  constexpr double height = 0.3, sigma = 0.2;
  KinematicsParameters parameters;
  parameters.cp_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma);
  const Kinematics kinematics(parameters);

  const auto graph = kinematics.contactHeightObjectives(phase, gravity, height);
  gtsam::NonlinearFactorGraph explicit_graph;
  for (size_t k = 5; k <= 7; ++k) {
    explicit_graph.add(kinematics.contactHeightObjectives(
        Slice(k), contacts->contactPoints(), gravity, height));
  }
  EXPECT_LONGS_EQUAL(6, graph.size());
  EXPECT(graph.equals(explicit_graph));

  gtsam::Values values;
  double expected_error = 0;
  for (size_t k = 5; k <= 7; ++k) {
    const double residual = 0.1 * (k - 5);
    for (const auto& link : {LH, RF}) {
      values.insert(PoseKey(link->id(), k),
                    gtsam::Pose3(gtsam::Rot3(), Point3(0, height + residual, 0)));
      expected_error += 0.5 * residual * residual / (sigma * sigma);
    }
  }
  DOUBLES_EQUAL(expected_error, graph.error(values), 1e-10);
  DOUBLES_EQUAL(expected_error, explicit_graph.error(values), 1e-10);
}

TEST(Phase, ContactHeightSingletonAndFlight) {
  using namespace contact_goals_example;
  const auto contacts = std::make_shared<FootContactConstraintSpec>(
      std::vector<LinkSharedPtr>{LH, RF}, contact_in_com);
  const Kinematics kinematics;
  const gtsam::Vector3 gravity(0, 0, -9.81);
  const auto singleton =
      kinematics.contactHeightObjectives(Phase(9, 9, contacts), gravity);
  EXPECT_LONGS_EQUAL(2, singleton.size());
  EXPECT(singleton.keys().count(PoseKey(LH->id(), 9)) == 1);
  EXPECT(singleton.keys().count(PoseKey(RF->id(), 9)) == 1);
  const auto empty = std::make_shared<FootContactConstraintSpec>();
  EXPECT_LONGS_EQUAL(
      0, kinematics.contactHeightObjectives(Phase(5, 7, empty), gravity).size());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
