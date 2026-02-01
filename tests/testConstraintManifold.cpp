/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testConstraintManifold.cpp
 * @brief Test constraint manifold.
 * @author Yetong Zhang
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/slam/BetweenFactor.h>

#include "gtdynamics/manifold/Retractor.h"

using namespace gtsam;
using namespace gtdynamics;

/** Simple example Pose3 with between constraints. */
TEST_UNSAFE(ConstraintManifold, connected_poses) {
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;

  // Constraints.
  gtsam::NonlinearEqualityConstraints constraints;
  auto noise = noiseModel::Unit::Create(6);
  auto factor12 = std::make_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(0, 0, 1)), noise);
  auto factor23 = std::make_shared<BetweenFactor<Pose3>>(
      x2_key, x3_key, Pose3(Rot3(), Point3(0, 1, 0)), noise);
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(factor12);
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(factor23);
  auto component = std::make_shared<ConnectedComponent>(constraints);

  // Create manifold values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  cm_base_values.insert(x2_key, Pose3(Rot3(), Point3(0, 0, 1)));
  cm_base_values.insert(x3_key, Pose3(Rot3(), Point3(0, 1, 1)));

  // Create constraint manifold with various tspacebasis and retractors
  std::vector<BasisType> basis_types{BasisType::MATRIX,
                                     BasisType::SPECIFY_VARIABLES,
                                     BasisType::SPARSE_MATRIX};
  std::vector<RetractType> retract_types{RetractType::UOPT,
                                         RetractType::FIX_VARS};

  BasisKeyFunc basis_key_func =
      [=](const ConnectedComponent::shared_ptr& cc) -> KeyVector {
    return KeyVector{x3_key};
  };

  for (const auto& basis_type : basis_types) {
    for (const auto& retract_type : retract_types) {
      auto params = std::make_shared<ConstraintManifold::Params>();
      params->basis_key_func = basis_key_func;
      params->basis_params->basis_type = basis_type;
      params->retract_params->retract_type = retract_type;
      if (basis_type == BasisType::SPECIFY_VARIABLES) {
        params->basis_params->use_basis_keys = true;
      }
      if (retract_type == RetractType::FIX_VARS) {
        params->retract_params->use_basis_keys = true;
      }
      ConstraintManifold manifold(component, cm_base_values, params, true);

      // Check recover
      Values values;
      Matrix H_recover_x2, H_recover_x3;
      EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)),
                          manifold.recover<Pose3>(x2_key, H_recover_x2)));
      EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 1, 1)),
                          manifold.recover<Pose3>(x3_key, H_recover_x3)));

      // Check recover jacobian
      std::function<Pose3(const ConstraintManifold&)> x2_recover_func =
          std::bind(&ConstraintManifold::recover<Pose3>, std::placeholders::_1,
                    x2_key, nullptr);
      EXPECT(assert_equal(numericalDerivative11<Pose3, ConstraintManifold, 6>(
                              x2_recover_func, manifold),
                          H_recover_x2));

      std::function<Pose3(const ConstraintManifold&)> x3_recover_func =
          std::bind(&ConstraintManifold::recover<Pose3>, std::placeholders::_1,
                    x3_key, nullptr);
      EXPECT(assert_equal(numericalDerivative11<Pose3, ConstraintManifold, 6>(
                              x3_recover_func, manifold),
                          H_recover_x3));

      // check retract
      Vector xi = (Vector(6) << 0, 0, 0, 0, 0, 1).finished();
      auto new_cm = manifold.retract(xi);
    }
  }
}

/** Dynamics manifold for cart-pole robot. */
TEST(ConstraintManifold_retract, cart_pole_dynamics) {
  // cart-pole robot setting
  auto robot = CreateRobotFromFile(kUrdfPath + std::string("cart_pole.urdf"))
                   .fixLink("l0");
  int j0_id = robot.joint("j0")->id(), j1_id = robot.joint("j1")->id();
  const gtsam::Vector3 gravity(0, 0, -10);
  OptimizerSetting opt;
  auto graph_builder = DynamicsGraph(opt, gravity);

  // constraints graph
  NonlinearFactorGraph constraints_graph;
  constraints_graph.add(graph_builder.dynamicsFactorGraph(robot, 0));

  // initial values
  Initializer initializer;
  Values values0 = initializer.ZeroValues(robot, 0, 0.0);
  Values known_values;
  for (const auto& joint : robot.joints()) {
    InsertJointAngle(&known_values, joint->id(),
                     JointAngle(values0, joint->id()));
    InsertJointVel(&known_values, joint->id(), JointVel(values0, joint->id()));
    InsertTorque(&known_values, joint->id(), 0.0);
  }
  for (const auto& link : robot.links()) {
    InsertPose(&known_values, link->id(), Pose(values0, link->id()));
    InsertTwist(&known_values, link->id(), Twist(values0, link->id()));
  }
  values0 = graph_builder.linearSolveFD(robot, 0, known_values);
  Values init_values = values0;

  // basis keys
  KeyVector basis_keys;
  basis_keys.push_back(JointAngleKey(j0_id, 0));
  basis_keys.push_back(JointAngleKey(j1_id, 0));
  basis_keys.push_back(JointVelKey(j0_id, 0));
  basis_keys.push_back(JointVelKey(j1_id, 0));
  basis_keys.push_back(JointAccelKey(j0_id, 0));
  basis_keys.push_back(JointAccelKey(j1_id, 0));
  BasisKeyFunc basis_key_func =
      [=](const ConnectedComponent::shared_ptr& cc) -> KeyVector {
    return basis_keys;
  };

  // constraint manifold
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph);
  auto cc_params = std::make_shared<ConstraintManifold::Params>();
  cc_params->retract_params->setFixVars();
  cc_params->basis_params->setFixVars();
  cc_params->basis_key_func = basis_key_func;
  auto cc = std::make_shared<ConnectedComponent>(constraints);
  auto cm = ConstraintManifold(cc, init_values, cc_params, true);

  // retract
  Vector xi = (Vector(6) << 1, 0, 0, 0, 0, 0).finished();
  auto new_cm = cm.retract(xi);
  // Check basis variables shall get the exact update.
  EXPECT(assert_equal(1., new_cm.recover<double>(JointAngleKey(j0_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointAngleKey(j1_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointVelKey(j0_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointVelKey(j1_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointAccelKey(j0_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointAccelKey(j1_id, 0))));

  // Check that all constraints shall be satisfied after retraction.
  EXPECT(assert_equal(0., cc->merit_graph_.error(new_cm.values())));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
