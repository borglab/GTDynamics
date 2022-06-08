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

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/optimizer/ConstraintManifold.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/slam/BetweenFactor.h>

#include <boost/format.hpp>

using namespace gtsam;
using namespace gtdynamics;

/** Simple example Pose3 with between constraints. */
TEST(ConstraintManifold, connected_poses) {
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;

  // Constraints.
  gtdynamics::EqualityConstraints constraints;
  auto noise = noiseModel::Unit::Create(6);
  auto factor12 = boost::make_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(0, 0, 1)), noise);
  auto factor23 = boost::make_shared<BetweenFactor<Pose3>>(
      x2_key, x3_key, Pose3(Rot3(), Point3(0, 1, 0)), noise);
  constraints.emplace_shared<gtdynamics::ZeroErrorFactorEquality>(factor12);
  constraints.emplace_shared<gtdynamics::ZeroErrorFactorEquality>(factor23);

  // Create manifold values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  cm_base_values.insert(x2_key, Pose3(Rot3(), Point3(0, 0, 1)));
  cm_base_values.insert(x3_key, Pose3(Rot3(), Point3(0, 1, 1)));

  // Construct manifold.
  auto component = boost::make_shared<ConnectedComponent>(constraints);
  ConstraintManifold manifold(component, cm_base_values);

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
                x2_key, boost::none);
  EXPECT(assert_equal(numericalDerivative11<Pose3, ConstraintManifold, 6>(
                          x2_recover_func, manifold),
                      H_recover_x2));

  std::function<Pose3(const ConstraintManifold&)> x3_recover_func =
      std::bind(&ConstraintManifold::recover<Pose3>, std::placeholders::_1,
                x3_key, boost::none);
  EXPECT(assert_equal(numericalDerivative11<Pose3, ConstraintManifold, 6>(
                          x3_recover_func, manifold),
                      H_recover_x3));
}

/** Dynamics manifold for cart-pole robot. */
TEST(ConstraintManifold_retract, cart_pole_dynamics) {
  // cart-pole robot setting
  auto robot = CreateRobotFromFile(kUrdfPath + std::string("cart_pole.urdf"))
                   .fixLink("l0");
  int j0_id = robot.joint("j0")->id(), j1_id = robot.joint("j1")->id();
  int l1_id = robot.link("l1")->id(), l2_id = robot.link("l2")->id(),
      l0_id = robot.link("l0")->id();
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

  // constraint manifold
  auto constraints = ConstraintsFromGraph(constraints_graph);
  auto cc_params = boost::make_shared<ConstraintManifold::Params>();
  cc_params->retract_type =
      ConstraintManifold::Params::RetractType::PARTIAL_PROJ;
  cc_params->basis_type =
      ConstraintManifold::Params::BasisType::SPECIFY_VARIABLES;
  auto cc = boost::make_shared<ConnectedComponent>(constraints);
  auto cm =
      ConstraintManifold(cc, init_values, cc_params, false, true, basis_keys);

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
  EXPECT(assert_equal(0., cc->merit_graph.error(new_cm.values())));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
