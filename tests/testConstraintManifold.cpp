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
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/Initializer.h>

#include <gtdynamics/optimizer/ConstraintManifold.h>

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
  constraints.emplace_shared<gtdynamics::FactorEquality>(factor12);
  constraints.emplace_shared<gtdynamics::FactorEquality>(factor23);

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

// cart-pole robot setting
auto robot = CreateRobotFromFile(kUrdfPath + std::string("cart_pole.urdf"))
                  .fixLink("l0");
int j0_id = robot.joint("j0")->id(), j1_id = robot.joint("j1")->id();
int l1_id = robot.link("l1")->id(), l2_id = robot.link("l2")->id(),
    l0_id = robot.link("l0")->id();


std::string pose_str(const Pose3& pose) {
  const Point3 trans = pose.translation();
  const Rot3 rot = pose.rotation();
  const Vector3 xyz = rot.xyz();
  std::stringstream buffer;
  buffer << boost::format(
                "[t: [%0.4f, %0.4f, %0.4f],\tR: [%0.4f, %0.4f, %0.4f]]") %
                trans.x() % trans.y() % trans.z() % xyz(0) % xyz(1) % xyz(2);
  return buffer.str();
}

std::string vec_str(const Vector& vec) {
  std::stringstream buffer;
  buffer << boost::format("[%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f]") %
                vec(0) % vec(1) % vec(2) % vec(3) % vec(4) % vec(5);
  return buffer.str();
}


void print_state(const Values& values) {
  const Pose3 pose1 = Pose(values, l1_id), pose2 = Pose(values, l2_id);
  const Vector6 twist1 = Twist(values, l1_id), twist2 = Twist(values, l2_id);
  const Vector6 twista1 = TwistAccel(values, l1_id),
                twista2 = TwistAccel(values, l2_id);
  const Vector6 wrench11 = Wrench(values, l1_id, j0_id),
                wrench01 = Wrench(values, l0_id, j0_id),
                wrench12 = Wrench(values, l1_id, j1_id),
                wrench22 = Wrench(values, l2_id, j1_id);
  const double q1 = JointAngle(values, j0_id), q2 = JointAngle(values, j1_id);
  const double v1 = JointVel(values, j0_id), v2 = JointVel(values, j1_id);
  const double a1 = JointAccel(values, j0_id), a2 = JointAccel(values, j1_id);
  const double t1 = Torque(values, j0_id), t2 = Torque(values, j1_id);
  std::cout << "q: " << boost::format("(%0.4f)") % q1 << "\t" << pose_str(pose1)
            << "\t" << boost::format("(%0.4f)") % q2 << "\t" << pose_str(pose2)
            << "\n";
  std::cout << "v: " << boost::format("(%0.4f)") % v1 << "\t" << vec_str(twist1)
            << "\t" << boost::format("(%0.4f)") % v2 << "\t" << vec_str(twist2)
            << "\n";
  std::cout << "a: " << boost::format("(%0.4f)") % a1 << "\t"
            << vec_str(twista1) << "\t" << boost::format("(%0.4f)") % a2 << "\t"
            << vec_str(twista2) << "\n";
  std::cout << "f1: " << vec_str(wrench01) << boost::format("(%0.4f)") % t1
            << "\t" << vec_str(wrench11) << "\n";
  std::cout << "f2: " << vec_str(wrench12) << boost::format("(%0.4f)") % t2
            << "\t" << vec_str(wrench22) << "\n";
}

TEST(ConstraintManifold, Kinematics) {

  const gtsam::Vector3 gravity(0, 0, -9.8);
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
  cc_params->retract_type = ConstraintManifold::Params::RetractType::PARTIAL_PROJ;
  cc_params->basis_type = ConstraintManifold::Params::BasisType::SPECIFY_VARIABLES;
  auto cc = boost::make_shared<ConnectedComponent>(constraints);
  auto cm = ConstraintManifold(cc, init_values, cc_params, false, true, basis_keys);
  print_state(cm.values());

  // retract
  Vector xi = (Vector(6)<<1, 0, 0, 0, 0, 0).finished();
  std::cout << "retract\n";
  auto new_cm = cm.retract(xi);
  std::cout << "retract done\n";
  print_state(new_cm.values());
  std::cout << "constraint violation: " << cc->merit_graph.error(new_cm.values()) << "\n";
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
