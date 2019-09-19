/**
 * @file  testDynamics.cpp
 * @brief test forward and inverse dynamics factor graph
 * @Author: Mandy Xie
 */

#include <Arm.h>
#include <DHLink.h>
#include <MotionPlanner.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {
// RR link example
vector<DH_Link> dh_rr = {DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0),
                                 Z_3x3),
                         DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0),
                                 Z_3x3)};
auto robot = Arm<DH_Link>(dh_rr);
auto dof = robot.numLinks();

// get robot jTi list at rest
auto jMi = robot.jTi_list(Vector::Zero(dof));
// get robot screw_axes for all links
auto screw_axes = robot.screwAxes();

Pose3 base_pose = Pose3();
Vector base_twist = Vector6::Zero(), base_acceleration = Vector6::Zero(),
       external_wrench = Vector6::Zero();

// TODO (Mandy): why can not use the following way:
// Vector3 gravity;
// gravity << 0, -9.8, 0;
// error: ‘gravity’ does not name a type
Vector3 gravity = (Vector(3) << 0, -9.8, 0).finished();

// Forward dynamics known torque, calculate joint acceleration
Vector known_torque = Vector::Zero(dof);
// Inverse dynamics known joint acceleration, calculate torque
Vector known_qAccel = (Vector(2) << -9.8, 19.6).finished();
// know com pose
vector<Pose3> pose = {Pose3(Rot3(), Point3(1, 0, 0)),
                      Pose3(Rot3(), Point3(3, 0, 0))};

// initial values
Vector start_q = Vector::Zero(dof), start_qVel = Vector::Zero(dof),
       start_qAccel = Vector::Zero(dof), start_torque = Vector::Zero(dof);

Vector twists = Vector6::Zero(), accels = Vector6::Zero(),
       wrenches = Vector6::Zero();

gtsam::noiseModel::Base::shared_ptr
    bv_cost_model = noiseModel::Constrained::All(6),
    ba_cost_model = noiseModel::Constrained::All(6),
    v_cost_model = noiseModel::Constrained::All(6),
    a_cost_model = noiseModel::Constrained::All(6),
    f_cost_model = noiseModel::Constrained::All(6),
    t_cost_model = noiseModel::Constrained::All(1),
    qa_cost_model = noiseModel::Constrained::All(1),
    qv_cost_model = noiseModel::Constrained::All(1),
    q_cost_model = noiseModel::Constrained::All(1),
    tf_cost_model = noiseModel::Constrained::All(6),
    p_cost_model = noiseModel::Constrained::All(6);
}  // namespace example

/**
 * Test forward dynamics with gravity
 */
TEST(FD_factor_graph, optimization) {
  NonlinearFactorGraph graph;
  // add base pose factor
  graph.add(
      BasePoseFactor(PoseKey(0, 0), example::p_cost_model, example::base_pose));
  // add base twist factor
  graph.add(BaseTwistFactor(TwistKey(0, 0), example::bv_cost_model,
                            example::base_twist));
  // add base acceleration factor
  graph.add(BaseTwistAccelFactor(TwistAccelKey(0, 0), example::ba_cost_model,
                                 example::base_acceleration));
  for (int j = 1; j <= example::dof; ++j) {
    // add twist factor
    graph.add(TwistFactor(TwistKey(j - 1, 0), TwistKey(j, 0),
                          JointAngleKey(j, 0), JointVelKey(j, 0),
                          example::v_cost_model, example::jMi[j - 1],
                          example::screw_axes[j - 1]));
    // add twist acceleration factor
    graph.add(TwistAccelFactor(TwistKey(j, 0), TwistAccelKey(j - 1, 0),
                               TwistAccelKey(j, 0), JointAngleKey(j, 0),
                               JointVelKey(j, 0), JointAccelKey(j, 0),
                               example::a_cost_model, example::jMi[j - 1],
                               example::screw_axes[j - 1]));
    // add wrench factor
    if (j < example::dof) {
      graph.add(WrenchFactor(TwistKey(j, 0), TwistAccelKey(j, 0),
                             WrenchKey(j, 0), WrenchKey(j + 1, 0),
                             PoseKey(j, 0), JointAngleKey(j + 1, 0),
                             example::f_cost_model, example::jMi[j],
                             example::robot.link(j - 1).inertiaMatrix(),
                             example::screw_axes[j], example::gravity));
    }
    // add torque factor
    graph.add(TorqueFactor(WrenchKey(j, 0), TorqueKey(j, 0),
                           example::t_cost_model, example::screw_axes[j - 1]));

    graph.add(
        PriorFactor<double>(JointAngleKey(j, 0), 0, example::q_cost_model));
    graph.add(
        PriorFactor<double>(JointVelKey(j, 0), 0, example::qv_cost_model));
    graph.add(PriorFactor<double>(TorqueKey(j, 0), example::known_torque[j - 1],
                                  example::t_cost_model));
    graph.add(PriorFactor<Pose3>(PoseKey(j, 0), example::pose[j - 1],
                                 example::p_cost_model));
  }
  // add tool wrench factor
  graph.add(ToolWrenchFactor(
      TwistKey(example::dof, 0), TwistAccelKey(example::dof, 0),
      WrenchKey(example::dof, 0), PoseKey(example::dof, 0),
      example::tf_cost_model, example::jMi[example::dof],
      example::robot.link(example::dof - 1).inertiaMatrix(),
      example::external_wrench, example::gravity));

  // set initial values
  Values init_values;

  init_values.insert(PoseKey(0, 0), example::base_pose);
  init_values.insert(TwistKey(0, 0), example::base_twist);
  init_values.insert(TwistAccelKey(0, 0), example::base_acceleration);
  for (int j = 1; j <= example::dof; ++j) {
    init_values.insert(PoseKey(j, 0), example::pose[j - 1]);
    init_values.insert(TwistKey(j, 0), example::twists);
    init_values.insert(TwistAccelKey(j, 0), example::accels);
    init_values.insert(WrenchKey(j, 0), example::wrenches);
    init_values.insert(TorqueKey(j, 0), example::start_torque[j - 1]);
    init_values.insert(JointAngleKey(j, 0), example::start_q[j - 1]);
    init_values.insert(JointVelKey(j, 0), example::start_qVel[j - 1]);
    init_values.insert(JointAccelKey(j, 0), example::start_qAccel[j - 1]);
  }

  // using Guassian Newton optimizer
  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();

  Vector actual_qAccel = Vector::Zero(example::dof);
  for (int j = 1; j <= example::dof; ++j) {
    actual_qAccel[j - 1] = result.atDouble(JointAccelKey(j, 0));
  }
  Vector expected_qAccel = example::known_qAccel;

  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

/**
 * Test inverse dynamics with gravity
 */
TEST(ID_factor_graph, optimization) {
  NonlinearFactorGraph graph;
  // add base pose factor
  graph.add(
      BasePoseFactor(PoseKey(0, 0), example::p_cost_model, example::base_pose));
  // add base twist factor
  graph.add(BaseTwistFactor(TwistKey(0, 0), example::bv_cost_model,
                            example::base_twist));
  // add base acceleration factor
  graph.add(BaseTwistAccelFactor(TwistAccelKey(0, 0), example::ba_cost_model,
                                 example::base_acceleration));
  for (int j = 1; j <= example::dof; ++j) {
    // add twist factor
    graph.add(TwistFactor(TwistKey(j - 1, 0), TwistKey(j, 0),
                          JointAngleKey(j, 0), JointVelKey(j, 0),
                          example::v_cost_model, example::jMi[j - 1],
                          example::screw_axes[j - 1]));
    // add twist acceleration factor
    graph.add(TwistAccelFactor(TwistKey(j, 0), TwistAccelKey(j - 1, 0),
                               TwistAccelKey(j, 0), JointAngleKey(j, 0),
                               JointVelKey(j, 0), JointAccelKey(j, 0),
                               example::a_cost_model, example::jMi[j - 1],
                               example::screw_axes[j - 1]));
    // add wrench factor
    if (j < example::dof) {
      graph.add(WrenchFactor(TwistKey(j, 0), TwistAccelKey(j, 0),
                             WrenchKey(j, 0), WrenchKey(j + 1, 0),
                             PoseKey(j, 0), JointAngleKey(j + 1, 0),
                             example::f_cost_model, example::jMi[j],
                             example::robot.link(j - 1).inertiaMatrix(),
                             example::screw_axes[j], example::gravity));
    }
    // add torque factor
    graph.add(TorqueFactor(WrenchKey(j, 0), TorqueKey(j, 0),
                           example::t_cost_model, example::screw_axes[j - 1]));

    graph.add(
        PriorFactor<double>(JointAngleKey(j, 0), 0, example::q_cost_model));
    graph.add(
        PriorFactor<double>(JointVelKey(j, 0), 0, example::qv_cost_model));
    graph.add(PriorFactor<double>(JointAccelKey(j, 0),
                                  example::known_qAccel[j - 1],
                                  example::qa_cost_model));
    graph.add(PriorFactor<Pose3>(PoseKey(j, 0), example::pose[j - 1],
                                 example::p_cost_model));
  }
  // add tool wrench factor
  graph.add(ToolWrenchFactor(
      TwistKey(example::dof, 0), TwistAccelKey(example::dof, 0),
      WrenchKey(example::dof, 0), PoseKey(example::dof, 0),
      example::tf_cost_model, example::jMi[example::dof],
      example::robot.link(example::dof - 1).inertiaMatrix(),
      example::external_wrench, example::gravity));

  // set initial values
  Values init_values;

  init_values.insert(PoseKey(0, 0), example::base_pose);
  init_values.insert(TwistKey(0, 0), example::base_twist);
  init_values.insert(TwistAccelKey(0, 0), example::base_acceleration);
  for (int j = 1; j <= example::dof; ++j) {
    init_values.insert(PoseKey(j, 0), example::pose[j - 1]);
    init_values.insert(TwistKey(j, 0), example::twists);
    init_values.insert(TwistAccelKey(j, 0), example::accels);
    init_values.insert(WrenchKey(j, 0), example::wrenches);
    init_values.insert(TorqueKey(j, 0), example::start_torque[j - 1]);
    init_values.insert(JointAngleKey(j, 0), example::start_q[j - 1]);
    init_values.insert(JointVelKey(j, 0), example::start_qVel[j - 1]);
    init_values.insert(JointAccelKey(j, 0), example::start_qAccel[j - 1]);
  }

  // using Guassian Newton optimizer
  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();

  Vector actual_torque = Vector::Zero(example::dof);
  for (int j = 1; j <= example::dof; ++j) {
    actual_torque[j - 1] = result.atDouble(TorqueKey(j, 0));
  }
  Vector expected_torque = example::known_torque;

  EXPECT(assert_equal(actual_torque, expected_torque));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}