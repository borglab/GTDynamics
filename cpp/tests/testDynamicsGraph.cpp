/**
 * @file  testDynamicsGraph.cpp
 * @brief test forward and inverse dynamics factor graph
 * @Author: Yetong Zhang
 */

#include <RobotModels.h>
#include <DynamicsGraph.h>
#include <UniversalRobot.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <utils.h>

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
using namespace robot;
using namespace gtsam;

int DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE = 0;
int DEBUG_FOUR_BAR_LINKAGE_ILS_EXAMPLE = 0;

// Test forward dynamics with gravity
TEST(dynamicsFactorGraph_FD, simple_urdf_eq_mass)
{
  using namespace simple_urdf_eq_mass;
  Vector torques = Vector::Ones(my_robot.numJoints());

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();
  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles, joint_vels, torques));
  // still need to add pose and twist priors since no link is fixed in this case
  for (auto link : my_robot.links())
  {
    int i = link->getID();
    graph.add(PriorFactor<Pose3>(PoseKey(i, 0), link->getComPose(), noiseModel::Constrained::All(6)));
    graph.add(PriorFactor<Vector6>(TwistKey(i, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  }

  // optimize
  Values init_values = DynamicsGraphBuilder::zeroValues(my_robot, 0);
  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();

  Vector actual_qAccel = DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  Vector expected_qAccel = (Vector(1) << 4).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

TEST(dynamicsFactorGraph_FD, four_bar_linkage)
{
  // Load the robot from urdf file
  using namespace four_bar_linkage;
  Vector torques = (Vector(4) << 1, 0, 1, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();

  NonlinearFactorGraph prior_factors = graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles, joint_vels, torques);
  // still need to add pose and twist priors since no link is fixed in this case
  for (auto link : my_robot.links())
  {
    int i = link->getID();
    prior_factors.add(PriorFactor<Pose3>(PoseKey(i, 0), link->getComPose(), noiseModel::Constrained::All(6)));
    prior_factors.add(PriorFactor<Vector6>(TwistKey(i, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  }

  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(prior_factors);

  Values init_values = DynamicsGraphBuilder::zeroValues(my_robot, 0);

  // test the four bar linkage FD in the free-floating scenario
  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();
  Vector actual_qAccel = DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  Vector expected_qAccel = (Vector(4) << 1, -1, 1, -1).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));

  // A planar four bar linkage in 3D space should throw an ILS error with the
  // constraints specified.
  graph = graph_builder.dynamicsFactorGraph(my_robot, 0, gravity);
  graph.add(prior_factors);
  GaussNewtonOptimizer optimizer1(graph, init_values);
  THROWS_EXCEPTION(optimizer1.optimize());

  // test the condition when we fix link "l1"
  my_robot.getLinkByName("l1")->fix();
  graph = graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(prior_factors);

  GaussNewtonOptimizer optimizer2(graph, init_values);
  optimizer2.optimize();
  result = optimizer2.values();
  actual_qAccel = DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  expected_qAccel = (Vector(4) << 0.25, -0.25, 0.25, -0.25).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

TEST(dynamicsFactorGraph_FD, jumping_robot)
{
  using namespace jumping_robot;
  double torque3 = 0;
  double torque2 = 0.5;
  Vector torques = (Vector(6) << 0, torque2, torque3, torque3, torque2, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();
  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles, joint_vels, torques));

  // set initial values
  Values init_values = DynamicsGraphBuilder::zeroValues(my_robot, 0);

  // test jumping robot FD
  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();

  // check acceleration
  auto expected_qAccel = Vector(6);
  double m1 = 0.31;
  double m2 = 0.28;
  double m3 = 0.54;
  double link_radius = 0.02;
  double l = 0.55;
  double theta = 0.0 / 180.0 * M_PI;
  double acc =
      (torque3 - torque2 * 2 - (0.5 * m1 + 1.5 * m2 + 1.0 * m3) * 9.8 * l * std::sin(theta)) /
      (std::pow(l, 2) * (1.0 / 4 * m1 + (1.0 / 4 + 2 * std::pow(std::sin(theta), 2)) * m2 + 2 * std::pow(std::sin(theta), 2) * m3) +
       (std::pow(l, 2) + 3 * std::pow(link_radius, 2)) * (1.0 / 12 * m1 + 1.0 / 12 * m2));
  expected_qAccel << acc, -2 * acc, acc, acc, -2 * acc, acc;
  Vector actual_qAccel = DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

TEST(collocationFactors, simple_urdf)
{
  using namespace simple_urdf;
  double dt = 1;
  int t = 0;
  int j = my_robot.joints()[0]->getID();

  NonlinearFactorGraph prior_factors;
  prior_factors.add(PriorFactor<double>(JointAngleKey(j, t), 1, noiseModel::Constrained::All(1)));
  prior_factors.add(PriorFactor<double>(JointVelKey(j, t), 1, noiseModel::Constrained::All(1)));
  prior_factors.add(PriorFactor<double>(JointAccelKey(j, t), 1, noiseModel::Constrained::All(1)));
  prior_factors.add(PriorFactor<double>(JointAccelKey(j, t + 1), 2, noiseModel::Constrained::All(1)));

  auto graph_builder = DynamicsGraphBuilder();

  Values init_values;
  init_values.insert(JointAngleKey(j, t), 0.0);
  init_values.insert(JointVelKey(j, t), 0.0);
  init_values.insert(JointAccelKey(j, t), 0.0);
  init_values.insert(JointAngleKey(j, t + 1), 0.0);
  init_values.insert(JointVelKey(j, t + 1), 0.0);
  init_values.insert(JointAccelKey(j, t + 1), 0.0);

  // test trapezoidal
  NonlinearFactorGraph trapezoidal_graph;
  trapezoidal_graph.add(graph_builder.collocationFactors(my_robot, t, dt, DynamicsGraphBuilder::CollocationScheme::Trapezoidal));
  trapezoidal_graph.add(prior_factors);
  Values trapezoidal_result = graph_builder.optimize(trapezoidal_graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(2.75, trapezoidal_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.5, trapezoidal_result.atDouble(JointVelKey(j, t + 1))));

  // test Euler
  NonlinearFactorGraph euler_graph;
  euler_graph.add(graph_builder.collocationFactors(my_robot, t, dt, DynamicsGraphBuilder::CollocationScheme::Euler));
  euler_graph.add(prior_factors);
  Values euler_result = graph_builder.optimize(euler_graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(2.0, euler_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.0, euler_result.atDouble(JointVelKey(j, t + 1))));

  // test the scenario with dt as a variable
  int phase = 0;
  init_values.insert(TimeKey(phase), double(0));
  prior_factors.add(PriorFactor<double>(TimeKey(phase), dt, gtsam::noiseModel::Constrained::All(1)));

  // multi-phase euler
  NonlinearFactorGraph mp_euler_graph;
  mp_euler_graph.add(graph_builder.multiPhaseCollocationFactors(my_robot, t, phase, DynamicsGraphBuilder::CollocationScheme::Euler));
  mp_euler_graph.add(prior_factors);
  Values mp_euler_result = graph_builder.optimize(mp_euler_graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointVelKey(j, t + 1))));

  // multi-phase trapezoidal
  NonlinearFactorGraph mp_trapezoidal_graph;
  mp_trapezoidal_graph.add(graph_builder.collocationFactors(my_robot, t, dt, DynamicsGraphBuilder::CollocationScheme::Trapezoidal));
  mp_trapezoidal_graph.add(prior_factors);
  Values mp_trapezoidal_result = graph_builder.optimize(mp_trapezoidal_graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(2.75, mp_trapezoidal_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.5, mp_trapezoidal_result.atDouble(JointVelKey(j, t + 1))));
}

// test forward dynamics of a trajectory
TEST(dynamicsTrajectoryFG, simple_urdf_eq_mass)
{
  using namespace simple_urdf_eq_mass;
  my_robot.getLinkByName("l1")->fix();
  int j = my_robot.joints()[0]->getID();
  auto graph_builder = DynamicsGraphBuilder();

  int num_steps = 2;
  double dt = 1;
  vector<Vector> torques_seq;
  for (int i = 0; i <= num_steps; i++)
  {
    torques_seq.emplace_back((Vector(1) << i * 1.0 + 1.0).finished());
  }

  Values init_values = DynamicsGraphBuilder::zeroValuesTrajectory(my_robot, num_steps);

  // test Euler
  NonlinearFactorGraph euler_graph = graph_builder.trajectoryFG(my_robot, num_steps, dt, DynamicsGraphBuilder::CollocationScheme::Euler, gravity, planar_axis);
  euler_graph.add(graph_builder.trajectoryFDPriors(my_robot, num_steps, joint_angles, joint_vels, torques_seq));
  Values euler_result = graph_builder.optimize(euler_graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(0.0, euler_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.0, euler_result.atDouble(JointVelKey(j, 1))));
  EXPECT(assert_equal(2.0, euler_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(assert_equal(1.0, euler_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(3.0, euler_result.atDouble(JointVelKey(j, 2))));
  EXPECT(assert_equal(3.0, euler_result.atDouble(JointAccelKey(j, 2))));

  // test trapezoidal
  NonlinearFactorGraph trapezoidal_graph = graph_builder.trajectoryFG(my_robot, num_steps, dt, DynamicsGraphBuilder::CollocationScheme::Trapezoidal, gravity, planar_axis);
  trapezoidal_graph.add(graph_builder.trajectoryFDPriors(my_robot, num_steps, joint_angles, joint_vels, torques_seq));
  Values trapezoidal_result = graph_builder.optimize(trapezoidal_graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(0.75, trapezoidal_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.5, trapezoidal_result.atDouble(JointVelKey(j, 1))));
  EXPECT(assert_equal(2.0, trapezoidal_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(assert_equal(3.5, trapezoidal_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(4.0, trapezoidal_result.atDouble(JointVelKey(j, 2))));
  EXPECT(assert_equal(3.0, trapezoidal_result.atDouble(JointAccelKey(j, 2))));

  // test the scenario with dt as a variable
  vector<int> phase_steps {1,1};
  double dt0 = 1;
  double dt1 = 2;
  NonlinearFactorGraph mp_prior_graph = graph_builder.trajectoryFDPriors(my_robot, num_steps, joint_angles, joint_vels, torques_seq);
  mp_prior_graph.add(PriorFactor<double>(TimeKey(0), dt0, gtsam::noiseModel::Constrained::All(1)));
  mp_prior_graph.add(PriorFactor<double>(TimeKey(1), dt1, gtsam::noiseModel::Constrained::All(1)));
  init_values = DynamicsGraphBuilder::zeroValuesTrajectory(my_robot, num_steps, 2);

  // multi-phase Euler
  NonlinearFactorGraph mp_euler_graph = graph_builder.multiPhaseTrajectoryFG(my_robot, phase_steps, DynamicsGraphBuilder::CollocationScheme::Euler, gravity, planar_axis);
  mp_euler_graph.add(mp_prior_graph);
  Values mp_euler_result =graph_builder.optimize(mp_euler_graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);

  // t        0   1   2
  // dt         1   2
  // torque   1   2   3
  // q        0   0   2
  // v        0   1   5
  // a        1   2   3
  EXPECT(assert_equal(0.0, mp_euler_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.0, mp_euler_result.atDouble(JointVelKey(j, 1))));
  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(5.0, mp_euler_result.atDouble(JointVelKey(j, 2))));
  EXPECT(assert_equal(3.0, mp_euler_result.atDouble(JointAccelKey(j, 2))));

  // multi-phase Trapezoidal
  NonlinearFactorGraph mp_trapezoidal_graph = graph_builder.multiPhaseTrajectoryFG(my_robot, phase_steps, DynamicsGraphBuilder::CollocationScheme::Trapezoidal, gravity, planar_axis);
  mp_trapezoidal_graph.add(mp_prior_graph);
  Values mp_trapezoidal_result =graph_builder.optimize(mp_trapezoidal_graph, init_values, DynamicsGraphBuilder::OptimizerType::PDL);

  // t        0     1     2
  // dt          1     2
  // torque   1     2     3
  // q        0     0.75  8.75
  // v        0     1.5   6.5
  // a        1     2     3
  EXPECT(assert_equal(0.75, mp_trapezoidal_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.5, mp_trapezoidal_result.atDouble(JointVelKey(j, 1))));
  EXPECT(assert_equal(2.0, mp_trapezoidal_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(assert_equal(8.75, mp_trapezoidal_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(6.5, mp_trapezoidal_result.atDouble(JointVelKey(j, 2))));
  EXPECT(assert_equal(3.0, mp_trapezoidal_result.atDouble(JointAccelKey(j, 2))));
}


// check joint limit factors
TEST(jointlimitFactors, simple_urdf)
{
  using namespace simple_urdf;
  auto graph_builder = DynamicsGraphBuilder();
  NonlinearFactorGraph joint_limit_factors = graph_builder.jointLimitFactors(my_robot, 0);

  // 4 joint limit factors per joint (angle, velocity, acceleration, torque).
  EXPECT(assert_equal((long)my_robot.joints().size() * 4, joint_limit_factors.keys().size()));
}

int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}