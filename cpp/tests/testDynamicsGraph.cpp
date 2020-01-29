/**
 * @file  testDynamicsGraph.cpp
 * @brief test forward and inverse dynamics factor graph
 * @Author: Yetong Zhang, Alejandro Escontrela
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

// Test forward dynamics with gravity of a two-link robot, with base link fixed
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
    graph.add(PriorFactor<Pose3>(PoseKey(i, 0), link->Twcom(), noiseModel::Constrained::All(6)));
    graph.add(PriorFactor<Vector6>(TwistKey(i, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  }

  Values result = graph_builder.optimize(graph, DynamicsGraphBuilder::zeroValues(my_robot, 0), DynamicsGraphBuilder::OptimizerType::GaussNewton);

  Vector actual_qAccel = DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  Vector expected_qAccel = (Vector(1) << 4).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 1e-3));
}

// Test forward dynamics with gravity of a four-bar linkage
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
    prior_factors.add(PriorFactor<Pose3>(PoseKey(i, 0), link->Twcom(), noiseModel::Constrained::All(6)));
    prior_factors.add(PriorFactor<Vector6>(TwistKey(i, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  }

  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(prior_factors);

  Values init_values = DynamicsGraphBuilder::zeroValues(my_robot, 0);

  // test the four bar linkage FD in the free-floating scenario
  Values result = graph_builder.optimize(graph, init_values, DynamicsGraphBuilder::OptimizerType::LM);
  Vector actual_qAccel = DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  Vector expected_qAccel = (Vector(4) << 1, -1, 1, -1).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 1e-4));

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

  result = graph_builder.optimize(graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);
  actual_qAccel = DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  expected_qAccel = (Vector(4) << 0.25, -0.25, 0.25, -0.25).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));

}


int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}