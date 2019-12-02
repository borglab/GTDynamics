/**
 * @file  testDynamicsGraph.cpp
 * @brief test forward and inverse dynamics factor graph
 * @Author: Yetong Zhang
 */

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

int DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE = 1;
int DEBUG_FOUR_BAR_LINKAGE_ILS_EXAMPLE = 0;

// print the info of links and joints of the robot
void print_robot(UniversalRobot& this_robot) {
  for (const auto& link: this_robot.links()) {
    cout<<link->name() << ":\n";
    cout<<"\tlink pose: " << link->getLinkPose().rotation().rpy().transpose() << ", " << link->getLinkPose().translation() << "\n";
    cout<<"\tcom pose: " << link->getComPose().rotation().rpy().transpose() << ", " << link->getComPose().translation() << "\n";
  }

  for (const auto& joint: this_robot.joints()) {
    cout << joint->name() << ":\n";
    cout<<"\tparent: " << joint->parentLink()->name() << "\tchild: " << joint->childLink().lock()->name() << "\n";
    cout<<"\tscrew axis: " << joint->screwAxis().transpose() << "\n";
    cout<<"\tpMc: " << joint->pMc().rotation().rpy().transpose() << ", " << joint->pMc().translation() << "\n";
    cout<<"\tpMc_com: " << joint->pMcCom().rotation().rpy().transpose() << ", " << joint->pMcCom().translation() << "\n";
  }
}

// print the factors of the factor graph
void print_graph(const NonlinearFactorGraph& graph) {
  for (auto& factor: graph) {
    for (auto& key: factor->keys()) {
      auto symb = LabeledSymbol(key);
      cout << symb.chr() << int(symb.label()) << "_" << symb.index() << "\t";
    }
    cout << "\n";
  }
}

// print the values
void print_values(const Values& result) {
      for (auto& key: result.keys()) {
      auto symb = LabeledSymbol(key);
      cout << symb.chr() << int(symb.label()) << "_" << symb.index() << " ";
      result.at(key).print();
      cout << "\n";
    }
}

// Test forward dynamics with gravity
TEST(FD_factor_graph, optimization) {

  // Load the robot from urdf file
  UniversalRobot simple_robot = UniversalRobot("../../../urdfs/test/four_bar_linkage_pure.urdf");
  print_robot(simple_robot);

  Vector twists = Vector6::Zero(), accels = Vector6::Zero(),
      wrenches = Vector6::Zero();
  Vector q = Vector::Zero(simple_robot.numJoints());
  Vector v = Vector::Zero(simple_robot.numJoints());
  Vector a = Vector::Zero(simple_robot.numJoints());
  Vector torque = Vector::Zero(simple_robot.numJoints());
  Vector3 gravity = (Vector(3) << 0, -9.8, 0).finished();
  Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();
  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(simple_robot, 0, gravity, planar_axis);

  // specify known values
  for (auto link: simple_robot.links()) {
    int i = link -> getID();
    graph.add(PriorFactor<Pose3>(PoseKey(i, 0), link -> getComPose(), noiseModel::Constrained::All(6)));
    graph.add(PriorFactor<Vector6>(TwistKey(i, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  }
  for (auto joint: simple_robot.joints()) {
    int j = joint -> getID();
    graph.add(PriorFactor<double>(JointAngleKey(j, 0), 0, noiseModel::Constrained::All(1)));
    graph.add(PriorFactor<double>(JointVelKey(j, 0), 0, noiseModel::Constrained::All(1)));
    graph.add(PriorFactor<double>(TorqueKey(j, 0), 0, noiseModel::Constrained::All(1)));
  }

  // set initial values
  Values init_values;
  for (auto link: simple_robot.links()) {
    int i = link -> getID();
    init_values.insert(PoseKey(i, 0), link -> getComPose());
    init_values.insert(TwistKey(i, 0), twists);
    init_values.insert(TwistAccelKey(i, 0), accels);
  }
  for (auto joint: simple_robot.joints()) {
    int j = joint -> getID();
    init_values.insert(WrenchKey(joint->parentLink()->getID(), j, 0), wrenches);
    init_values.insert(WrenchKey(joint->childLink().lock()->getID(), j, 0), wrenches);
    Vector torque0 = Vector::Zero(1);
    // torque0 << 1;
    init_values.insert(TorqueKey(j, 0), torque0[0]);
    init_values.insert(JointAngleKey(j, 0), q[0]);
    init_values.insert(JointVelKey(j, 0), v[0]);
    init_values.insert(JointAccelKey(j, 0), a[0]);
  }
  print_values(init_values);

  // graph.print("", MultiRobotKeyFormatter);
  if(DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE) {
    print_graph(graph);
  }

  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();

  if (DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE) {
    for (auto& key: result.keys()) {
      auto symb = LabeledSymbol(key);
      cout << symb.chr() << int(symb.label()) << "_" << symb.index() << " ";
      result.at(key).print();
      cout << "\n";
    }
    // result.print();

    cout << "error: " << graph.error(result) << "\n";
  }

}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}