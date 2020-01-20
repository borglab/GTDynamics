/**
 * @file  testJumpingRobot.cpp
 * @brief test forward dynamics for four-bar linkage
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
#include <fstream>

using namespace std;
using namespace robot;
using namespace gtsam;

class JumpingRobot
{
private:
  DynamicsGraphBuilder graph_builder_;
  UniversalRobot robot_;

public:
  explicit JumpingRobot() : graph_builder_(DynamicsGraphBuilder()),
                            robot_(UniversalRobot("../../../urdfs/test/jumping_robot.urdf"))
  {
    robot_.getLinkByName("l0")->fix();
  }

  NonlinearFactorGraph actuatorFG() const
  {

  }
};

// Test forward dynamics with gravity
TEST(FD_factor_graph, optimization)
{

  // Load the robot from urdf file
  UniversalRobot jumping_robot = UniversalRobot("../../../urdfs/test/jumping_robot.urdf");
  jumping_robot.getLinkByName("l0")->fix();
  jumping_robot.printRobot();

  Vector twists = Vector6::Zero(), accels = Vector6::Zero(),
         wrenches = Vector6::Zero();
  Vector q = Vector::Zero(jumping_robot.numJoints());
  Vector v = Vector::Zero(jumping_robot.numJoints());
}

int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}