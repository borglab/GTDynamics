/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testNonlinearDynamicsEliminateableGraph.cpp
 * @brief Test non-linear eliminateble dynamics factor graph
 * @Author: Mandy Xie
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/NonlinearDynamicsBayesNet.h>
#include <gtdynamics/dynamics/NonlinearDynamicsEliminateableGraph.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtdynamics/utils/utils.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtdynamics;
using namespace gtsam;
using namespace std;

namespace example {
// noise model
auto cost_model = noiseModel::Gaussian::Covariance(I_1x1);
Key torque_key = Symbol('t', 1), wrench_key = Symbol('F', 1);
}  // namespace example

// Test nonlinear eliminatebale dynamics graph
TEST(NonlinearDynamicsEliminateableGraph, nonlinearElimination) {
  using example::cost_model;
  using example::torque_key;
  using example::wrench_key;
  // Create a torque factor
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  TorqueFactor torqueFactor(wrench_key, torque_key, cost_model, screw_axis);
  // Create a non-linear dynamic factor graph with only a torque factor
  auto NLEDG = NonlinearDynamicsEliminateableGraph();
  NLEDG.emplace_shared<TorqueFactor>(torqueFactor);
  // // peform partial elimination
  NonlinearDynamicsBayesNet::shared_ptr actualBN;
  NonlinearDynamicsEliminateableGraph::shared_ptr actualRemainingNLEDG;
  auto ordering = Ordering(boost::assign::list_of(torque_key));
  boost::tie(actualBN, actualRemainingNLEDG) =
      NLEDG.eliminatePartialSequential(ordering);

  // expected Bayes net
  auto torqueConditional = NonlinearDynamicsConditional(
      wrench_key, torque_key, cost_model, screw_axis, torque_key);
  auto expectedBN = NonlinearDynamicsBayesNet();
  expectedBN.emplace_shared<NonlinearDynamicsConditional>(torqueConditional);
  // expected remaining NLEDG is empty
  auto expectedRemainingNLEDG = NonlinearDynamicsEliminateableGraph();

  // check if the result matches
  EXPECT(assert_equal(*actualBN, expectedBN));
  EXPECT(assert_equal(*actualRemainingNLEDG, expectedRemainingNLEDG));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
