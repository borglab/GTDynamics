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

TEST(DynamicsGraph, forward_dynamics_rr) {

    using namespace simple_urdf;

    robot::DynamicsGraphBuilder dg_builder = robot::DynamicsGraphBuilder();

    my_robot.getLinkByName("l1")->fix();

    gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
    gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 0, 0, 1).finished();

    gtsam::NonlinearFactorGraph dfg = dg_builder.dynamicsFactorGraph(
        my_robot, 0, gravity, planar_axis);

    gtsam::Vector theta = (gtsam::Vector(2) << 0, 0).finished();
    gtsam::Vector theta_dot = (gtsam::Vector(2) << 0, 0).finished();
    gtsam::Vector tau = (gtsam::Vector(2) << 0, 0).finished();
    gtsam::NonlinearFactorGraph fd_priors = dg_builder.forwardDynamicsPriors(
        my_robot, 0, theta, theta_dot, tau);

    dfg.add(fd_priors);

    gtsam::Values init_values = dg_builder.zeroValues(my_robot, 0);

    gtsam::Values results = dg_builder.optimize(dfg, init_values,
        robot::DynamicsGraphBuilder::OptimizerType::GaussNewton);

    dg_builder.print_values(results);

    std::cout << "Optimization error: " << dfg.error(results) << std::endl;

}

int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}