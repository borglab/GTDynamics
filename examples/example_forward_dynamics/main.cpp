#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace gtdynamics;
int main(int argc, char** argv) {
  // Load the robot and build a nonlinear factor graph of kinodynamics
  // constraints.
  auto simple_rr = CreateRobotFromFile("../simple_rr.sdf", "simple_rr_sdf");
  simple_rr.print();

  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  auto graph_builder = DynamicsGraph(gravity);
  auto kdfg = graph_builder.dynamicsFactorGraph(simple_rr,
                                                0  // timestep
                                                );


    



  // Specify the forward dynamics priors and add them to the factor graph.
  gtsam::Vector theta = (gtsam::Vector(2) << 0, 0).finished();
  gtsam::Vector theta_dot = (gtsam::Vector(2) << 0, 0).finished();
  gtsam::Vector tau = (gtsam::Vector(2) << 0, 0).finished();
  auto fd_priors =
      graph_builder.forwardDynamicsPriors(simple_rr, 0, theta, theta_dot, tau);
  kdfg.add(fd_priors);

  // Initialize solution.
  auto init_values = ZeroValues(simple_rr, 0);

  // Compute the forward dynamics.
  gtsam::LevenbergMarquardtOptimizer optimizer(kdfg, init_values);
  gtsam::Values results = optimizer.optimize();
  graph_builder.printValues(results);

  return 0;
}
