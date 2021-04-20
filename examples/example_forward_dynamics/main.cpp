#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace gtdynamics;
int main(int argc, char** argv) {
  // Load the robot
  auto simple_rr =
      CreateRobotFromFile(SDF_PATH + "/test/simple_rr.sdf", "simple_rr_sdf");
  simple_rr.print();

  gtsam::Vector3 gravity(0, 0, -9.8);

  // Build a nonlinear factor graph of kinodynamics constraints.
  auto graph_builder = DynamicsGraph(gravity);
  auto kdfg = graph_builder.dynamicsFactorGraph(simple_rr,
                                                0  // timestep
  );

  // Specify the forward dynamics priors and add them to the factor graph.
  gtsam::Values known_values;
  for (auto&& joint : simple_rr.joints()) {
    int j = joint->id();
    InsertJointAngle(&known_values, j, 0, 0.0);
    InsertJointVel(&known_values, j, 0, 0.0);
    InsertTorque(&known_values, j, 0, 0.0);
  }
  auto fd_priors =
      graph_builder.forwardDynamicsPriors(simple_rr, 0, known_values);
  kdfg.add(fd_priors);

  // Initialize solution.
  auto init_values = ZeroValues(simple_rr, 0);

  // Compute the forward dynamics.
  gtsam::LevenbergMarquardtOptimizer optimizer(kdfg, init_values);
  gtsam::Values results = optimizer.optimize();
  graph_builder.printValues(results);

  return 0;
}
