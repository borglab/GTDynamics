/**
 * @file  testJumpingRobot.cpp
 * @brief test forward dynamics for four-bar linkage
 * @Author: Yetong Zhang
 */

#include <PneumaticActuator.h>
#include <DynamicsGraph.h>
#include <UniversalRobot.h>
#include <Simulator.h>
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

void merge(Values& values, const Values& new_values) {
  for (Key &key : new_values.keys())
    {
      if (!values.exists(key)) {
        values.insert(key, new_values.at(key));
      }
    }
}

class JumpingRobot
{
private:
  DynamicsGraphBuilder graph_builder_;
  gtsam::Vector3 gravity_;
  gtsam::Vector3 planar_axis_;
  UniversalRobot robot_;
  vector<PneumaticActuator> actuators_;
  Simulator simulator_;

  UniversalRobot loadRobot()
  {
    UniversalRobot jumping_robot = UniversalRobot("../../../urdfs/test/jumping_robot.urdf");
    jumping_robot.getLinkByName("l0")->fix();
    return jumping_robot;
  }

  vector<PneumaticActuator> getActuators(const Vector &rest_angles)
  {
    vector<PneumaticActuator> actuators;

    // coefficients for pressure factor
    const double c1 = -12.05020559, c2 = 8.88481485, c3 = -85.56821655, t0 = 0.224;
    const vector<double> pressure_coeffs{t0, c1, c2, c3};

    // coefficients for pneumatic actuator factor
    const double p00 = -17.39, p10 = 1.11, p01 = 2.22, p20 = -0.9486,
                 p11 = -0.4481, p02 = -0.0003159, p30 = 0.1745, p21 = 0.01601,
                 p12 = 0.0001081, p03 = -7.703e-07;
    const vector<double> pneumatic_coeffs{p00, p10, p01, p20, p11, p02, p30, p21, p12, p03};

    // coefficients for actuator joint factor
    const double k = 1000;
    const double r = 0.04;

    actuators.emplace_back(PneumaticActuator(2, pressure_coeffs, pneumatic_coeffs, k, r, rest_angles[1], false));
    actuators.emplace_back(PneumaticActuator(3, pressure_coeffs, pneumatic_coeffs, k, r, rest_angles[2], true));
    actuators.emplace_back(PneumaticActuator(4, pressure_coeffs, pneumatic_coeffs, k, r, rest_angles[3], true));
    actuators.emplace_back(PneumaticActuator(5, pressure_coeffs, pneumatic_coeffs, k, r, rest_angles[4], false));
    return actuators;
  }

public:
  /**
   * Construct jumping robot
   * Keyword arguments:
        rest_angles        -- angles at rest for each joint
        init_angles        -- initial joint angles
        int_vels           -- initial joint velocities
   */
  explicit JumpingRobot(const Vector &rest_angles,
                        const Vector &init_angles,
                        const Vector &init_vels) : graph_builder_(DynamicsGraphBuilder()),
                                                   gravity_((Vector(3) << 0, 0, -9.8).finished()),
                                                   planar_axis_((Vector(3) << 1, 0, 0).finished()),
                                                   robot_(loadRobot()),
                                                   actuators_(getActuators(rest_angles)),
                                                   simulator_(Simulator(robot_, init_angles, init_vels, gravity_, planar_axis_))
  {
  }

  Vector computeTorques(const int t, const double dt, const Vector& init_pressures, const Vector& start_times, Values& values) {
      Vector torques = Vector::Zero(robot_.numJoints());

      Vector qs = simulator_.getJointAngles();
      for (int actuator_idx = 0; actuator_idx < actuators_.size(); actuator_idx++) {
        auto& actuator = actuators_[actuator_idx];
        int joint_idx = actuator_idx + 1;
        double angle = qs[joint_idx];
        double delta_t = t*dt - start_times[actuator_idx];
        double init_pressure = init_pressures[actuator_idx];
        Values actuator_values = actuator.computeResult(t, angle, delta_t, init_pressure);
        merge(values, actuator_values);
        double torque = actuator_values.atDouble(TorqueKey(actuator.j(), t));
        torques(joint_idx) = torque;
      }
      return torques;
  }

  Values simulate(const int num_steps, const double dt, const Vector& init_pressures, const Vector& start_times)
  {
    // reset simulator
    simulator_.reset();

    // simulate to get values
    Values values;
    for (int t=0; t<num_steps; t++) {
      Vector torques = computeTorques(t, dt, init_pressures, start_times, values);
      simulator_.step(torques, dt);
    }
    Vector torques = computeTorques(num_steps, dt, init_pressures, start_times, values);
    simulator_.forwardDynamics(torques);
    merge(values, simulator_.getValues());
    return values;
  }

  NonlinearFactorGraph trajectoryFG(const int num_steps, const double dt) const
  {
    NonlinearFactorGraph graph;
    auto collocation = DynamicsGraphBuilder::CollocationScheme::Trapezoidal;

    // dynamics graph
    graph.add(graph_builder_.trajectoryFG(robot_, num_steps, dt, collocation, gravity_, planar_axis_));

    // actuator graph
    for (int t = 0; t <= num_steps; t++)
    {
      for (auto &actuator : actuators_)
      {
        graph.add(actuator.actuatorFactorGraph(t));
      }
    }

    // priors
    for (int t = 0; t <= num_steps; t++)
    {
      graph.add(PriorFactor<double>(TimeKey(t), t * dt,
                                    gtsam::noiseModel::Constrained::All(1)));
    }
    return graph;
  }
};

// Test forward dynamics with gravity
TEST(FD_factor_graph, optimization)
{
  Vector rest_angles = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  Vector init_angles = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  Vector init_vels = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  JumpingRobot jr(rest_angles, init_angles, init_vels);

  int num_steps = 2;
  double dt = 0.1;
  Vector init_pressures = (Vector(4) << 240, 240, 240, 240).finished();
  Vector start_times = (Vector(4) << 0, 0, 0, 0).finished();

  cout << "start simulate\n";
  Values init_values = jr.simulate(num_steps, dt, init_pressures, start_times);
  cout << "construct trajectory FG\n";
  NonlinearFactorGraph graph = jr.trajectoryFG(num_steps, dt);
  cout << "graph:\n";
  DynamicsGraphBuilder::print_graph(graph);

  cout << "values:\n";
  DynamicsGraphBuilder::print_values(init_values);
  std::cout << graph.error(init_values)<<"\n";

  Values results = DynamicsGraphBuilder::optimize(graph, init_values, DynamicsGraphBuilder::OptimizerType::LM);
  std::cout << graph.error(results)<<"\n";

}

int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}