/**
 * @file  Simulator.h
 * @brief robot Simulator using forward dynamics factor graph
 * @Author:Yetong Zhang
 */
#pragma once

#include <DynamicsGraph.h>
#include <UniversalRobot.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/optional.hpp>

namespace robot {
/**
 * Simulator is a class which simulate robot arm motion using forward
 * dynamics
 */
class Simulator {
private:
  UniversalRobot robot_;
  int t_;
  DynamicsGraphBuilder graph_builder_;
  gtsam::Vector qs_, vs_, as_, torques_;
  boost::optional<gtsam::Vector3> gravity_;
  boost::optional<gtsam::Vector3> planar_axis_;
  gtsam::Values results_;

public:
  /**
   * Constructor
   * Keyword arguments:
   *  time_step                -- Simulator time step
   *  robot                    -- robotic manipulator
   *  initialJointAngles       -- initial joint angles
   *  initialJointVelocities   -- initial joint velocities
   *  initialConditions        -- condition to specify for the first timestep
   */
  Simulator(
      const UniversalRobot &robot, const gtsam::Vector &initialJointAngles,
      const gtsam::Vector &initialJointVelocities,
      //  const boost::optional<gtsam::Values> &initialConditions = boost::none,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none)
      : robot_(robot), t_(0), graph_builder_(DynamicsGraphBuilder()),
        qs_(initialJointAngles), vs_(initialJointVelocities),
        as_(gtsam::Vector::Zero(robot.numJoints())),
        torques_(gtsam::Vector::Zero(robot.numJoints())), gravity_(gravity),
        planar_axis_(planar_axis) {}
  ~Simulator() {}

  // compute forward dynamics for one step, and update the joint accelerations
  void forwardDynamics(const gtsam::Vector &known_torques) {
    // construct Dynamics Factor Graph
    gtsam::NonlinearFactorGraph graph =
        graph_builder_.dynamicsFactorGraph(robot_, t_, gravity_, planar_axis_);

    // add prior factors
    graph.add(graph_builder_.forwardDynamicsPriors(robot_, t_, qs_, vs_,
                                                   known_torques));

    // optimize factor graph
    gtsam::Values init_values = DynamicsGraphBuilder::zeroValues(robot_, t_);

    // DynamicsGraphBuilder::print_graph(graph);
    // DynamicsGraphBuilder::print_values(init_values);

    gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
    optimizer.optimize();

    auto result = optimizer.values();
    results_.insert(result);

    // DynamicsGraphBuilder::saveGraph("../../../visualization/factor_graph.json",
    //                                 graph, result, robot_, t_, false);
    // update values
    torques_ = known_torques;
    as_ = DynamicsGraphBuilder::jointAccels(robot_, result, t_);
  }

  // integrate through a time step
  void integration(const double dt) {
    gtsam::Vector vs_new = (vs_ + dt * as_).eval();
    gtsam::Vector qs_new = (qs_ + dt * vs_ + 0.5 * as_ * std::pow(dt, 2)).eval();
    vs_ = vs_new;
    qs_ = qs_new;
  }

  // simulation for one step with given torques
  void step(const gtsam::Vector &known_torques, const double dt) {
    forwardDynamics(known_torques);
    integration(dt);
    t_++;
  }

  // simulation for the specified sequence of torques
  gtsam::Values simulate(const std::vector<gtsam::Vector> torques_seq, const double dt) {
    for (const gtsam::Vector& known_torques: torques_seq) {
      step(known_torques, dt);
    }
    return results_;
  }

  /// return joint angle values
  const gtsam::Vector &getJointAngles() const { return qs_; }

  /// return joint velocity values
  const gtsam::Vector &getJointVelocities() const { return vs_; }

  /// return joint acceleration values
  const gtsam::Vector &getJointAccelerations() const { return as_; }

  /// return joint torque values
  const gtsam::Vector &getJointTorques() const { return torques_; }
};

} // namespace robot
