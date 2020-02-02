/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Simulator.h
 * @brief robot Simulator using forward dynamics factor graph
 * @Author:Yetong Zhang
 */
#pragma once

#include <DynamicsGraph.h>
#include <UniversalRobot.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

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
  gtsam::Vector initial_angles_, initial_vels_;
  boost::optional<gtsam::Vector3> gravity_;
  boost::optional<gtsam::Vector3> planar_axis_;
  gtsam::Vector qs_, vs_, as_;
  gtsam::Values results_;

 public:
  /**
   * Constructor
   * Keyword arguments:
   *  time_step                -- Simulator time step
   *  robot                    -- robotic manipulator
   *  initialJointAngles       -- initial joint angles
   *  initialJointVelocities   -- initial joint velocities
   */
  Simulator(const UniversalRobot &robot,
            const gtsam::Vector &initialJointAngles,
            const gtsam::Vector &initialJointVelocities,
            const boost::optional<gtsam::Vector3> &gravity = boost::none,
            const boost::optional<gtsam::Vector3> &planar_axis = boost::none)
      : robot_(robot),
        t_(0),
        graph_builder_(DynamicsGraphBuilder()),
        initial_angles_(initialJointAngles),
        initial_vels_(initialJointVelocities),
        gravity_(gravity),
        planar_axis_(planar_axis) {
    reset();
  }
  ~Simulator() {}

  /* reset simulation. */
  void reset(const double t = 0) {
    t_ = t;
    qs_ = initial_angles_;
    vs_ = initial_vels_;
    as_ = gtsam::Vector::Zero(robot_.numJoints());
    results_ = gtsam::Values();
  }

  gtsam::Values getInitValuesFromPrev(const int t) {
    gtsam::Values values;
    for (auto &link : robot_.links()) {
      int i = link->getID();
      values.insert(PoseKey(i, t), results_.at(PoseKey(i, t - 1)));
      values.insert(TwistKey(i, t), results_.at(TwistKey(i, t - 1)));
      values.insert(TwistAccelKey(i, t), results_.at(TwistAccelKey(i, t - 1)));
    }
    for (auto &joint : robot_.joints()) {
      int j = joint->getID();
      auto parent_link = joint->parentLink().lock();
      auto child_link = joint->childLink().lock();
      if (!parent_link->isFixed()) {
        values.insert(WrenchKey(parent_link->getID(), j, t),
                      results_.at(WrenchKey(parent_link->getID(), j, t - 1)));
      }
      if (!child_link->isFixed()) {
        values.insert(WrenchKey(child_link->getID(), j, t),
                      results_.at(WrenchKey(child_link->getID(), j, t - 1)));
      }
      values.insert(TorqueKey(j, t), results_.at(TorqueKey(j, t - 1)));
      values.insert(JointAngleKey(j, t), results_.at(JointAngleKey(j, t - 1)));
      values.insert(JointVelKey(j, t), results_.at(JointVelKey(j, t - 1)));
      values.insert(JointAccelKey(j, t), results_.at(JointAccelKey(j, t - 1)));
    }
    return values;
  }

  /**
   * perform forward dynamics to calculate accelerations, update a_, add new
   * values to results_ Keyword arguments: torques                   -- torques
   * for the time step
   */
  void forwardDynamics(const gtsam::Vector &torques) {
    gtsam::Values result;
    if (results_.size() == 0) {  // first step
      gtsam::NonlinearFactorGraph graph = graph_builder_.dynamicsFactorGraph(
          robot_, t_, gravity_, planar_axis_);
      gtsam::Values init_values = graph_builder_.zeroValues(robot_, t_);
      result = graph_builder_.optimize(graph, init_values,
                                       DynamicsGraphBuilder::OptimizerType::LM);
      // result = graph_builder_.iterativeSolveFD(robot_, t_, qs_, vs_, torques,
      // gravity_, planar_axis_);
    } else {
      gtsam::NonlinearFactorGraph graph = graph_builder_.dynamicsFactorGraph(
          robot_, t_, gravity_, planar_axis_);
      graph.add(
          graph_builder_.forwardDynamicsPriors(robot_, t_, qs_, vs_, torques));
      // gtsam::Values init_values = DynamicsGraphBuilder::zeroValues(robot_,
      // t_);
      gtsam::Values init_values = getInitValuesFromPrev(t_);
      result = graph_builder_.optimize(graph, init_values,
                                       DynamicsGraphBuilder::OptimizerType::LM);

      // make sure the optimization converge to global minimum
      double error = graph.error(result);
      if (error > 1) {
        result = graph_builder_.optimize(
            graph, init_values, DynamicsGraphBuilder::OptimizerType::PDL);
        error = graph.error(result);
        if (error > 1) {
          std::cout << "t= " << t_ << "\terror= " << error << "\n";
          throw std::runtime_error("Fail to optimize for dynamics graph");
        }
      }
    }

    results_.insert(result);

    // DynamicsGraphBuilder::saveGraphMultiSteps("../../../visualization/factor_graph.json",
    // graph, result, robot_, t_, false); std::cout << "error " <<
    // graph.error(result) << "\n";

    // update values
    as_ = DynamicsGraphBuilder::jointAccels(robot_, result, t_);
  }

  /**
   * integrate to calculate new q, v for one time step, update q_, v_
   * Keyword arguments:
   *  torques                   -- torques for the time step
   *  dt                        -- duration for the time step
   */
  void integration(const double dt) {
    gtsam::Vector vs_new = (vs_ + dt * as_).eval();
    // gtsam::Vector qs_new = (qs_ + dt * vs_).eval();
    gtsam::Vector qs_new =
        (qs_ + dt * vs_ + 0.5 * as_ * std::pow(dt, 2)).eval();
    vs_ = vs_new;
    qs_ = qs_new;
  }

  /**
   * simulate for one time step, update q_, v_, a_, t_, add the new values into
   * result_ Keyword arguments: torques                   -- torques for the
   * time step dt                        -- duration for the time step
   */
  void step(const gtsam::Vector &torques, const double dt) {
    forwardDynamics(torques);
    integration(dt);
    t_++;
  }

  /* simulation for the specified sequence of torques */
  gtsam::Values simulate(const std::vector<gtsam::Vector> torques_seq,
                         const double dt) {
    for (const gtsam::Vector &torques : torques_seq) {
      step(torques, dt);
    }
    return results_;
  }

  /* return joint angle values. */
  const gtsam::Vector &getJointAngles() const { return qs_; }

  /* return joint velocity values. */
  const gtsam::Vector &getJointVelocities() const { return vs_; }

  /* return joint acceleration values. */
  const gtsam::Vector &getJointAccelerations() const { return as_; }

  /* return all values during simulation. */
  const gtsam::Values &getValues() const { return results_; }
};

}  // namespace robot
