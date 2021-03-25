/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DynamicsGraph.h
 * @brief Builds a dynamics graph from a Robot object.
 * @author Yetong Zhang, Alejandro Escontrela
 */

#include "gtdynamics/dynamics/DynamicsGraph.h"

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "gtdynamics/factors/ContactDynamicsFrictionConeFactor.h"
#include "gtdynamics/factors/ContactDynamicsMomentFactor.h"
#include "gtdynamics/factors/ContactKinematicsAccelFactor.h"
#include "gtdynamics/factors/ContactKinematicsPoseFactor.h"
#include "gtdynamics/factors/ContactKinematicsTwistFactor.h"
#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/utils/JsonSaver.h"
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/values.h"

using gtsam::Double_;
using gtsam::GaussianFactorGraph;
using gtsam::I_1x1;
using gtsam::I_6x6;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector6;
using gtsam::Z_6x1;

namespace gtdynamics {

GaussianFactorGraph DynamicsGraph::linearDynamicsGraph(
    const Robot &robot, const int t, const gtsam::Values &known_values) {
  GaussianFactorGraph graph;
  auto all_constrained = gtsam::noiseModel::Constrained::All(6);
  for (auto &&link : robot.links()) {
    int i = link->id();
    if (link->isFixed()) {
      // prior on twist acceleration for fixed link
      // A_i = 0
      graph.add(internal::TwistAccelKey(i, t), I_6x6, Z_6x1, all_constrained);
    } else {
      // wrench factor
      // G_i * A_i - F_i_j1 - .. - F_i_jn  = ad(V_i)^T * G_i * V*i + m_i * R_i^T
      // * g
      const auto &connected_joints = link->getJoints();
      const gtsam::Matrix6 G_i = link->inertiaMatrix();
      const double m_i = link->mass();
      const Pose3 T_wi = Pose(known_values, i, t);
      const Vector6 V_i = Twist(known_values, i, t);
      Vector6 rhs = Pose3::adjointMap(V_i).transpose() * G_i * V_i;
      if (gravity_) {
        Vector gravitational_force =
            T_wi.rotation().transpose() * (*gravity_) * m_i;
        for (int i = 3; i < 6; ++i) {
          rhs[i] += gravitational_force[i - 3];
        }
      }
      auto accel_key = internal::TwistAccelKey(i, t);
      if (connected_joints.size() == 0) {
        graph.add(accel_key, G_i, rhs, all_constrained);
      } else if (connected_joints.size() == 1) {
        graph.add(accel_key, G_i,
                  internal::WrenchKey(i, connected_joints[0]->id(), t), -I_6x6,
                  rhs, all_constrained);
      } else if (connected_joints.size() == 2) {
        graph.add(accel_key, G_i,
                  internal::WrenchKey(i, connected_joints[0]->id(), t), -I_6x6,
                  internal::WrenchKey(i, connected_joints[1]->id(), t), -I_6x6,
                  rhs, all_constrained);
      }
    }
  }

  OptimizerSetting opt_;
  for (auto &&joint : robot.joints()) {
    graph += joint->linearAFactors(t, known_values, opt_, planar_axis_);
    graph += joint->linearDynamicsFactors(t, known_values, opt_, planar_axis_);
  }

  return graph;
}

GaussianFactorGraph DynamicsGraph::linearFDPriors(
    const Robot &robot, const int t, const gtsam::Values &torques) {
  OptimizerSetting opt_ = OptimizerSetting();
  GaussianFactorGraph graph;
  for (auto &&joint : robot.joints()) graph += joint->linearFDPriors(t, torques, opt_);
  return graph;
}

GaussianFactorGraph DynamicsGraph::linearIDPriors(
    const Robot &robot, const int t, const gtsam::Values &joint_accels) {
  GaussianFactorGraph graph;
  auto all_constrained = gtsam::noiseModel::Constrained::All(1);
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    double accel = JointAccel(joint_accels, j, t);
    gtsam::Vector1 rhs(accel);
    graph.add(internal::JointAccelKey(j, t), I_1x1, rhs, all_constrained);
  }
  return graph;
}

Values DynamicsGraph::linearSolveFD(const Robot &robot, const int t,
                                    const gtsam::Values &known_values) {
  // construct and solve linear graph
  GaussianFactorGraph graph = linearDynamicsGraph(robot, t, known_values);
  GaussianFactorGraph priors = linearFDPriors(robot, t, known_values);
  graph += priors;
  gtsam::VectorValues results = graph.optimize();

  // arrange values
  Values values = known_values;
  try {
    // Copy accelerations and wrenches to result.
    for (auto &&joint : robot.joints()) {
      int j = joint->id();
      int i1 = joint->parent()->id();
      int i2 = joint->child()->id();
      InsertJointAccel(&values, j, t, JointAccel(results, j, t)[0]);
      InsertWrench(&values, i1, j, t, Wrench(results, i1, j, t));
      InsertWrench(&values, i2, j, t, Wrench(results, i2, j, t));
    }
    for (auto &&link : robot.links()) {
      int i = link->id();
      InsertTwistAccel(&values, i, t, TwistAccel(results, i, t));
    }
  } catch (const gtsam::ValuesKeyAlreadyExists &e) {
    std::cerr << "key already exists:" << _GTDKeyFormatter(e.key()) << '\n';
    throw std::invalid_argument(
        "linearSolveFD: known_values should contain no accelerations or "
        "wrenches");
  }
  return values;
}

Values DynamicsGraph::linearSolveID(const Robot &robot, const int t,
                                    const gtsam::Values &known_values) {
  // construct and solve linear graph
  GaussianFactorGraph graph = linearDynamicsGraph(robot, t, known_values);
  GaussianFactorGraph priors = linearIDPriors(robot, t, known_values);
  graph += priors;

  gtsam::VectorValues results = graph.optimize();

  // arrange values
  Values values = known_values;
  try {
    // Copy torques and wrenches to result.
    for (auto &&joint : robot.joints()) {
      int j = joint->id();
      int i1 = joint->parent()->id();
      int i2 = joint->child()->id();
      std::string name = joint->name();
      InsertTorque(&values, j, t, Torque(results, j, t)[0]);
      InsertWrench(&values, i1, j, t, Wrench(results, i1, j, t));
      InsertWrench(&values, i2, j, t, Wrench(results, i2, j, t));
    }
    for (auto &&link : robot.links()) {
      int i = link->id();
      std::string name = link->name();
      InsertTwistAccel(&values, i, t, TwistAccel(results, i, t));
    }
  } catch (const gtsam::ValuesKeyAlreadyExists &e) {
    std::cerr << "key already exists:" << _GTDKeyFormatter(e.key()) << '\n';
    throw std::invalid_argument(
        "linearSolveID: known_values should contain no torques, "
        "wrenches, or twist accelerations.");
  }
  return values;
}

gtsam::NonlinearFactorGraph DynamicsGraph::qFactors(
    const Robot &robot, const int t,
    const boost::optional<ContactPoints> &contact_points) const {
  NonlinearFactorGraph graph;
  for (auto &&link : robot.links()) graph.add(link->qFactors(t, opt_));
  for (auto &&joint : robot.joints()) graph.add(joint->qFactors(t, opt_));

  // TODO(frank): clearly document this behavior
  gtsam::Vector3 gravity;
  if (gravity_)
    gravity = *gravity_;
  else
    gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();

  // Add contact factors.
  for (auto &&link : robot.links()) {
    int i = link->id();
    // Check if the link has contact points. If so, add pose constraints.
    if (contact_points) {
      for (auto &&contact_point : *contact_points) {
        if (contact_point.first != link->name()) continue;

        ContactKinematicsPoseFactor contact_pose_factor(
            internal::PoseKey(i, t), opt_.cp_cost_model,
            gtsam::Pose3(gtsam::Rot3(), -contact_point.second.point), gravity,
            contact_point.second.height);
        graph.add(contact_pose_factor);
      }
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::vFactors(
    const Robot &robot, const int t,
    const boost::optional<ContactPoints> &contact_points) const {
  NonlinearFactorGraph graph;
  for (auto &&link : robot.links()) graph.add(link->vFactors(t, opt_));
  for (auto &&joint : robot.joints()) graph.add(joint->vFactors(t, opt_));

  // Add contact factors.
  for (auto &&link : robot.links()) {
    int i = link->id();
    // Check if the link has contact points. If so, add twist constraints.
    if (contact_points) {
      for (auto &&contact_point : *contact_points) {
        if (contact_point.first != link->name()) continue;

        ContactKinematicsTwistFactor contact_twist_factor(
            internal::TwistKey(i, t), opt_.cv_cost_model,
            gtsam::Pose3(gtsam::Rot3(), -contact_point.second.point));
        graph.add(contact_twist_factor);
      }
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::aFactors(
    const Robot &robot, const int t,
    const boost::optional<ContactPoints> &contact_points) const {
  NonlinearFactorGraph graph;
  for (auto &&link : robot.links()) graph.add(link->aFactors(t, opt_));
  for (auto &&joint : robot.joints()) graph.add(joint->aFactors(t, opt_));

  // Add contact factors.
  for (auto &&link : robot.links()) {
    int i = link->id();

    // Check if the link has contact points. If so, add accel constraints.
    if (contact_points) {
      for (auto &&contact_point : *contact_points) {
        if (contact_point.first != link->name()) continue;

        ContactKinematicsAccelFactor contact_accel_factor(
            internal::TwistAccelKey(i, t), opt_.ca_cost_model,
            gtsam::Pose3(gtsam::Rot3(), -contact_point.second.point));
        graph.add(contact_accel_factor);
      }
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::dynamicsFactors(
    const Robot &robot, const int t,
    const boost::optional<ContactPoints> &contact_points,
    const boost::optional<double> &mu) const {
  NonlinearFactorGraph graph;

  // TODO(frank): clearly document this behavior
  gtsam::Vector3 gravity;
  if (gravity_)
    gravity = *gravity_;
  else
    gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();

  double mu_;  // Static friction coefficient.
  if (mu)
    mu_ = *mu;
  else
    mu_ = 1.0;

  for (auto &&link : robot.links()) {
    int i = link->id();
    if (!link->isFixed()) {
      const auto &connected_joints = link->getJoints();
      std::vector<DynamicsSymbol> wrenches;

      // Add wrench keys for joints.
      for (auto &&joint : connected_joints)
        wrenches.push_back(internal::WrenchKey(i, joint->id(), t));

      // Add wrench keys for contact points.
      if (contact_points) {
        for (auto &&contact_point : *contact_points) {
          if (contact_point.first != link->name()) continue;

          wrenches.push_back(ContactWrenchKey(i, contact_point.second.id, t));

          // Add contact dynamics constraints.
          graph.add(ContactDynamicsFrictionConeFactor(
              internal::PoseKey(i, t),
              ContactWrenchKey(i, contact_point.second.id, t),
              opt_.cfriction_cost_model, mu_, gravity));

          graph.add(ContactDynamicsMomentFactor(
              ContactWrenchKey(i, contact_point.second.id, t),
              opt_.cm_cost_model,
              gtsam::Pose3(gtsam::Rot3(), -contact_point.second.point)));
        }
      }

      graph.add(link->dynamicsFactors(t, opt_, wrenches, gravity));
    }
  }

  for (auto &&joint : robot.joints())
    graph.add(joint->dynamicsFactors(t, opt_, planar_axis_));

  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::dynamicsFactorGraph(
    const Robot &robot, const int t,
    const boost::optional<ContactPoints> &contact_points,
    const boost::optional<double> &mu) const {
  NonlinearFactorGraph graph;
  graph.add(qFactors(robot, t, contact_points));
  graph.add(vFactors(robot, t, contact_points));
  graph.add(aFactors(robot, t, contact_points));
  graph.add(dynamicsFactors(robot, t, contact_points, mu));
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::trajectoryFG(
    const Robot &robot, const int num_steps, const double dt,
    const DynamicsGraph::CollocationScheme collocation,
    const boost::optional<ContactPoints> &contact_points,
    const boost::optional<double> &mu) const {
  NonlinearFactorGraph graph;
  for (int t = 0; t < num_steps + 1; t++) {
    graph.add(dynamicsFactorGraph(robot, t,
                                  contact_points, mu));
    if (t < num_steps) {
      graph.add(collocationFactors(robot, t, dt, collocation));
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::multiPhaseTrajectoryFG(
    const std::vector<Robot> &robots, const std::vector<int> &phase_steps,
    const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs,
    const CollocationScheme collocation,
    const boost::optional<std::vector<ContactPoints>> &phase_contact_points,
    const boost::optional<double> &mu) const {
  NonlinearFactorGraph graph;
  int num_phases = robots.size();

  // add dynamcis for each step
  int t = 0;
  graph.add(dynamicsFactorGraph(robots[0], t));

  for (int phase = 0; phase < num_phases; phase++) {
    // in-phase
    for (int phase_step = 0; phase_step < phase_steps[phase] - 1;
         phase_step++) {
      graph.add(dynamicsFactorGraph(robots[phase], ++t));
    }
    // transition
    if (phase == num_phases - 1) {
      graph.add(dynamicsFactorGraph(robots[phase], ++t));
    } else {
      t++;
      graph.add(transition_graphs[phase]);
    }
  }

  // add collocation factors
  t = 0;
  for (int phase = 0; phase < num_phases; phase++) {
    for (int phase_step = 0; phase_step < phase_steps[phase]; phase_step++) {
      graph.add(
          multiPhaseCollocationFactors(robots[phase], t++, phase, collocation));
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph
DynamicsGraph::collocationFactors(const Robot &robot, const int t,
                                  const double dt,
                                  const CollocationScheme collocation) const {
  gtsam::ExpressionFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    Double_ q0_expr(internal::JointAngleKey(j, t));
    Double_ q1_expr(internal::JointAngleKey(j, t + 1));
    Double_ v0_expr(internal::JointVelKey(j, t));
    Double_ v1_expr(internal::JointVelKey(j, t + 1));
    Double_ a0_expr(internal::JointAccelKey(j, t));
    Double_ a1_expr(internal::JointAccelKey(j, t + 1));
    switch (collocation) {
    case CollocationScheme::Euler:
      graph.addExpressionFactor(q0_expr + dt * v0_expr - q1_expr, 0.0,
                                opt_.q_col_cost_model);
      graph.addExpressionFactor(v0_expr + dt * a0_expr - v1_expr, 0.0,
                                opt_.v_col_cost_model);
      break;
    case CollocationScheme::Trapezoidal:
      graph.addExpressionFactor(q0_expr + 0.5 * dt * v0_expr +
                                    0.5 * dt * v1_expr - q1_expr,
                                0.0, opt_.q_col_cost_model);
      graph.addExpressionFactor(v0_expr + 0.5 * dt * a0_expr +
                                    0.5 * dt * a1_expr - v1_expr,
                                0.0, opt_.v_col_cost_model);
      break;
    default:
      throw std::runtime_error(
          "runge-kutta and hermite-simpson not implemented yet");
      break;
    }
  }
  NonlinearFactorGraph nonlinear_graph;
  nonlinear_graph.add(graph);
  return nonlinear_graph;
}

// the * operator for doubles in expression factor does not work well yet
double multDouble(const double &d1, const double &d2,
                  gtsam::OptionalJacobian<1, 1> H1,
                  gtsam::OptionalJacobian<1, 1> H2) {
  if (H1)
    *H1 = gtsam::I_1x1 * d2;
  if (H2)
    *H2 = gtsam::I_1x1 * d1;
  return d1 * d2;
}

gtsam::NonlinearFactorGraph DynamicsGraph::multiPhaseCollocationFactors(
    const Robot &robot, const int t, const int phase,
    const CollocationScheme collocation) const {
  gtsam::ExpressionFactorGraph graph;
  Double_ phase_expr(PhaseKey(phase));
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    Double_ q0_expr(internal::JointAngleKey(j, t));
    Double_ q1_expr(internal::JointAngleKey(j, t + 1));
    Double_ v0_expr(internal::JointVelKey(j, t));
    Double_ v1_expr(internal::JointVelKey(j, t + 1));
    Double_ a0_expr(internal::JointAccelKey(j, t));
    Double_ a1_expr(internal::JointAccelKey(j, t + 1));

    if (collocation == CollocationScheme::Euler) {
      Double_ v0dt(multDouble, phase_expr, v0_expr);
      Double_ a0dt(multDouble, phase_expr, a0_expr);
      graph.addExpressionFactor(q0_expr + v0dt - q1_expr, 0.0,
                                opt_.q_col_cost_model);
      graph.addExpressionFactor(v0_expr + a0dt - v1_expr, 0.0,
                                opt_.v_col_cost_model);
    } else if (collocation == CollocationScheme::Trapezoidal) {
      Double_ v0dt(multDouble, phase_expr, v0_expr);
      Double_ a0dt(multDouble, phase_expr, a0_expr);
      Double_ v1dt(multDouble, phase_expr, v1_expr);
      Double_ a1dt(multDouble, phase_expr, a1_expr);
      graph.addExpressionFactor(q0_expr + 0.5 * v0dt + 0.5 * v1dt - q1_expr,
                                0.0, opt_.q_col_cost_model);
      graph.addExpressionFactor(v0_expr + 0.5 * a0dt + 0.5 * a1dt - v1_expr,
                                0.0, opt_.v_col_cost_model);
    } else {
      throw std::runtime_error(
          "runge-kutta and hermite-simpson not implemented yet");
    }
  }
  NonlinearFactorGraph nonlinear_graph;
  nonlinear_graph.add(graph);
  return nonlinear_graph;
}

gtsam::NonlinearFactorGraph
DynamicsGraph::forwardDynamicsPriors(const Robot &robot, const int t,
                                     const gtsam::Values &known_values) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.addPrior(internal::JointAngleKey(j, t),
                   JointAngle(known_values, j, t), opt_.prior_q_cost_model);
    graph.addPrior(internal::JointVelKey(j, t), JointVel(known_values, j, t),
                   opt_.prior_qv_cost_model);
    graph.addPrior(internal::TorqueKey(j, t), Torque(known_values, j, t),
                   opt_.prior_t_cost_model);
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::forwardDynamicsPriors(
    const Robot &robot, const int t, const gtsam::Vector &joint_angles,
    const gtsam::Vector &joint_vels, const gtsam::Vector &torques) const {
  gtsam::NonlinearFactorGraph graph;
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints.at(idx);
    int j = joint->id();
    graph.addPrior(internal::JointAngleKey(j, t), joint_angles(idx),
                   opt_.prior_q_cost_model);
    graph.addPrior(internal::JointVelKey(j, t), joint_vels(idx),
                   opt_.prior_qv_cost_model);
    graph.addPrior(internal::TorqueKey(j, t), torques(idx),
                   opt_.prior_t_cost_model);
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::inverseDynamicsPriors(
    const Robot &robot, const int t, const gtsam::Vector &joint_angles,
    const gtsam::Vector &joint_vels, const gtsam::Vector &joint_accels) const {
  gtsam::NonlinearFactorGraph graph;
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->id();
    graph.addPrior(internal::JointAngleKey(j, t), joint_angles[idx],
                   opt_.prior_q_cost_model);
    graph.addPrior(internal::JointVelKey(j, t), joint_vels[idx],
                   opt_.prior_qv_cost_model);
    graph.addPrior(internal::JointAccelKey(j, t), joint_accels[idx],
                   opt_.prior_qa_cost_model);
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::trajectoryFDPriors(
    const Robot &robot, const int num_steps, const gtsam::Vector &joint_angles,
    const gtsam::Vector &joint_vels,
    const std::vector<gtsam::Vector> &torques_seq) const {
  gtsam::NonlinearFactorGraph graph;
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    int j = joints[idx]->id();
    graph.addPrior(internal::JointAngleKey(j, 0), joint_angles[idx],
                   opt_.prior_q_cost_model);
    graph.addPrior(internal::JointVelKey(j, 0), joint_vels[idx],
                   opt_.prior_qv_cost_model);
  }
  for (int t = 0; t <= num_steps; t++) {
    for (int idx = 0; idx < robot.numJoints(); idx++) {
      int j = joints[idx]->id();
      graph.addPrior(internal::TorqueKey(j, t), torques_seq[t][idx],
                     opt_.prior_t_cost_model);
    }
  }

  return graph;
}

gtsam::NonlinearFactorGraph
DynamicsGraph::jointLimitFactors(const Robot &robot, const int t) const {
  NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints())
    graph.add(joint->jointLimitFactors(t, opt_));
  return graph;
}

gtsam::NonlinearFactorGraph
DynamicsGraph::targetAngleFactors(const Robot &robot, const int t,
                                  const std::string &joint_name,
                                  const double target_angle) const {
  NonlinearFactorGraph graph;
  int j = robot.joint(joint_name)->id();
  graph.addPrior(internal::JointAngleKey(j, t), target_angle,
                 opt_.prior_q_cost_model);
  return graph;
}

gtsam::NonlinearFactorGraph
DynamicsGraph::targetPoseFactors(const Robot &robot, const int t,
                                 const std::string &link_name,
                                 const gtsam::Pose3 &target_pose) const {
  NonlinearFactorGraph graph;
  int i = robot.link(link_name)->id();
  graph.addPrior(internal::PoseKey(i, t), target_pose, opt_.bp_cost_model);
  return graph;
}

gtsam::Vector DynamicsGraph::jointAccels(const Robot &robot,
                                         const gtsam::Values &result,
                                         const int t) {
  gtsam::Vector joint_accels = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->id();
    joint_accels[idx] = JointAccel(result, j, t);
  }
  return joint_accels;
}

gtsam::Vector DynamicsGraph::jointVels(const Robot &robot,
                                       const gtsam::Values &result,
                                       const int t) {
  gtsam::Vector joint_vels = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->id();
    joint_vels[idx] = JointVel(result, j, t);
  }
  return joint_vels;
}

gtsam::Vector DynamicsGraph::jointAngles(const Robot &robot,
                                         const gtsam::Values &result,
                                         const int t) {
  gtsam::Vector joint_angles = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->id();
    joint_angles[idx] = JointAngle(result, j, t);
  }
  return joint_angles;
}

gtsam::Vector DynamicsGraph::jointTorques(const Robot &robot,
                                          const gtsam::Values &result,
                                          const int t) {
  gtsam::Vector joint_torques = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->id();
    joint_torques[idx] = Torque(result, j, t);
  }
  return joint_torques;
}

JointValueMap DynamicsGraph::jointAccelsMap(const Robot &robot,
                                          const gtsam::Values &result,
                                          const int t) {
  JointValueMap joint_accels;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    std::string name = joint->name();
    joint_accels[name] = JointAccel(result, j, t);
  }
  return joint_accels;
}

JointValueMap DynamicsGraph::jointVelsMap(const Robot &robot,
                                        const gtsam::Values &result,
                                        const int t) {
  JointValueMap joint_vels;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    std::string name = joint->name();
    joint_vels[name] = JointVel(result, j, t);
  }
  return joint_vels;
}

JointValueMap DynamicsGraph::jointAnglesMap(const Robot &robot,
                                          const gtsam::Values &result,
                                          const int t) {
  JointValueMap joint_angles;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    std::string name = joint->name();
    joint_angles[name] = JointAngle(result, j, t);
  }
  return joint_angles;
}

JointValueMap DynamicsGraph::jointTorquesMap(const Robot &robot,
                                           const gtsam::Values &result,
                                           const int t) {
  JointValueMap joint_torques;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    std::string name = joint->name();
    joint_torques[name] = Torque(result, j, t);
  }
  return joint_torques;
}

void printKey(const gtsam::Key &key) {
  auto symb = DynamicsSymbol(key);
  std::cout << (std::string)(symb) << "\t";
}

// print the factors of the factor graph
void DynamicsGraph::printValues(const gtsam::Values &values) {
  std::cout << "values:\n";
  for (auto &key : values.keys()) {
    printKey(key);
    std::cout << "\n";
    values.at(key).print();
    std::cout << "\n";
  }
}

// print the factors of the factor graph
void DynamicsGraph::printGraph(const gtsam::NonlinearFactorGraph &graph) {
  std::cout << "graph:\n";
  for (auto &factor : graph) {
    for (auto &key : factor->keys()) {
      printKey(key);
    }
    std::cout << "\n";
  }
}

// using radial location to locate the variables
gtsam::Vector3 radial_location(double r, double i, int n) {
  double theta = M_PI * 2 / n * i;
  double x = r * cos(theta);
  double y = r * sin(theta);
  return (gtsam::Vector(3) << x, y, 0).finished();
}

// using radial location to locate the variables
gtsam::Vector3 corner_location(double r, double j, int n) {
  double theta = M_PI * 2 / n * (j + 0.5);
  double x = r * cos(theta);
  double y = r * sin(theta);
  return (gtsam::Vector(3) << x, y, 0).finished();
}

JsonSaver::LocationType get_locations(const Robot &robot, const int t,
                                      bool radial) {
  JsonSaver::LocationType locations;

  if (radial) {
    int n = robot.numLinks();
    for (auto &&link : robot.links()) {
      int i = link->id();
      locations[internal::PoseKey(i, t)] = radial_location(2, i, n);
      locations[internal::TwistKey(i, t)] = radial_location(3, i, n);
      locations[internal::TwistAccelKey(i, t)] = radial_location(4, i, n);
    }

    for (auto &&joint : robot.joints()) {
      int j = joint->id(); 
      locations[internal::JointAngleKey(j, t)] = corner_location(2.5, j, n);
      locations[internal::JointVelKey(j, t)] = corner_location(3.5, j, n);
      locations[internal::JointAccelKey(j, t)] = corner_location(4.5, j, n);
      locations[internal::TorqueKey(j, t)] = corner_location(6, j, n);
      int i1 = joint->parent()->id();
      int i2 = joint->child()->id();
      locations[internal::WrenchKey(i1, j, t)] = corner_location(5.5, j - 0.25, n);
      locations[internal::WrenchKey(i2, j, t)] = corner_location(5.5, j + 0.25, n);
    }
  } else {
    for (auto &&link : robot.links()) {
      int i = link->id();
      locations[internal::PoseKey(i, t)] = gtsam::Vector3(i, 0, 0);
      locations[internal::TwistKey(i, t)] = gtsam::Vector3(i, 1, 0);
      locations[internal::TwistAccelKey(i, t)] = gtsam::Vector3(i, 2, 0);
    }

    for (auto &&joint : robot.joints()) {
      int j = joint->id();
      locations[internal::JointAngleKey(j, t)] =
          (gtsam::Vector(3) << j + 0.5, 0.5, 0).finished();
      locations[internal::JointVelKey(j, t)] =
          (gtsam::Vector(3) << j + 0.5, 1.5, 0).finished();
      locations[internal::JointAccelKey(j, t)] =
          (gtsam::Vector(3) << j + 0.5, 2.5, 0).finished();
      int i1 = joint->parent()->id();
      int i2 = joint->child()->id();
      locations[internal::WrenchKey(i1, j, t)] =
          (gtsam::Vector(3) << j + 0.25, 3.5, 0).finished();
      locations[internal::WrenchKey(i2, j, t)] =
          (gtsam::Vector(3) << j + 0.75, 3.5, 0).finished();
      locations[internal::TorqueKey(j, t)] =
          (gtsam::Vector(3) << j + 0.5, 4.5, 0).finished();
    }
  }
  return locations;
}

void DynamicsGraph::saveGraph(const std::string &file_path,
                              const gtsam::NonlinearFactorGraph &graph,
                              const gtsam::Values &values, const Robot &robot,
                              const int t, bool radial) {
  std::ofstream json_file;
  json_file.open(file_path);
  JsonSaver::LocationType locations = get_locations(robot, t, radial);
  JsonSaver::SaveFactorGraph(graph, json_file, values, locations);
  json_file.close();
}

void DynamicsGraph::saveGraphMultiSteps(
    const std::string &file_path, const gtsam::NonlinearFactorGraph &graph,
    const gtsam::Values &values, const Robot &robot, const int num_steps,
    bool radial) {
  std::ofstream json_file;
  json_file.open(file_path);
  JsonSaver::LocationType locations;

  for (int t = 0; t <= num_steps; t++) {
    JsonSaver::LocationType locations_t = get_locations(robot, t, radial);
    gtsam::Vector offset = (gtsam::Vector(3) << 20.0 * t, 0, 0).finished();
    for (auto it = locations_t.begin(); it != locations_t.end(); it++) {
      auto key = it->first;
      locations_t[key] = locations_t[key] + offset;
    }
    locations.insert(locations_t.begin(), locations_t.end());
  }

  JsonSaver::SaveFactorGraph(graph, json_file, values, locations);
  json_file.close();
}

/* classify the variables into different clusters */
typedef std::pair<std::string, int> ClusterInfo;

inline ClusterInfo getCluster(const gtsam::Key &key) {
  const DynamicsSymbol symb(key);
  const std::string label = symb.label();
  const int t = symb.time();
  if (label == "q" || label == "p") {
    return ClusterInfo("q", t);
  }
  if (label == "v" || label == "V") {
    return ClusterInfo("v", t);
  }
  if (label == "a" || label == "A") {
    return ClusterInfo("a", t);
  }
  if (label == "T" || label == "F") {
    return ClusterInfo("f", t);
  }
  if (label == "ti" || label == "Pi") {
    return ClusterInfo("s", t);
  }
  return ClusterInfo("o", t);
}

void DynamicsGraph::saveGraphTraj(const std::string &file_path,
                                  const gtsam::NonlinearFactorGraph &graph,
                                  const gtsam::Values &values,
                                  const int num_steps) {
  std::map<std::string, gtsam::Values> clustered_values;
  std::map<std::string, gtsam::NonlinearFactorGraph> clustered_graphs;

  // cluster the values
  for (const auto &key : values.keys()) {
    ClusterInfo cluster_info = getCluster(key);
    int t = cluster_info.second;
    std::string category = cluster_info.first;
    std::string cluster_name = category + std::to_string(t);
    if (category == "s") {
      cluster_name = "control";
    }
    if (clustered_values.find(cluster_name) == clustered_values.end()) {
      clustered_values[cluster_name] = gtsam::Values();
    }
    clustered_values[cluster_name].insert(key, values.at(key));
  }

  // cluster the factors
  for (const auto &factor : graph) {
    std::set<int> time_steps;
    std::set<std::string> v_categories;
    for (gtsam::Key key : factor->keys()) {
      ClusterInfo cluster_info = getCluster(key);
      v_categories.insert(cluster_info.first);
      if (cluster_info.first != "s") {
        time_steps.insert(cluster_info.second);
      }
    }
    std::string cluster_name;
    int t = *std::min_element(time_steps.begin(), time_steps.end());

    if (time_steps.size() == 2) {  // collocation factors
      if (v_categories.find("q") != v_categories.end()) {
        cluster_name = "cQ" + std::to_string(t) + "_" + std::to_string(t + 1);
      } else {
        cluster_name = "cV" + std::to_string(t) + "_" + std::to_string(t + 1);
      }
    } else if (time_steps.size() == 1) {
      if (v_categories.find("o") != v_categories.end()) {
        cluster_name = "O" + std::to_string(t);
      } else if (v_categories.find("f") != v_categories.end()) {
        cluster_name = "F" + std::to_string(t);
      } else if (v_categories.find("a") != v_categories.end()) {
        cluster_name = "A" + std::to_string(t);
      } else if (v_categories.find("v") != v_categories.end()) {
        cluster_name = "V" + std::to_string(t);
      } else if (v_categories.find("q") != v_categories.end()) {
        cluster_name = "Q" + std::to_string(t);
      } else {
        cluster_name = "S";
      }
    } else {
      cluster_name = "S";
    }
    if (clustered_graphs.find(cluster_name) == clustered_graphs.end()) {
      clustered_graphs[cluster_name] = gtsam::NonlinearFactorGraph();
    }
    clustered_graphs[cluster_name].add(factor);
  }

  // specify locations
  JsonSaver::StrLocationType locations;
  for (int t = 0; t <= num_steps; t++) {
    locations["q" + std::to_string(t)] =
        (gtsam::Vector(3) << t, 0, 0).finished();
    locations["v" + std::to_string(t)] =
        (gtsam::Vector(3) << t, 1, 0).finished();
    locations["a" + std::to_string(t)] =
        (gtsam::Vector(3) << t, 2, 0).finished();
    locations["f" + std::to_string(t)] =
        (gtsam::Vector(3) << t, 3, 0).finished();
    locations["o" + std::to_string(t)] =
        (gtsam::Vector(3) << t, 4, 0).finished();
    locations['V' + std::to_string(t)] =
        (gtsam::Vector(3) << t, 0.5, 0).finished();
    locations['A' + std::to_string(t)] =
        (gtsam::Vector(3) << t, 1.5, 0).finished();
    locations['F' + std::to_string(t)] =
        (gtsam::Vector(3) << t, 2.5, 0).finished();
  }

  // save to file
  std::ofstream json_file;
  json_file.open(file_path);
  JsonSaver::SaveClusteredGraph(json_file, clustered_graphs, clustered_values,
                                values, locations);
  json_file.close();
}

}  // namespace gtdynamics
