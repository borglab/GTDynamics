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
#include <gtsam/nonlinear/ExpressionFactor.h>
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
using gtsam::Key;
using gtsam::ExpressionFactor;

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
      const auto &connected_joints = link->joints();
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
    const Robot &robot, const int k,
    const boost::optional<ContactPoints> &contact_points) const {
  NonlinearFactorGraph graph;
  for (auto &&link : robot.links())
    if (link->isFixed())
      graph.addPrior(internal::PoseKey(link->id(), k), link->getFixedPose(),
                     opt_.bp_cost_model);

  // TODO(frank): call Kinematics::graph<Slice> instead
  for (auto &&joint : robot.joints()) {
    graph.emplace_shared<PoseFactor>(
        internal::PoseKey(joint->parent()->id(), k),
        internal::PoseKey(joint->child()->id(), k),
        internal::JointAngleKey(joint->id(), k), opt_.p_cost_model, joint);
  }

  // TODO(frank): whoever write this should clean up this mess.
  gtsam::Vector3 gravity;
  if (gravity_)
    gravity = *gravity_;
  else
    gravity = gtsam::Vector3(0, 0, -9.8);

  // Add contact factors.
  for (auto &&link : robot.links()) {
    int i = link->id();
    // Check if the link has contact points. If so, add pose constraints.
    if (contact_points) {
      for (auto &&contact_point : *contact_points) {
        if (contact_point.first != link->name()) continue;

        // TODO(frank): #179 make sure height is handled correctly.
        ContactKinematicsPoseFactor contact_pose_factor(
            internal::PoseKey(i, k), opt_.cp_cost_model,
            contact_point.second.point, gravity);
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
  for (auto &&link : robot.links())
    if (link->isFixed())
      graph.addPrior<gtsam::Vector6>(internal::TwistKey(link->id(), t),
                                     gtsam::Z_6x1, opt_.bv_cost_model);

  for (auto &&joint : robot.joints())
    graph.emplace_shared<TwistFactor>(internal::TwistKey(joint->parent()->id(), t),
                                    internal::TwistKey(joint->child()->id(), t),
                                    internal::JointAngleKey(joint->id(), t),
                                    internal::JointVelKey(joint->id(), t),
                                    opt_.v_cost_model, joint);

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
  for (auto &&link : robot.links())
    if (link->isFixed())
      graph.addPrior<gtsam::Vector6>(internal::TwistAccelKey(link->id(), t),
                                     gtsam::Z_6x1, opt_.ba_cost_model);
  for (auto &&joint : robot.joints())
    graph.emplace_shared<TwistAccelFactor>(
      internal::TwistKey(joint->child()->id(), t),
      internal::TwistAccelKey(joint->parent()->id(), t),
      internal::TwistAccelKey(joint->child()->id(), t),
      internal::JointAngleKey(joint->id(), t), internal::JointVelKey(joint->id(), t),
      internal::JointAccelKey(joint->id(), t), opt_.a_cost_model,
      boost::static_pointer_cast<const JointTyped>(joint));

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

// TODO(frank): migrate to Dynamics::graph<Slice>
gtsam::NonlinearFactorGraph DynamicsGraph::dynamicsFactors(
    const Robot &robot, const int k,
    const boost::optional<ContactPoints> &contact_points,
    const boost::optional<double> &mu) const {
  NonlinearFactorGraph graph;

  // TODO(frank): whoever write this should clean up this mess.
  gtsam::Vector3 gravity;
  if (gravity_)
    gravity = *gravity_;
  else
    gravity = gtsam::Vector3(0, 0, -9.8);

  double mu_;  // Static friction coefficient.
  if (mu)
    mu_ = *mu;
  else
    mu_ = 1.0;

  for (auto &&link : robot.links()) {
    int i = link->id();
    if (!link->isFixed()) {
      const auto &connected_joints = link->joints();
      std::vector<DynamicsSymbol> wrench_keys;

      // Add wrench keys for joints.
      for (auto &&joint : connected_joints)
        wrench_keys.push_back(internal::WrenchKey(i, joint->id(), k));

      // Add wrench keys for contact points.
      if (contact_points) {
        for (auto &&contact_point : *contact_points) {
          if (contact_point.first != link->name()) continue;
          auto wrench_key = ContactWrenchKey(i, contact_point.second.id, k);
          wrench_keys.push_back(wrench_key);

          // Add contact dynamics constraints.
          graph.emplace_shared<ContactDynamicsFrictionConeFactor>(
              internal::PoseKey(i, k), wrench_key, opt_.cfriction_cost_model,
              mu_, gravity);

          graph.emplace_shared<ContactDynamicsMomentFactor>(
              wrench_key, opt_.cm_cost_model,
              gtsam::Pose3(gtsam::Rot3(), -contact_point.second.point));
        }
      }

      // add wrench factor for link
      graph.emplace_shared<WrenchFactor>(
          internal::TwistKey(link->id(), k),
          internal::TwistAccelKey(link->id(), k), wrench_keys,
          internal::PoseKey(link->id(), k), opt_.fa_cost_model,
          link->inertiaMatrix(), gravity);
    }
  }

  // TODO(frank): use Statics<Slice> calls
  // TODO(frank): sort out const shared ptr mess
  for (auto &&joint : robot.joints()) {
    auto j = joint->id(), child_id = joint->child()->id();
    auto const_joint = boost::static_pointer_cast<const JointTyped>(joint);
    graph.emplace_shared<WrenchEquivalenceFactor>(opt_.f_cost_model,
                                                  const_joint, k);
    graph.emplace_shared<TorqueFactor>(opt_.t_cost_model, const_joint, k);
    if (planar_axis_)
      graph.emplace_shared<WrenchPlanarFactor>(opt_.planar_cost_model,
                                               *planar_axis_, const_joint, k);
  }
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
    const CollocationScheme collocation,
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
    const Robot &robot, const std::vector<int> &phase_steps,
    const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs,
    const CollocationScheme collocation,
    const boost::optional<std::vector<ContactPoints>> &phase_contact_points,
    const boost::optional<double> &mu) const {
  NonlinearFactorGraph graph;
  int num_phases = phase_steps.size();

  // Return either ContactPoints or None if none specified for phase p
  auto contact_points =
      [&phase_contact_points](int p) -> boost::optional<ContactPoints> {
    if (phase_contact_points) return (*phase_contact_points)[p];
    return boost::none;
  };

  // First slice, k==0
  graph.add(dynamicsFactorGraph(robot, 0, contact_points(0), mu));

  int k = 0;
  for (int p = 0; p < num_phases; p++) {
    // in-phase
    // add dynamics for each step
    for (int step = 0; step < phase_steps[p] - 1; step++) {
      graph.add(dynamicsFactorGraph(robot, ++k, contact_points(p), mu));
    }
    if (p == num_phases - 1) {
      // Last slice, k==K-1
      graph.add(dynamicsFactorGraph(robot, ++k, contact_points(p), mu));
    } else {
      // transition
      graph.add(transition_graphs[p]);
      k++;
    }
  }

  // add collocation factors
  k = 0;
  for (int p = 0; p < num_phases; p++) {
    for (int step = 0; step < phase_steps[p]; step++) {
      graph.add(multiPhaseCollocationFactors(robot, k++, p, collocation));
    }
  }
  return graph;
}

void DynamicsGraph::addCollocationFactorDouble(
    NonlinearFactorGraph* graph, const Key x0_key, const Key x1_key,
    const Key v0_key, const Key v1_key, const double dt,
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const CollocationScheme collocation) {
  Double_ x0_expr(x0_key);
  Double_ x1_expr(x1_key);
  Double_ v0_expr(v0_key);
  Double_ v1_expr(v1_key);
  if (collocation == CollocationScheme::Euler) {
    graph->add(
        ExpressionFactor(cost_model, 0.0, x0_expr + dt * v0_expr - x1_expr));
  } else if (collocation == CollocationScheme::Trapezoidal) {
    graph->add(ExpressionFactor(
        cost_model, 0.0,
        x0_expr + 0.5 * dt * v0_expr + 0.5 * dt * v1_expr - x1_expr));
  } else {
    throw std::runtime_error(
        "runge-kutta and hermite-simpson not implemented yet");
  }
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

void DynamicsGraph::addMultiPhaseCollocationFactorDouble(
    NonlinearFactorGraph* graph, const Key x0_key, const Key x1_key,
    const Key v0_key, const Key v1_key, const Key phase_key,
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const CollocationScheme collocation) {
  Double_ phase_expr(phase_key);
  Double_ x0_expr(x0_key);
  Double_ x1_expr(x1_key);
  Double_ v0_expr(v0_key);
  Double_ v1_expr(v1_key);
  Double_ v0dt(multDouble, phase_expr, v0_expr);

  if (collocation == CollocationScheme::Euler) {
    graph->add(ExpressionFactor(cost_model, 0.0, x0_expr + v0dt - x1_expr));
  } else if (collocation == CollocationScheme::Trapezoidal) {
    Double_ v1dt(multDouble, phase_expr, v1_expr);
    graph->add(ExpressionFactor(cost_model, 0.0,
                               x0_expr + 0.5 * v0dt + 0.5 * v1dt - x1_expr));
  } else {
    throw std::runtime_error(
        "runge-kutta and hermite-simpson not implemented yet");
  }
}

gtsam::NonlinearFactorGraph DynamicsGraph::jointCollocationFactors(
    const int j, const int t, const double dt,
    const CollocationScheme collocation) const {
  NonlinearFactorGraph graph;
  Key q0_key = internal::JointAngleKey(j, t),
      q1_key = internal::JointAngleKey(j, t + 1),
      v0_key = internal::JointVelKey(j, t),
      v1_key = internal::JointVelKey(j, t + 1),
      a0_key = internal::JointAccelKey(j, t),
      a1_key = internal::JointAccelKey(j, t + 1);
  addCollocationFactorDouble(&graph, q0_key, q1_key, v0_key, v1_key, dt,
                             opt_.q_col_cost_model, collocation);
  addCollocationFactorDouble(&graph, v0_key, v1_key, a0_key, a1_key, dt,
                             opt_.v_col_cost_model, collocation);
  return graph;
}

gtsam::NonlinearFactorGraph
DynamicsGraph::collocationFactors(const Robot &robot, const int t,
                                  const double dt,
                                  const CollocationScheme collocation) const {
  NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.add(jointCollocationFactors(j, t, dt, collocation));
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::jointMultiPhaseCollocationFactors(
    const int j, const int t, const int phase,
    const CollocationScheme collocation) const {
  Key phase_key = PhaseKey(phase), q0_key = internal::JointAngleKey(j, t),
      q1_key = internal::JointAngleKey(j, t + 1),
      v0_key = internal::JointVelKey(j, t),
      v1_key = internal::JointVelKey(j, t + 1),
      a0_key = internal::JointAccelKey(j, t),
      a1_key = internal::JointAccelKey(j, t + 1);

  gtsam::NonlinearFactorGraph graph;
  addMultiPhaseCollocationFactorDouble(&graph, q0_key, q1_key, v0_key, v1_key,
                                       phase_key, opt_.q_col_cost_model,
                                       collocation);
  addMultiPhaseCollocationFactorDouble(&graph, v0_key, v1_key, a0_key, a1_key,
                                       phase_key, opt_.v_col_cost_model,
                                       collocation);
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::multiPhaseCollocationFactors(
    const Robot &robot, const int t, const int phase,
    const CollocationScheme collocation) const {
  NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.add(jointMultiPhaseCollocationFactors(j, t, phase, collocation));
  }
  return graph;
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

gtsam::NonlinearFactorGraph DynamicsGraph::inverseDynamicsPriors(
    const Robot &robot, const int t, const gtsam::Values &known_values) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.addPrior(internal::JointAngleKey(j, t),
                   JointAngle(known_values, j, t), opt_.prior_q_cost_model);
    graph.addPrior(internal::JointVelKey(j, t), JointVel(known_values, j, t),
                   opt_.prior_qv_cost_model);
    graph.addPrior(internal::JointAccelKey(j, t),
                   JointAccel(known_values, j, t), opt_.prior_qa_cost_model);
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::trajectoryFDPriors(
    const Robot &robot, const int num_steps,
    const gtsam::Values &known_values) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.addPrior(internal::JointAngleKey(j, 0),
                   JointAngle(known_values, j, 0), opt_.prior_q_cost_model);
    graph.addPrior(internal::JointVelKey(j, 0), JointVel(known_values, j, 0),
                   opt_.prior_qv_cost_model);
    for (int t = 0; t <= num_steps; t++) {
      graph.addPrior(internal::TorqueKey(j, t), Torque(known_values, j, t),
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
