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

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/utils/Slice.h>
#include <gtdynamics/utils/JsonSaver.h>
#include <gtdynamics/utils/utils.h>
#include <gtdynamics/utils/values.h>
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

using gtsam::Double_;
using gtsam::ExpressionFactor;
using gtsam::GaussianFactorGraph;
using gtsam::I_1x1;
using gtsam::I_6x6;
using gtsam::Key;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector6;
using gtsam::Z_6x1;

namespace gtdynamics {

GaussianFactorGraph DynamicsGraph::linearDynamicsGraph(
    const Robot &robot, const int k, const gtsam::Values &known_values) const {
  GaussianFactorGraph graph;
  auto all_constrained = gtsam::noiseModel::Constrained::All(6);
  for (auto &&link : robot.links()) {
    int i = link->id();
    if (link->isFixed()) {
      // prior on twist acceleration for fixed link
      // A_i = 0
      graph.add(TwistAccelKey(i, k), I_6x6, Z_6x1, all_constrained);
    } else {
      // wrench factor
      // G_i * A_i - F_i_j1 - .. - F_i_jn  = ad(V_i)^T * G_i * V*i + m_i * R_i^T
      // * g
      const auto &connected_joints = link->joints();
      const gtsam::Matrix6 G_i = link->inertiaMatrix();
      const double m_i = link->mass();
      const Pose3 T_wi = Pose(known_values, i, k);
      const Vector6 V_i = Twist(known_values, i, k);
      Vector6 rhs = Pose3::adjointMap(V_i).transpose() * G_i * V_i;

      // Compute gravitational forces. If gravity=(0, 0, 0), this will be zeros.
      Vector gravitational_force = T_wi.rotation().transpose() * gravity_ * m_i;
      for (int i = 3; i < 6; ++i) {
        rhs[i] += gravitational_force[i - 3];
      }

      auto accel_key = TwistAccelKey(i, k);
      if (connected_joints.size() == 0) {
        graph.add(accel_key, G_i, rhs, all_constrained);
      } else if (connected_joints.size() == 1) {
        graph.add(accel_key, G_i, WrenchKey(i, connected_joints[0]->id(), k),
                  -I_6x6, rhs, all_constrained);
      } else if (connected_joints.size() == 2) {
        graph.add(accel_key, G_i, WrenchKey(i, connected_joints[0]->id(), k),
                  -I_6x6, WrenchKey(i, connected_joints[1]->id(), k), -I_6x6,
                  rhs, all_constrained);
      }
    }
  }

  OptimizerSetting opt_;
  for (auto &&joint : robot.joints()) {
    graph.push_back(joint->linearAFactors(k, known_values, opt_, planar_axis_));
    graph.push_back(
        joint->linearDynamicsFactors(k, known_values, opt_, planar_axis_));
  }

  return graph;
}

GaussianFactorGraph DynamicsGraph::linearFDPriors(
    const Robot &robot, const int k, const gtsam::Values &torques) {
  OptimizerSetting opt_ = OptimizerSetting();
  GaussianFactorGraph graph;
  for (auto &&joint : robot.joints())
    graph.push_back(joint->linearFDPriors(k, torques, opt_));
  return graph;
}

GaussianFactorGraph DynamicsGraph::linearIDPriors(
    const Robot &robot, const int k, const gtsam::Values &joint_accels) {
  GaussianFactorGraph graph;
  auto all_constrained = gtsam::noiseModel::Constrained::All(1);
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    double accel = JointAccel(joint_accels, j, k);
    gtsam::Vector1 rhs(accel);
    graph.add(JointAccelKey(j, k), I_1x1, rhs, all_constrained);
  }
  return graph;
}

Values DynamicsGraph::linearSolveFD(const Robot &robot, const int k,
                                    const gtsam::Values &known_values) const {
  // construct and solve linear graph
  GaussianFactorGraph graph = linearDynamicsGraph(robot, k, known_values);
  GaussianFactorGraph priors = linearFDPriors(robot, k, known_values);
  graph.push_back(priors);
  gtsam::VectorValues results = graph.optimize();

  // arrange values
  Values values = known_values;
  try {
    // Copy accelerations and wrenches to result.
    for (auto &&joint : robot.joints()) {
      int j = joint->id();
      int i1 = joint->parent()->id();
      int i2 = joint->child()->id();
      InsertJointAccel(&values, j, k, JointAccel(results, j, k)[0]);
      InsertWrench(&values, i1, j, k, Wrench(results, i1, j, k));
      InsertWrench(&values, i2, j, k, Wrench(results, i2, j, k));
    }
    for (auto &&link : robot.links()) {
      int i = link->id();
      InsertTwistAccel(&values, i, k, TwistAccel(results, i, k));
    }
  } catch (const gtsam::ValuesKeyAlreadyExists &e) {
    std::cerr << "key already exists:" << _GTDKeyFormatter(e.key()) << '\n';
    throw std::invalid_argument(
        "linearSolveFD: known_values should contain no accelerations or "
        "wrenches");
  }
  return values;
}

Values DynamicsGraph::linearSolveID(const Robot &robot, const int k,
                                    const gtsam::Values &known_values) {
  // construct and solve linear graph
  GaussianFactorGraph graph = linearDynamicsGraph(robot, k, known_values);
  GaussianFactorGraph priors = linearIDPriors(robot, k, known_values);
  graph.push_back(priors);

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
      InsertTorque(&values, j, k, Torque(results, j, k)[0]);
      InsertWrench(&values, i1, j, k, Wrench(results, i1, j, k));
      InsertWrench(&values, i2, j, k, Wrench(results, i2, j, k));
    }
    for (auto &&link : robot.links()) {
      int i = link->id();
      std::string name = link->name();
      InsertTwistAccel(&values, i, k, TwistAccel(results, i, k));
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
    const std::optional<PointOnLinks> &contact_points) const {
  const Slice slice(k);
  return kinematics_.qFactors(slice, robot, contact_points, gravity_);
}

gtsam::NonlinearFactorGraph DynamicsGraph::vFactors(
    const Robot &robot, const int k,
    const std::optional<PointOnLinks> &contact_points) const {
  const Slice slice(k);
  return kinematics_.vFactors(slice, robot, contact_points);
}

gtsam::NonlinearFactorGraph DynamicsGraph::aFactors(
    const Robot &robot, const int k,
    const std::optional<PointOnLinks> &contact_points) const {
  return dynamics_.aFactors(Slice(k), robot, contact_points);
}

// TODO(frank): migrate to Dynamics::graph<Slice>
gtsam::NonlinearFactorGraph DynamicsGraph::dynamicsFactors(
    const Robot &robot, const int k,
    const std::optional<PointOnLinks> &contact_points,
    const std::optional<double> &mu) const {
  NonlinearFactorGraph graph;

  double mu_;  // Static friction coefficient.
  if (mu)
    mu_ = *mu;
  else
    mu_ = 1.0;

  for (auto &&link : robot.links()) {
    int i = link->id();
    if (!link->isFixed()) {
      const auto &connected_joints = link->joints();
      std::vector<gtsam::Key> wrench_keys;

      // Add wrench keys for joints.
      for (auto &&joint : connected_joints)
        wrench_keys.push_back(WrenchKey(i, joint->id(), k));

      // Add wrench keys for contact points.
      if (contact_points) {
        for (auto &&cp : *contact_points) {
          if (cp.link->id() != i) continue;
          // TODO(frank): allow multiple contact points on one link, id = 0,1,..
          auto wrench_key = ContactWrenchKey(i, 0, k);
          wrench_keys.push_back(wrench_key);

          // Add contact dynamics constraints.
          graph.emplace_shared<ContactDynamicsFrictionConeFactor>(
              PoseKey(i, k), wrench_key, opt_.cfriction_cost_model, mu_,
              gravity_);

          graph.emplace_shared<ContactDynamicsMomentFactor>(
              wrench_key, opt_.cm_cost_model,
              gtsam::Pose3(gtsam::Rot3(), -cp.point));
        }
      }

      // add wrench factor for link
      graph.add(
          WrenchFactor(opt_.fa_cost_model, link, wrench_keys, k, gravity_));
    }
  }

  // TODO(frank): use Statics<Slice> calls
  // TODO(frank): sort out const shared ptr mess
  for (auto &&joint : robot.joints()) {
    auto j = joint->id(), child_id = joint->child()->id();
    auto const_joint = joint;
    graph.add(WrenchEquivalenceFactor(opt_.f_cost_model, const_joint, k));
    graph.add(TorqueFactor(opt_.t_cost_model, const_joint, k));
    if (planar_axis_) {
      graph.add(WrenchPlanarFactor(opt_.planar_cost_model, *planar_axis_,
                                   const_joint, k));
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::dynamicsFactorGraph(
    const Robot &robot, const int k,
    const std::optional<PointOnLinks> &contact_points,
    const std::optional<double> &mu) const {
  NonlinearFactorGraph graph;
  graph.add(qFactors(robot, k, contact_points));
  graph.add(vFactors(robot, k, contact_points));
  graph.add(aFactors(robot, k, contact_points));
  graph.add(dynamicsFactors(robot, k, contact_points, mu));
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::trajectoryFG(
    const Robot &robot, const int num_steps, const double dt,
    const CollocationScheme collocation,
    const std::optional<PointOnLinks> &contact_points,
    const std::optional<double> &mu) const {
  NonlinearFactorGraph graph;
  for (int k = 0; k <= num_steps; k++) {
    graph.add(dynamicsFactorGraph(robot, k, contact_points, mu));
    if (k < num_steps) {
      graph.add(collocationFactors(robot, k, dt, collocation));
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::multiPhaseTrajectoryFG(
    const Robot &robot, const std::vector<int> &phase_steps,
    const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs,
    const CollocationScheme collocation,
    const std::optional<std::vector<PointOnLinks>> &phase_contact_points,
    const std::optional<double> &mu) const {
  NonlinearFactorGraph graph;
  int num_phases = phase_steps.size();

  // Return either PointOnLinks or None if none specified for phase p
  auto contact_points =
      [&phase_contact_points](int p) -> std::optional<PointOnLinks> {
    if (phase_contact_points) return (*phase_contact_points)[p];
    return {};
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
    NonlinearFactorGraph *graph, const Key x0_key, const Key x1_key,
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
  if (H1) *H1 = gtsam::I_1x1 * d2;
  if (H2) *H2 = gtsam::I_1x1 * d1;
  return d1 * d2;
}

std::shared_ptr<gtsam::ExpressionFactor<double>>
DynamicsGraph::collocationFactorDouble(
    const gtsam::Key x0_key, const gtsam::Key x1_key, const gtsam::Key v0_key,
    const gtsam::Key v1_key, const gtsam::Key phase_key,
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const CollocationScheme collocation) {
  Double_ phase_expr(phase_key);
  Double_ x0_expr(x0_key);
  Double_ x1_expr(x1_key);
  Double_ v0_expr(v0_key);
  Double_ v1_expr(v1_key);
  Double_ v0dt(multDouble, phase_expr, v0_expr);

  if (collocation == CollocationScheme::Euler) {
    return std::make_shared<ExpressionFactor<double>>(cost_model, 0.0,
                                                      x0_expr + v0dt - x1_expr);
  } else if (collocation == CollocationScheme::Trapezoidal) {
    Double_ v1dt(multDouble, phase_expr, v1_expr);
    return std::make_shared<ExpressionFactor<double>>(
        cost_model, 0.0, x0_expr + 0.5 * v0dt + 0.5 * v1dt - x1_expr);
  } else {
    throw std::runtime_error(
        "runge-kutta and hermite-simpson not implemented yet");
  }
}

std::shared_ptr<gtsam::ExpressionFactor<double>>
DynamicsGraph::multiPhaseJointCollocationQFactor(const size_t j, const size_t k,
                        const gtsam::Key phase_key,
                        const gtsam::noiseModel::Base::shared_ptr &cost_model,
                        const CollocationScheme collocation) {
  return collocationFactorDouble(JointAngleKey(j, k), JointAngleKey(j, k + 1),
                                 JointVelKey(j, k), JointVelKey(j, k + 1),
                                 phase_key, cost_model, collocation);
}

std::shared_ptr<gtsam::ExpressionFactor<double>>
DynamicsGraph::multiPhaseJointCollocationVFactor(
    const size_t j, const size_t k, const gtsam::Key phase_key,
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const CollocationScheme collocation) {
  return collocationFactorDouble(JointVelKey(j, k), JointVelKey(j, k + 1),
                                 JointAccelKey(j, k), JointAccelKey(j, k + 1),
                                 phase_key, cost_model, collocation);
}

void DynamicsGraph::addMultiPhaseCollocationFactorDouble(
    NonlinearFactorGraph *graph, const Key x0_key, const Key x1_key,
    const Key v0_key, const Key v1_key, const Key phase_key,
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const CollocationScheme collocation) {
  graph->add(collocationFactorDouble(x0_key, x1_key, v0_key, v1_key, phase_key,
                                     cost_model, collocation));
}

gtsam::NonlinearFactorGraph DynamicsGraph::jointCollocationFactors(
    const int j, const int k, const double dt,
    const CollocationScheme collocation) const {
  NonlinearFactorGraph graph;
  Key q0_key = JointAngleKey(j, k), q1_key = JointAngleKey(j, k + 1),
      v0_key = JointVelKey(j, k), v1_key = JointVelKey(j, k + 1),
      a0_key = JointAccelKey(j, k), a1_key = JointAccelKey(j, k + 1);
  addCollocationFactorDouble(&graph, q0_key, q1_key, v0_key, v1_key, dt,
                             opt_.q_col_cost_model, collocation);
  addCollocationFactorDouble(&graph, v0_key, v1_key, a0_key, a1_key, dt,
                             opt_.v_col_cost_model, collocation);
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::collocationFactors(
    const Robot &robot, const int k, const double dt,
    const CollocationScheme collocation) const {
  NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.add(jointCollocationFactors(j, k, dt, collocation));
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::jointMultiPhaseCollocationFactors(
    const int j, const int k, const int phase,
    const CollocationScheme collocation) const {
  Key phase_key = PhaseKey(phase), q0_key = JointAngleKey(j, k),
      q1_key = JointAngleKey(j, k + 1), v0_key = JointVelKey(j, k),
      v1_key = JointVelKey(j, k + 1), a0_key = JointAccelKey(j, k),
      a1_key = JointAccelKey(j, k + 1);

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
    const Robot &robot, const int k, const int phase,
    const CollocationScheme collocation) const {
  NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.add(jointMultiPhaseCollocationFactors(j, k, phase, collocation));
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::forwardDynamicsPriors(
    const Robot &robot, const int k, const gtsam::Values &known_values) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.addPrior(JointAngleKey(j, k), JointAngle(known_values, j, k),
                   opt_.prior_q_cost_model);
    graph.addPrior(JointVelKey(j, k), JointVel(known_values, j, k),
                   opt_.prior_qv_cost_model);
    graph.addPrior(TorqueKey(j, k), Torque(known_values, j, k),
                   opt_.prior_t_cost_model);
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::inverseDynamicsPriors(
    const Robot &robot, const int k, const gtsam::Values &known_values) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.addPrior(JointAngleKey(j, k), JointAngle(known_values, j, k),
                   opt_.prior_q_cost_model);
    graph.addPrior(JointVelKey(j, k), JointVel(known_values, j, k),
                   opt_.prior_qv_cost_model);
    graph.addPrior(JointAccelKey(j, k), JointAccel(known_values, j, k),
                   opt_.prior_qa_cost_model);
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::trajectoryFDPriors(
    const Robot &robot, const int num_steps,
    const gtsam::Values &known_values) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    graph.addPrior(JointAngleKey(j, 0), JointAngle(known_values, j, 0),
                   opt_.prior_q_cost_model);
    graph.addPrior(JointVelKey(j, 0), JointVel(known_values, j, 0),
                   opt_.prior_qv_cost_model);
    for (int k = 0; k <= num_steps; k++) {
      graph.addPrior(TorqueKey(j, k), Torque(known_values, j, k),
                     opt_.prior_t_cost_model);
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::jointLimitFactors(
    const Robot &robot, const int k) const {
  NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints())
    graph.add(joint->jointLimitFactors(k, opt_));
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::targetAngleFactors(
    const Robot &robot, const int k, const std::string &joint_name,
    const double target_angle) const {
  NonlinearFactorGraph graph;
  int j = robot.joint(joint_name)->id();
  graph.addPrior(JointAngleKey(j, k), target_angle, opt_.prior_q_cost_model);
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraph::targetPoseFactors(
    const Robot &robot, const int k, const std::string &link_name,
    const gtsam::Pose3 &target_pose) const {
  NonlinearFactorGraph graph;
  int i = robot.link(link_name)->id();
  graph.addPrior(PoseKey(i, k), target_pose, opt_.bp_cost_model);
  return graph;
}

gtsam::Vector DynamicsGraph::jointAccels(const Robot &robot,
                                         const gtsam::Values &result,
                                         const int k) {
  gtsam::Vector joint_accels = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->id();
    joint_accels[idx] = JointAccel(result, j, k);
  }
  return joint_accels;
}

gtsam::Vector DynamicsGraph::jointVels(const Robot &robot,
                                       const gtsam::Values &result,
                                       const int k) {
  gtsam::Vector joint_vels = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->id();
    joint_vels[idx] = JointVel(result, j, k);
  }
  return joint_vels;
}

gtsam::Vector DynamicsGraph::jointAngles(const Robot &robot,
                                         const gtsam::Values &result,
                                         const int k) {
  gtsam::Vector joint_angles = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->id();
    joint_angles[idx] = JointAngle(result, j, k);
  }
  return joint_angles;
}

gtsam::Vector DynamicsGraph::jointTorques(const Robot &robot,
                                          const gtsam::Values &result,
                                          const int k) {
  gtsam::Vector joint_torques = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->id();
    joint_torques[idx] = Torque(result, j, k);
  }
  return joint_torques;
}

JointValueMap DynamicsGraph::jointAccelsMap(const Robot &robot,
                                            const gtsam::Values &result,
                                            const int k) {
  JointValueMap joint_accels;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    std::string name = joint->name();
    joint_accels[name] = JointAccel(result, j, k);
  }
  return joint_accels;
}

JointValueMap DynamicsGraph::jointVelsMap(const Robot &robot,
                                          const gtsam::Values &result,
                                          const int k) {
  JointValueMap joint_vels;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    std::string name = joint->name();
    joint_vels[name] = JointVel(result, j, k);
  }
  return joint_vels;
}

JointValueMap DynamicsGraph::jointAnglesMap(const Robot &robot,
                                            const gtsam::Values &result,
                                            const int k) {
  JointValueMap joint_angles;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    std::string name = joint->name();
    joint_angles[name] = JointAngle(result, j, k);
  }
  return joint_angles;
}

JointValueMap DynamicsGraph::jointTorquesMap(const Robot &robot,
                                             const gtsam::Values &result,
                                             const int k) {
  JointValueMap joint_torques;
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    std::string name = joint->name();
    joint_torques[name] = Torque(result, j, k);
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

JsonSaver::LocationType get_locations(const Robot &robot, const int k,
                                      bool radial) {
  JsonSaver::LocationType locations;

  if (radial) {
    int n = robot.numLinks();
    for (auto &&link : robot.links()) {
      int i = link->id();
      locations[PoseKey(i, k)] = radial_location(2, i, n);
      locations[TwistKey(i, k)] = radial_location(3, i, n);
      locations[TwistAccelKey(i, k)] = radial_location(4, i, n);
    }

    for (auto &&joint : robot.joints()) {
      int j = joint->id();
      locations[JointAngleKey(j, k)] = corner_location(2.5, j, n);
      locations[JointVelKey(j, k)] = corner_location(3.5, j, n);
      locations[JointAccelKey(j, k)] = corner_location(4.5, j, n);
      locations[TorqueKey(j, k)] = corner_location(6, j, n);
      int i1 = joint->parent()->id();
      int i2 = joint->child()->id();
      locations[WrenchKey(i1, j, k)] = corner_location(5.5, j - 0.25, n);
      locations[WrenchKey(i2, j, k)] = corner_location(5.5, j + 0.25, n);
    }
  } else {
    for (auto &&link : robot.links()) {
      int i = link->id();
      locations[PoseKey(i, k)] = gtsam::Vector3(i, 0, 0);
      locations[TwistKey(i, k)] = gtsam::Vector3(i, 1, 0);
      locations[TwistAccelKey(i, k)] = gtsam::Vector3(i, 2, 0);
    }

    for (auto &&joint : robot.joints()) {
      int j = joint->id();
      locations[JointAngleKey(j, k)] =
          (gtsam::Vector(3) << j + 0.5, 0.5, 0).finished();
      locations[JointVelKey(j, k)] =
          (gtsam::Vector(3) << j + 0.5, 1.5, 0).finished();
      locations[JointAccelKey(j, k)] =
          (gtsam::Vector(3) << j + 0.5, 2.5, 0).finished();
      int i1 = joint->parent()->id();
      int i2 = joint->child()->id();
      locations[WrenchKey(i1, j, k)] =
          (gtsam::Vector(3) << j + 0.25, 3.5, 0).finished();
      locations[WrenchKey(i2, j, k)] =
          (gtsam::Vector(3) << j + 0.75, 3.5, 0).finished();
      locations[TorqueKey(j, k)] =
          (gtsam::Vector(3) << j + 0.5, 4.5, 0).finished();
    }
  }
  return locations;
}

void DynamicsGraph::saveGraph(const std::string &file_path,
                              const gtsam::NonlinearFactorGraph &graph,
                              const gtsam::Values &values, const Robot &robot,
                              const int k, bool radial) {
  std::ofstream json_file;
  json_file.open(file_path);
  JsonSaver::LocationType locations = get_locations(robot, k, radial);
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

  for (int k = 0; k <= num_steps; k++) {
    JsonSaver::LocationType locations_t = get_locations(robot, k, radial);
    gtsam::Vector offset = (gtsam::Vector(3) << 20.0 * k, 0, 0).finished();
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
  const int k = symb.time();
  if (label == "q" || label == "p") {
    return ClusterInfo("q", k);
  }
  if (label == "v" || label == "V") {
    return ClusterInfo("v", k);
  }
  if (label == "a" || label == "A") {
    return ClusterInfo("a", k);
  }
  if (label == "T" || label == "F") {
    return ClusterInfo("f", k);
  }
  if (label == "ti" || label == "Pi") {
    return ClusterInfo("s", k);
  }
  return ClusterInfo("o", k);
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
    int k = cluster_info.second;
    std::string category = cluster_info.first;
    std::string cluster_name = category + std::to_string(k);
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
    int k = *std::min_element(time_steps.begin(), time_steps.end());

    if (time_steps.size() == 2) {  // collocation factors
      if (v_categories.find("q") != v_categories.end()) {
        cluster_name = "cQ" + std::to_string(k) + "_" + std::to_string(k + 1);
      } else {
        cluster_name = "cV" + std::to_string(k) + "_" + std::to_string(k + 1);
      }
    } else if (time_steps.size() == 1) {
      if (v_categories.find("o") != v_categories.end()) {
        cluster_name = "O" + std::to_string(k);
      } else if (v_categories.find("f") != v_categories.end()) {
        cluster_name = "F" + std::to_string(k);
      } else if (v_categories.find("a") != v_categories.end()) {
        cluster_name = "A" + std::to_string(k);
      } else if (v_categories.find("v") != v_categories.end()) {
        cluster_name = "V" + std::to_string(k);
      } else if (v_categories.find("q") != v_categories.end()) {
        cluster_name = "Q" + std::to_string(k);
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
  for (int k = 0; k <= num_steps; k++) {
    locations["q" + std::to_string(k)] =
        (gtsam::Vector(3) << k, 0, 0).finished();
    locations["v" + std::to_string(k)] =
        (gtsam::Vector(3) << k, 1, 0).finished();
    locations["a" + std::to_string(k)] =
        (gtsam::Vector(3) << k, 2, 0).finished();
    locations["f" + std::to_string(k)] =
        (gtsam::Vector(3) << k, 3, 0).finished();
    locations["o" + std::to_string(k)] =
        (gtsam::Vector(3) << k, 4, 0).finished();
    locations['V' + std::to_string(k)] =
        (gtsam::Vector(3) << k, 0.5, 0).finished();
    locations['A' + std::to_string(k)] =
        (gtsam::Vector(3) << k, 1.5, 0).finished();
    locations['F' + std::to_string(k)] =
        (gtsam::Vector(3) << k, 2.5, 0).finished();
  }

  // save to file
  std::ofstream json_file;
  json_file.open(file_path);
  JsonSaver::SaveClusteredGraph(json_file, clustered_graphs, clustered_values,
                                values, locations);
  json_file.close();
}

}  // namespace gtdynamics
