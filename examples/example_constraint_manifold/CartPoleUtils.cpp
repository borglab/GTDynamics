/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CartPoleUtils.cpp
 * @brief Cart pole utilities implementations.
 * @author Yetong Zhang
 */

#include "CartPoleUtils.h"

#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtdynamics {

using gtsam::Key;
using gtsam::KeyVector;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;

/* ************************************************************************* */
NonlinearFactorGraph CartPole::getDynamicsGraph(size_t num_steps) const {
  NonlinearFactorGraph graph;
  // kinodynamic constraints at each step
  for (size_t k = 0; k <= num_steps; k++) {
    graph.add(graph_builder.dynamicsFactorGraph(robot, k));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph CartPole::getUnactuatedGraph(size_t num_steps) const {
  NonlinearFactorGraph graph;
  for (int k = 0; k <= num_steps; k++)
    graph.addPrior(TorqueKey(j1_id, k), 0.0,
                   graph_builder.opt().prior_t_cost_model);
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph CartPole::initStateGraph() const {
  // Initial conditions
  NonlinearFactorGraph graph;
  graph.addPrior(JointAngleKey(j0_id, 0), X_i[0], prior_model);
  graph.addPrior(JointVelKey(j0_id, 0), X_i[1], prior_model);
  graph.addPrior(JointAccelKey(j0_id, 0), X_i[2], prior_model);
  graph.addPrior(JointAngleKey(j1_id, 0), X_i[3], prior_model);
  graph.addPrior(JointVelKey(j1_id, 0), X_i[4], prior_model);
  // graph.addPrior(JointAccelKey(j1_id, 0), X_i[5], prior_model);
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph CartPole::finalStateGraph(size_t num_steps) const {
  NonlinearFactorGraph graph;
  graph.addPrior(JointAngleKey(j0_id, num_steps), X_T[0], pos_objectives_model);
  graph.addPrior(JointAngleKey(j1_id, num_steps), X_T[3], pos_objectives_model);
  graph.addPrior(JointVelKey(j0_id, num_steps), X_T[1], objectives_model);
  graph.addPrior(JointAccelKey(j0_id, num_steps), X_T[2], objectives_model);
  graph.addPrior(JointVelKey(j1_id, num_steps), X_T[4], objectives_model);
  graph.addPrior(JointAccelKey(j1_id, num_steps), X_T[5], objectives_model);
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph CartPole::minTorqueCosts(size_t num_steps) const {
  NonlinearFactorGraph graph;
  for (int k = 0; k <= num_steps; k++)
    graph.emplace_shared<MinTorqueFactor>(TorqueKey(j0_id, k), control_model);
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph CartPole::getCollocation(size_t num_steps, double dt) {
  NonlinearFactorGraph graph;
  for (int k = 0; k < num_steps; k++) {
    graph.add(
        graph_builder.collocationFactors(robot, k, dt, collocation_scheme));
  }
  return graph;
}

/* ************************************************************************* */
Values CartPole::getInitValuesZero(size_t num_steps) {
  Initializer initializer;
  Values values0 = initializer.ZeroValues(robot, 0, 0.0);
  Values known_values;
  for (const auto& joint : robot.joints()) {
    InsertJointAngle(&known_values, joint->id(),
                     JointAngle(values0, joint->id()));
    InsertJointVel(&known_values, joint->id(), JointVel(values0, joint->id()));
    InsertTorque(&known_values, joint->id(), 0.0);
  }
  for (const auto& link : robot.links()) {
    InsertPose(&known_values, link->id(), Pose(values0, link->id()));
    InsertTwist(&known_values, link->id(), Twist(values0, link->id()));
  }
  values0 = graph_builder.linearSolveFD(robot, 0, known_values);

  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    for (const auto& joint : robot.joints()) {
      InsertJointAngle(&values, joint->id(), k,
                       JointAngle(values0, joint->id()));
      InsertJointVel(&values, joint->id(), k, JointVel(values0, joint->id()));
      InsertJointAccel(&values, joint->id(), k,
                       JointAccel(values0, joint->id()));
      InsertTorque(&values, joint->id(), k, Torque(values0, joint->id()));
      for (const auto& link : joint->links()) {
        InsertWrench(&values, link->id(), joint->id(), k,
                     Wrench(values0, link->id(), joint->id()));
      }
    }
    for (const auto& link : robot.links()) {
      InsertPose(&values, link->id(), k, Pose(values0, link->id()));
      InsertTwist(&values, link->id(), k, Twist(values0, link->id()));
      InsertTwistAccel(&values, link->id(), k, TwistAccel(values0, link->id()));
    }
  }
  return values;
}

/* ************************************************************************* */
Values CartPole::getInitValues(size_t num_steps, std::string option) {
  if (option == "zero") {
    return getInitValuesZero(num_steps);
  } else if (option == "interp") {
    return getInitValuesInterp(num_steps);
  } else if (option == "infeasible") {
    return getInitValuesInfeasible(num_steps);
  } else {
    return getInitValuesZero(num_steps);
  }
}

/* ************************************************************************* */
Values CartPole::getInitValuesInterp(size_t num_steps) {
  Initializer initializer;

  double init_q0 = X_i(0);
  double terminal_q0 = X_T(0);
  double init_q1 = X_i(3);
  double terminal_q1 = X_T(3);

  Values init_values;
  for (size_t k = 0; k <= num_steps; k++) {
    double q0 =
        init_q0 + (terminal_q0 - init_q0) * (double)k / (double)num_steps;
    double q1 =
        init_q1 + (terminal_q1 - init_q1) * (double)k / (double)num_steps;
    Values known_values;
    InsertJointAngle(&known_values, j0_id, k, q0);
    InsertJointAngle(&known_values, j1_id, k, q1);
    known_values = robot.forwardKinematics(known_values, k);
    InsertTorque(&known_values, j0_id, k, 0.0);
    InsertTorque(&known_values, j1_id, k, 0.0);
    Values values = graph_builder.linearSolveFD(robot, k, known_values);
    init_values.insert(values);
  }
  return init_values;
}

/* ************************************************************************* */
Values CartPole::getInitValuesInfeasible(size_t num_steps) {
  Initializer initializer;
  auto init_values_infeasible =
      initializer.ZeroValuesTrajectory(robot, num_steps, 0, 0.0);
  return init_values_infeasible;
}
/* ************************************************************************* */
void CartPole::exportTrajectory(const Values& results, size_t num_steps,
                                double dt, std::string file_name) const {
  std::ofstream traj_file;
  traj_file.open(file_name);
  traj_file << "t,x,xdot,xddot,xtau,theta,thetadot,thetaddot,thetatau\n";
  double t_elapsed = 0;
  for (int t = 0; t <= num_steps; t++, t_elapsed += dt) {
    std::vector<gtsam::Key> keys = {
        JointAngleKey(j0_id, t), JointVelKey(j0_id, t),
        JointAccelKey(j0_id, t), TorqueKey(j0_id, t),
        JointAngleKey(j1_id, t), JointVelKey(j1_id, t),
        JointAccelKey(j1_id, t), TorqueKey(j1_id, t)};
    std::vector<std::string> vals = {std::to_string(t_elapsed)};
    for (auto&& k : keys) {
      vals.push_back(std::to_string(results.atDouble(k)));
    }
    std::string vals_str = "";
    for (size_t j = 0; j < vals.size(); j++) {
      vals_str += vals[j] + (j != vals.size() - 1 ? "," : "");
    }
    traj_file << vals_str << "\n";
  }
  traj_file.close();
}

/* ************************************************************************* */
void CartPole::printJointAngles(const Values& values, size_t num_steps) const {
  for (size_t k = 0; k <= num_steps; k++) {
    std::cout << "step " << k << ":";
    for (const auto& joint : robot.joints()) {
      double angle = JointAngle(values, joint->id(), k);
      std::cout << "\t" << angle;
    }
    std::cout << "\n";
  }
}

/* ************************************************************************* */
OptimizerSetting CartPole::getOptSetting() const {
  auto opt = gtdynamics::OptimizerSetting();
  // opt.bp_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  // opt.bv_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  // opt.ba_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  // opt.p_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  // opt.v_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  // opt.a_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  // opt.f_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  // opt.fa_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  // opt.t_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  // opt.cp_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  // opt.cfriction_cost_model =
  //     gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  // opt.cv_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  // opt.ca_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  // opt.planar_cost_model =
  //     gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  // opt.prior_q_cost_model =
  //     gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  // opt.prior_qv_cost_model =
  //     gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  // opt.prior_qa_cost_model =
  //     gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  // opt.prior_t_cost_model =
  //     gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  // opt.q_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(1,
  // sigma_collocation); opt.v_col_cost_model =
  // gtsam::noiseModel::Isotropic::Sigma(1, sigma_collocation);
  // opt.time_cost_model = gtsam::noiseModel::Isotropic::Sigma(1,
  // sigma_dynamics); opt.pose_col_cost_model =
  //     gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  // opt.twist_col_cost_model =
  //     gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  return opt;
}

/* ************************************************************************* */
BasisKeyFunc CartPole::getBasisKeyFunc(bool unactuated_as_constraint) const {
  if (unactuated_as_constraint) {
    BasisKeyFunc basis_key_func =
        [=](const KeyVector& keys) -> KeyVector {
      KeyVector basis_keys;
      for (const Key& key : keys) {
        auto symb = DynamicsSymbol(key);
        if (symb.label() == "q") {
          basis_keys.push_back(key);
        }
        if (symb.label() == "v") {
          basis_keys.push_back(key);
        }
        if (symb.label() == "T" and symb.jointIdx() == j0_id) {
          basis_keys.push_back(key);
        }
      }
      return basis_keys;
    };
    return basis_key_func;
  } else {
    BasisKeyFunc basis_key_func =
        [](const KeyVector& keys) -> KeyVector {
      KeyVector basis_keys;
      for (const Key& key : keys) {
        auto symb = DynamicsSymbol(key);
        if (symb.label() == "q") {
          basis_keys.push_back(key);
        }
        if (symb.label() == "v") {
          basis_keys.push_back(key);
        }
        if (symb.label() == "T") {
          basis_keys.push_back(key);
        }
      }
      return basis_keys;
    };
    return basis_key_func;
  }
}

}  // namespace gtdynamics
