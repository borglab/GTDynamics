#include "utils/DynamicsSymbol.h"
#include "utils/values.h"
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/scenarios/IECartPoleWithLimits.h>
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using gtdynamics::DoubleExpressionEquality, gtdynamics::EqualityConstraints;
using gtdynamics::DoubleExpressionInequality, gtdynamics::InequalityConstraints;
using gtdynamics::InsertTorque, gtdynamics::InsertJointAngle,
    gtdynamics::InsertJointVel;
using gtdynamics::JointAngleKey, gtdynamics::JointVelKey,
    gtdynamics::JointAccelKey, gtdynamics::TorqueKey;

namespace gtsam {

/* ************************************************************************* */
void IECartPoleWithLimits::PrintValues(const Values &values,
                                       const size_t num_steps) {
  std::cout << std::setw(12) << "x" << std::setw(12) << "v" << std::setw(12)
            << "a" << std::setw(12) << "q" << std::setw(12) << "w"
            << std::setw(12) << "qa" << std::setw(12) << "f"
            << "\n";
  for (size_t k = 0; k <= num_steps; k++) {
    double q = values.atDouble(JointAngleKey(1, k));
    double w = values.atDouble(JointVelKey(1, k));
    double qa = values.atDouble(JointAccelKey(1, k));
    double x = values.atDouble(JointAngleKey(0, k));
    double v = values.atDouble(JointVelKey(0, k));
    double a = values.atDouble(JointAccelKey(0, k));
    double f = values.atDouble(TorqueKey(0, k));
    std::cout << std::setprecision(5) << std::setw(12) << x << std::setw(12)
              << v << std::setw(12) << a << std::setw(12) << q << std::setw(12)
              << w << std::setw(12) << qa << std::setw(12) << f << "\n";
  }
}

/* ************************************************************************* */
void IECartPoleWithLimits::PrintDelta(const VectorValues &values,
                                      const size_t num_steps) {
  std::cout << std::setw(12) << "x" << std::setw(12) << "v" << std::setw(12)
            << "a" << std::setw(12) << "q" << std::setw(12) << "w"
            << std::setw(12) << "qa" << std::setw(12) << "f"
            << "\n";
  for (size_t k = 0; k <= num_steps; k++) {
    double q = values.at(JointAngleKey(1, k))(0);
    double w = values.at(JointVelKey(1, k))(0);
    double qa = values.at(JointAccelKey(1, k))(0);
    double x = values.at(JointAngleKey(0, k))(0);
    double v = values.at(JointVelKey(0, k))(0);
    double a = values.at(JointAccelKey(0, k))(0);
    double f = values.at(TorqueKey(0, k))(0);
    std::cout << std::setprecision(5) << std::setw(12) << x << std::setw(12)
              << v << std::setw(12) << a << std::setw(12) << q << std::setw(12)
              << w << std::setw(12) << qa << std::setw(12) << f << "\n";
  }
}

/* ************************************************************************* */
void IECartPoleWithLimits::ExportValues(const Values &values, size_t num_steps,
                                        const std::string &file_path) {
  std::ofstream traj_file;
  traj_file.open(file_path);
  double dt = 0.1;
  traj_file << "t,x,xdot,xddot,xtau,theta,thetadot,thetaddot,thetatau\n";
  double t_elapsed = 0;
  for (int t = 0; t <= num_steps; t++, t_elapsed += dt) {
    std::vector<gtsam::Key> keys = {JointAngleKey(0, t), JointVelKey(0, t),
                                    JointAccelKey(0, t), TorqueKey(0, t),
                                    JointAngleKey(1, t), JointVelKey(1, t),
                                    JointAccelKey(1, t), TorqueKey(1, t)};
    std::vector<std::string> vals = {std::to_string(t_elapsed)};
    for (auto &&k : keys) {
      vals.push_back(std::to_string(values.atDouble(k)));
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
gtdynamics::EqualityConstraints
IECartPoleWithLimits::eConstraints(const int k) const {
  auto graph = graph_builder.dynamicsFactorGraph(robot, k);
  graph.addPrior(TorqueKey(r_joint_id, k), 0.0,
                 graph_builder.opt().prior_t_cost_model);
  auto e_constraints = gtdynamics::ConstraintsFromGraph(graph);
  return e_constraints;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IECartPoleWithLimits::iConstraints(const int k) const {
  Key p_joint_key = JointAngleKey(p_joint_id, k);
  Key force_key = TorqueKey(p_joint_id, k);
  InequalityConstraints i_constraints;
  Double_ x_expr(p_joint_key);
  Double_ f_expr(force_key);
  Double_ x_min_expr = x_expr - Double_(x_min);
  Double_ x_max_expr = Double_(x_max) - x_expr;
  Double_ f_min_expr = f_expr - Double_(f_min);
  Double_ f_max_expr = Double_(f_max) - f_expr;
  i_constraints.emplace_shared<DoubleExpressionInequality>(x_min_expr, tol);
  i_constraints.emplace_shared<DoubleExpressionInequality>(x_max_expr, tol);
  i_constraints.emplace_shared<DoubleExpressionInequality>(f_min_expr, tol);
  i_constraints.emplace_shared<DoubleExpressionInequality>(f_max_expr, tol);
  return i_constraints;
}

/* ************************************************************************* */
NonlinearFactorGraph
IECartPoleWithLimits::finalStateCosts(size_t num_steps) const {
  NonlinearFactorGraph graph;
  if (constrain_final_x) {
    graph.addPrior(JointAngleKey(p_joint_id, num_steps), X_T[0],
                   pos_objectives_model);
  }
  graph.addPrior(JointAngleKey(r_joint_id, num_steps), X_T[3],
                 pos_objectives_model);
  graph.addPrior(JointVelKey(p_joint_id, num_steps), X_T[1], objectives_model);
  graph.addPrior(JointVelKey(r_joint_id, num_steps), X_T[4], objectives_model);
  return graph;
}

/* ************************************************************************* */
gtdynamics::EqualityConstraints
IECartPoleWithLimits::initStateConstraints() const {
  EqualityConstraints constraints;
  Key r_joint_key = JointAngleKey(r_joint_id, 0);
  Key p_joint_key = JointAngleKey(p_joint_id, 0);
  Key r_vel_key = JointVelKey(r_joint_id, 0);
  Key p_vel_key = JointVelKey(p_joint_id, 0);
  Double_ r_joint_expr(r_joint_key);
  Double_ p_joint_expr(p_joint_key);
  Double_ r_vel_expr(r_vel_key);
  Double_ p_vel_expr(p_vel_key);
  constraints.emplace_shared<DoubleExpressionEquality>(
      r_joint_expr - Double_(X_i(3)), tol);
  constraints.emplace_shared<DoubleExpressionEquality>(
      p_joint_expr - Double_(X_i(0)), tol);
  constraints.emplace_shared<DoubleExpressionEquality>(
      r_vel_expr - Double_(X_i(4)), tol);
  constraints.emplace_shared<DoubleExpressionEquality>(
      p_vel_expr - Double_(X_i(1)), tol);
  return constraints;
}

/* ************************************************************************* */
gtdynamics::EqualityConstraints
IECartPoleWithLimits::finalStateConstraints(size_t num_steps) const {
  EqualityConstraints constraints;
  Key r_joint_key = gtdynamics::JointAngleKey(r_joint_id, num_steps);
  Key p_joint_key = gtdynamics::JointAngleKey(p_joint_id, num_steps);
  Key r_vel_key = gtdynamics::JointVelKey(r_joint_id, num_steps);
  Key p_vel_key = gtdynamics::JointVelKey(p_joint_id, num_steps);
  Double_ r_joint_expr(r_joint_key);
  Double_ p_joint_expr(p_joint_key);
  Double_ r_vel_expr(r_vel_key);
  Double_ p_vel_expr(p_vel_key);
  constraints.emplace_shared<DoubleExpressionEquality>(
      r_joint_expr - Double_(X_T(3)), tol);
  if (constrain_final_x) {
    constraints.emplace_shared<DoubleExpressionEquality>(
        p_joint_expr - Double_(X_T(0)), tol);
  }
  constraints.emplace_shared<DoubleExpressionEquality>(
      r_vel_expr - Double_(X_T(4)), tol);
  constraints.emplace_shared<DoubleExpressionEquality>(
      p_vel_expr - Double_(X_T(1)), tol);
  return constraints;
}

/* ************************************************************************* */
NonlinearFactorGraph
IECartPoleWithLimits::minTorqueCosts(size_t num_steps) const {
  NonlinearFactorGraph graph;
  for (int k = 0; k <= num_steps; k++)
    graph.emplace_shared<gtdynamics::MinTorqueFactor>(
        gtdynamics::TorqueKey(p_joint_id, k), control_model);
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IECartPoleWithLimits::collocationCosts(size_t num_steps,
                                                            double dt) const {
  NonlinearFactorGraph graph;
  for (int k = 0; k < num_steps; k++) {
    graph.add(
        graph_builder.collocationFactors(robot, k, dt, collocation_scheme));
  }
  return graph;
}

/* ************************************************************************* */
Values IECartPoleWithLimits::computeValues(const size_t &k, const double &x,
                                           const double &v, const double &q,
                                           const double &w) const {
  Values known_values;
  Key p_joint_key = JointAngleKey(p_joint_id, k);
  Key p_vel_key = JointVelKey(p_joint_id, k);
  Key r_joint_key = JointAngleKey(r_joint_id, k);
  Key r_vel_key = JointVelKey(r_joint_id, k);
  known_values.insert(p_joint_key, x);
  known_values.insert(p_vel_key, v);
  known_values.insert(r_joint_key, q);
  known_values.insert(r_vel_key, w);
  known_values = robot.forwardKinematics(known_values, k, "l0");

  InsertTorque(&known_values, p_joint_id, k, 0.0);
  InsertTorque(&known_values, r_joint_id, k, 0.0);
  Values values = graph_builder.linearSolveFD(robot, k, known_values);
  return values;
}

/* ************************************************************************* */
Values IECartPoleWithLimits::getInitValuesZero(size_t num_steps) const {
  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    values.insert(computeValues(k, 0, 0, 0, 0));
  }
  return values;
}

/* ************************************************************************* */
Values IECartPoleWithLimits::getInitValuesInterp(size_t num_steps) const {
  gtdynamics::Initializer initializer;

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
    init_values.insert(computeValues(k, q0, 0, q1, 0));
  }
  return init_values;
}

BasisKeyFunc IECartPoleWithLimits::getBasisKeyFunc() const {
  BasisKeyFunc basis_key_func = [=](const ConnectedComponent::shared_ptr &cc) -> KeyVector {
    KeyVector basis_keys;
    size_t k = gtdynamics::DynamicsSymbol(*cc->keys_.begin()).time();
    if (k == 0) {
      basis_keys.push_back(gtdynamics::TorqueKey(p_joint_id, k));
      return basis_keys;
    }
    // if (k == num_steps) {
    //   basis_keys.push_back(gtdynamics::TorqueKey(cp.p_joint_id, k));
    //   if (!cp.constrain_final_x) {
    //     basis_keys.push_back(gtdynamics::JointAngleKey(cp.p_joint_id, k));
    //   }
    //   return basis_keys;
    // }
    for (const Key &key : cc->keys_) {
      auto symb = gtdynamics::DynamicsSymbol(key);
      if (symb.label() == "q") {
        basis_keys.push_back(key);
      }
      if (symb.label() == "v") {
        basis_keys.push_back(key);
      }
      if (symb.label() == "T" and symb.jointIdx() == p_joint_id) {
        basis_keys.push_back(key);
      }
    }
    return basis_keys;
  };
  return basis_key_func;
}



/* ************************************************************************* */
IEConstraintManifold CartPoleWithLimitsRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices) const {
  Values new_values = manifold->values().retract(delta);

  // delta.print("==============================delta============================\n",
  // gtdynamics::GTDKeyFormatter);

  size_t k = gtdynamics::DynamicsSymbol(new_values.keys().front()).time();
  IndexSet active_indices;

  // limit x to bounded values
  double x = new_values.atDouble(JointAngleKey(cp_.p_joint_id, k));
  if (x < cp_.x_min || blocking_indices && blocking_indices->exists(0)) {
    x = cp_.x_min;
    active_indices.insert(0);
  }
  if (x > cp_.x_max || blocking_indices && blocking_indices->exists(1)) {
    x = cp_.x_max;
    active_indices.insert(1);
  }

  // limit f to bounded values
  double f = new_values.atDouble(TorqueKey(cp_.p_joint_id, k));
  if (f < cp_.f_min || blocking_indices && blocking_indices->exists(2)) {
    f = cp_.f_min;
    active_indices.insert(2);
  }
  if (f > cp_.f_max || blocking_indices && blocking_indices->exists(3)) {
    f = cp_.f_max;
    active_indices.insert(3);
  }

  // compute rest values
  double q = new_values.atDouble(JointAngleKey(cp_.r_joint_id, k));
  double w = new_values.atDouble(JointVelKey(cp_.r_joint_id, k));
  double v = new_values.atDouble(JointVelKey(cp_.p_joint_id, k));
  Values known_values;
  InsertJointAngle(&known_values, cp_.p_joint_id, k, x);
  InsertJointAngle(&known_values, cp_.r_joint_id, k, q);
  InsertJointVel(&known_values, cp_.p_joint_id, k, v);
  InsertJointVel(&known_values, cp_.r_joint_id, k, w);
  known_values = cp_.robot.forwardKinematics(known_values, k);
  InsertTorque(&known_values, cp_.p_joint_id, k, f);
  InsertTorque(&known_values, cp_.r_joint_id, k, 0.0);
  Values result = cp_.graph_builder.linearSolveFD(cp_.robot, k, known_values);

  // result.print("==============================result============================\n",
  // gtdynamics::GTDKeyFormatter);

  return manifold->createWithNewValues(result, active_indices);
}

} // namespace gtsam
