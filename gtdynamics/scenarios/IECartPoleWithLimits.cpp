#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/scenarios/IECartPoleWithLimits.h>
#include <gtdynamics/utils/Initializer.h>

#include <iomanip>

namespace gtdynamics {
using namespace gtsam;

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
    std::vector<Key> keys = {JointAngleKey(0, t), JointVelKey(0, t),
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
NonlinearEqualityConstraints
IECartPoleWithLimits::eConstraints(const int k) const {
  auto graph = graph_builder_.dynamicsFactorGraph(robot_, k);
  graph.addPrior(TorqueKey(r_joint_id_, k), 0.0,
                 graph_builder_.opt().prior_t_cost_model);
  auto e_constraints = NonlinearEqualityConstraints::FromCostGraph(graph);
  return e_constraints;
}

/* ************************************************************************* */
NonlinearInequalityConstraints
IECartPoleWithLimits::iConstraints(const int k) const {
  Key p_joint_key = JointAngleKey(p_joint_id_, k);
  Key force_key = TorqueKey(p_joint_id_, k);
  NonlinearInequalityConstraints i_constraints;
  Double_ x_expr(p_joint_key);
  Double_ f_expr(force_key);
  Double_ x_min_expr = x_expr - Double_(x_min_);
  Double_ x_max_expr = Double_(x_max_) - x_expr;
  Double_ f_min_expr = f_expr - Double_(f_min_);
  Double_ f_max_expr = Double_(f_max_) - f_expr;
  i_constraints.push_back(
      ScalarExpressionInequalityConstraint::GeqZero(x_min_expr, tol_));
  i_constraints.push_back(
      ScalarExpressionInequalityConstraint::GeqZero(x_max_expr, tol_));
  i_constraints.push_back(
      ScalarExpressionInequalityConstraint::GeqZero(f_min_expr, tol_));
  i_constraints.push_back(
      ScalarExpressionInequalityConstraint::GeqZero(f_max_expr, tol_));
  return i_constraints;
}

/* ************************************************************************* */
NonlinearFactorGraph
IECartPoleWithLimits::finalStateCosts(size_t num_steps) const {
  NonlinearFactorGraph graph;
  if (constrain_final_x_) {
    graph.addPrior(JointAngleKey(p_joint_id_, num_steps), target_state_[0],
                   pos_objectives_model_);
  }
  graph.addPrior(JointAngleKey(r_joint_id_, num_steps), target_state_[3],
                 pos_objectives_model_);
  graph.addPrior(JointVelKey(p_joint_id_, num_steps), target_state_[1],
                 objectives_model_);
  graph.addPrior(JointVelKey(r_joint_id_, num_steps), target_state_[4],
                 objectives_model_);
  return graph;
}

/* ************************************************************************* */
NonlinearEqualityConstraints
IECartPoleWithLimits::initStateConstraints() const {
  NonlinearEqualityConstraints constraints;
  Key r_joint_key = JointAngleKey(r_joint_id_, 0);
  Key p_joint_key = JointAngleKey(p_joint_id_, 0);
  Key r_vel_key = JointVelKey(r_joint_id_, 0);
  Key p_vel_key = JointVelKey(p_joint_id_, 0);
  Double_ r_joint_expr(r_joint_key);
  Double_ p_joint_expr(p_joint_key);
  Double_ r_vel_expr(r_vel_key);
  Double_ p_vel_expr(p_vel_key);
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      r_joint_expr - Double_(initial_state_(3)), 0.0, Vector1(tol_));
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      p_joint_expr - Double_(initial_state_(0)), 0.0, Vector1(tol_));
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      r_vel_expr - Double_(initial_state_(4)), 0.0, Vector1(tol_));
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      p_vel_expr - Double_(initial_state_(1)), 0.0, Vector1(tol_));
  return constraints;
}

/* ************************************************************************* */
NonlinearEqualityConstraints
IECartPoleWithLimits::finalStateConstraints(size_t num_steps) const {
  NonlinearEqualityConstraints constraints;
  Key r_joint_key = JointAngleKey(r_joint_id_, num_steps);
  Key p_joint_key = JointAngleKey(p_joint_id_, num_steps);
  Key r_vel_key = JointVelKey(r_joint_id_, num_steps);
  Key p_vel_key = JointVelKey(p_joint_id_, num_steps);
  Double_ r_joint_expr(r_joint_key);
  Double_ p_joint_expr(p_joint_key);
  Double_ r_vel_expr(r_vel_key);
  Double_ p_vel_expr(p_vel_key);
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      r_joint_expr - Double_(target_state_(3)), 0.0, Vector1(tol_));
  if (constrain_final_x_) {
    constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
        p_joint_expr - Double_(target_state_(0)), 0.0, Vector1(tol_));
  }
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      r_vel_expr - Double_(target_state_(4)), 0.0, Vector1(tol_));
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      p_vel_expr - Double_(target_state_(1)), 0.0, Vector1(tol_));
  return constraints;
}

/* ************************************************************************* */
NonlinearFactorGraph
IECartPoleWithLimits::minTorqueCosts(size_t num_steps) const {
  NonlinearFactorGraph graph;
  for (int k = 0; k <= num_steps; k++)
    graph.emplace_shared<MinTorqueFactor>(
        TorqueKey(p_joint_id_, k), control_model_);
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IECartPoleWithLimits::collocationCosts(size_t num_steps,
                                                            double dt) const {
  NonlinearFactorGraph graph;
  for (int k = 0; k < num_steps; k++) {
    graph.add(
        graph_builder_.collocationFactors(robot_, k, dt, collocation_scheme_));
  }
  return graph;
}

/* ************************************************************************* */
Values IECartPoleWithLimits::computeValues(const size_t &k, const double &x,
                                           const double &v, const double &q,
                                           const double &w) const {
  Values known_values;
  Key p_joint_key = JointAngleKey(p_joint_id_, k);
  Key p_vel_key = JointVelKey(p_joint_id_, k);
  Key r_joint_key = JointAngleKey(r_joint_id_, k);
  Key r_vel_key = JointVelKey(r_joint_id_, k);
  known_values.insert(p_joint_key, x);
  known_values.insert(p_vel_key, v);
  known_values.insert(r_joint_key, q);
  known_values.insert(r_vel_key, w);
  known_values = robot_.forwardKinematics(known_values, k, "l0");

  InsertTorque(&known_values, p_joint_id_, k, 0.0);
  InsertTorque(&known_values, r_joint_id_, k, 0.0);
  Values values = graph_builder_.linearSolveFD(robot_, k, known_values);
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
  Initializer initializer;

  double init_q0 = initial_state_(0);
  double terminal_q0 = target_state_(0);
  double init_q1 = initial_state_(3);
  double terminal_q1 = target_state_(3);

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
  BasisKeyFunc basis_key_func = [=](const KeyVector& keys) -> KeyVector {
    KeyVector basis_keys;
    size_t k = DynamicsSymbol(*keys.begin()).time();
    if (k == 0) {
      basis_keys.push_back(TorqueKey(p_joint_id_, k));
      return basis_keys;
    }
    // if (k == num_steps) {
    //   basis_keys.push_back(TorqueKey(cp.p_joint_id_, k));
    //   if (!cp.constrain_final_x_) {
    //     basis_keys.push_back(JointAngleKey(cp.p_joint_id_, k));
    //   }
    //   return basis_keys;
    // }
    for (const Key &key : keys) {
      auto symb = DynamicsSymbol(key);
      if (symb.label() == "q") {
        basis_keys.push_back(key);
      }
      if (symb.label() == "v") {
        basis_keys.push_back(key);
      }
      if (symb.label() == "T" and symb.jointIdx() == p_joint_id_) {
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
    const std::optional<IndexSet> &blocking_indices,
    IERetractInfo* retract_info) const {
  Values new_values = manifold->values().retract(delta);

  // delta.print("==============================delta============================\n",
  // GTDKeyFormatter);

  size_t k = DynamicsSymbol(new_values.keys().front()).time();
  IndexSet active_indices;

  // limit x to bounded values
  double x = new_values.atDouble(JointAngleKey(cp_.p_joint_id_, k));
  if (x < cp_.x_min_ || blocking_indices && blocking_indices->exists(0)) {
    x = cp_.x_min_;
    active_indices.insert(0);
  }
  if (x > cp_.x_max_ || blocking_indices && blocking_indices->exists(1)) {
    x = cp_.x_max_;
    active_indices.insert(1);
  }

  // limit f to bounded values
  double f = new_values.atDouble(TorqueKey(cp_.p_joint_id_, k));
  if (f < cp_.f_min_ || blocking_indices && blocking_indices->exists(2)) {
    f = cp_.f_min_;
    active_indices.insert(2);
  }
  if (f > cp_.f_max_ || blocking_indices && blocking_indices->exists(3)) {
    f = cp_.f_max_;
    active_indices.insert(3);
  }

  // compute rest values
  double q = new_values.atDouble(JointAngleKey(cp_.r_joint_id_, k));
  double w = new_values.atDouble(JointVelKey(cp_.r_joint_id_, k));
  double v = new_values.atDouble(JointVelKey(cp_.p_joint_id_, k));
  Values known_values;
  InsertJointAngle(&known_values, cp_.p_joint_id_, k, x);
  InsertJointAngle(&known_values, cp_.r_joint_id_, k, q);
  InsertJointVel(&known_values, cp_.p_joint_id_, k, v);
  InsertJointVel(&known_values, cp_.r_joint_id_, k, w);
  known_values = cp_.robot_.forwardKinematics(known_values, k);
  InsertTorque(&known_values, cp_.p_joint_id_, k, f);
  InsertTorque(&known_values, cp_.r_joint_id_, k, 0.0);
  Values result =
      cp_.graph_builder_.linearSolveFD(cp_.robot_, k, known_values);

  // result.print("==============================result============================\n",
  // GTDKeyFormatter);

  return manifold->createWithNewValues(result, active_indices);
}

} // namespace gtdynamics
