#include <gtdynamics/scenarios/IECartPoleWithFriction.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
double IECartPoleWithFriction::computeFx(const double q, const double v,
                                         const double a,
                                         OptionalJacobian<1, 1> H_q,
                                         OptionalJacobian<1, 1> H_v,
                                         OptionalJacobian<1, 1> H_a) const {
  double s = sin(q), c = cos(q);
  if (H_q)
    H_q->setConstant(m * v * v * r * s - m * a * r * c);
  if (H_v)
    H_v->setConstant(-2 * m * r * v * c);
  if (H_a)
    H_a->setConstant(-m * r * s);
  return -m * v * v * r * c - m * a * r * s;
}

/* ************************************************************************* */
double IECartPoleWithFriction::computeFy(const double q, const double v,
                                         const double a,
                                         OptionalJacobian<1, 1> H_q,
                                         OptionalJacobian<1, 1> H_v,
                                         OptionalJacobian<1, 1> H_a) const {
  double s = sin(q), c = cos(q);
  if (H_q)
    H_q->setConstant(-m * v * v * r * c - m * a * r * s);
  if (H_v)
    H_v->setConstant(-2 * m * v * r * s);
  if (H_a)
    H_a->setConstant(m * r * c);
  return (M + m) * g - m * v * v * r * s + m * a * r * c;
}

double IECartPoleWithFriction::computeTau(const double q, const double a,
                                          OptionalJacobian<1, 1> H_q,
                                          OptionalJacobian<1, 1> H_a) const {
  if (H_q)
    H_q->setConstant(-m * g * r * sin(q));
  if (H_a)
    H_a->setConstant(m * r * r);
  return m * r * r * a + m * g * r * cos(q);
}

/* ************************************************************************* */
Double_ IECartPoleWithFriction::balanceFxExpr(const Double_ &q_expr,
                                              const Double_ &v_expr,
                                              const Double_ &a_expr,
                                              const Double_ &fx_expr) const {
  auto compute_fx_function = [&](const double q, const double v, const double a,
                                 OptionalJacobian<1, 1> H_q = {},
                                 OptionalJacobian<1, 1> H_v = {},
                                 OptionalJacobian<1, 1> H_a = {}) {
    return computeFx(q, v, a, H_q, H_v, H_a);
  };
  Double_ compute_fx_expr(compute_fx_function, q_expr, v_expr, a_expr);
  return fx_expr - compute_fx_expr;
}

/* ************************************************************************* */
Double_ IECartPoleWithFriction::balanceFyExpr(const Double_ &q_expr,
                                              const Double_ &v_expr,
                                              const Double_ &a_expr,
                                              const Double_ &fy_expr) const {
  auto compute_fy_function = [&](const double q, const double v, const double a,
                                 OptionalJacobian<1, 1> H_q = {},
                                 OptionalJacobian<1, 1> H_v = {},
                                 OptionalJacobian<1, 1> H_a = {}) {
    return computeFy(q, v, a, H_q, H_v, H_a);
  };
  Double_ compute_fy_expr(compute_fy_function, q_expr, v_expr, a_expr);
  return fy_expr - compute_fy_expr;
}

/* ************************************************************************* */
Double_ IECartPoleWithFriction::balanceRotExpr(const Double_ &q_expr,
                                               const Double_ &a_expr,
                                               const Double_ &tau_expr) const {
  auto compute_tau_function = [&](const double q, const double a,
                                 OptionalJacobian<1, 1> H_q = {},
                                 OptionalJacobian<1, 1> H_a = {}) {
    return computeTau(q, a, H_q, H_a);
  };
  Double_ compute_tau_expr(compute_tau_function, q_expr, a_expr);
  
  return tau_expr - compute_tau_expr;
}

/* ************************************************************************* */
Double_
IECartPoleWithFriction::frictionConeExpr1(const Double_ &fx_expr,
                                          const Double_ &fy_expr) const {
  return mu * fy_expr - fx_expr;
}

/* ************************************************************************* */
Double_
IECartPoleWithFriction::frictionConeExpr2(const Double_ &fx_expr,
                                          const Double_ &fy_expr) const {
  return mu * fy_expr + fx_expr;
}

/* ************************************************************************* */
gtdynamics::EqualityConstraints
IECartPoleWithFriction::eConstraints(const int k) const {
  gtdynamics::EqualityConstraints constraints;
  Double_ q_expr(QKey(k));
  Double_ v_expr(VKey(k));
  Double_ a_expr(AKey(k));
  Double_ tau_expr(TauKey(k));
  Double_ fx_expr(FxKey(k));
  Double_ fy_expr(FyKey(k));
  constraints.emplace_shared<gtdynamics::DoubleExpressionEquality>(
      balanceFxExpr(q_expr, v_expr, a_expr, fx_expr), 1.0);
  constraints.emplace_shared<gtdynamics::DoubleExpressionEquality>(
      balanceFyExpr(q_expr, v_expr, a_expr, fy_expr), 1.0);
  constraints.emplace_shared<gtdynamics::DoubleExpressionEquality>(
      balanceRotExpr(q_expr, a_expr, tau_expr), 1.0);
  return constraints;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IECartPoleWithFriction::iConstraints(const int k) const {
  gtdynamics::InequalityConstraints constraints;
  Double_ fx_expr(FxKey(k));
  Double_ fy_expr(FyKey(k));
  constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
      frictionConeExpr1(fx_expr, fy_expr), 1.0);
  constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
      frictionConeExpr2(fx_expr, fy_expr), 1.0);
  if (include_torque_limits) {
    Double_ torque_expr(TauKey(k));
    Double_ lower_limit_expr = torque_expr - Double_(tau_min);
    Double_ upper_limit_expr = Double_(tau_max) - torque_expr;
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
        lower_limit_expr, 1.0);
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
        upper_limit_expr, 1.0);
  }
  return constraints;
}

/* ************************************************************************* */
Values IECartPoleWithFriction::computeValues(const size_t &k, const double &q,
                                             const double &v,
                                             const double &a) const {
  Values values;
  double tau = computeTau(q, a);
  double fx = computeFx(q, v, a);
  double fy = computeFy(q, v, a);
  values.insertDouble(QKey(k), q);
  values.insertDouble(VKey(k), v);
  values.insertDouble(AKey(k), a);
  values.insertDouble(TauKey(k), tau);
  values.insertDouble(FxKey(k), fx);
  values.insertDouble(FyKey(k), fy);
  return values;
}

/* ************************************************************************* */
void IECartPoleWithFriction::PrintValues(const Values &values,
                                         const size_t num_steps) {
  std::cout << std::setw(12) << "q" << std::setw(12) << "v" << std::setw(12)
            << "a" << std::setw(12) << "tau" << std::setw(12) << "fx"
            << std::setw(12) << "fy"
            << "\n";
  for (size_t k = 0; k <= num_steps; k++) {
    double q = values.atDouble(QKey(k));
    double v = values.atDouble(VKey(k));
    double a = values.atDouble(AKey(k));
    double tau = values.atDouble(TauKey(k));
    double fx = values.atDouble(FxKey(k));
    double fy = values.atDouble(FyKey(k));
    std::cout << std::setprecision(5) << std::setw(12) << q << std::setw(12)
              << v << std::setw(12) << a << std::setw(12) << tau
              << std::setw(12) << fx << std::setw(12) << fy << "\n";
  }
}

/* ************************************************************************* */
void IECartPoleWithFriction::PrintDelta(const VectorValues &values,
                                        const size_t num_steps) {
  std::cout << std::setw(12) << "q" << std::setw(12) << "v" << std::setw(12)
            << "a" << std::setw(12) << "tau" << std::setw(12) << "fx"
            << std::setw(12) << "fy"
            << "\n";
  for (size_t k = 0; k <= num_steps; k++) {
    double q = values.at(QKey(k))(0);
    double v = values.at(VKey(k))(0);
    double a = values.at(AKey(k))(0);
    double tau = values.at(TauKey(k))(0);
    double fx = values.at(FxKey(k))(0);
    double fy = values.at(FyKey(k))(0);
    std::cout << std::setw(12) << q << std::setw(12) << v << std::setw(12) << a
              << std::setw(12) << tau << std::setw(12) << fx << std::setw(12)
              << fy << "\n";
  }
}

/* ************************************************************************* */
void IECartPoleWithFriction::ExportValues(const Values &values,
                                          const size_t num_steps,
                                          const std::string &file_path) {
  std::ofstream file;
  file.open(file_path);
  for (int k = 0; k <= num_steps; k++) {
    double q = values.atDouble(QKey(k));
    double v = values.atDouble(VKey(k));
    double a = values.atDouble(AKey(k));
    double tau = values.atDouble(TauKey(k));
    double fx = values.atDouble(FxKey(k));
    double fy = values.atDouble(FyKey(k));
    file << q << " " << v << " " << a << " " << tau << " " << fx << " " << fy
         << "\n";
  }
  file.close();
}

/* ************************************************************************* */
void IECartPoleWithFriction::ExportVector(const VectorValues &values,
                                          const size_t num_steps,
                                          const std::string &file_path) {
  std::ofstream file;
  file.open(file_path);
  for (int k = 0; k <= num_steps; k++) {
    double q = values.at(QKey(k))(0);
    double v = values.at(VKey(k))(0);
    double a = values.at(AKey(k))(0);
    double tau = values.at(TauKey(k))(0);
    double fx = values.at(FxKey(k))(0);
    double fy = values.at(FyKey(k))(0);
    file << q << " " << v << " " << a << " " << tau << " " << fx << " " << fy
         << "\n";
  }
  file << "\n";
  file.close();
}

/* ************************************************************************* */
void UpdateBoundary(const double &a, const double &b, const size_t &index,
                    double &ub, double &lb, bool &is_ub, bool &is_lb,
                    size_t &ub_index, size_t &lb_index) {
  double c = b / a;
  if (a > 0) {
    if (!is_lb || (is_lb && (c > lb))) {
      is_lb = true;
      lb = c;
      lb_index = index;
    }
  } else {
    if (!is_ub || (is_ub && (c < ub))) {
      is_ub = true;
      ub = c;
      ub_index = index;
    }
  }
}

/* ************************************************************************* */
IEConstraintManifold CartPoleWithFrictionRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices,
    IERetractInfo* retract_info) const {

  Values new_values = manifold->values().retract(delta);
  int k = Symbol(new_values.keys().front()).index();
  double q = new_values.atDouble(QKey(k));
  double v = new_values.atDouble(VKey(k));
  double a = new_values.atDouble(AKey(k));
  double a1 = cp_.mu * cp_.m * cp_.r * cos(q) + cp_.m * cp_.r * sin(q);
  double a2 = cp_.mu * cp_.m * cp_.r * cos(q) - cp_.m * cp_.r * sin(q);
  double b1 = -cp_.mu * (cp_.M + cp_.m) * cp_.g +
              cp_.mu * cp_.m * v * v * cp_.r * sin(q) -
              cp_.m * v * v * cp_.r * cos(q);
  double b2 = -cp_.mu * (cp_.M + cp_.m) * cp_.g +
              cp_.mu * cp_.m * v * v * cp_.r * sin(q) +
              cp_.m * v * v * cp_.r * cos(q);
  double ub, lb;
  bool is_ub = false, is_lb = false;
  size_t ub_index, lb_index;
  UpdateBoundary(a1, b1, 0, ub, lb, is_ub, is_lb, ub_index, lb_index);
  UpdateBoundary(a2, b2, 1, ub, lb, is_ub, is_lb, ub_index, lb_index);

  if (is_ub && is_lb && (lb >= ub)) {
    return retract1(manifold, delta);
  }

  IndexSet active_indices;
  if (blocking_indices) {
    if (blocking_indices->exists(0)) {
      a = b1 / a1;
    } else if (blocking_indices->exists(1)) {
      a = b2 / a2;
    }
  }

  if (is_ub && !is_lb) {
    if (a > ub) {
      a = ub;
      active_indices.insert(ub_index);
    }
  } else if (is_lb && !is_ub) {
    if (a < lb) {
      a = lb;
      active_indices.insert(lb_index);
    }
    a = std::max(lb, a);
  } else if (is_ub && is_lb && (lb <= ub)) {
    if (a < lb) {
      a = lb;
      active_indices.insert(lb_index);
    }
    if (a > ub) {
      a = ub;
      active_indices.insert(ub_index);
    }
  }

  if (blocking_indices && active_indices.size() == 0) {
    active_indices.insert(blocking_indices->begin(), blocking_indices->end());
  }
  new_values = cp_.computeValues(k, q, v, a);
  return manifold->createWithNewValues(new_values, active_indices);
}

/* ************************************************************************* */
IEConstraintManifold
CartPoleWithFrictionRetractor::retract1(const IEConstraintManifold *manifold,
                                        const VectorValues &delta) const {

  const gtdynamics::InequalityConstraints &i_constraints =
      *manifold->iConstraints();
  const gtdynamics::EqualityConstraints &e_constraints =
      *manifold->eConstraints();

  NonlinearFactorGraph graph;

  // optimize barrier
  Values new_values = manifold->values().retract(delta);
  // std::cout << "new_values\n";
  // PrintValues(new_values, 0);

  for (const Key &key : new_values.keys()) {
    SharedNoiseModel prior_noise;
    auto symb = Symbol(key);
    if (symb.chr() == 'q') {
      prior_noise = noiseModel::Isotropic::Sigma(1, 1e-2);
    } else if (symb.chr() == 'v') {
      prior_noise = noiseModel::Isotropic::Sigma(1, 1e-2);
    } else {
      prior_noise = noiseModel::Isotropic::Sigma(1, 1.0);
    }
    graph.addPrior<double>(key, new_values.atDouble(key), prior_noise);
  }
  for (const auto &constraint : e_constraints) {
    graph.add(constraint->createFactor(10.0));
  }
  for (const auto &constraint : i_constraints) {
    graph.add(constraint->createBarrierFactor(10.0));
  }

  LevenbergMarquardtParams params;
  // params.setVerbosityLM("SUMMARY");
  // params.setlambdaUpperBound(1e10);
  LevenbergMarquardtOptimizer optimizer(graph, manifold->values(), params);
  Values opt_values = optimizer.optimize();
  // std::cout << "opt_values\n";
  // PrintValues(opt_values, 0);

  // for (const auto& factor : graph) {
  //   factor->print();
  //   std::cout << "error: " << factor->error(opt_values) << "\n";
  // }

  // collect active indices
  IndexSet active_indices;
  for (size_t constraint_idx = 0; constraint_idx < i_constraints.size();
       constraint_idx++) {
    if (!i_constraints.at(constraint_idx)->feasible(opt_values)) {
      active_indices.insert(constraint_idx);
    }
  }

  // std::cout << "active indices:\n";
  // active_indices.print();

  // final optimization to make strictly feasible solution
  NonlinearFactorGraph graph1;
  for (const auto &constraint : e_constraints) {
    graph1.add(constraint->createFactor(1.0));
  }
  for (const auto &constraint_idx : active_indices) {
    graph1.add(i_constraints.at(constraint_idx)
                   ->createEqualityConstraint()
                   ->createFactor(1.0));
  }
  for (const Key &key : new_values.keys()) {
    SharedNoiseModel prior_noise;
    auto symb = Symbol(key);
    if (symb.chr() == 'q') {
      prior_noise = noiseModel::Isotropic::Sigma(1, 1.0);
      graph1.addPrior<double>(key, opt_values.atDouble(key), prior_noise);
    } else if (symb.chr() == 'v') {
      prior_noise = noiseModel::Isotropic::Sigma(1, 1.0);
      graph1.addPrior<double>(key, opt_values.atDouble(key), prior_noise);
    }
  }

  // std::cout << "optimize for feasibility\n";
  // active_indices.print("active indices\n");
  LevenbergMarquardtOptimizer optimizer1(graph1, opt_values, params);
  Values result = optimizer1.optimize();

  // std::cout << "result\n";
  // PrintValues(result, 0);

  return manifold->createWithNewValues(result, active_indices);
}

/* ************************************************************************* */
IEConstraintManifold CPBarrierRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices,
    IERetractInfo* retract_info) const {

  const gtdynamics::InequalityConstraints &i_constraints =
      *manifold->iConstraints();
  const gtdynamics::EqualityConstraints &e_constraints =
      *manifold->eConstraints();

  NonlinearFactorGraph graph;

  // optimize barrier
  Values new_values = manifold->values().retract(delta);
  auto prior_noise = noiseModel::Unit::Create(1);
  double mu = 1e2;
  for (const Key &key : new_values.keys()) {
    Symbol symb(key);
    if (symb.chr() == 'q' || symb.chr() == 'v' || symb.chr() == 'a') {
      graph.addPrior<double>(key, new_values.atDouble(key), prior_noise);
    }
  }
  for (const auto &constraint : e_constraints) {
    graph.add(constraint->createFactor(mu));
  }
  for (size_t idx = 0; idx < i_constraints.size(); idx++) {
    const auto &constraint = i_constraints.at(idx);
    if (blocking_indices && blocking_indices->exists(idx)) {
      graph.add(constraint->createL2Factor(mu));
    } else {
      graph.add(constraint->createBarrierFactor(mu));
    }
  }

  LevenbergMarquardtParams params;
  // params.setVerbosityLM("SUMMARY");
  // params.setlambdaUpperBound(1e10);
  LevenbergMarquardtOptimizer optimizer(graph, manifold->values(), params);
  Values opt_values = optimizer.optimize();

  // collect active indices
  IndexSet active_indices;
  if (blocking_indices) {
    active_indices = *blocking_indices;
  }
  for (size_t idx = 0; idx < i_constraints.size(); idx++) {
    if (!i_constraints.at(idx)->feasible(opt_values)) {
      active_indices.insert(idx);
    }
  }

  // final optimization to make strictly feasible solution
  NonlinearFactorGraph graph1;
  for (const auto &constraint : e_constraints) {
    graph1.add(constraint->createFactor(1.0));
  }
  for (const auto &constraint_idx : active_indices) {
    graph1.add(i_constraints.at(constraint_idx)->createL2Factor(1.0));
  }
  // std::cout << "optimize for feasibility\n";
  // active_indices.print("active indices\n");
  LevenbergMarquardtOptimizer optimizer1(graph1, opt_values, params);
  Values result = optimizer1.optimize();

  return manifold->createWithNewValues(result, active_indices);
}

} // namespace gtsam
