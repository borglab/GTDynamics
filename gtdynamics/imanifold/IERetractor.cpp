/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IERetractor.cpp
 * @brief Tagent space basis implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/imanifold/IECartPoleWithFriction.h>
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IERetractor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
IEConstraintManifold
IERetractor::moveToBoundary(const IEConstraintManifold *manifold,
                            const IndexSet &blocking_indices) const {
  VectorValues delta = manifold->values().zeroVectors();
  return retract(manifold, delta, blocking_indices);
}

/* ************************************************************************* */
IEConstraintManifold HalfSphereRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices) const {
  Key key = manifold->values().keys().front();
  Point3 p = manifold->values().at<Point3>(key);
  Vector3 v = delta.at(key);
  Point3 new_p = p + v;

  if (blocking_indices && blocking_indices->size()>0) {
    new_p.z() = 0;
  }
  new_p.z() = std::max(0.0, new_p.z());
  new_p = normalize(new_p);

  Values new_values;
  new_values.insert(key, new_p);
  return manifold->createWithNewValues(new_values);
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
    const std::optional<IndexSet> &blocking_indices) const {

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
    }
    else if (blocking_indices->exists(1)) {
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

  if (blocking_indices && active_indices.size()==0) {
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
      manifold->eCC()->constraints_;

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
IEConstraintManifold BarrierRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices) const {

  const gtdynamics::InequalityConstraints &i_constraints =
      *manifold->iConstraints();
  const gtdynamics::EqualityConstraints &e_constraints =
      manifold->eCC()->constraints_;

  NonlinearFactorGraph graph;

  // optimize barrier
  Values new_values = manifold->values().retract(delta);
  auto prior_noise = noiseModel::Unit::Create(1);
  for (const Key &key : new_values.keys()) {
    graph.addPrior<double>(key, new_values.atDouble(key), prior_noise);
  }
  for (const auto &constraint : e_constraints) {
    graph.add(constraint->createFactor(1e-1));
  }
  for (const auto &constraint : i_constraints) {
    graph.add(constraint->createBarrierFactor(1e-1));
  }

  LevenbergMarquardtParams params;
  // params.setVerbosityLM("SUMMARY");
  // params.setlambdaUpperBound(1e10);
  LevenbergMarquardtOptimizer optimizer(graph, manifold->values(), params);
  Values opt_values = optimizer.optimize();

  // collect active indices
  IndexSet active_indices;
  for (size_t constraint_idx = 0; constraint_idx < i_constraints.size();
       constraint_idx++) {
    if (!i_constraints.at(constraint_idx)->feasible(opt_values)) {
      active_indices.insert(constraint_idx);
    }
  }

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
  // std::cout << "optimize for feasibility\n";
  // active_indices.print("active indices\n");
  LevenbergMarquardtOptimizer optimizer1(graph1, opt_values, params);
  Values result = optimizer1.optimize();

  return manifold->createWithNewValues(result, active_indices);
}


/* ************************************************************************* */
IEConstraintManifold CPBarrierRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices) const {

  const gtdynamics::InequalityConstraints &i_constraints =
      *manifold->iConstraints();
  const gtdynamics::EqualityConstraints &e_constraints =
      manifold->eCC()->constraints_;

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
  for (size_t idx=0; idx<i_constraints.size(); idx++) {
    const auto& constraint = i_constraints.at(idx);
    if (blocking_indices && blocking_indices->exists(idx)) {
      graph.add(constraint->createL2Factor(mu));
    }
    else {
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
