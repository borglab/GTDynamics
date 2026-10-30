/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file CartPoleWithFrictionRetractor.cpp
 * @brief CartPoleWithFrictionRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/scenarios/CartPoleWithFrictionRetractor.h>
#include <gtdynamics/scenarios/IECartPoleWithFriction.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <iomanip>

namespace gtdynamics {
using namespace gtsam;

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
    IERetractionInfo *retract_info) const {
  Values new_values = manifold->values().retract(delta);
  int k = Symbol(new_values.keys().front()).index();
  double q = new_values.atDouble(QKey(k));
  double v = new_values.atDouble(VKey(k));
  double a = new_values.atDouble(AKey(k));
  double a1 = cp_.friction_coefficient_ * cp_.mass_ * cp_.length_ * cos(q) +
              cp_.mass_ * cp_.length_ * sin(q);
  double a2 = cp_.friction_coefficient_ * cp_.mass_ * cp_.length_ * cos(q) -
              cp_.mass_ * cp_.length_ * sin(q);
  double b1 =
      -cp_.friction_coefficient_ * (cp_.cart_mass_ + cp_.mass_) * cp_.gravity_ +
      cp_.friction_coefficient_ * cp_.mass_ * v * v * cp_.length_ * sin(q) -
      cp_.mass_ * v * v * cp_.length_ * cos(q);
  double b2 =
      -cp_.friction_coefficient_ * (cp_.cart_mass_ + cp_.mass_) * cp_.gravity_ +
      cp_.friction_coefficient_ * cp_.mass_ * v * v * cp_.length_ * sin(q) +
      cp_.mass_ * v * v * cp_.length_ * cos(q);
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
  const NonlinearInequalityConstraints &i_constraints =
      *manifold->iConstraints();
  const NonlinearEqualityConstraints &e_constraints = *manifold->eConstraints();

  NonlinearFactorGraph graph;

  // optimize penalty
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
    graph.add(constraint->penaltyFactor(10.0));
  }
  for (const auto &constraint : i_constraints) {
    graph.add(constraint->penaltyFactor(10.0));
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
    graph1.add(constraint->penaltyFactor(1.0));
  }
  for (const auto &constraint_idx : active_indices) {
    graph1.add(i_constraints.at(constraint_idx)
                   ->createEqualityConstraint()
                   ->penaltyFactor(1.0));
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

} // namespace gtdynamics
