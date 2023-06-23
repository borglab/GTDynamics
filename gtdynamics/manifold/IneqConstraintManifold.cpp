#include "manifold/ConnectedComponent.h"
#include "optimizer/EqualityConstraint.h"
#include <gtdynamics/manifold/IneqConstraintManifold.h>
#include <gtdynamics/manifold/Retractor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

double point3_z(const Point3& p, OptionalJacobian<1, 3> H) {
  if (H) {
    *H << 0, 0, 1;
  }
  return p.z();
}

/// dot product of VectorValues with different structures
double vv_dot(const VectorValues &v1, const VectorValues v2) {
  double sum = 0;
  for (const auto &it : v1) {
    if (v2.find(it.first) != v2.end()) {
      sum += it.second.dot(v2.at(it.first));
    }
  }
  return sum;
}

std::pair<IndexSet, VectorValues>
IneqConstraintManifold::identifyBlockingConstraints(
    const VectorValues &delta) const {
  // TODO: implement algorithm 16.3 in Nocedal

  // linearize active constraints
  std::map<size_t, VectorValues> active_jacobians;
  for (const auto &constraint_idx : active_indices_) {
    auto factor = i_constraints_.at(constraint_idx)
                      ->createEqualityConstraint()
                      ->createFactor(1.0);
    auto linear_factor = factor->linearize(values_);
    auto jacobian = linear_factor->jacobian().first;
    VectorValues jacobian_vv;
    size_t slot = 0;
    for (auto it = linear_factor->begin(); it != linear_factor->end(); it++) {
      size_t dim = linear_factor->getDim(it);
      jacobian_vv.insert(*it, jacobian.block(0, slot, 1, dim).transpose());
      slot += dim;
    }
    active_jacobians.insert(std::make_pair(constraint_idx, jacobian_vv));
  }

  VectorValues g = delta;
  IndexSet blocking_constraints;
  while (true) {
    // Check blocking constraints
    double max_violation = 0;
    size_t max_vio_idx = 0;
    for (const auto &constraint_idx : active_indices_) {
      if (blocking_constraints.find(constraint_idx) ==
          blocking_constraints.end()) {
        double violation =
            std::max(0.0, -vv_dot(active_jacobians.at(constraint_idx), g));
        if (violation > max_violation) {
          max_violation = violation;
          max_vio_idx = constraint_idx;
        }
      }
    }
    if (max_violation == 0) {
      return std::make_pair(blocking_constraints, g);
    }

    // add blocking constriants and solve
    blocking_constraints.insert(max_vio_idx);
    GaussianFactorGraph graph;
    // prior for each
    for (const auto &it : delta) {
      size_t dim = it.second.size();
      graph.add(it.first, it.second, Matrix::Identity(dim, dim),
                noiseModel::Unit::Create(dim));
    }
    // constraint
    for (const auto &constraint_idx : blocking_constraints) {
      auto factor = i_constraints_.at(constraint_idx)
                        ->createEqualityConstraint()
                        ->createFactor(1.0);
      auto constraint_factor =
          factor->cloneWithNewNoiseModel(noiseModel::Constrained::All(1));
      auto linear_factor = constraint_factor->linearize(values_);
      graph.add(linear_factor);
    }

    g = graph.optimize();
  }
}

/** Retraction of the constraint manifold, e.g., retraction required for gtsam
 * manifold type. Note: Jacobians are set as zero since they are not required
 * for optimization. */
IneqConstraintManifold::shared_ptr
IneqConstraintManifold::retract(const gtsam::VectorValues &delta,
                                const IndexSet &tight_indices) const {
  // retract base values
  Values values = values_.retract(delta);
  IndexSet active_indices = tight_indices;

  while (true) {
    // retract by minimizing tight constraints
    if (active_indices.size() > 0) {
      NonlinearFactorGraph graph;
      for (const auto &constraint_idx : active_indices) {
        graph.add(i_constraints_.at(constraint_idx)->createL2Factor(1.0));
      }
      Values opt_values;
      for (const Key &key : graph.keys()) {
        opt_values.insert(key, values.at(key));
      }
      LevenbergMarquardtParams params;
      params.setVerbosityLM("SUMMARY");
      params.setlambdaUpperBound(1e10);
      auto optimizer = LevenbergMarquardtOptimizer(graph, opt_values, params);
      opt_values = optimizer.optimize();
      values.update(opt_values);
    }

    // check other constriant violation
    double max_violation = 0;
    size_t max_vio_idx = 0;
    for (size_t constraint_idx = 0; constraint_idx < i_constraints_.size();
         constraint_idx++) {
      if (active_indices.find(constraint_idx) == active_indices.end()) {
        const auto &constraint = i_constraints_.at(constraint_idx);
        if (!constraint->feasible(values)) {
          double violation = constraint->toleranceScaledViolation(values);
          if (violation > max_violation) {
            max_violation = violation;
            max_vio_idx = constraint_idx;
          }
        }
      }
    }

    if (max_violation == 0) {
      break;
    } else {
      active_indices.insert(max_vio_idx);
    }
  }

  return std::make_shared<IneqConstraintManifold>(i_constraints_, values,
                                                  active_indices);
}

IneqConstraintManifold::shared_ptr
IneqConstraintManifold::retractBarrier(const gtsam::VectorValues &delta,
                                       const IndexSet &tight_indices) const {
  NonlinearFactorGraph graph;

  // add priors on target value
  Values values = values_.retract(delta);
  // for (const auto &constraint : i_constraints_) {
  //   std::cout << "constriant evaluation: " << (*constraint)(values) << "\n";
  // }

  auto prior_noise = noiseModel::Unit::Create(1);
  for (const Key &key : values.keys()) {
    graph.addPrior<double>(key, values.atDouble(key), prior_noise);
  }

  // add tight constraints
  for (const auto &constraint_idx : tight_indices) {
    graph.add(i_constraints_.at(constraint_idx)->createL2Factor(1.0));
  }

  // add barrier constraints
  for (size_t constraint_idx = 0; constraint_idx < i_constraints_.size();
       constraint_idx++) {
    if (tight_indices.find(constraint_idx) == tight_indices.end()) {
      graph.add(i_constraints_.at(constraint_idx)->createBarrierFactor(1.0));
    }
  }

  // optimize
  // std::cout << "optimize with barriers\n";
  LevenbergMarquardtParams params;
  // params.setVerbosityLM("SUMMARY");
  params.setlambdaUpperBound(1e10);
  LevenbergMarquardtOptimizer optimizer(graph, values_, params);
  Values opt_values = optimizer.optimize();
  // opt_values.print();

  // collect active indices
  IndexSet active_indices = tight_indices;
  for (size_t constraint_idx = 0; constraint_idx < i_constraints_.size();
       constraint_idx++) {
    if (!i_constraints_.at(constraint_idx)->feasible(opt_values)) {
      active_indices.insert(constraint_idx);
    }
  }

  // final optimization to make strictly feasible solution
  NonlinearFactorGraph graph1;
  for (const auto &constraint_idx : active_indices) {
    graph1.add(i_constraints_.at(constraint_idx)
                   ->createEqualityConstraint()
                   ->createFactor(1.0));
  }
  // std::cout << "optimize for feasibility\n";
  // active_indices.print("active indices\n");
  LevenbergMarquardtOptimizer optimizer1(graph1, opt_values, params);
  Values new_values = optimizer1.optimize();

  return std::make_shared<IneqConstraintManifold>(i_constraints_, new_values,
                                                  active_indices);
}

gtsam::Values
retractEqualities(const gtdynamics::EqualityConstraints &constraints,
                  const gtsam::Values &values, const VectorValues &delta) {
  auto component = std::make_shared<ConnectedComponent>(constraints);
  auto retractor_params = std::make_shared<RetractParams>();
  auto retractor = Retractor::create(retractor_params, component);

  Values new_values = retractor->retract(values, delta);
  return new_values;
}

gtsam::Values
retractEqualities(const gtdynamics::EqualityConstraints &constraints,
                  const gtsam::Values &values) {
  auto component = std::make_shared<ConnectedComponent>(constraints);
  auto retractor_params = std::make_shared<RetractParams>();
  auto retractor = Retractor::create(retractor_params, component);

  Values new_values = retractor->retractConstraints(values);
  return new_values;
}

/// Retract using barrier
IneqConstraintManifold::shared_ptr
IneqConstraintManifold::retractLineSearch(const gtsam::VectorValues &delta,
                                          const IndexSet &tight_indices) const {

  double scale = 1.0;
  double alpha = 0.8;

  gtdynamics::EqualityConstraints e_constraints;
  for (const auto &constraint_idx : tight_indices) {
    e_constraints.emplace_back(
        i_constraints_.at(constraint_idx)->createEqualityConstraint());
  }

  int blocking_idx = -1;
  Values values;
  while (true) {
    values = retractEqualities(e_constraints, values_, scale * delta);

    values.print();

    // check all other constraints are feasible
    bool feasible = true;
    double max_violation = 0;
    for (size_t i = 0; i < i_constraints_.size(); i++) {
      if (tight_indices.find(i) == tight_indices.end()) {
        if (!i_constraints_.at(i)->feasible(values)) {
          feasible = false;
          double violation =
              i_constraints_.at(i)->toleranceScaledViolation(values);
          if (violation > max_violation) {
            blocking_idx = i;
            max_violation = violation;
          }
        }
      }
    }
    if (feasible) {
      break;
    }
    scale *= alpha;
  }

  IndexSet active_indices = tight_indices;
  if (blocking_idx != -1) {
    active_indices.insert(blocking_idx);
    e_constraints.emplace_back(
        i_constraints_.at(blocking_idx)->createEqualityConstraint());
    values = retractEqualities(e_constraints, values);
    return std::make_shared<IneqConstraintManifold>(i_constraints_, values,
                                                    active_indices);
  }
  return std::make_shared<IneqConstraintManifold>(i_constraints_, values,
                                                  active_indices);
}



Values
CollectManifoldValues(const std::vector<IneqConstraintManifold::shared_ptr> &manifolds) {
  Values values;
  for (const auto &manifold : manifolds) {
    values.insert(manifold->values());
  }
  return values;
}



} // namespace gtsam
