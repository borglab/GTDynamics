/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  OptimizationBenchmark.h
 * @brief Helper functions for benchmarking constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include "gtdynamics/imanifold/IELMOptimizerState.h"
#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtdynamics/optimizer/BarrierOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>
#include <iostream>
#include <ostream>

using gtsam::LevenbergMarquardtParams;
using gtsam::Values;

namespace gtsam {

/** Equality-constrained optimization problem, in the form of
 * argmin_x ||f(X)||^2
 * s.t.     h(X) = 0
 *          g(X) >= 0
 * where X represents the variables, ||f(X)||^2 represents the quadratic cost
 * functions, h(X)=0 represents the constraints.
 */
struct IEConsOptProblem : public gtdynamics::EqConsOptProblem {
  gtdynamics::InequalityConstraints i_constraints_;

  /// Constructor.
  IEConsOptProblem(const gtsam::NonlinearFactorGraph &costs,
                   const gtdynamics::EqualityConstraints &e_constraints,
                   const gtdynamics::InequalityConstraints &i_constraints,
                   const gtsam::Values &values)
      : EqConsOptProblem(costs, e_constraints, values),
        i_constraints_(i_constraints) {}

  const gtdynamics::EqualityConstraints &eConstraints() const {
    return constraints();
  }
  const gtdynamics::InequalityConstraints &iConstraints() const {
    return i_constraints_;
  }

  /// Evaluate the dimension of constriants.
  size_t eConstraintsDimension() const {
    return EqConsOptProblem::constraintsDimension();
  }

  /// Evaluate the constraint violation (as L2 norm).
  double evaluateEConstraintViolationL2Norm(const gtsam::Values &values) const {
    return EqConsOptProblem::evaluateConstraintViolationL2Norm(values);
  }

  /// Return a graph of merit factors of constraints.
  double evaluateIConstraintViolationL2Norm(const gtsam::Values &values) const {
    double violation = 0;
    for (const auto &constraint : i_constraints_) {
      violation += pow(constraint->toleranceScaledViolation(values), 2);
    }
    return sqrt(violation);
  }

  virtual gtsam::NonlinearFactorGraph
  constraintsGraph(double mu = 1.0) const override {
    gtsam::NonlinearFactorGraph graph = EqConsOptProblem::constraintsGraph(mu);
    for (const auto &constraint : i_constraints_) {
      graph.add(constraint->createBarrierFactor(mu));
    }
    return graph;
  }
};

struct IEIterSummary {
  size_t num_inner_iters;
  double cost;
  double e_violation;
  double i_violation;
};

struct IEResultSummary {
  std::string exp_name;
  size_t variable_dim;
  size_t factor_dim;
  size_t total_inner_iters;
  size_t total_iters;
  double cost;
  double e_violation;
  double i_violation;
  std::vector<IEIterSummary> iters_summary;

  void printLatex(std::ostream &latex_os) const;

  void exportFile(const std::string &file_path) const;
};

typedef std::vector<internal::LevenbergMarquardtState> LMItersDetail;

struct BarrierIterDetail {
  double mu;
  Values values;
  BarrierIterDetail(const double &_mu, const Values &_values)
      : mu(_mu), values(_values) {}
};

typedef std::vector<BarrierIterDetail> BarrierItersDetail;

/** Run optimization using soft constraints, e.g., treating constraints as
 * costs.
 */
std::pair<IEResultSummary, LMItersDetail> OptimizeSoftConstraints(
    const IEConsOptProblem &problem,
    LevenbergMarquardtParams lm_params = LevenbergMarquardtParams(),
    double mu = 100);

/** Run constrained optimization using the penalty method. */
std::pair<IEResultSummary, BarrierItersDetail>
OptimizeBarrierMethod(const IEConsOptProblem &problem,
                      const gtdynamics::BarrierParameters &params);

/** Run constrained optimization using the Augmented Lagrangian method. */
std::pair<IEResultSummary, IEGDItersDetails>
OptimizeIEGD(const IEConsOptProblem &problem, const gtsam::GDParams &params,
             const IEConstraintManifold::Params::shared_ptr &iecm_params);

/** Run constrained optimization using the Augmented Lagrangian method. */
std::pair<IEResultSummary, IELMItersDetails>
OptimizeIELM(const IEConsOptProblem &problem,
             const gtsam::LevenbergMarquardtParams &params,
             const gtsam::IELMParams &ie_params,
             const IEConstraintManifold::Params::shared_ptr &iecm_params);

} // namespace gtsam
