/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstrainedOptBenchmarkIE.h
 * @brief Helper functions for benchmarking constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmcopt/IEGDOptimizer.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/constrained_optimizer/SQPOptimizer.h>
#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/base/timing.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>

#include <iostream>
#include <ostream>

#if defined(GTDYNAMICS_WITH_IFOPT)
#include <gtdynamics/constrained_optimizer/IPOptOptimizer.h>
#else
namespace gtdynamics {
using IPItersDetails = std::vector<size_t>;
}
#endif

namespace gtdynamics {

using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using EqualityConstraints = gtsam::NonlinearEqualityConstraints;
using InequalityConstraints = gtsam::NonlinearInequalityConstraints;

struct PenaltyParameters : public gtsam::PenaltyOptimizerParams {
  double &initial_mu;
  double &mu_increase_rate;
  size_t &num_iterations;
  LevenbergMarquardtParams &lm_params;
  std::vector<LevenbergMarquardtParams> iters_lm_params;
  bool store_iter_details = false;
  bool store_lm_details = false;

  PenaltyParameters()
      : gtsam::PenaltyOptimizerParams(), initial_mu(initialMuEq),
        mu_increase_rate(muEqIncreaseRate), num_iterations(maxIterations),
        lm_params(lmParams) {}
};

struct AugmentedLagrangianParameters : public gtsam::AugmentedLagrangianParams {
  double &initial_mu_e;
  double &initial_mu_i;
  double &mu_increase_rate;
  double &mu_increase_threshold;
  size_t &num_iterations;
  double &dual_step_size_factor_e;
  double &dual_step_size_factor_i;
  double &max_dual_step_size_e;
  double &max_dual_step_size_i;
  LevenbergMarquardtParams &lm_params;
  bool store_iter_details = false;
  bool store_lm_details = false;

  AugmentedLagrangianParameters()
      : gtsam::AugmentedLagrangianParams(), initial_mu_e(initialMuEq),
        initial_mu_i(initialMuIneq),
        mu_increase_rate(muEqIncreaseRate),
        mu_increase_threshold(muIncreaseThreshold),
        num_iterations(maxIterations),
        dual_step_size_factor_e(dualStepSizeFactorEq),
        dual_step_size_factor_i(dualStepSizeFactorIneq),
        max_dual_step_size_e(maxDualStepSizeEq),
        max_dual_step_size_i(maxDualStepSizeIneq), lm_params(lmParams) {}
};

using PenaltyItersDetails = gtsam::PenaltyOptimizer::Progress;
using AugmentedLagrangianItersDetails =
    gtsam::AugmentedLagrangianOptimizer::Progress;

Values ProjectValues(const IEConsOptProblem &problem, const Values &values,
                     double sigma = 1e3);

void EvaluateCostTerms(std::ostream &os,
                       const std::vector<NonlinearFactorGraph> &graphs,
                       const Values &values, const Values &proj_values,
                       std::string exp_name);

/// Summary for each iteration of IE-constrained optimization.
class IEIterSummary {
 public:
  size_t accum_iters = 0;  // accumulative lm iters up to current step
  size_t accum_inner_iters = 0;
  double cost = 0;
  double e_violation = 0;
  double i_violation = 0;
  double projected_cost = 0;

  // Default constructor.
  IEIterSummary() {}

  // Constructor.
  IEIterSummary(const IEConsOptProblem &problem, const Values &values,
                const size_t _accum_iters = 0,
                const size_t _accum_inner_iters = 0,
                bool eval_projected_cost = true)
      : accum_iters(_accum_iters), accum_inner_iters(_accum_inner_iters) {
    evaluate(problem, values, eval_projected_cost);
  }

  // evaluate cost, constraint violation, and projected cost.
  void evaluate(const IEConsOptProblem &problem, const Values &values,
                bool eval_projected_cost = true);
};

class IEItersSummary : public std::vector<IEIterSummary> {
 public:
  using Base = std::vector<IEIterSummary>;
  using Base::Base;

  void addAccumulative(const IEConsOptProblem &problem, const Values &values,
                       const size_t step_lm_iters,
                       const size_t step_lm_inner_iters,
                       bool eval_projected_cost = true);
};

/// Summary of IE-constrained optimization result.
class IEResultSummary {
 public:
  std::string exp_name;
  size_t variable_dim;
  size_t factor_dim;
  size_t total_inner_iters;
  size_t total_iters;
  Values values;
  Values projected_values;
  double cost;
  double e_violation;
  double i_violation;
  double projected_cost;
  double optimization_time;
  IEItersSummary iters_summary;

  void evaluate(const IEConsOptProblem &problem);

  void printLatex(std::ostream &latex_os) const;

  void exportFile(const std::string &file_path) const;
};

typedef std::vector<gtsam::internal::LevenbergMarquardtState> LMItersDetail;

/** Run optimization using soft constraints, e.g., treating constraints as
 * costs.
 */
std::pair<IEResultSummary, LMItersDetail> OptimizeIE_Soft(
    const IEConsOptProblem &problem,
    LevenbergMarquardtParams lm_params = LevenbergMarquardtParams(),
    double mu = 100, bool eval_projected_cost = true);

/** Run constrained optimization using the penalty method. */
std::pair<IEResultSummary, PenaltyItersDetails> OptimizeIE_Penalty(
    const IEConsOptProblem &problem,
    const gtsam::PenaltyOptimizerParams::shared_ptr &params,
    bool eval_projected_cost = true);

/** Run constrained optimization using augmented Lagrangian method. */
std::pair<IEResultSummary, AugmentedLagrangianItersDetails>
OptimizeIE_AugmentedLagrangian(
    const IEConsOptProblem &problem,
    const gtsam::AugmentedLagrangianParams::shared_ptr &params,
    bool eval_projected_cost = true);

/** Run SQP method. */
std::pair<IEResultSummary, SQPItersDetails> OptimizeIE_SQP(
    const IEConsOptProblem &problem, const SQPParams::shared_ptr &params,
    bool eval_projected_cost = true);

std::pair<IEResultSummary, IPItersDetails> OptimizeIE_IPOPT(
    const IEConsOptProblem &problem, bool eval_projected_cost = true);

/** Run e-manifold optimization, with added penalty for i-constraints. */
std::pair<IEResultSummary, IELMItersDetails> OptimizeIE_CMOpt(
    const IEConsOptProblem &problem, const IELMParams &ielm_params,
    const IEConstraintManifold::Params::shared_ptr &iecm_params,
    double mu = 100, bool eval_projected_cost = true);

/** Run constrained optimization using the Augmented Lagrangian method. */
std::pair<IEResultSummary, IEGDItersDetails> OptimizeIE_CMCOptGD(
    const IEConsOptProblem &problem, const GDParams &params,
    const IEConstraintManifold::Params::shared_ptr &iecm_params,
    bool eval_projected_cost = true);

/** Run constrained optimization using the Augmented Lagrangian method. */
std::pair<IEResultSummary, IELMItersDetails> OptimizeIE_CMCOptLM(
    const IEConsOptProblem &problem, const IELMParams &ielm_params,
    const IEConstraintManifold::Params::shared_ptr &iecm_params,
    std::string exp_name = "CMOpt(IE)", bool eval_projected_cost = true);

}  // namespace gtdynamics
