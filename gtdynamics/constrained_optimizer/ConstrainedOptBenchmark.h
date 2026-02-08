/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstrainedOptBenchmark.h
 * @brief Helper functions for benchmarking constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtdynamics/cmopt/NonlinearMOptimizer.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Values.h>

#include <functional>
#include <iostream>
#include <ostream>
#include <set>
#include <string>
#include <vector>

namespace gtdynamics {

using gtsam::LevenbergMarquardtParams;
using gtsam::Values;

/** Supported optimization methods in constrained benchmark runs. */
enum class BenchmarkMethod { SOFT, PENALTY, AUGMENTED_LAGRANGIAN, CM_F, CM_I };

/** Runtime options shared by constrained benchmark examples. */
struct BenchmarkRunOptions {
  std::string benchmark_id = "benchmark";
  std::string benchmark_csv_path;
  std::set<BenchmarkMethod> methods = {BenchmarkMethod::SOFT,
                                       BenchmarkMethod::PENALTY,
                                       BenchmarkMethod::AUGMENTED_LAGRANGIAN,
                                       BenchmarkMethod::CM_F,
                                       BenchmarkMethod::CM_I};
  bool verbose_benchmark = false;
  bool verbose_retractor = false;
  double soft_mu = 1.0;
  double constraint_unit_scale = 1.0;
  size_t cm_f_retractor_max_iterations = 10;
  size_t cm_i_retractor_max_iterations = 1;
  bool cm_i_retract_final = true;
};

/** CLI defaults for benchmark examples. */
struct BenchmarkCliDefaults {
  std::string benchmark_id = "benchmark";
  std::set<BenchmarkMethod> default_methods = {BenchmarkMethod::SOFT,
                                               BenchmarkMethod::PENALTY,
                                               BenchmarkMethod::AUGMENTED_LAGRANGIAN,
                                               BenchmarkMethod::CM_F,
                                               BenchmarkMethod::CM_I};
  bool enable_num_steps = false;
  size_t default_num_steps = 0;
};

/** Parsed CLI options shared by benchmark examples. */
struct ParsedBenchmarkCli {
  BenchmarkRunOptions run_options;
  size_t num_steps = 0;
  std::vector<std::string> unknown_args;
};

/** Print common benchmark usage flags. */
void PrintBenchmarkUsage(std::ostream& os, const char* program_name,
                         const BenchmarkCliDefaults& defaults);

/** Parse common benchmark CLI arguments, keeping unknown args for examples. */
ParsedBenchmarkCli ParseBenchmarkCli(int argc, char** argv,
                                     const BenchmarkCliDefaults& defaults);

/** Return the canonical short token for a benchmark method. */
std::string BenchmarkMethodToken(BenchmarkMethod method);

/** Return the display label used in benchmark output. */
std::string BenchmarkMethodLabel(BenchmarkMethod method);

/** Build a deterministic output file name under kDataPath for a method. */
std::string BenchmarkMethodDataPath(const BenchmarkRunOptions& options,
                                    BenchmarkMethod method,
                                    const std::string& suffix);

/** Common benchmark runner to remove duplicated orchestration in examples. */
class ConstrainedOptBenchmarkRunner {
 public:
  using ProblemFactory = std::function<EConsOptProblem()>;
  using LmConfig =
      std::function<void(BenchmarkMethod method, LevenbergMarquardtParams*)>;
  using MoptFactory =
      std::function<ManifoldOptimizerParameters(BenchmarkMethod method)>;
  using ResultCallback =
      std::function<void(BenchmarkMethod method, const Values& result)>;

  explicit ConstrainedOptBenchmarkRunner(BenchmarkRunOptions options);

  void setProblemFactory(ProblemFactory factory);
  void setOuterLmBaseParams(LevenbergMarquardtParams params);
  void setOuterLmConfig(LmConfig callback);
  void setMoptFactory(MoptFactory factory);
  void setResultCallback(ResultCallback callback);

  /** Run selected methods and append benchmark rows to CSV. */
  void run(std::ostream& latex_os);

 private:
  BenchmarkRunOptions options_;
  ProblemFactory problem_factory_;
  LevenbergMarquardtParams outer_lm_params_;
  LmConfig outer_lm_config_;
  MoptFactory mopt_factory_;
  ResultCallback result_callback_;
};

/// Default parameters for manifold optimization.
ManifoldOptimizerParameters DefaultMoptParams();

/// Default parameters for manifold optimization, with basis constructed by
/// specifying variables.
ManifoldOptimizerParameters
DefaultMoptParamsSV(const BasisKeyFunc &basis_key_func);

/** Run optimization using soft constraints, e.g., treating constraints as
 * costs.
 */
Values OptimizeE_SoftConstraints(
    const EConsOptProblem &problem, std::ostream &latex_os,
    LevenbergMarquardtParams lm_params = LevenbergMarquardtParams(),
    double mu = 100, double constraint_unit_scale = 1.0);

/** Run optimization using constraint manifold. */
Values
OptimizeE_CMOpt(const EConsOptProblem &problem, std::ostream &latex_os,
                ManifoldOptimizerParameters mopt_params = DefaultMoptParams(),
                LevenbergMarquardtParams lm_params = LevenbergMarquardtParams(),
                std::string exp_name = "Constraint Manifold",
                double constraint_unit_scale = 1.0);

/** Run constrained optimization using the penalty method. */
Values OptimizeE_Penalty(const EConsOptProblem &problem, std::ostream &latex_os,
                         gtsam::PenaltyOptimizerParams::shared_ptr params =
                             std::make_shared<gtsam::PenaltyOptimizerParams>(),
                         double constraint_unit_scale = 1.0);

/** Run constrained optimization using the Augmented Lagrangian method. */
Values OptimizeE_AugmentedLagrangian(
    const EConsOptProblem &problem, std::ostream &latex_os,
    gtsam::AugmentedLagrangianParams::shared_ptr params =
        std::make_shared<gtsam::AugmentedLagrangianParams>(),
    double constraint_unit_scale = 1.0);

} // namespace gtdynamics
