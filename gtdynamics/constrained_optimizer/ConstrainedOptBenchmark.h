/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstrainedOptBenchmark.h
 * @brief Shared benchmark runner and CLI utilities for constrained optimization
 * examples.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtdynamics/cmopt/NonlinearMOptimizer.h>
#include <gtsam/base/timing.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
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

/// Common benchmark runner to remove duplicated orchestration in examples.
class ConstrainedOptBenchmark {
 public:
  /// Supported optimization methods in constrained benchmark runs.
  enum class Method { SOFT, PENALTY, AUGMENTED_LAGRANGIAN, CM_F, CM_I };

  /// Runtime options shared by constrained benchmark examples.
  struct Options {
    /// Identifier used as filename prefix and benchmark row label.
    std::string id = "benchmark";
    /// Optional explicit CSV output path (defaults under kDataPath when empty).
    std::string csvPath;
    /// Set of optimization methods to execute for one benchmark run.
    std::set<Method> methods = {Method::SOFT, Method::PENALTY,
                                Method::AUGMENTED_LAGRANGIAN, Method::CM_F,
                                Method::CM_I};
    /// If true, enable summary logging for outer optimizers.
    bool verbose = false;
    /// If true, enable summary logging for manifold retractor optimizers.
    bool verboseRetractor = false;
    /// Soft-constraint penalty weight.
    double softMu = 1.0;
    /// Scale factor applied to reported constraint violation norms.
    double constraintUnitScale = 1.0;
    /// Retractor max iterations for CM(F).
    size_t cmFRetractorMaxIterations = 10;
    /// Retractor max iterations for CM(I).
    size_t cmIRetractorMaxIterations = 1;
    /// Whether CM(I) should apply a final projection/retraction.
    bool cmIRetractFinal = true;
  };

  /// Defaults for CLI parsing shared by benchmark examples.
  struct CliDefaults {
    /// Default identifier used when not provided from CLI.
    std::string id = "benchmark";
    /// Default methods to run when --methods is not specified.
    std::set<Method> defaultMethods = {Method::SOFT, Method::PENALTY,
                                       Method::AUGMENTED_LAGRANGIAN,
                                       Method::CM_F, Method::CM_I};
    /// Whether parser accepts --num-steps.
    bool enableNumSteps = false;
    /// Default --num-steps value when enabled.
    size_t defaultNumSteps = 0;
  };

  /// Parsed CLI options shared by benchmark examples.
  struct ParsedCli {
    /// Effective runtime options.
    Options runOptions;
    /// Effective number of steps (meaningful only when enabled by defaults).
    size_t numSteps = 0;
    /// Unrecognized args preserved for example-specific parsing/validation.
    std::vector<std::string> unknownArgs;
  };

  using ProblemFactory = std::function<EConsOptProblem()>;
  using LmConfig =
      std::function<void(Method method, LevenbergMarquardtParams*)>;
  using MoptFactory = std::function<ManifoldOptimizerParameters(Method method)>;
  using ResultCallback =
      std::function<void(Method method, const Values& result)>;

  /// Create a benchmark runner with selected runtime options.
  explicit ConstrainedOptBenchmark(Options options);

  /// Set factory used to create a fresh optimization problem per method run.
  void setProblemFactory(ProblemFactory factory);

  /// Set baseline outer LM parameters before per-method customization.
  void setOuterLmBaseParams(LevenbergMarquardtParams params);

  /// Set callback to customize outer LM parameters per method.
  void setOuterLmConfig(LmConfig callback);

  /// Set callback to construct manifold optimizer parameters per CM method.
  void setMoptFactory(MoptFactory factory);

  /// Set callback invoked after each method completes with resulting values.
  void setResultCallback(ResultCallback callback);

  /// Run selected methods and append rows to CSV.
  void run(std::ostream& latexOs);

  /// Create default CM parameters using unconstrained retraction + dense basis.
  static ManifoldOptimizerParameters DefaultMoptParams();

  /// Create default CM parameters using basis-key elimination + basis
  /// retraction.
  static ManifoldOptimizerParameters DefaultMoptParamsSV(
      const BasisKeyFunc& basisKeyFunc);

  /// Print common CLI usage flags for benchmark examples.
  static void PrintUsage(std::ostream& os, const char* programName,
                         const CliDefaults& defaults);

  /// Parse common CLI arguments and preserve unknown args for example parsing.
  static ParsedCli ParseCli(int argc, char** argv, const CliDefaults& defaults);

  /// Return the canonical short token for a method.
  static std::string MethodToken(Method method);

  /// Return the display label used in benchmark output.
  static std::string MethodLabel(Method method);

  /// Build a deterministic output file name under kDataPath for a method.
  static std::string MethodDataPath(const Options& options, Method method,
                                    const std::string& suffix);

 private:
  static Values OptimizeSoftConstraints(const EConsOptProblem& problem,
                                        std::ostream& latexOs,
                                        LevenbergMarquardtParams lmParams,
                                        double mu, double constraintUnitScale);

  static Values OptimizeCmOpt(const EConsOptProblem& problem,
                              std::ostream& latexOs,
                              ManifoldOptimizerParameters moptParams,
                              LevenbergMarquardtParams lmParams,
                              const std::string& expName,
                              double constraintUnitScale);

  static Values OptimizePenalty(
      const EConsOptProblem& problem, std::ostream& latexOs,
      gtsam::PenaltyOptimizerParams::shared_ptr params,
      double constraintUnitScale);

  static Values OptimizeAugmentedLagrangian(
      const EConsOptProblem& problem, std::ostream& latexOs,
      gtsam::AugmentedLagrangianParams::shared_ptr params,
      double constraintUnitScale);

  Options options_;
  ProblemFactory problemFactory_;
  LevenbergMarquardtParams outerLmParams_;
  LmConfig outerLmConfig_;
  MoptFactory moptFactory_;
  ResultCallback resultCallback_;
};

}  // namespace gtdynamics
