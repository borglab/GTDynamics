/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/* Constrained optimization benchmark implementations. */

#include <gtdynamics/cmopt/Retractor.h>
#include <gtdynamics/cmopt/TspaceBasis.h>
#include <gtdynamics/config.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gtdynamics {
namespace {

constexpr const char* kBenchmarkCsvEnv = "GTDYN_BENCHMARK_CSV";
constexpr const char* kBenchmarkIdEnv = "GTDYN_BENCHMARK_ID";

std::string CsvEscape(const std::string& value) {
  std::string escaped = "\"";
  for (char ch : value) {
    if (ch == '"')
      escaped += "\"\"";
    else
      escaped += ch;
  }
  escaped += "\"";
  return escaped;
}

std::string ToLower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

std::string Trim(const std::string& text) {
  const auto first = text.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) return "";
  const auto last = text.find_last_not_of(" \t\n\r");
  return text.substr(first, last - first + 1);
}

std::vector<std::string> Split(const std::string& text, char delim) {
  std::vector<std::string> parts;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, delim)) parts.push_back(item);
  return parts;
}

std::vector<BenchmarkMethod> OrderedMethods() {
  return {BenchmarkMethod::SOFT, BenchmarkMethod::PENALTY,
          BenchmarkMethod::AUGMENTED_LAGRANGIAN, BenchmarkMethod::CM_F,
          BenchmarkMethod::CM_I};
}

std::string MethodsListString(const std::set<BenchmarkMethod>& methods) {
  std::vector<std::string> tokens;
  for (BenchmarkMethod method : OrderedMethods()) {
    if (methods.count(method) == 0) continue;
    tokens.push_back(BenchmarkMethodToken(method));
  }

  std::ostringstream oss;
  for (size_t i = 0; i < tokens.size(); ++i) {
    if (i > 0) oss << ",";
    oss << tokens[i];
  }
  return oss.str();
}

std::set<BenchmarkMethod> ParseMethods(const std::string& methodsText) {
  std::set<BenchmarkMethod> methods;
  for (const auto& tokenRaw : Split(methodsText, ',')) {
    const std::string token = ToLower(Trim(tokenRaw));
    if (token.empty()) continue;
    if (token == "all") {
      const auto methods = OrderedMethods();
      return std::set<BenchmarkMethod>(methods.begin(), methods.end());
    }
    if (token == "soft") {
      methods.insert(BenchmarkMethod::SOFT);
      continue;
    }
    if (token == "penalty") {
      methods.insert(BenchmarkMethod::PENALTY);
      continue;
    }
    if (token == "alm" || token == "augmented_lagrangian") {
      methods.insert(BenchmarkMethod::AUGMENTED_LAGRANGIAN);
      continue;
    }
    if (token == "f" || token == "cmf") {
      methods.insert(BenchmarkMethod::CM_F);
      continue;
    }
    if (token == "i" || token == "cmi") {
      methods.insert(BenchmarkMethod::CM_I);
      continue;
    }
    throw std::invalid_argument("Unknown method token: " + tokenRaw);
  }

  if (methods.empty()) {
    throw std::invalid_argument("No valid methods selected.");
  }

  return methods;
}

std::filesystem::path EnsureDataPath() {
  const std::filesystem::path path(kDataPath);
  std::filesystem::create_directories(path);
  return path;
}

std::string DefaultBenchmarkCsvPath(
    const ConstrainedOptBenchmark::Options& options) {
  const std::filesystem::path path =
      EnsureDataPath() / (options.id + "_benchmark.csv");
  return path.string();
}

class ScopedEnvVar {
 public:
  ScopedEnvVar(const char* key, std::string value) : key_(key) {
    const char* existing = std::getenv(key_);
    if (existing) {
      hadOldValue_ = true;
      oldValue_ = existing;
    }
    setenv(key_, value.c_str(), 1);
  }

  ~ScopedEnvVar() {
    if (hadOldValue_) {
      setenv(key_, oldValue_.c_str(), 1);
    } else {
      unsetenv(key_);
    }
  }

 private:
  const char* key_;
  bool hadOldValue_ = false;
  std::string oldValue_;
};

void AppendBenchmarkCsv(const std::string& method, size_t fDim, size_t vDim,
                        double timeS, size_t numIters, double constraintL2,
                        double cost) {
  const char* csvPath = std::getenv(kBenchmarkCsvEnv);
  if (!csvPath || std::string(csvPath).empty()) return;

  const char* benchNameEnv = std::getenv(kBenchmarkIdEnv);
  std::string benchName = benchNameEnv ? benchNameEnv : "";

  bool writeHeader = true;
  {
    std::ifstream in(csvPath);
    writeHeader = !in.good() || in.peek() == std::ifstream::traits_type::eof();
  }

  std::ofstream out(csvPath, std::ios::app);
  if (!out) return;

  if (writeHeader) {
    out << "benchmark,method,f_dim,v_dim,time_s,iters,constraint_l2,cost\n";
  }
  out << CsvEscape(benchName) << "," << CsvEscape(method) << "," << fDim << ","
      << vDim << "," << std::setprecision(12) << timeS << "," << numIters << ","
      << std::scientific << std::setprecision(12) << constraintL2 << ","
      << std::defaultfloat << std::setprecision(12) << cost << "\n";
}

void PrintLatex(std::ostream& latexOs, const std::string& expName, size_t fDim,
                size_t vDim, double time, size_t numIters, double constraintVio,
                double cost) {
  std::cout << "[BENCH] " << expName << ": f_dim=" << fDim << ", v_dim=" << vDim
            << ", time_s=" << std::setprecision(6) << std::defaultfloat << time
            << ", iters=" << numIters << ", constraint_l2=" << std::scientific
            << std::setprecision(3) << constraintVio
            << ", cost=" << std::defaultfloat << std::setprecision(6) << cost
            << "\n";
  latexOs << "& " + expName + " & $" << fDim << " \\times " << vDim << "$ & "
          << std::setprecision(4) << time << std::defaultfloat << " & "
          << numIters << " & " << std::scientific << std::setprecision(2)
          << constraintVio << std::defaultfloat << " & " << std::fixed
          << std::setprecision(2) << cost << std::defaultfloat << "\\\\\n";
  AppendBenchmarkCsv(expName, fDim, vDim, time, numIters, constraintVio, cost);
}

}  // namespace

void PrintBenchmarkUsage(std::ostream& os, const char* programName,
                         const BenchmarkCliDefaults& defaults) {
  os << "Usage: " << programName << " [benchmark options] [example options]\n"
     << "Benchmark options:\n"
     << "  --methods LIST            Comma list from "
        "{soft,penalty,alm,f,i,all}.\n"
     << "  --benchmark-id NAME       Prefix for generated benchmark "
        "artifacts.\n"
     << "  --benchmark-csv PATH      CSV path for benchmark rows (default: "
        "kDataPath/NAME_benchmark.csv).\n"
     << "  --verbose-benchmark       Enable outer LM summary output.\n"
     << "  --verbose-retractor       Enable retraction LM summary output.\n";
  if (defaults.enableNumSteps) {
    os << "  --num-steps N             Number of trajectory steps (default: "
       << defaults.defaultNumSteps << ").\n";
  }
  os << "Defaults: benchmark-id=" << defaults.id
     << ", methods=" << MethodsListString(defaults.defaultMethods) << "\n";
}

ParsedBenchmarkCli ParseBenchmarkCli(int argc, char** argv,
                                     const BenchmarkCliDefaults& defaults) {
  ParsedBenchmarkCli parsed;
  parsed.runOptions.id = defaults.id;
  parsed.runOptions.methods = defaults.defaultMethods;
  parsed.numSteps = defaults.defaultNumSteps;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      PrintBenchmarkUsage(std::cout, argv[0], defaults);
      std::exit(0);
    } else if (arg == "--verbose-benchmark") {
      parsed.runOptions.verbose = true;
    } else if (arg == "--verbose-retractor") {
      parsed.runOptions.verboseRetractor = true;
    } else if (arg == "--methods") {
      if (i + 1 >= argc)
        throw std::invalid_argument("Missing value for --methods");
      parsed.runOptions.methods = ParseMethods(argv[++i]);
    } else if (arg.rfind("--methods=", 0) == 0) {
      parsed.runOptions.methods =
          ParseMethods(arg.substr(std::string("--methods=").size()));
    } else if (arg == "--benchmark-id") {
      if (i + 1 >= argc)
        throw std::invalid_argument("Missing value for --benchmark-id");
      parsed.runOptions.id = argv[++i];
    } else if (arg.rfind("--benchmark-id=", 0) == 0) {
      parsed.runOptions.id = arg.substr(std::string("--benchmark-id=").size());
    } else if (arg == "--benchmark-csv") {
      if (i + 1 >= argc)
        throw std::invalid_argument("Missing value for --benchmark-csv");
      parsed.runOptions.csvPath = argv[++i];
    } else if (arg.rfind("--benchmark-csv=", 0) == 0) {
      parsed.runOptions.csvPath =
          arg.substr(std::string("--benchmark-csv=").size());
    } else if (defaults.enableNumSteps &&
               (arg == "--num-steps" || arg == "-n")) {
      if (i + 1 >= argc)
        throw std::invalid_argument("Missing value for --num-steps");
      parsed.numSteps = std::stoul(argv[++i]);
    } else if (defaults.enableNumSteps && arg.rfind("--num-steps=", 0) == 0) {
      parsed.numSteps =
          std::stoul(arg.substr(std::string("--num-steps=").size()));
    } else if (defaults.enableNumSteps &&
               std::all_of(arg.begin(), arg.end(),
                           [](unsigned char c) { return std::isdigit(c); })) {
      parsed.numSteps = std::stoul(arg);
    } else {
      parsed.unknownArgs.push_back(arg);
    }
  }

  if (defaults.enableNumSteps && parsed.numSteps == 0) {
    throw std::invalid_argument("--num-steps must be greater than 0");
  }

  return parsed;
}

std::string BenchmarkMethodToken(BenchmarkMethod method) {
  switch (method) {
    case BenchmarkMethod::SOFT:
      return "soft";
    case BenchmarkMethod::PENALTY:
      return "penalty";
    case BenchmarkMethod::AUGMENTED_LAGRANGIAN:
      return "alm";
    case BenchmarkMethod::CM_F:
      return "cm_f";
    case BenchmarkMethod::CM_I:
      return "cm_i";
  }
  return "unknown";
}

std::string BenchmarkMethodLabel(BenchmarkMethod method) {
  switch (method) {
    case BenchmarkMethod::SOFT:
      return "Soft Constraint";
    case BenchmarkMethod::PENALTY:
      return "Penalty Method";
    case BenchmarkMethod::AUGMENTED_LAGRANGIAN:
      return "Augmented Lagrangian";
    case BenchmarkMethod::CM_F:
      return "Constraint Manifold (F)";
    case BenchmarkMethod::CM_I:
      return "Constraint Manifold (I)";
  }
  return "Unknown";
}

std::string BenchmarkMethodDataPath(
    const ConstrainedOptBenchmark::Options& options, BenchmarkMethod method,
    const std::string& suffix) {
  const std::filesystem::path path =
      EnsureDataPath() /
      (options.id + "_" + BenchmarkMethodToken(method) + suffix);
  return path.string();
}

ConstrainedOptBenchmark::ConstrainedOptBenchmark(
    ConstrainedOptBenchmark::Options options)
    : options_(std::move(options)) {
  moptFactory_ = [](BenchmarkMethod) {
    return ConstrainedOptBenchmark::DefaultMoptParams();
  };
}

void ConstrainedOptBenchmark::setProblemFactory(ProblemFactory factory) {
  problemFactory_ = std::move(factory);
}

void ConstrainedOptBenchmark::setOuterLmBaseParams(
    LevenbergMarquardtParams params) {
  outerLmParams_ = std::move(params);
}

void ConstrainedOptBenchmark::setOuterLmConfig(LmConfig callback) {
  outerLmConfig_ = std::move(callback);
}

void ConstrainedOptBenchmark::setMoptFactory(MoptFactory factory) {
  moptFactory_ = std::move(factory);
}

void ConstrainedOptBenchmark::setResultCallback(ResultCallback callback) {
  resultCallback_ = std::move(callback);
}

void ConstrainedOptBenchmark::run(std::ostream& latexOs) {
  if (!problemFactory_) {
    throw std::runtime_error(
        "ConstrainedOptBenchmark requires a problem factory.");
  }

  if (options_.methods.empty()) {
    throw std::runtime_error("ConstrainedOptBenchmark methods set is empty.");
  }

  const std::string csvPath = options_.csvPath.empty()
                                  ? DefaultBenchmarkCsvPath(options_)
                                  : options_.csvPath;

  ScopedEnvVar benchCsvEnv(kBenchmarkCsvEnv, csvPath);
  ScopedEnvVar benchIdEnv(kBenchmarkIdEnv, options_.id);

  for (BenchmarkMethod method : OrderedMethods()) {
    if (options_.methods.count(method) == 0) continue;

    LevenbergMarquardtParams lmParams = outerLmParams_;
    if (options_.verbose) {
      lmParams.setVerbosityLM("SUMMARY");
    }
    if (outerLmConfig_) {
      outerLmConfig_(method, &lmParams);
    }

    auto problem = problemFactory_();

    Values result;
    switch (method) {
      case BenchmarkMethod::SOFT: {
        std::cout << "soft constraints:\n";
        result =
            OptimizeSoftConstraints(problem, latexOs, lmParams, options_.softMu,
                                    options_.constraintUnitScale);
        break;
      }
      case BenchmarkMethod::PENALTY: {
        std::cout << "penalty method:\n";
        auto penaltyParams = std::make_shared<gtsam::PenaltyOptimizerParams>();
        penaltyParams->lmParams = lmParams;
        result = OptimizePenalty(problem, latexOs, penaltyParams,
                                 options_.constraintUnitScale);
        break;
      }
      case BenchmarkMethod::AUGMENTED_LAGRANGIAN: {
        std::cout << "augmented lagrangian:\n";
        auto almParams = std::make_shared<gtsam::AugmentedLagrangianParams>();
        almParams->lmParams = lmParams;
        result = OptimizeAugmentedLagrangian(problem, latexOs, almParams,
                                             options_.constraintUnitScale);
        break;
      }
      case BenchmarkMethod::CM_F:
      case BenchmarkMethod::CM_I: {
        const bool feasible = method == BenchmarkMethod::CM_F;
        std::cout << "constraint manifold basis variables ("
                  << (feasible ? "feasible" : "infeasible") << "):\n";

        auto moptParams = moptFactory_(method);
        auto* retractLm =
            &moptParams.cc_params->retractor_creator->params()->lm_params;
        if (options_.verboseRetractor) {
          retractLm->setVerbosityLM("SUMMARY");
        }

        retractLm->setMaxIterations(feasible
                                        ? options_.cmFRetractorMaxIterations
                                        : options_.cmIRetractorMaxIterations);

        if (!feasible && options_.cmIRetractFinal) {
          moptParams.retract_final = true;
        }

        result = OptimizeCmOpt(problem, latexOs, moptParams, lmParams,
                               BenchmarkMethodLabel(method),
                               options_.constraintUnitScale);
        break;
      }
    }

    if (resultCallback_) {
      resultCallback_(method, result);
    }
  }
}

Values ConstrainedOptBenchmark::OptimizeSoftConstraints(
    const EConsOptProblem& problem, std::ostream& latexOs,
    LevenbergMarquardtParams lmParams, double mu, double constraintUnitScale) {
  NonlinearFactorGraph graph = problem.costs_;
  graph.add(problem.constraints().penaltyGraph(mu));

  LevenbergMarquardtOptimizer optimizer(graph, problem.initValues(), lmParams);
  auto optimizationStart = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimizationEnd = std::chrono::system_clock::now();
  const auto optimizationTimeMs =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimizationEnd -
                                                            optimizationStart);
  const double optimizationTime = optimizationTimeMs.count() * 1e-3;

  PrintLatex(
      latexOs, "Soft Constraint",
      problem.costsDimension() + problem.constraintsDimension(),
      problem.valuesDimension(), optimizationTime,
      optimizer.getInnerIterations(),
      problem.evaluateEConstraintViolationL2Norm(result) * constraintUnitScale,
      problem.evaluateCost(result));
  return result;
}

ManifoldOptimizerParameters ConstrainedOptBenchmark::DefaultMoptParams() {
  ManifoldOptimizerParameters moptParams;
  auto retractorParams = std::make_shared<RetractParams>();
  moptParams.cc_params->retractor_creator =
      std::make_shared<UoptRetractorCreator>(retractorParams);
  auto basisParams = std::make_shared<TspaceBasisParams>();
  basisParams->always_construct_basis = false;
  moptParams.cc_params->basis_creator =
      std::make_shared<OrthonormalBasisCreator>(basisParams);
  return moptParams;
}

ManifoldOptimizerParameters ConstrainedOptBenchmark::DefaultMoptParamsSV(
    const BasisKeyFunc& basisKeyFunc) {
  ManifoldOptimizerParameters moptParams;
  auto retractorParams = std::make_shared<RetractParams>();
  retractorParams->use_basis_keys = true;
  moptParams.cc_params->retractor_creator =
      std::make_shared<BasisRetractorCreator>(basisKeyFunc, retractorParams);
  auto basisParams = std::make_shared<TspaceBasisParams>();
  basisParams->use_basis_keys = true;
  basisParams->always_construct_basis = false;
  moptParams.cc_params->basis_creator =
      std::make_shared<EliminationBasisCreator>(basisKeyFunc, basisParams);
  return moptParams;
}

Values ConstrainedOptBenchmark::OptimizeCmOpt(
    const EConsOptProblem& problem, std::ostream& latexOs,
    ManifoldOptimizerParameters moptParams, LevenbergMarquardtParams lmParams,
    const std::string& expName, double constraintUnitScale) {
  NonlinearMOptimizer optimizer(moptParams, lmParams);
  auto moptProblem = optimizer.initializeMoptProblem(
      problem.costs(), problem.constraints(), problem.initValues());

  auto optimizationStart = std::chrono::system_clock::now();
  auto result = optimizer.optimizeMOpt(moptProblem);
  auto optimizationEnd = std::chrono::system_clock::now();
  const auto optimizationTimeMs =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimizationEnd -
                                                            optimizationStart);
  const double optimizationTime = optimizationTimeMs.count() * 1e-3;

  const auto problemDim = moptProblem.problemDimension();
  PrintLatex(
      latexOs, "\\textbf{" + expName + "}", problemDim.first, problemDim.second,
      optimizationTime, 0,
      problem.evaluateEConstraintViolationL2Norm(result) * constraintUnitScale,
      problem.evaluateCost(result));

  return result;
}

Values ConstrainedOptBenchmark::OptimizePenalty(
    const EConsOptProblem& problem, std::ostream& latexOs,
    gtsam::PenaltyOptimizerParams::shared_ptr params,
    double constraintUnitScale) {
  gtsam::NonlinearFactorGraph graph = problem.costs();
  graph.add(problem.constraints());
  gtsam::PenaltyOptimizer optimizer(graph, problem.initValues(), params);

  auto optimizationStart = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimizationEnd = std::chrono::system_clock::now();
  const auto optimizationTimeUs =
      std::chrono::duration_cast<std::chrono::microseconds>(optimizationEnd -
                                                            optimizationStart);
  const double optimizationTime = optimizationTimeUs.count() * 1e-6;

  PrintLatex(
      latexOs, "Penalty Method",
      problem.costsDimension() + problem.constraintsDimension(),
      problem.valuesDimension(), optimizationTime, 0,
      problem.evaluateEConstraintViolationL2Norm(result) * constraintUnitScale,
      problem.evaluateCost(result));

  return result;
}

Values ConstrainedOptBenchmark::OptimizeAugmentedLagrangian(
    const EConsOptProblem& problem, std::ostream& latexOs,
    gtsam::AugmentedLagrangianParams::shared_ptr params,
    double constraintUnitScale) {
  gtsam::NonlinearFactorGraph graph = problem.costs();
  graph.add(problem.constraints());
  gtsam::AugmentedLagrangianOptimizer optimizer(graph, problem.initValues(),
                                                params);

  auto optimizationStart = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimizationEnd = std::chrono::system_clock::now();
  const auto optimizationTimeMs =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimizationEnd -
                                                            optimizationStart);
  const double optimizationTime = optimizationTimeMs.count() * 1e-3;

  PrintLatex(
      latexOs, "Augmented Lagrangian",
      problem.costsDimension() + problem.constraintsDimension(),
      problem.valuesDimension(), optimizationTime, 0,
      problem.evaluateEConstraintViolationL2Norm(result) * constraintUnitScale,
      problem.evaluateCost(result));

  return result;
}

}  // namespace gtdynamics
