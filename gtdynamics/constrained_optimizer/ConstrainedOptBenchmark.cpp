/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstrainedOptBenchmark.cpp
 * @brief Constrained optimization benchmark implementations.
 * @author Yetong Zhang
 */

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
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
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

std::set<BenchmarkMethod> ParseMethods(const std::string& methods_text) {
  std::set<BenchmarkMethod> methods;
  for (const auto& token_raw : Split(methods_text, ',')) {
    const std::string token = ToLower(Trim(token_raw));
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
    throw std::invalid_argument("Unknown method token: " + token_raw);
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

std::string DefaultBenchmarkCsvPath(const BenchmarkRunOptions& options) {
  const std::filesystem::path path =
      EnsureDataPath() / (options.benchmark_id + "_benchmark.csv");
  return path.string();
}

class ScopedEnvVar {
 public:
  ScopedEnvVar(const char* key, std::string value) : key_(key) {
    const char* existing = std::getenv(key_);
    if (existing) {
      had_old_value_ = true;
      old_value_ = existing;
    }
    setenv(key_, value.c_str(), 1);
  }

  ~ScopedEnvVar() {
    if (had_old_value_) {
      setenv(key_, old_value_.c_str(), 1);
    } else {
      unsetenv(key_);
    }
  }

 private:
  const char* key_;
  bool had_old_value_ = false;
  std::string old_value_;
};

void AppendBenchmarkCsv(const std::string& method, size_t f_dim, size_t v_dim,
                        double time_s, size_t num_iters, double constraint_l2,
                        double cost) {
  const char* csv_path = std::getenv(kBenchmarkCsvEnv);
  if (!csv_path || std::string(csv_path).empty()) return;

  const char* bench_name_env = std::getenv(kBenchmarkIdEnv);
  std::string bench_name = bench_name_env ? bench_name_env : "";

  bool write_header = true;
  {
    std::ifstream in(csv_path);
    write_header =
        !in.good() || in.peek() == std::ifstream::traits_type::eof();
  }

  std::ofstream out(csv_path, std::ios::app);
  if (!out) return;

  if (write_header) {
    out << "benchmark,method,f_dim,v_dim,time_s,iters,constraint_l2,cost\n";
  }
  out << CsvEscape(bench_name) << "," << CsvEscape(method) << "," << f_dim
      << "," << v_dim << "," << std::setprecision(12) << time_s << ","
      << num_iters << "," << std::scientific << std::setprecision(12)
      << constraint_l2 << "," << std::defaultfloat << std::setprecision(12)
      << cost << "\n";
}

void PrintLatex(std::ostream& latex_os, const std::string& exp_name, size_t f_dim,
                size_t v_dim, double time, size_t num_iters,
                double constraint_vio, double cost) {
  std::cout << "[BENCH] " << exp_name << ": f_dim=" << f_dim
            << ", v_dim=" << v_dim << ", time_s=" << std::setprecision(6)
            << std::defaultfloat << time << ", iters=" << num_iters
            << ", constraint_l2=" << std::scientific << std::setprecision(3)
            << constraint_vio << ", cost=" << std::defaultfloat
            << std::setprecision(6) << cost << "\n";
  latex_os << "& " + exp_name + " & $" << f_dim << " \\times " << v_dim
           << "$ & " << std::setprecision(4) << time << std::defaultfloat
           << " & " << num_iters << " & " << std::scientific
           << std::setprecision(2) << constraint_vio << std::defaultfloat
           << " & " << std::fixed << std::setprecision(2) << cost
           << std::defaultfloat << "\\\\\n";
  AppendBenchmarkCsv(exp_name, f_dim, v_dim, time, num_iters, constraint_vio,
                     cost);
}

}  // namespace

void PrintBenchmarkUsage(std::ostream& os, const char* program_name,
                         const BenchmarkCliDefaults& defaults) {
  os << "Usage: " << program_name << " [benchmark options] [example options]\n"
     << "Benchmark options:\n"
     << "  --methods LIST            Comma list from {soft,penalty,alm,f,i,all}.\n"
     << "  --benchmark-id NAME       Prefix for generated benchmark artifacts.\n"
     << "  --benchmark-csv PATH      CSV path for benchmark rows (default: kDataPath/NAME_benchmark.csv).\n"
     << "  --verbose-benchmark       Enable outer LM summary output.\n"
     << "  --verbose-retractor       Enable retraction LM summary output.\n";
  if (defaults.enable_num_steps) {
    os << "  --num-steps N             Number of trajectory steps (default: "
       << defaults.default_num_steps << ").\n";
  }
  os << "Defaults: benchmark-id=" << defaults.benchmark_id
     << ", methods=" << MethodsListString(defaults.default_methods) << "\n";
}

ParsedBenchmarkCli ParseBenchmarkCli(int argc, char** argv,
                                     const BenchmarkCliDefaults& defaults) {
  ParsedBenchmarkCli parsed;
  parsed.run_options.benchmark_id = defaults.benchmark_id;
  parsed.run_options.methods = defaults.default_methods;
  parsed.num_steps = defaults.default_num_steps;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      PrintBenchmarkUsage(std::cout, argv[0], defaults);
      std::exit(0);
    } else if (arg == "--verbose-benchmark") {
      parsed.run_options.verbose_benchmark = true;
    } else if (arg == "--verbose-retractor") {
      parsed.run_options.verbose_retractor = true;
    } else if (arg == "--methods") {
      if (i + 1 >= argc) throw std::invalid_argument("Missing value for --methods");
      parsed.run_options.methods = ParseMethods(argv[++i]);
    } else if (arg.rfind("--methods=", 0) == 0) {
      parsed.run_options.methods =
          ParseMethods(arg.substr(std::string("--methods=").size()));
    } else if (arg == "--benchmark-id") {
      if (i + 1 >= argc) throw std::invalid_argument("Missing value for --benchmark-id");
      parsed.run_options.benchmark_id = argv[++i];
    } else if (arg.rfind("--benchmark-id=", 0) == 0) {
      parsed.run_options.benchmark_id =
          arg.substr(std::string("--benchmark-id=").size());
    } else if (arg == "--benchmark-csv") {
      if (i + 1 >= argc) throw std::invalid_argument("Missing value for --benchmark-csv");
      parsed.run_options.benchmark_csv_path = argv[++i];
    } else if (arg.rfind("--benchmark-csv=", 0) == 0) {
      parsed.run_options.benchmark_csv_path =
          arg.substr(std::string("--benchmark-csv=").size());
    } else if (defaults.enable_num_steps &&
               (arg == "--num-steps" || arg == "-n")) {
      if (i + 1 >= argc) throw std::invalid_argument("Missing value for --num-steps");
      parsed.num_steps = std::stoul(argv[++i]);
    } else if (defaults.enable_num_steps && arg.rfind("--num-steps=", 0) == 0) {
      parsed.num_steps = std::stoul(arg.substr(std::string("--num-steps=").size()));
    } else if (defaults.enable_num_steps &&
               std::all_of(arg.begin(), arg.end(),
                           [](unsigned char c) { return std::isdigit(c); })) {
      parsed.num_steps = std::stoul(arg);
    } else {
      parsed.unknown_args.push_back(arg);
    }
  }

  if (defaults.enable_num_steps && parsed.num_steps == 0) {
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

std::string BenchmarkMethodDataPath(const BenchmarkRunOptions& options,
                                    BenchmarkMethod method,
                                    const std::string& suffix) {
  const std::filesystem::path path =
      EnsureDataPath() / (options.benchmark_id + "_" +
                          BenchmarkMethodToken(method) + suffix);
  return path.string();
}

ConstrainedOptBenchmarkRunner::ConstrainedOptBenchmarkRunner(
    BenchmarkRunOptions options)
    : options_(std::move(options)) {
  mopt_factory_ =
      [](BenchmarkMethod) { return DefaultMoptParams(); };
}

void ConstrainedOptBenchmarkRunner::setProblemFactory(ProblemFactory factory) {
  problem_factory_ = std::move(factory);
}

void ConstrainedOptBenchmarkRunner::setOuterLmBaseParams(
    LevenbergMarquardtParams params) {
  outer_lm_params_ = std::move(params);
}

void ConstrainedOptBenchmarkRunner::setOuterLmConfig(LmConfig callback) {
  outer_lm_config_ = std::move(callback);
}

void ConstrainedOptBenchmarkRunner::setMoptFactory(MoptFactory factory) {
  mopt_factory_ = std::move(factory);
}

void ConstrainedOptBenchmarkRunner::setResultCallback(ResultCallback callback) {
  result_callback_ = std::move(callback);
}

void ConstrainedOptBenchmarkRunner::run(std::ostream& latex_os) {
  if (!problem_factory_) {
    throw std::runtime_error("ConstrainedOptBenchmarkRunner requires a problem factory.");
  }

  if (options_.methods.empty()) {
    throw std::runtime_error("ConstrainedOptBenchmarkRunner methods set is empty.");
  }

  const std::string csv_path = options_.benchmark_csv_path.empty()
                                   ? DefaultBenchmarkCsvPath(options_)
                                   : options_.benchmark_csv_path;

  ScopedEnvVar bench_csv_env(kBenchmarkCsvEnv, csv_path);
  ScopedEnvVar bench_id_env(kBenchmarkIdEnv, options_.benchmark_id);

  for (BenchmarkMethod method : OrderedMethods()) {
    if (options_.methods.count(method) == 0) continue;

    LevenbergMarquardtParams lm_params = outer_lm_params_;
    if (options_.verbose_benchmark) {
      lm_params.setVerbosityLM("SUMMARY");
    }
    if (outer_lm_config_) {
      outer_lm_config_(method, &lm_params);
    }

    auto problem = problem_factory_();

    Values result;
    switch (method) {
      case BenchmarkMethod::SOFT: {
        std::cout << "soft constraints:\n";
        result = OptimizeE_SoftConstraints(problem, latex_os, lm_params,
                                           options_.soft_mu,
                                           options_.constraint_unit_scale);
        break;
      }
      case BenchmarkMethod::PENALTY: {
        std::cout << "penalty method:\n";
        auto penalty_params = std::make_shared<gtsam::PenaltyOptimizerParams>();
        penalty_params->lmParams = lm_params;
        result = OptimizeE_Penalty(problem, latex_os, penalty_params,
                                   options_.constraint_unit_scale);
        break;
      }
      case BenchmarkMethod::AUGMENTED_LAGRANGIAN: {
        std::cout << "augmented lagrangian:\n";
        auto alm_params =
            std::make_shared<gtsam::AugmentedLagrangianParams>();
        alm_params->lmParams = lm_params;
        result = OptimizeE_AugmentedLagrangian(problem, latex_os, alm_params,
                                               options_.constraint_unit_scale);
        break;
      }
      case BenchmarkMethod::CM_F:
      case BenchmarkMethod::CM_I: {
        const bool feasible = method == BenchmarkMethod::CM_F;
        std::cout << "constraint manifold basis variables ("
                  << (feasible ? "feasible" : "infeasible") << "):\n";

        auto mopt_params = mopt_factory_(method);
        auto* retract_lm =
            &mopt_params.cc_params->retractor_creator->params()->lm_params;
        if (options_.verbose_retractor) {
          retract_lm->setVerbosityLM("SUMMARY");
        }

        retract_lm->setMaxIterations(
            feasible ? options_.cm_f_retractor_max_iterations
                     : options_.cm_i_retractor_max_iterations);

        if (!feasible && options_.cm_i_retract_final) {
          mopt_params.retract_final = true;
        }

        result = OptimizeE_CMOpt(problem, latex_os, mopt_params, lm_params,
                                 BenchmarkMethodLabel(method),
                                 options_.constraint_unit_scale);
        break;
      }
    }

    if (result_callback_) {
      result_callback_(method, result);
    }
  }
}

/* ************************************************************************* */
Values OptimizeE_SoftConstraints(const EConsOptProblem &problem,
                                 std::ostream &latex_os,
                                 LevenbergMarquardtParams lm_params, double mu,
                                 double constraint_unit_scale) {
  NonlinearFactorGraph graph = problem.costs_;
  graph.add(problem.constraints().penaltyGraph(mu));

  LevenbergMarquardtOptimizer optimizer(graph, problem.initValues(), lm_params);
  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  PrintLatex(latex_os, "Soft Constraint",
             problem.costsDimension() + problem.constraintsDimension(),
             problem.valuesDimension(), optimization_time,
             optimizer.getInnerIterations(),
             problem.evaluateEConstraintViolationL2Norm(result) *
                 constraint_unit_scale,
             problem.evaluateCost(result));
  return result;
}

/* ************************************************************************* */
ManifoldOptimizerParameters DefaultMoptParams() {
  ManifoldOptimizerParameters mopt_params;
  auto retractor_params = std::make_shared<RetractParams>();
  mopt_params.cc_params->retractor_creator =
      std::make_shared<UoptRetractorCreator>(retractor_params);
  auto basis_params = std::make_shared<TspaceBasisParams>();
  basis_params->always_construct_basis = false;
  mopt_params.cc_params->basis_creator =
      std::make_shared<OrthonormalBasisCreator>(basis_params);
  return mopt_params;
}

/* ************************************************************************* */
ManifoldOptimizerParameters
DefaultMoptParamsSV(const BasisKeyFunc &basis_key_func) {
  ManifoldOptimizerParameters mopt_params;
  auto retractor_params = std::make_shared<RetractParams>();
  retractor_params->use_basis_keys = true;
  mopt_params.cc_params->retractor_creator =
      std::make_shared<BasisRetractorCreator>(basis_key_func, retractor_params);
  auto basis_params = std::make_shared<TspaceBasisParams>();
  basis_params->use_basis_keys = true;
  basis_params->always_construct_basis = false;
  mopt_params.cc_params->basis_creator =
      std::make_shared<EliminationBasisCreator>(basis_key_func, basis_params);
  return mopt_params;
}

/* ************************************************************************* */
Values OptimizeE_CMOpt(const EConsOptProblem &problem, std::ostream &latex_os,
                       ManifoldOptimizerParameters mopt_params,
                       LevenbergMarquardtParams lm_params, std::string exp_name,
                       double constraint_unit_scale) {
  NonlinearMOptimizer optimizer(mopt_params, lm_params);
  auto mopt_problem = optimizer.initializeMoptProblem(
      problem.costs(), problem.constraints(), problem.initValues());

  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimizeMOpt(mopt_problem);
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  auto problem_dim = mopt_problem.problemDimension();
  PrintLatex(latex_os, "\\textbf{" + exp_name + "}", problem_dim.first,
             problem_dim.second, optimization_time,
             //   intermediate_result.num_iters.at(0), TODO
             0,
             problem.evaluateEConstraintViolationL2Norm(result) *
                 constraint_unit_scale,
             problem.evaluateCost(result));

  return result;
}

/* ************************************************************************* */
Values OptimizeE_Penalty(const EConsOptProblem &problem, std::ostream &latex_os,
                         gtsam::PenaltyOptimizerParams::shared_ptr params,
                         double constraint_unit_scale) {
  // params->store_iter_details = true; // Not yet in gtsam, TODO
  gtsam::NonlinearFactorGraph graph = problem.costs();
  graph.add(problem.constraints());
  gtsam::PenaltyOptimizer optimizer(graph, problem.initValues(), params);

  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::microseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-6;

  PrintLatex(latex_os, "Penalty Method",
             problem.costsDimension() + problem.constraintsDimension(),
             problem.valuesDimension(), optimization_time,
             //   std::accumulate(intermediate_result.num_iters.begin(),
             //                   intermediate_result.num_iters.end(), 0), TODO
             0,
             problem.evaluateEConstraintViolationL2Norm(result) *
                 constraint_unit_scale,
             problem.evaluateCost(result));

  return result;
}

/* ************************************************************************* */
Values
OptimizeE_AugmentedLagrangian(const EConsOptProblem &problem,
                              std::ostream &latex_os,
                              gtsam::AugmentedLagrangianParams::shared_ptr params,
                              double constraint_unit_scale) {
  gtsam::NonlinearFactorGraph graph = problem.costs();
  graph.add(problem.constraints());
  gtsam::AugmentedLagrangianOptimizer optimizer(graph, problem.initValues(),
                                                params);

  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  PrintLatex(latex_os, "Augmented Lagrangian",
             problem.costsDimension() + problem.constraintsDimension(),
             problem.valuesDimension(), optimization_time,
             //   std::accumulate(intermediate_result.num_iters.begin(),
             //                   intermediate_result.num_iters.end(), 0), TODO
             0,
             problem.evaluateEConstraintViolationL2Norm(result) *
                 constraint_unit_scale,
             problem.evaluateCost(result));

  return result;
}

}  // namespace gtdynamics
