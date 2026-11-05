/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file CmcBenchmark.h
 * @brief Headless measurement mode for the original CMC example programs.
 * @author GTDynamics contributors
 */
#pragma once
#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmarkIE.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace cmc_benchmark {
inline std::string method, output, variant;
inline size_t stopAfter = 1, calls = 0;
/// Stop after the requested solves, before plotting or exporting unused data.
struct Finished {};

/** Enable --benchmark LM|GD|CM and write one JSON line per completed solve.
 * --benchmark-stop-after retains preceding solves in multistage examples.
 * Normal example execution is unchanged when --benchmark is absent.
 */
inline void configure(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--benchmark-variant" && i + 1 < argc)
      variant = argv[++i];
    else if (arg == "--benchmark" && i + 1 < argc)
      method = argv[++i];
    else if (arg == "--benchmark-output" && i + 1 < argc)
      output = argv[++i];
    else if (arg == "--benchmark-stop-after" && i + 1 < argc)
      stopAfter = std::stoul(argv[++i]);
  }
  if (!method.empty() && method != "LM" && method != "GD" && method != "CM")
    throw std::invalid_argument("--benchmark must be LM, GD, or CM");
  if (!stopAfter)
    throw std::invalid_argument("benchmark solve count must be positive");
  if (!method.empty() && output.empty())
    throw std::invalid_argument("--benchmark requires --benchmark-output");
  if (!output.empty()) {
    std::ofstream stream(output);
    if (!stream)
      throw std::runtime_error("Cannot write benchmark output: " + output);
  }
  if (!method.empty()) std::srand(0);
}

/** Skip unrelated solvers only in the explicitly selected benchmark mode. */
template <class Function>
auto other(Function function) -> decltype(function()) {
  if (!method.empty()) return {};
  return function();
}

/** Run the original benchmark helper and capture its numerical result before
 * plotting/export code. Time excludes this additional result serialization.
 */
template <class Function>
auto measure(const gtdynamics::IEConsOptProblem& problem,
             const std::string& selected, Function function)
    -> decltype(function()) {
  if (method.empty()) return function();
  if (method != selected) return {};
  const auto start = std::chrono::steady_clock::now();
  auto result = function();
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  const auto& summary = result.first;
  std::ofstream stream(output, std::ios::app);
  stream << std::setprecision(17) << "{\"method\":" << std::quoted(selected)
         << ",\"stage\":" << calls
         << ",\"optimization_seconds\":" << summary.optimization_time
         << ",\"pipeline_seconds\":" << elapsed
         << ",\"iterations\":" << summary.total_iters
         << ",\"inner_iterations\":" << summary.total_inner_iters
         << ",\"cost\":" << summary.cost
         << ",\"equality_l2\":" << summary.e_violation
         << ",\"inequality_l2\":" << summary.i_violation
         << ",\"factor_costs\":[";
  bool first = true;
  for (const auto& factor : problem.costs()) {
    if (!first) stream << ',';
    first = false;
    stream << (factor ? factor->error(summary.values) : 0.0);
  }
  stream << "],\"coordinates_from_initial\":[";
  const auto delta =
      problem.initValues().localCoordinates(summary.values).vector();
  for (Eigen::Index i = 0; i < delta.size(); ++i) {
    if (i) stream << ',';
    stream << delta(i);
  }
  double equalityMax = 0, inequalityMax = 0;
  for (const auto& constraint : problem.eConstraints())
    equalityMax = std::max(
        equalityMax,
        constraint->unwhitenedError(summary.values).cwiseAbs().maxCoeff());
  for (const auto& constraint : problem.iConstraints())
    inequalityMax = std::max(
        inequalityMax, constraint->unwhitenedExpr(summary.values).maxCoeff());
  stream << "],\"equality_max\":" << equalityMax
         << ",\"inequality_max\":" << inequalityMax << "}\n";
  stream.close();
  if (++calls >= stopAfter) throw Finished{};
  return result;
}
}  // namespace cmc_benchmark
