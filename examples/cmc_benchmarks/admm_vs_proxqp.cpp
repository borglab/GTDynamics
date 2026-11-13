/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file admm_vs_proxqp.cpp
 * @brief Reproducible small-to-large coupled point-cone QP microbenchmark.
 *
 * Build with GTDYNAMICS_WITH_PROXQP=ON. This benchmarks only the *global QP*,
 * not CMC's nonlinear projection, and reports failures rather than timings
 * for incorrect/uncertified solutions.
 */
#include <gtdynamics/cmcopt/PointSplitAdmmSolver.h>
#include <gtdynamics/cmcopt/ProxQpSolver.h>
#include <gtsam/constrained/LinearConstraint.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

using namespace gtsam;
namespace cmc = gtdynamics::internal;

int main(int argc, char** argv) {
#ifndef GTDYNAMICS_WITH_PROXQP
  std::cerr << "Build with GTDYNAMICS_WITH_PROXQP=ON to compare backends\n";
  return 2;
#else
  const int points = argc > 1 ? std::stoi(argv[1]) : 128;
  const double rho = argc > 2 ? std::stod(argv[2]) : 1.;
  const size_t threads = argc > 3 ? std::stoul(argv[3]) : 1;
  const size_t limit = argc > 4 ? std::stoul(argv[4]) : 10000;
  const int repeats = argc > 5 ? std::stoi(argv[5]) : 5;
  if (points < 1 || repeats < 1) return 2;

  // Every point has a 3D cone and independent bounds, while pairwise cost
  // factors couple all points into one sparse chain. Same QP for both backends.
  QpProblem problem;
  Values prototype;
  const Matrix3 I = Matrix3::Identity();
  for (int i = 0; i < points; ++i) {
    const Key key = i + 1;
    prototype.insert(key, Vector(Vector3::Zero()));
    const double phase = 0.025 * i;
    const Vector3 target(std::cos(phase) - 0.6,
                         std::sin(phase) - 0.6, 0.4);
    problem.addCost(JacobianFactor(key, I, target));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(key, -I, Vector3::Zero())));
    if (i)
      problem.addCost(JacobianFactor(
          key - 1, 0.5 * I, key, -0.5 * I, Vector3::Zero()));
  }
  cmc::PointSplitAdmmParams config;
  config.rho = rho;
  config.projectionThreads = threads;

  std::cout << "points,variables,rows,backend,repeat,seconds,iterations,"
               "converged,objective,primal,dual,stationarity,factorization,projections\n";
  auto run = [&](const std::string& name, int repeat) {
    const auto start = std::chrono::steady_clock::now();
    if (name == "admm") {
      const auto result = cmc::solvePointSplitAdmm(problem, prototype,
                                                   std::nullopt, limit, config);
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();
      const double objective = result.converged ?
          problem.costs().error(result.values) :
          std::numeric_limits<double>::quiet_NaN();
      std::cout << points << "," << 3 * points << "," << 3 * points
                << ",admm," << repeat << "," << elapsed << ","
                << result.iterations << "," << result.converged << ","
                << objective << "," << result.primalResidual << ","
                << result.dualResidual << ","
                << result.stationarityResidual << ","
                << result.factorizationSeconds << ","
                << result.projectionSeconds << "\n";
      if (!result.converged) std::cerr << result.message << "\n";
      return result.converged;
    }
    const auto result =
        cmc::solveProxQp(problem, prototype, std::nullopt, limit);
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    const double objective = result.converged ?
        problem.costs().error(result.values) :
        std::numeric_limits<double>::quiet_NaN();
    std::cout << points << "," << 3 * points << "," << 3 * points
              << ",proxqp," << repeat << "," << elapsed << ","
              << result.iterations << "," << result.converged << ","
              << objective << ",,,,,\n";
    if (!result.converged) std::cerr << result.message << "\n";
    return result.converged;
  };
  // Warm up both backends, then alternate execution order to reduce bias.
  const bool warmAdmm = run("admm", -1);
  const bool warmProx = run("proxqp", -1);
  bool ok = warmAdmm && warmProx;
  for (int i = 0; i < repeats; ++i) {
    if (i % 2) {
      ok = run("proxqp", i) && ok;
      ok = run("admm", i) && ok;
    } else {
      ok = run("admm", i) && ok;
      ok = run("proxqp", i) && ok;
    }
  }
  return ok ? 0 : 1;
#endif
}
