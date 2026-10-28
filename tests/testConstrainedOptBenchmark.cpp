/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/** @file testConstrainedOptBenchmark.cpp
 * @brief Check configurable penalty/AL schedules against the PR #363 equations.
 */
#include <CppUnitLite/TestHarness.h>
#include <examples/example_constraint_manifold/CableRobotBenchmark.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <cmath>
#include <cstdio>
#include <sstream>

using namespace gtsam;
using namespace gtdynamics;

TEST(ConstrainedOptBenchmark, OriginalPenaltyAndMultiplierSchedules) {
  const Key key = Symbol('x', 0);
  const double target = 100.0;
  const double weight = 100.0;
  NonlinearFactorGraph costs, equalityCosts;
  costs.addPrior(key, target, noiseModel::Isotropic::Sigma(1, 0.1));
  equalityCosts.addPrior(key, 1.0, noiseModel::Unit::Create(1));
  const auto constraints =
      NonlinearEqualityConstraints::FromCostGraph(equalityCosts);
  Values initial;
  initial.insert(key, 0.0);
  ConstrainedOptBenchmark::Options options;
  options.methods = {ConstrainedOptBenchmark::Method::PENALTY,
                     ConstrainedOptBenchmark::Method::AUGMENTED_LAGRANGIAN};
  options.csvPath = "testConstrainedOptBenchmark.csv";
  ConstrainedOptBenchmark runner(options);
  runner.setProblemFactory(
      [&]() { return EConsOptProblem(costs, constraints, initial); });
  bool penaltyConfigured = false, almConfigured = false;
  runner.setPenaltyConfig([&](PenaltyOptimizerParams *params) {
    configureCableRobotPenalty(params);
    penaltyConfigured = true;
  });
  runner.setAugmentedLagrangianConfig([&](AugmentedLagrangianParams *params) {
    configureCableRobotAugmentedLagrangian(params);
    almConfigured = true;
  });
  size_t numResults = 0;
  runner.setResultCallback(
      [&](ConstrainedOptBenchmark::Method method, const Values &values) {
        ++numResults;
        double expected;
        if (method == ConstrainedOptBenchmark::Method::PENALTY) {
          const double mu = std::pow(2.0, 14);
          expected = (weight * target + mu) / (weight + mu);
        } else {
          // Solve the original scalar AL subproblems analytically, updating
          // lambda without the newer cap and doubling mu only on insufficient
          // contraction.
          double mu = 1, lambda = 0, previous = 0;
          expected = previous;
          for (int iteration = 0; iteration < 12; ++iteration) {
            expected = (weight * target + mu - lambda) / (weight + mu);
            lambda += mu * (expected - 1);
            if (std::abs(expected - 1) >= 0.25 * std::abs(previous - 1))
              mu *= 2;
            previous = expected;
          }
          EXPECT(mu > 10);  // Exercise uncapped dual updates.
        }
        DOUBLES_EQUAL(expected, values.at<double>(key), 1e-5);
      });
  std::ostringstream latex;
  runner.run(latex);
  EXPECT(penaltyConfigured && almConfigured);
  EXPECT_LONGS_EQUAL(2, numResults);
  std::remove(options.csvPath.c_str());
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
