/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main_quadruped.cpp
 * @brief Trajectory optimization for a legged robot with contacts.
 * @author Alejandro Escontrela
 */

#include <gtdynamics/config.h>
#include <gtdynamics/dynamics/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/dynamics/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/mechanics/TorqueFactor.h>
#include <gtdynamics/mechanics/WrenchEquivalenceFactor.h>
#include <gtdynamics/dynamics/WrenchFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "QuadrupedUtils.h"
#include "gtdynamics/cmopt/ConstraintManifold.h"
#include "gtdynamics/cmopt/TspaceBasis.h"
#include "gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h"
#include "gtdynamics/constrained_optimizer/ConstrainedOptimizer.h"
#include "gtdynamics/factors/ContactPointFactor.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/values.h"

using namespace gtdynamics;
using namespace gtsam;

namespace {
constexpr double kConstraintUnitScale = 1e-3;
}  // namespace

void TrajectoryOptimization(
    size_t numSteps, const ConstrainedOptBenchmark::Options& runOptions) {
  /// Initialize vision60 robot
  Vision60Robot vision60;
  vision60.express_redundancy = false;

  /// Scenario
  double dt = 0.1;
  Pose3 basePoseInit(Rot3::Identity(), Point3(0, 0, vision60.nominal_height));
  Vector6 baseTwistInit = Vector6::Zero();

  // std::vector<Pose3> des_poses {Pose3(Rot3::Identity(), Point3(0, 0,
  // vision60.nominal_height + 0.1)), Pose3(Rot3::Rz(0.1), Point3(0, 0,
  // vision60.nominal_height + 0.2)), Pose3(Rot3::Rz(0.1), Point3(0, 0,
  // vision60.nominal_height + 0.1))};

  // std::vector<double> des_poses_t {1.0, 2.0, 3.0};

  std::vector<Pose3> desPoses{
      Pose3(Rot3::Ry(-0.2), Point3(0.2, 0, vision60.nominal_height + 0.2))};
  const double horizonTime = numSteps * dt;
  std::vector<double> desPosesT{std::min(3.0, horizonTime)};

  /// Get constraints, costs, and initial values.
  auto constraintsGraph = vision60.getConstraintsGraphTrajectory(numSteps);
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraintsGraph);
  auto initValues = vision60.getInitValuesTrajectory(numSteps, dt, basePoseInit,
                                                     desPoses, desPosesT);
  NonlinearFactorGraph collocationCosts =
      vision60.collocationCosts(numSteps, dt);
  NonlinearFactorGraph boundaryCosts = vision60.boundaryCosts(
      basePoseInit, baseTwistInit, desPoses, desPosesT, dt);
  NonlinearFactorGraph minTorqueCosts = vision60.minTorqueCosts(numSteps);
  NonlinearFactorGraph frictionConeCosts = vision60.frictionConeCosts(numSteps);
  NonlinearFactorGraph costs;
  costs.add(collocationCosts);
  costs.add(boundaryCosts);
  costs.add(minTorqueCosts);
  costs.add(frictionConeCosts);
  auto evaluateCosts = [=](const Values& values) {
    std::cout << "collocation costs:\t" << collocationCosts.error(values)
              << "\n";
    std::cout << "boundary costs:\t" << boundaryCosts.error(values) << "\n";
    std::cout << "min torque costs:\t" << minTorqueCosts.error(values) << "\n";
    std::cout << "friction cone costs:\t" << frictionConeCosts.error(values)
              << "\n";
  };

  /// Export initial trajectory
  evaluateCosts(initValues);
  vision60.exportTrajectory(
      initValues, numSteps,
      std::string(kDataPath) + runOptions.id + "_init_traj.csv");

  auto createProblem = [=]() {
    return EConsOptProblem(costs, constraints, initValues);
  };

  LevenbergMarquardtParams lmParams;
  lmParams.minModelFidelity = 0.3;
  lmParams.linearSolverType =
      gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
  lmParams.setMaxIterations(30);
  lmParams.relativeErrorTol = 1e-3;
  lmParams.setlambdaUpperBound(1e2);

  ConstrainedOptBenchmark runner(runOptions);
  runner.setProblemFactory(createProblem);
  runner.setOuterLmBaseParams(lmParams);
  runner.setOuterLmConfig([](ConstrainedOptBenchmark::Method method,
                             LevenbergMarquardtParams* params) {
    if (method == ConstrainedOptBenchmark::Method::CM_F ||
        method == ConstrainedOptBenchmark::Method::CM_I) {
      params->setlambdaInitial(1e1);
    }
  });
  runner.setMoptFactory([&](ConstrainedOptBenchmark::Method) {
    auto moptParams = ConstrainedOptBenchmark::DefaultMoptParamsSV(
        vision60.getBasisKeyFunc());
    moptParams.cc_params->retractor_creator->params()->use_basis_keys = true;
    moptParams.cc_params->retractor_creator->params()->sigma = 1.0;
    moptParams.cc_params->retractor_creator->params()->apply_base_retraction =
        true;
    moptParams.cc_params->retractor_creator->params()->check_feasible = true;
    moptParams.cc_params->retractor_creator->params()
        ->lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    moptParams.cc_params->retractor_creator->params()
        ->lm_params.setlambdaUpperBound(1e2);
    return moptParams;
  });
  runner.setResultCallback(
      [&](ConstrainedOptBenchmark::Method method, const Values& result) {
        evaluateCosts(result);
        vision60.exportTrajectory(result, numSteps,
                                  ConstrainedOptBenchmark::MethodDataPath(
                                      runOptions, method, "_traj.csv"));
      });

  std::ostringstream latexOs;
  runner.run(latexOs);
  std::cout << latexOs.str();
}

int main(int argc, char** argv) {
  try {
    ConstrainedOptBenchmark::CliDefaults defaults;
    defaults.id = "quadruped";
    defaults.enableNumSteps = true;
    defaults.defaultNumSteps = 30;
    defaults.defaultMethods = {ConstrainedOptBenchmark::Method::SOFT,
                               ConstrainedOptBenchmark::Method::CM_I};

    auto parsed = ConstrainedOptBenchmark::ParseCli(argc, argv, defaults);
    if (!parsed.unknownArgs.empty()) {
      throw std::invalid_argument("Unknown option: " +
                                  parsed.unknownArgs.front());
    }

    parsed.runOptions.softMu = 1e2;
    parsed.runOptions.constraintUnitScale = kConstraintUnitScale;

    std::cout << "Using num_steps=" << parsed.numSteps << "\n";
    std::cout << "Methods: " << "\n";
    for (ConstrainedOptBenchmark::Method method : parsed.runOptions.methods) {
      std::cout << "  - " << ConstrainedOptBenchmark::MethodToken(method)
                << "\n";
    }
    TrajectoryOptimization(parsed.numSteps, parsed.runOptions);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    ConstrainedOptBenchmark::CliDefaults defaults;
    defaults.id = "quadruped";
    defaults.enableNumSteps = true;
    defaults.defaultNumSteps = 30;
    defaults.defaultMethods = {ConstrainedOptBenchmark::Method::SOFT,
                               ConstrainedOptBenchmark::Method::CM_I};
    ConstrainedOptBenchmark::PrintUsage(std::cerr, argv[0], defaults);
    return 1;
  }
}
