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

#include "QuadrupedUtils.h"

#include <gtdynamics/config.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
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

#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
#include <gtdynamics/factors/TorqueFactor.h>

#include "gtdynamics/factors/ContactPointFactor.h"
#include "gtdynamics/constrained_optimizer/ConstrainedOptimizer.h"
#include "gtdynamics/cmopt/ConstraintManifold.h"
#include "gtdynamics/cmopt/TspaceBasis.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/values.h"
#include "gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h"
#include "gtdynamics/universal_robot/Link.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

using namespace gtdynamics;
using namespace gtsam;

namespace {
constexpr double kConstraintUnitScale = 1e-3;
}  // namespace

void TrajectoryOptimization(size_t num_steps,
                            const BenchmarkRunOptions& run_options) {
  /// Initialize vision60 robot
  Vision60Robot vision60;
  vision60.express_redundancy = false;

  /// Scenario
  double dt = 0.1;
  Pose3 base_pose_init(Rot3::Identity(), Point3(0, 0, vision60.nominal_height));
  Vector6 base_twist_init = Vector6::Zero();

  // std::vector<Pose3> des_poses {Pose3(Rot3::Identity(), Point3(0, 0, vision60.nominal_height + 0.1)),
  // Pose3(Rot3::Rz(0.1), Point3(0, 0, vision60.nominal_height + 0.2)),
  // Pose3(Rot3::Rz(0.1), Point3(0, 0, vision60.nominal_height + 0.1))};
  
  // std::vector<double> des_poses_t {1.0, 2.0, 3.0};
  
  std::vector<Pose3> des_poses{
      Pose3(Rot3::Ry(-0.2), Point3(0.2, 0, vision60.nominal_height + 0.2))};
  const double horizon_time = num_steps * dt;
  std::vector<double> des_poses_t{std::min(3.0, horizon_time)};

  /// Get constraints, costs, and initial values.
  auto constraints_graph = vision60.getConstraintsGraphTrajectory(num_steps);
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph);
  auto init_values = vision60.getInitValuesTrajectory(
      num_steps, dt, base_pose_init, des_poses, des_poses_t);
  NonlinearFactorGraph collocation_costs =
      vision60.collocationCosts(num_steps, dt);
  NonlinearFactorGraph boundary_costs = vision60.boundaryCosts(
      base_pose_init, base_twist_init, des_poses, des_poses_t, dt);
  NonlinearFactorGraph min_torque_costs = vision60.minTorqueCosts(num_steps);
  NonlinearFactorGraph friction_cone_costs = vision60.frictionConeCosts(num_steps);
  NonlinearFactorGraph costs;
  costs.add(collocation_costs);
  costs.add(boundary_costs);
  costs.add(min_torque_costs);
  costs.add(friction_cone_costs);
  auto EvaluateCosts = [=] (const Values& values) {
    std::cout << "collocation costs:\t" << collocation_costs.error(values) << "\n";
    std::cout << "boundary costs:\t" << boundary_costs.error(values) << "\n";
    std::cout << "min torque costs:\t" << min_torque_costs.error(values) << "\n";
    std::cout << "friction cone costs:\t" << friction_cone_costs.error(values) << "\n";
  };

  /// Export initial trajectory
  EvaluateCosts(init_values);
  vision60.exportTrajectory(
      init_values, num_steps,
      std::string(kDataPath) + run_options.benchmark_id + "_init_traj.csv");

  auto create_problem = [=]() { return EConsOptProblem(costs, constraints, init_values); };

  LevenbergMarquardtParams lm_params;
  lm_params.minModelFidelity = 0.3;
  lm_params.linearSolverType =
      gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
  lm_params.setMaxIterations(30);
  lm_params.relativeErrorTol = 1e-3;
  lm_params.setlambdaUpperBound(1e2);

  ConstrainedOptBenchmarkRunner runner(run_options);
  runner.setProblemFactory(create_problem);
  runner.setOuterLmBaseParams(lm_params);
  runner.setOuterLmConfig(
      [](BenchmarkMethod method, LevenbergMarquardtParams* params) {
        if (method == BenchmarkMethod::CM_F || method == BenchmarkMethod::CM_I) {
          params->setlambdaInitial(1e1);
        }
      });
  runner.setMoptFactory([&](BenchmarkMethod) {
    auto mopt_params = DefaultMoptParamsSV(vision60.getBasisKeyFunc());
    mopt_params.cc_params->retractor_creator->params()->use_basis_keys = true;
    mopt_params.cc_params->retractor_creator->params()->sigma = 1.0;
    mopt_params.cc_params->retractor_creator->params()->apply_base_retraction =
        true;
    mopt_params.cc_params->retractor_creator->params()->check_feasible = true;
    mopt_params.cc_params->retractor_creator->params()
        ->lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    mopt_params.cc_params->retractor_creator->params()->lm_params.setlambdaUpperBound(
        1e2);
    return mopt_params;
  });
  runner.setResultCallback([&](BenchmarkMethod method, const Values& result) {
    EvaluateCosts(result);
    vision60.exportTrajectory(
        result, num_steps,
        BenchmarkMethodDataPath(run_options, method, "_traj.csv"));
  });

  std::ostringstream latex_os;
  runner.run(latex_os);
  std::cout << latex_os.str();
}


int main(int argc, char **argv) {
  try {
    BenchmarkCliDefaults defaults;
    defaults.benchmark_id = "quadruped";
    defaults.enable_num_steps = true;
    defaults.default_num_steps = 30;
    defaults.default_methods = {BenchmarkMethod::SOFT, BenchmarkMethod::CM_I};

    auto parsed = ParseBenchmarkCli(argc, argv, defaults);
    if (!parsed.unknown_args.empty()) {
      throw std::invalid_argument("Unknown option: " + parsed.unknown_args.front());
    }

    parsed.run_options.soft_mu = 1e2;
    parsed.run_options.constraint_unit_scale = kConstraintUnitScale;

    std::cout << "Using num_steps=" << parsed.num_steps << "\n";
    std::cout << "Methods: " << "\n";
    for (BenchmarkMethod method : parsed.run_options.methods) {
      std::cout << "  - " << BenchmarkMethodToken(method) << "\n";
    }
    TrajectoryOptimization(parsed.num_steps, parsed.run_options);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    BenchmarkCliDefaults defaults;
    defaults.benchmark_id = "quadruped";
    defaults.enable_num_steps = true;
    defaults.default_num_steps = 30;
    defaults.default_methods = {BenchmarkMethod::SOFT, BenchmarkMethod::CM_I};
    PrintBenchmarkUsage(std::cerr, argv[0], defaults);
    return 1;
  }
}
