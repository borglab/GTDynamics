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
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace gtdynamics;
using namespace gtsam;

namespace {
struct QuadrupedArgs {
  size_t num_steps = 30;
  bool run_soft = true;
  bool run_cm_f = false;
  bool run_cm_i = true;
};

void PrintUsage(const char* progname) {
  std::cerr << "Usage: " << progname
            << " [--num-steps N|-n N] [--methods LIST|-m LIST]\n"
               "  LIST is comma-separated tokens from {soft,f,i,all}\n"
               "  Defaults: --num-steps 30, --methods soft,i\n";
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

void ParseMethods(const std::string& methods_text, QuadrupedArgs* args) {
  args->run_soft = false;
  args->run_cm_f = false;
  args->run_cm_i = false;

  for (const auto& token_raw : Split(methods_text, ',')) {
    const std::string token = ToLower(Trim(token_raw));
    if (token.empty()) continue;
    if (token == "all") {
      args->run_soft = true;
      args->run_cm_f = true;
      args->run_cm_i = true;
      continue;
    }
    if (token == "soft") {
      args->run_soft = true;
      continue;
    }
    if (token == "f" || token == "cmf") {
      args->run_cm_f = true;
      continue;
    }
    if (token == "i" || token == "cmi") {
      args->run_cm_i = true;
      continue;
    }
    throw std::invalid_argument("Unknown method token: " + token_raw);
  }

  if (!args->run_soft && !args->run_cm_f && !args->run_cm_i) {
    throw std::invalid_argument("No valid methods selected.");
  }
}

QuadrupedArgs ParseArgs(int argc, char** argv) {
  QuadrupedArgs args;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else if (arg == "-n" || arg == "--num-steps") {
      if (i + 1 >= argc) throw std::invalid_argument("Missing value for --num-steps");
      args.num_steps = std::stoul(argv[++i]);
    } else if (arg.rfind("--num-steps=", 0) == 0) {
      args.num_steps = std::stoul(arg.substr(std::string("--num-steps=").size()));
    } else if (arg == "-m" || arg == "--methods") {
      if (i + 1 >= argc) throw std::invalid_argument("Missing value for --methods");
      ParseMethods(argv[++i], &args);
    } else if (arg.rfind("--methods=", 0) == 0) {
      ParseMethods(arg.substr(std::string("--methods=").size()), &args);
    } else if (std::all_of(arg.begin(), arg.end(),
                           [](unsigned char c) { return std::isdigit(c); })) {
      // Backward-compatible positional form: `<prog> 5`
      args.num_steps = std::stoul(arg);
    } else {
      throw std::invalid_argument("Unknown argument: " + arg);
    }
  }

  if (args.num_steps == 0) {
    throw std::invalid_argument("--num-steps must be greater than 0");
  }

  return args;
}
}  // namespace

void TrajectoryOptimization(const QuadrupedArgs& args) {
  const size_t num_steps = args.num_steps;
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
  auto init_values = vision60.getInitValuesTrajectory(num_steps, dt, base_pose_init, des_poses, des_poses_t);
  NonlinearFactorGraph collocation_costs = vision60.collocationCosts(num_steps, dt);
  NonlinearFactorGraph boundary_costs = vision60.boundaryCosts(base_pose_init, base_twist_init, des_poses, des_poses_t, dt);
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
  vision60.exportTrajectory(init_values, num_steps,
                            std::string(kDataPath) + "init_traj.csv");


  /// Construct problem
  auto problem = EConsOptProblem(costs, constraints, init_values);
  std::ostringstream latex_os;
  LevenbergMarquardtParams lm_params;
  lm_params.minModelFidelity = 0.3;
  lm_params.linearSolverType =
      gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
  lm_params.setMaxIterations(30);
  lm_params.relativeErrorTol = 1e-3;
  // lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e2);

  const double kConstraintUnitScale = 1e-3;

  // optimize soft constraints
  if (args.run_soft) {
    std::cout << "soft constraints:\n";
    auto soft_result = OptimizeE_SoftConstraints(problem, latex_os, lm_params, 1e2,
                                                 kConstraintUnitScale);
    EvaluateCosts(soft_result);
    vision60.exportTrajectory(soft_result, num_steps,
                              std::string(kDataPath) + "soft_traj.csv");
  } else {
    std::cout << "soft constraints: skipped\n";
  }

  // // optimize penalty method
  // std::cout << "penalty method:\n";
  // PenaltyOptimizerParams penalty_params;
  // penalty_params.lm_params = lm_params;
  // auto penalty_result =
  //     OptimizeE_Penalty(problem, latex_os, penalty_params, kConstraintUnitScale);
  // EvaluateCosts(penalty_result);

  // // optimize augmented lagrangian
  // std::cout << "augmented lagrangian:\n";
  // AugmentedLagrangianParameters almParams;
  // almParams.lm_params = lm_params;
  // auto almResult =
  //     OptimizeE_AugmentedLagrangian(problem, latex_os, almParams, kConstraintUnitScale);
  // EvaluateCosts(almResult);

  if (args.run_cm_f || args.run_cm_i) {
    lm_params.setlambdaInitial(1e1);
    auto mopt_params = DefaultMoptParamsSV(vision60.getBasisKeyFunc());
    mopt_params.cc_params->retractor_creator->params()->use_basis_keys = true;
    mopt_params.cc_params->retractor_creator->params()->sigma = 1.0;
    mopt_params.cc_params->retractor_creator->params()->apply_base_retraction =
        true;

    mopt_params.cc_params->retractor_creator->params()->lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    mopt_params.cc_params->retractor_creator->params()->lm_params.setlambdaUpperBound(
        1e2);
    mopt_params.cc_params->retractor_creator->params()->lm_params.setMaxIterations(
        10);
    mopt_params.cc_params->retractor_creator->params()->check_feasible = true;

    if (args.run_cm_f) {
      std::cout << "constraint manifold basis variables feasible:\n";
      auto cm_result = OptimizeE_CMOpt(problem, latex_os, mopt_params, lm_params,
                                       "Constraint Manifold (F)",
                                       kConstraintUnitScale);
      EvaluateCosts(cm_result);
      vision60.exportTrajectory(cm_result, num_steps,
                                std::string(kDataPath) + "cm_traj.csv");
    } else {
      std::cout << "constraint manifold basis variables feasible: skipped\n";
    }

    if (args.run_cm_i) {
      std::cout << "constraint manifold basis variables infeasible:\n";
      mopt_params.cc_params->retractor_creator->params()->lm_params.setMaxIterations(
          1);
      mopt_params.retract_final = true;
      auto cm_infeasible_result = OptimizeE_CMOpt(
          problem, latex_os, mopt_params, lm_params, "Constraint Manifold (I)",
          kConstraintUnitScale);
      EvaluateCosts(cm_infeasible_result);
      vision60.exportTrajectory(cm_infeasible_result, num_steps,
                                std::string(kDataPath) + "cm_infeas_traj.csv");
    } else {
      std::cout
          << "constraint manifold basis variables infeasible: skipped\n";
    }
  } else {
    std::cout << "constraint manifold methods: skipped\n";
  }


  std::cout << latex_os.str();
}


int main(int argc, char **argv) {
  try {
    const auto args = ParseArgs(argc, argv);
    std::cout << "Using num_steps=" << args.num_steps << "\n";
    std::cout << "Methods: soft=" << args.run_soft
              << ", f=" << args.run_cm_f
              << ", i=" << args.run_cm_i << "\n";
    TrajectoryOptimization(args);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
