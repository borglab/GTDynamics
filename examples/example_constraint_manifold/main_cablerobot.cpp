/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main_cablerobot.cpp
 * @brief Kinematic trajectory planning problem of reaching a target pose with a
 * cable robot. Benchmarking dynamic factor graph, constraint manifold, manually
 * specified manifold.
 * @author Yetong Zhang
 * @author Gerry Chen
 */

#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>
#include <gtdynamics/cablerobot/factors/CableLengthFactor.h>
#include <gtdynamics/cablerobot/factors/CableTensionFactor.h>
#include <gtdynamics/cablerobot/factors/CableVelocityFactor.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/JointLimitFactor.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace gtsam;
using namespace gtdynamics;

namespace {

struct CableRobotArgs {
  bool verbose_benchmark = false;
  bool verbose_retractor = false;
};

void PrintUsage(const char* program_name) {
  std::cout
      << "Usage: " << program_name << " [args]\n"
      << "Options:\n"
      << "  --verbose-benchmark   Enable outer LM summary output.\n"
      << "  --verbose-retractor   Enable retraction LM summary output.\n"
      << "  --help                Show this message.\n";
}

CableRobotArgs ParseArgs(int argc, char** argv) {
  CableRobotArgs args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--verbose-benchmark") {
      args.verbose_benchmark = true;
    } else if (arg == "--verbose-retractor") {
      args.verbose_retractor = true;
    } else if (arg == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::invalid_argument("Unknown option: " + arg);
    }
  }
  return args;
}
}  // namespace

const size_t num_steps = 10;
auto joint_noise = noiseModel::Isotropic::Sigma(1, 1.0);
auto joint_limit_noise = noiseModel::Isotropic::Sigma(1, 1e-2);
auto target_noise = noiseModel::Isotropic::Sigma(6, 1e-2);

const double kCdprWidth = 3, kCdprHeight = 2;
const std::array<Point3, 4> kCdprFrameMountPoints = {
    Point3{kCdprWidth, 0, 0},  //
    Point3{kCdprWidth, kCdprHeight, 0}, Point3{0, kCdprHeight, 0},
    Point3{0, 0, 0}};
const double kCdprEeWidth = 0.1, kCdprEeHeight = 0.1;
const std::array<Point3, 4> kCdprEeMountPoints = {
    Point3{kCdprEeWidth, 0, 0}, Point3{kCdprEeWidth, kCdprEeHeight, 0},
    Point3{0, kCdprEeHeight, 0}, Point3{0, 0, 0}};

const Key kEeId = 1;
const Pose3 target_pose = Pose3(Rot3(), Point3(0.2, 0.5, 0.0));
double joint_limit_low = 0;
double joint_limit_high = 3.6;

/** Factor graph of all kinematic constraints. */
NonlinearFactorGraph get_constraints_graph() {
  NonlinearFactorGraph constraints_graph;
  auto graph_builder = DynamicsGraph();  //
  // kinematics constraints at each step
  for (size_t k = 0; k <= num_steps; ++k) {
    for (size_t ci = 0; ci < 4; ++ci) {
      constraints_graph.emplace_shared<CableLengthFactor>(
          JointAngleKey(ci, k), PoseKey(kEeId, k),
          gtsam::noiseModel::Isotropic::Sigma(1, 0.001),
          kCdprFrameMountPoints[ci], kCdprEeMountPoints[ci]);
    }
  }
  // constraints_graph.print("constraints_graph", GTDKeyFormatter);
  // // prior constraints at first step
  constraints_graph.addPrior(PoseKey(kEeId, 0),
                             Pose3(Rot3(), Point3(1.5, 1.0, 0)),
                             gtsam::noiseModel::Isotropic::Sigma(6, 0.001));
  return constraints_graph;
}

/** Cost for planning, includes joint rotation costs, joint limit costs, cost
 * for reaching target pose. */
NonlinearFactorGraph get_costs() {
  NonlinearFactorGraph costs;
  // rotation costs
  NonlinearFactorGraph rotation_costs;
  for (size_t k = 0; k < num_steps; k++) {
    for (size_t ci = 0; ci < 4; ++ci) {
      rotation_costs.emplace_shared<BetweenFactor<double>>(
          JointAngleKey(ci, k), JointAngleKey(ci, k + 1), 0.0, joint_noise);
    }
  }
  // joint limit costs;
  NonlinearFactorGraph joint_limit_costs;
  for (size_t k = 0; k <= num_steps; k++) {
    for (size_t ci = 0; ci < 4; ++ci) {
      joint_limit_costs.emplace_shared<JointLimitFactor>(
          JointAngleKey(ci, k), joint_limit_noise, joint_limit_low,
          joint_limit_high, 0.0);
    }
  }
  // target cost
  NonlinearFactorGraph target_costs;
  costs.addPrior(PoseKey(kEeId, num_steps), target_pose, target_noise);
  costs.add(rotation_costs);
  costs.add(target_costs);
  costs.add(joint_limit_costs);
  return costs;
}

/** Initial values specifed by 0 for all joint angles. */
Values get_init_values() {
  Values init_values;
  for (size_t k = 0; k <= num_steps; k++) {
    for (size_t ci = 0; ci < 4; ++ci) {
      InsertJointAngle(&init_values, ci, k, 1.8);
    }
    InsertPose(&init_values, kEeId, k, Pose3(Rot3(), Point3(1.5, 1.0, 0)));
  }
  return init_values;
}

/** Print joint angles for all steps. */
void print_joint_angles(const Values& values) {
  for (size_t k = 0; k <= num_steps; k++) {
    std::cout << "step " << k << ": ";
    for (size_t ci = 0; ci < 4; ++ci) {
      double angle = JointAngle(values, ci, k);
      std::cout << "\t" << angle;
    }
    std::cout << "\n";
  }
}

/** Compare simple kinematic planning tasks of a cable robot using (1) dynamics
 * factor graph (2) constraint manifold  */
void kinematic_planning(const CableRobotArgs& args) {
  // Create constrained optimization problem.
  auto constraints_graph = get_constraints_graph();
  auto costs = get_costs();
  auto init_values = get_init_values();
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph);
  auto problem = EConsOptProblem(costs, constraints, init_values);

  std::ostringstream latex_os;
  LevenbergMarquardtParams lm_params;
  lm_params.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
  if (args.verbose_benchmark) {
    lm_params.setVerbosityLM("SUMMARY");
    std::cout << "[BENCH] Verbose mode enabled for cable_robot benchmark.\n";
  }

  // optimize soft constraints
  std::cout << "soft constraints:\n";
  auto soft_result =
      OptimizeE_SoftConstraints(problem, latex_os, lm_params, 1.0);

  // optimize penalty method
  std::cout << "penalty method:\n";
  auto penalty_params = std::make_shared<gtsam::PenaltyOptimizerParams>();
  penalty_params->lmParams = lm_params;
  auto penalty_result =
      OptimizeE_Penalty(problem, latex_os, penalty_params);

  // optimize augmented lagrangian
  std::cout << "augmented lagrangian:\n";
  auto almParams = std::make_shared<gtsam::AugmentedLagrangianParams>();
  almParams->lmParams = lm_params;
  auto almResult =
      OptimizeE_AugmentedLagrangian(problem, latex_os, almParams);

  // optimize constraint manifold specify variables (feasible)
  std::cout << "constraint manifold basis variables (feasible):\n";
  auto mopt_params = DefaultMoptParams();
  mopt_params.cc_params->retractor_creator->params()->lm_params.linearSolverType =
      gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
  mopt_params.cc_params->retractor_creator->params()->lm_params.setlambdaUpperBound(
      1e2);
  mopt_params.cc_params->retractor_creator->params()->lm_params.setMaxIterations(
      10);
  if (args.verbose_retractor) {
    mopt_params.cc_params->retractor_creator->params()->lm_params.setVerbosityLM(
        "SUMMARY");
    std::cout << "[BENCH] Retraction LM verbosity enabled.\n";
  }
  auto cm_basis_result = OptimizeE_CMOpt(
      problem, latex_os, mopt_params, lm_params, "Constraint Manifold (F)");

  // optimize constraint manifold specify variables (infeasible)
  std::cout << "constraint manifold basis variables (infeasible):\n";
  mopt_params.cc_params->retractor_creator->params()->lm_params.setMaxIterations(1);
  auto cm_basis_infeasible_result = OptimizeE_CMOpt(
      problem, latex_os, mopt_params, lm_params, "Constraint Manifold (I)");

  std::cout << latex_os.str();
}

int main(int argc, char** argv) {
  try {
    const CableRobotArgs args = ParseArgs(argc, argv);
    kinematic_planning(args);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
