/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main_cablerobot.cpp
 * @brief Cable robot kinematic trajectory benchmark using constrained
 * optimization methods.
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
#include <iostream>
#include <stdexcept>
#include <string>

using namespace gtsam;
using namespace gtdynamics;

namespace {
constexpr size_t kNumSteps = 10;
const auto kJointNoise = noiseModel::Isotropic::Sigma(1, 1.0);
const auto kJointLimitNoise = noiseModel::Isotropic::Sigma(1, 1e-2);
const auto kTargetNoise = noiseModel::Isotropic::Sigma(6, 1e-2);

constexpr double kJointLimitLow = 0.0;
constexpr double kJointLimitHigh = 3.6;

const Key kEeId = 1;
const Pose3 kTargetPose = Pose3(Rot3(), Point3(0.2, 0.5, 0.0));

}  // namespace

namespace {
const double kCdprWidth = 3, kCdprHeight = 2;
const std::array<Point3, 4> kCdprFrameMountPoints = {
    Point3{kCdprWidth, 0, 0},  //
    Point3{kCdprWidth, kCdprHeight, 0}, Point3{0, kCdprHeight, 0},
    Point3{0, 0, 0}};
const double kCdprEeWidth = 0.1, kCdprEeHeight = 0.1;
const std::array<Point3, 4> kCdprEeMountPoints = {
    Point3{kCdprEeWidth, 0, 0}, Point3{kCdprEeWidth, kCdprEeHeight, 0},
    Point3{0, kCdprEeHeight, 0}, Point3{0, 0, 0}};
}  // namespace

/** Factor graph of all kinematic constraints. */
NonlinearFactorGraph get_constraints_graph() {
  NonlinearFactorGraph constraints_graph;
  auto graph_builder = DynamicsGraph();  //
  // kinematics constraints at each step
  for (size_t k = 0; k <= kNumSteps; ++k) {
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

/** Build joint rotation, limit, and terminal pose costs. */
NonlinearFactorGraph get_costs() {
  NonlinearFactorGraph costs;
  // rotation costs
  NonlinearFactorGraph rotation_costs;
  for (size_t k = 0; k < kNumSteps; k++) {
    for (size_t ci = 0; ci < 4; ++ci) {
      rotation_costs.emplace_shared<BetweenFactor<double>>(
          JointAngleKey(ci, k), JointAngleKey(ci, k + 1), 0.0, kJointNoise);
    }
  }
  // joint limit costs;
  NonlinearFactorGraph joint_limit_costs;
  for (size_t k = 0; k <= kNumSteps; k++) {
    for (size_t ci = 0; ci < 4; ++ci) {
      joint_limit_costs.emplace_shared<JointLimitFactor>(
          JointAngleKey(ci, k), kJointLimitNoise, kJointLimitLow,
          kJointLimitHigh, 0.0);
    }
  }
  // target cost
  NonlinearFactorGraph target_costs;
  costs.addPrior(PoseKey(kEeId, kNumSteps), kTargetPose, kTargetNoise);
  costs.add(rotation_costs);
  costs.add(target_costs);
  costs.add(joint_limit_costs);
  return costs;
}

/** Build an initial guess for cable lengths and end-effector pose. */
Values get_init_values() {
  Values init_values;
  for (size_t k = 0; k <= kNumSteps; k++) {
    for (size_t ci = 0; ci < 4; ++ci) {
      InsertJointAngle(&init_values, ci, k, 1.8);
    }
    InsertPose(&init_values, kEeId, k, Pose3(Rot3(), Point3(1.5, 1.0, 0)));
  }
  return init_values;
}

/** Print joint angles for all steps. */
void print_joint_angles(const Values& values) {
  for (size_t k = 0; k <= kNumSteps; k++) {
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
void kinematic_planning(const BenchmarkRunOptions& run_options) {
  // Create constrained optimization problem.
  auto constraints_graph = get_constraints_graph();
  auto costs = get_costs();
  auto init_values = get_init_values();
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph);
  auto problem = EConsOptProblem(costs, constraints, init_values);

  LevenbergMarquardtParams lm_params;
  lm_params.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
  ConstrainedOptBenchmarkRunner runner(run_options);
  runner.setProblemFactory([=]() { return EConsOptProblem(costs, constraints, init_values); });
  runner.setOuterLmBaseParams(lm_params);
  runner.setMoptFactory([](BenchmarkMethod) {
    auto mopt_params = DefaultMoptParams();
    mopt_params.cc_params->retractor_creator->params()->lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
    mopt_params.cc_params->retractor_creator->params()->lm_params.setlambdaUpperBound(
        1e2);
    return mopt_params;
  });

  std::ostringstream latex_os;
  runner.run(latex_os);
  std::cout << latex_os.str();
}

int main(int argc, char** argv) {
  try {
    BenchmarkCliDefaults defaults;
    defaults.benchmark_id = "cable_robot";
    auto parsed = ParseBenchmarkCli(argc, argv, defaults);
    if (!parsed.unknown_args.empty()) {
      throw std::invalid_argument("Unknown option: " + parsed.unknown_args.front());
    }
    kinematic_planning(parsed.run_options);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    BenchmarkCliDefaults defaults;
    defaults.benchmark_id = "cable_robot";
    PrintBenchmarkUsage(std::cerr, argv[0], defaults);
    return 1;
  }
}
