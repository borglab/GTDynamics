/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main_arm_kinematic_planning.cpp
 * @brief KUKA arm kinematic trajectory benchmark using constrained optimization
 * methods.
 * @author Yetong Zhang
 */

#include "SerialChain.h"

#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/JointLimitFactor.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/slam/BetweenFactor.h>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gtsam;
using namespace gtdynamics;

namespace {
constexpr size_t kNumSteps = 10;
const auto kJointNoise = noiseModel::Isotropic::Sigma(1, 1.0);
const auto kJointLimitNoise = noiseModel::Isotropic::Sigma(1, 1e-2);
const auto kTargetNoise = noiseModel::Isotropic::Sigma(6, 1e-2);
const std::string kSdfFile = "kuka_world.sdf";
const std::string kRobotName = "lbr_iiwa";
const std::string kBaseName = "lbr_iiwa_link_0";
const std::string kEeName = "lbr_iiwa_link_7";
const Pose3 kTargetPose = Pose3(Rot3(), Point3(0, 0.2, 1.0));
const Pose3 kBasePose;
constexpr double kJointLimitLow = -3.14;
constexpr double kJointLimitHigh = 3.14;

Robot& KukaRobot() {
  static Robot robot = CreateRobotFromFile(kSdfPath + kSdfFile, kRobotName);
  return robot;
}

struct ArmBenchmarkArgs {
  bool verbose_benchmark = false;
  bool verbose_retractor = false;
  bool cm_i_only = false;
  bool cm_f_only = false;
  bool skip_cm_f = false;
};

void PrintUsage(const char* program_name) {
  std::cout
      << "Usage: " << program_name << " [args]\n"
      << "Options:\n"
      << "  --verbose-benchmark     Enable outer LM summary output.\n"
      << "  --verbose-retractor     Enable retraction LM summary output.\n"
      << "  --cm-i-only             Run only CM(I) benchmark variant.\n"
      << "  --cm-f-only             Run only CM(F) benchmark variant.\n"
      << "  --skip-cm-f             Skip CM(F) in mixed run.\n"
      << "  --help                  Show this message.\n";
}

ArmBenchmarkArgs ParseArgs(int argc, char** argv) {
  ArmBenchmarkArgs args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--verbose-benchmark") {
      args.verbose_benchmark = true;
    } else if (arg == "--verbose-retractor") {
      args.verbose_retractor = true;
    } else if (arg == "--cm-i-only") {
      args.cm_i_only = true;
    } else if (arg == "--cm-f-only") {
      args.cm_f_only = true;
    } else if (arg == "--skip-cm-f") {
      args.skip_cm_f = true;
    } else if (arg == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::invalid_argument("Unknown option: " + arg);
    }
  }

  if (args.cm_i_only && args.cm_f_only) {
    throw std::invalid_argument(
        "Cannot combine --cm-i-only and --cm-f-only.");
  }
  return args;
}
}  // namespace

/** Functor version of JointLimitFactor, for creating expressions. Compute error
 * for joint limit error, to reproduce joint limit factor in expressions. */
class JointLimitFunctor {
protected:
  double low_, high_;

public:
  JointLimitFunctor(const double &low, const double &high)
      : low_(low), high_(high) {}

  double operator()(const double &q,
                    OptionalJacobian<1, 1> H_q = nullptr) const {
    if (q < low_) {
      if (H_q)
        *H_q = -I_1x1;
      return low_ - q;
    } else if (q <= high_) {
      if (H_q)
        *H_q = Z_1x1;
      return 0.0;
    } else {
      if (H_q)
        *H_q = I_1x1;
      return q - high_;
    }
  }
};

/** Function to manually define the basis keys for each constraint manifold. */
KeyVector FindBasisKeys(const KeyVector& keys) {
  KeyVector basis_keys;
  for (const Key& key : keys) {
    auto symbol = DynamicsSymbol(key);
    if (symbol.label() == "q") {
      basis_keys.push_back(key);
    }
  }
  return basis_keys;
}

/** Factor graph of all kinematic constraints. Include kinematic constraints at
 * each time step, and the priors for the first step. */
NonlinearFactorGraph get_constraints_graph() {
  auto& robot = KukaRobot();
  NonlinearFactorGraph constraints_graph;
  // kinematics constraints at each step
  auto graph_builder = DynamicsGraph();
  for (size_t k = 0; k <= kNumSteps; k++) {
    constraints_graph.add(graph_builder.qFactors(robot, k));
  }

  // // prior constraints at first step
  size_t k0 = 0;
  for (const auto& joint : robot.joints()) {
    constraints_graph.addPrior(JointAngleKey(joint->id(), k0), 0.0,
                               graph_builder.opt().prior_q_cost_model);
  }
  return constraints_graph;
}

/** Cost function for planning, includes cost of rotation joints, joint limit
 * costs, and cost for reaching target pose at end-effector. */
NonlinearFactorGraph get_costs() {
  auto& robot = KukaRobot();
  NonlinearFactorGraph costs;
  // rotation costs
  NonlinearFactorGraph rotation_costs;
  for (size_t k = 0; k < kNumSteps; k++) {
    for (const auto& joint : robot.joints()) {
      rotation_costs.emplace_shared<BetweenFactor<double>>(
          JointAngleKey(joint->id(), k), JointAngleKey(joint->id(), k + 1), 0.0,
          kJointNoise);
    }
  }
  // joint limit costs;
  NonlinearFactorGraph joint_limit_costs;
  for (size_t k = 0; k <= kNumSteps; k++) {
    for (const auto& joint : robot.joints()) {
      joint_limit_costs.emplace_shared<JointLimitFactor>(
          JointAngleKey(joint->id(), k), kJointLimitNoise, kJointLimitLow,
          kJointLimitHigh, 0.0);
    }
  }
  // target cost
  NonlinearFactorGraph target_costs;
  const auto& ee_link = robot.link(kEeName);
  costs.addPrior(PoseKey(ee_link->id(), kNumSteps), kTargetPose, kTargetNoise);
  costs.add(rotation_costs);
  costs.add(target_costs);
  costs.add(joint_limit_costs);
  return costs;
}

/** Initial values specified by 0 for all joint angles. */
Values get_init_values() {
  auto& robot = KukaRobot();
  Values init_values;
  Values fk_values;
  size_t k0 = 0;
  for (const auto& joint : robot.joints()) {
    InsertJointAngle(&fk_values, joint->id(), k0, 0.0);
  }
  fk_values = robot.forwardKinematics(fk_values, k0);
  // fk_values.print("", GTDKeyFormatter);
  for (size_t k = 0; k <= kNumSteps; k++) {
    for (const auto& joint : robot.joints()) {
      double angle = JointAngle(fk_values, joint->id(), k0);
      InsertJointAngle(&init_values, joint->id(), k, angle);
    }
    for (const auto& link : robot.links()) {
      Pose3 pose = Pose(fk_values, link->id(), k0);
      InsertPose(&init_values, link->id(), k, pose);
    }
  }
  return init_values;
}

/** Initial values of serial chain specified by 0 for all joint angles. */
Values get_init_values_sc() {
  auto& robot = KukaRobot();
  Values init_values;
  auto shared_robot = std::make_shared<Robot>(robot);
  Values joint_angles;
  for (const auto& joint : robot.joints()) {
    gtdynamics::InsertJointAngle(&joint_angles, joint->id(), 0.0);
  }
  for (size_t k = 1; k <= kNumSteps; k++) {
    Key sc_key = k;
    init_values.insert(sc_key, SerialChain<7>(shared_robot, kBaseName,
                                              kBasePose, joint_angles));
  }
  return init_values;
}

/** Same cost function, but imposed on the serial chain manifold. */
NonlinearFactorGraph get_costs_sc() {
  auto& robot = KukaRobot();
  NonlinearFactorGraph costs;
  // rotation costs
  Expression<SerialChain<7>> state1(1);
  for (const auto& joint : robot.joints()) {
    auto joint_func = std::bind(&SerialChain<7>::joint, std::placeholders::_1,
                                joint->name(), std::placeholders::_2);
    Expression<double> curr_joint(joint_func, state1);
    costs.addExpressionFactor<double>(kJointNoise, 0.0, curr_joint);
  }
  for (size_t k = 1; k < kNumSteps; k++) {
    Key curr_key = k;
    Key next_key = k + 1;
    Expression<SerialChain<7>> curr_state(curr_key);
    Expression<SerialChain<7>> next_state(next_key);
    for (const auto& joint : robot.joints()) {
      auto joint_func = std::bind(&SerialChain<7>::joint, std::placeholders::_1,
                                  joint->name(), std::placeholders::_2);
      Expression<double> curr_joint(joint_func, curr_state);
      Expression<double> next_joint(joint_func, next_state);
      Expression<double> joint_rotation_expr = next_joint - curr_joint;
      costs.addExpressionFactor<double>(kJointNoise, 0.0, joint_rotation_expr);
    }
  }

  // joint limit costs
  JointLimitFunctor joint_limit_functor(kJointLimitLow, kJointLimitHigh);
  for (size_t k = 1; k <= kNumSteps; k++) {
    Key curr_key = k;
    Expression<SerialChain<7>> curr_state(curr_key);
    for (const auto& joint : robot.joints()) {
      auto joint_func = std::bind(&SerialChain<7>::joint, std::placeholders::_1,
                                  joint->name(), std::placeholders::_2);
      Expression<double> curr_joint(joint_func, curr_state);
      Expression<double> joint_limit_error(joint_limit_functor, curr_joint);
      costs.addExpressionFactor<double>(kJointLimitNoise, 0.0,
                                        joint_limit_error);
    }
  }

  // target costs
  Key last_key = kNumSteps;
  Expression<SerialChain<7>> last_state(last_key);
  auto pose_func = std::bind(&SerialChain<7>::linkPose, std::placeholders::_1,
                             kEeName, std::placeholders::_2);
  Expression<Pose3> target_pose_expr(pose_func, last_state);
  costs.addExpressionFactor<Pose3>(kTargetNoise, kTargetPose, target_pose_expr);
  return costs;
}

/** Kinematic trajectory optimization using manually defined serial chain
 * manifold. */
Values optimize_serial_chain_manifold(const NonlinearFactorGraph& costs,
                                      const Values& init_values) {
  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  size_t graph_dim = 0;
  for (const auto& factor : costs) {
    graph_dim += factor->dim();
  }
  std::cout << "dimension: " << graph_dim << " x " << init_values.dim() << "\n";
  LevenbergMarquardtOptimizer optimizer(costs, init_values, params);
  gttic_(serial_chain_manifold);
  auto result = optimizer.optimize();
  gttoc_(serial_chain_manifold);
  return result;
}

/** Print joint angles for all steps. */
void print_joint_angles(const Values& values) {
  auto& robot = KukaRobot();
  for (size_t k = 0; k <= kNumSteps; k++) {
    std::cout << "step " << k << ":";
    for (const auto& joint : robot.joints()) {
      double angle = JointAngle(values, joint->id(), k);
      std::cout << "\t" << angle;
    }
    std::cout << "\n";
  }
}

/** Print joint angles of serial chain for all steps. */
void print_joint_angles_sc(const Values& values) {
  auto& robot = KukaRobot();
  for (size_t k = 1; k <= kNumSteps; k++) {
    Key state_key = k;
    auto state = values.at<SerialChain<7>>(state_key);
    std::cout << "step " << k << ":";
    for (const auto& joint : robot.joints()) {
      double angle = state.joint(joint->name());
      std::cout << "\t" << angle;
    }
    std::cout << "\n";
  }
}

/** Benchmark constrained optimizers on a KUKA arm kinematic planning problem. */
void kinematic_planning(const ArmBenchmarkArgs& args) {
  auto& robot = KukaRobot();
  // Create constrained optimization problem.
  robot.fixLink(kBaseName);
  const auto createProblem = []() {
    auto constraints_graph = get_constraints_graph();
    auto costs = get_costs();
    auto init_values = get_init_values();
    auto constraints =
        gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph);
    return EConsOptProblem(costs, constraints, init_values);
  };

  std::ostringstream latex_os;
  LevenbergMarquardtParams lm_params;
  if (args.cm_f_only) {
    std::cout << "[BENCH] CM(F)-only mode enabled.\n";
  }
  if (args.verbose_benchmark) {
    lm_params.setVerbosityLM("SUMMARY");
    std::cout << "[BENCH] Verbose mode enabled for ARM benchmark.\n";
  }
  if (args.cm_i_only) {
    lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    lm_params.setMaxIterations(20);
    lm_params.relativeErrorTol = 1e-3;
    std::cout << "[BENCH] I-only mode: using fast outer-LM settings "
                 "(SEQUENTIAL_CHOLESKY, maxIterations=20, relativeErrorTol=1e-3).\n";
  }
  if (args.cm_f_only) {
    lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    lm_params.setMaxIterations(30);
    lm_params.relativeErrorTol = 1e-3;
    lm_params.setlambdaUpperBound(1e2);
    std::cout << "[BENCH] F-only mode: using bounded outer-LM settings "
                 "(SEQUENTIAL_CHOLESKY, maxIterations=30, relativeErrorTol=1e-3, "
                 "lambdaUpperBound=1e2).\n";
  }

  if (!args.cm_i_only && !args.cm_f_only) {
    // optimize soft constraints
    std::cout << "soft constraints:\n";
    auto soft_problem = createProblem();
    auto soft_result =
        OptimizeE_SoftConstraints(soft_problem, latex_os, lm_params, 1.0);

    // optimize penalty method
    std::cout << "penalty method:\n";
    auto penalty_problem = createProblem();
    auto penalty_params = std::make_shared<gtsam::PenaltyOptimizerParams>();
    penalty_params->lmParams = lm_params;
    auto penalty_result =
        OptimizeE_Penalty(penalty_problem, latex_os, penalty_params);

    // optimize augmented lagrangian
    std::cout << "augmented lagrangian:\n";
    auto alm_problem = createProblem();
    auto almParams = std::make_shared<gtsam::AugmentedLagrangianParams>();
    almParams->lmParams = lm_params;
    auto almResult =
        OptimizeE_AugmentedLagrangian(alm_problem, latex_os, almParams);
  } else if (args.cm_i_only) {
    std::cout << "[BENCH] I-only mode: skipping soft, penalty, augmented "
                 "lagrangian, and CM(F).\n";
  } else {
    std::cout << "[BENCH] F-only mode: skipping soft, penalty, augmented "
                 "lagrangian, and CM(I).\n";
  }

  // optimize constraint manifold specify variables (feasible)
  auto mopt_params = DefaultMoptParamsSV(&FindBasisKeys);
  mopt_params.cc_params->retractor_creator->params()->lm_params.linearSolverType =
      gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
  mopt_params.cc_params->retractor_creator->params()->lm_params.setlambdaUpperBound(
      1e2);
  if (args.cm_f_only || (!args.cm_i_only && !args.skip_cm_f)) {
    mopt_params.cc_params->retractor_creator->params()->lm_params.setMaxIterations(
        10);
  }
  if (args.cm_i_only || args.skip_cm_f) {
    mopt_params.cc_params->retractor_creator->params()->lm_params.setMaxIterations(
        10);
  }
  if (args.verbose_retractor) {
    mopt_params.cc_params->retractor_creator->params()->lm_params.setVerbosityLM(
        "SUMMARY");
    std::cout << "[BENCH] Retraction LM verbosity enabled.\n";
  }
  if (args.cm_f_only || (!args.cm_i_only && !args.skip_cm_f)) {
    std::cout << "constraint manifold basis variables (feasible):\n";
    auto cm_f_problem = createProblem();
    auto cm_basis_result = OptimizeE_CMOpt(
        cm_f_problem, latex_os, mopt_params, lm_params,
        "Constraint Manifold (F)");
  } else if (args.skip_cm_f && !args.cm_i_only) {
    std::cout << "constraint manifold basis variables (feasible): skipped "
                 "(--skip-cm-f)\n";
  } else {
    std::cout
        << "constraint manifold basis variables (feasible): skipped (I-only mode)\n";
  }

  // // optimize constraint manifold specify variables (infeasible)
  if (!args.cm_f_only) {
    std::cout << "constraint manifold basis variables (infeasible):\n";
    auto cm_i_problem = createProblem();
    LevenbergMarquardtParams cm_i_lm_params = lm_params;
    if (args.skip_cm_f && !args.cm_i_only) {
      cm_i_lm_params.linearSolverType =
          gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
      cm_i_lm_params.setMaxIterations(20);
      cm_i_lm_params.relativeErrorTol = 1e-3;
      cm_i_lm_params.setlambdaUpperBound(1e2);
      std::cout << "[BENCH] CM(I) mode: using fast outer-LM settings "
                   "(SEQUENTIAL_CHOLESKY, maxIterations=20, relativeErrorTol=1e-3, "
                   "lambdaUpperBound=1e2).\n";
    }
    mopt_params.cc_params->retractor_creator->params()->lm_params.setMaxIterations(1);
    auto cm_basis_infeasible_result = OptimizeE_CMOpt(
        cm_i_problem, latex_os, mopt_params, cm_i_lm_params,
        "Constraint Manifold (I)");
  } else {
    std::cout
        << "constraint manifold basis variables (infeasible): skipped (F-only mode)\n";
  }

  // // optimize serial chain manifold
  // std::cout << "serial chain manifold\n";
  // Values sc_init_values = get_init_values_sc();
  // auto sc_costs = get_costs_sc();
  // auto sc_result = optimize_serial_chain_manifold(sc_costs, sc_init_values);
  // std::cout << "final cost: " << sc_costs.error(sc_result) << "\n";
  // print_joint_angles_sc(sc_result);

  std::cout << latex_os.str();
}

int main(int argc, char** argv) {
  try {
    const ArmBenchmarkArgs args = ParseArgs(argc, argv);
    kinematic_planning(args);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
