/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main_connected_poses.cpp
 * @brief Two connected Pose2 trajectories benchmarked with constrained
 * optimization methods.
 * @author Yetong Zhang
 */

#include <gtdynamics/cmopt/NonlinearMOptimizer.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cstdlib>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace gtsam;
using namespace gtdynamics;
using gtsam::symbol_shorthand::A, gtsam::symbol_shorthand::B;

namespace {
constexpr size_t kNumSteps = 100;
constexpr double kConstraintUnitScale = 1.0;
const auto kConstraintNoise = noiseModel::Isotropic::Sigma(3, 1e0);
const auto kPriorNoise = noiseModel::Isotropic::Sigma(3, 1e-1);
const auto kOdoNoise = noiseModel::Isotropic::Sigma(3, 1e-1);
const Vector kOdoSigma = (Vector(3) << 0.1, 0.1, 0.1).finished();

Sampler& OdoSampler() {
  static Sampler sampler(noiseModel::Diagonal::Sigmas(kOdoSigma));
  return sampler;
}

struct ConnectedPosesArgs {
  bool verbose_benchmark = false;
  bool verbose_retractor = false;
  bool debug_cm_components = false;
};

void PrintUsage(const char* program_name) {
  std::cout
      << "Usage: " << program_name << " [args]\n"
      << "Options:\n"
      << "  --verbose-benchmark   Enable outer LM summary output.\n"
      << "  --verbose-retractor   Enable retraction LM summary output.\n"
      << "  --debug-cm-components Print CM component/manifold diagnostics.\n"
      << "  --help                Show this message.\n";
}

ConnectedPosesArgs ParseArgs(int argc, char** argv) {
  ConnectedPosesArgs args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--verbose-benchmark") {
      args.verbose_benchmark = true;
    } else if (arg == "--verbose-retractor") {
      args.verbose_retractor = true;
    } else if (arg == "--debug-cm-components") {
      args.debug_cm_components = true;
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

Pose2 add_noise(const Pose2& pose, Sampler& sampler) {
  auto xi = sampler.sample();
  return pose.expmap(xi);
}

/** Build the connected-pose constraints between trajectories A and B. */
NonlinearFactorGraph get_constraints_graph(const Values& gt) {
  NonlinearFactorGraph constraints_graph;

  for (size_t k = 0; k <= kNumSteps; k++) {
    Pose2 pose_1 = gt.at<Pose2>(A(k));
    Pose2 pose_2 = gt.at<Pose2>(B(k));
    Pose2 rel_pose = pose_1.inverse() * pose_2;
    constraints_graph.emplace_shared<BetweenFactor<Pose2>>(A(k), B(k), rel_pose,
                                                           kConstraintNoise);
  }

  return constraints_graph;
}

double EvaluatePoseError(const Values& gt, const Values& result) {
  double error1 = 0;
  double error2 = 0;
  for (size_t k = 1; k <= kNumSteps; k++) {
    {
      Pose2 gt_pose = gt.at<Pose2>(A(k));
      Pose2 est_pose = result.at<Pose2>(A(k));
      Pose2 rel_pose = est_pose.inverse().compose(gt_pose);
      Matrix3 diff = rel_pose.matrix() - I_3x3;
      // std::cout << diff << "\n";
      // std::cout << diff.norm() << "\n";
      error1 += pow(diff.norm(), 2);
    }
    {
      Pose2 gt_pose = gt.at<Pose2>(B(k));
      Pose2 est_pose = result.at<Pose2>(B(k));
      Pose2 rel_pose = est_pose.inverse().compose(gt_pose);
      Matrix3 diff = rel_pose.matrix() - I_3x3;
      error2 += pow(diff.norm(), 2);
    }
  }
  std::cout << sqrt(error1 / kNumSteps) << "\t" << sqrt(error2 / kNumSteps)
            << "\n";
  return sqrt(error1 / kNumSteps) + sqrt(error2 / kNumSteps);
}

std::vector<std::vector<Pose2>> GetOdoMeasurements(const Values& gt) {
  std::vector<std::vector<Pose2>> odometryMeasurements;
  for (size_t k = 0; k < kNumSteps; k++) {
    Pose2 pose1_curr = gt.at<Pose2>(A(k));
    Pose2 pose1_next = gt.at<Pose2>(A(k + 1));
    Pose2 rel_pose_1 = pose1_curr.inverse() * pose1_next;
    Pose2 pose2_curr = gt.at<Pose2>(B(k));
    Pose2 pose2_next = gt.at<Pose2>(B(k + 1));
    Pose2 rel_pose_2 = pose2_curr.inverse() * pose2_next;
    rel_pose_1 = add_noise(rel_pose_1, OdoSampler());
    rel_pose_2 = add_noise(rel_pose_2, OdoSampler());
    odometryMeasurements.push_back(std::vector<Pose2>{rel_pose_1, rel_pose_2});
  }
  return odometryMeasurements;
}

/** Build priors and odometry costs for both connected trajectories. */
NonlinearFactorGraph get_costs(
    const Values& gt,
    const std::vector<std::vector<Pose2>>& odometryMeasurements) {
  NonlinearFactorGraph costs;

  costs.emplace_shared<PriorFactor<Pose2>>(A(0), gt.at<Pose2>(A(0)),
                                           kPriorNoise);
  costs.emplace_shared<PriorFactor<Pose2>>(B(0), gt.at<Pose2>(B(0)),
                                           kPriorNoise);

  for (size_t k = 0; k < kNumSteps; k++) {
    costs.emplace_shared<BetweenFactor<Pose2>>(
        A(k), A(k + 1), odometryMeasurements[k][0], kOdoNoise);
    costs.emplace_shared<BetweenFactor<Pose2>>(
        B(k), B(k + 1), odometryMeasurements[k][1], kOdoNoise);
  }
  return costs;
}

Values get_gt_values() {
  Values gt;
  for (size_t k = 0; k <= kNumSteps; k++) {
    gt.insert(A(k), Pose2(k * 0.1, 5.0, k * 0.1));
    gt.insert(B(k), Pose2(k * 0.1, -5.0, -k * 0.1));
  }
  return gt;
}

/** Build initial values by integrating the noisy odometry from the initial
 * pose. */
Values get_init_values(
    const Values& gt,
    const std::vector<std::vector<Pose2>>& odometryMeasurements) {
  Values init_values;
  init_values.insert(A(0), gt.at<Pose2>(A(0)));
  init_values.insert(B(0), gt.at<Pose2>(B(0)));
  for (size_t k = 1; k <= kNumSteps; k++) {
    Pose2 rel_pose1 = odometryMeasurements[k - 1][0];
    Pose2 rel_pose2 = odometryMeasurements[k - 1][1];
    Pose2 pose_1 = init_values.at<Pose2>(A(k - 1)).compose(rel_pose1);
    Pose2 pose_2 = init_values.at<Pose2>(B(k - 1)).compose(rel_pose2);
    init_values.insert(A(k), pose_1);
    init_values.insert(B(k), pose_2);
  }
  return init_values;
}

/** Function to manually define the basis keys for each constraint manifold. */
KeyVector FindBasisKeys(const KeyVector& keys) {
  KeyVector basis_keys;
  for (const Key& key : keys) {
    auto symbol = Symbol(key);
    if (symbol.chr() == 'a') {
      basis_keys.push_back(key);
    }
  }
  return basis_keys;
}

void PrintCMComponentDebug(const EConsOptProblem& problem,
                           const ManifoldOptimizerParameters& mopt_params,
                           const LevenbergMarquardtParams& lm_params,
                           bool debug_enabled) {
  if (!debug_enabled) return;

  std::cout << "[CM DEBUG] ===== start component/manifold dump =====\n";
  std::cout << "[CM DEBUG] constraints dim: " << problem.constraintsDimension()
            << ", costs dim: " << problem.costsDimension()
            << ", values dim: " << problem.valuesDimension() << "\n";

  NonlinearMOptimizer debug_optimizer(mopt_params, lm_params);
  auto mopt_problem = debug_optimizer.initializeMoptProblem(
      problem.costs(), problem.constraints(), problem.initValues());

  mopt_problem.print("[CM DEBUG] ");

  std::map<Key, Key> key_component_map;
  for (const Key& cm_key : mopt_problem.manifold_keys_) {
    const auto& cm = mopt_problem.values_.at(cm_key).cast<ConstraintManifold>();
    for (const Key& base_key : cm.values().keys()) {
      key_component_map[base_key] = cm_key;
    }
  }
  for (const Key& cm_key : mopt_problem.fixed_manifolds_.keys()) {
    const auto& cm =
        mopt_problem.fixed_manifolds_.at(cm_key).cast<ConstraintManifold>();
    for (const Key& base_key : cm.values().keys()) {
      key_component_map[base_key] = cm_key;
    }
  }

  std::cout << "[CM DEBUG] base_key -> manifold_key mapping:\n";
  for (const auto& it : key_component_map) {
    std::cout << "[CM DEBUG]   " << DefaultKeyFormatter(it.first) << " -> "
              << DefaultKeyFormatter(it.second) << "\n";
  }
  std::cout << "[CM DEBUG] ===== end component/manifold dump =====\n";
}

/** Benchmark constrained optimizers on the connected-pose estimation problem.
 */
void kinematic_planning(const ConnectedPosesArgs& args) {
  // Create constrained optimization problem.
  auto gt = get_gt_values();
  auto constraints_graph = get_constraints_graph(gt);
  std::vector<std::vector<Pose2>> odo_measurements = GetOdoMeasurements(gt);
  auto costs = get_costs(gt, odo_measurements);
  auto init_values = get_init_values(gt, odo_measurements);
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph);
  auto problem = EConsOptProblem(costs, constraints, init_values);

  std::ostringstream latex_os;
  LevenbergMarquardtParams lm_params;
  if (args.verbose_benchmark) {
    lm_params.setVerbosityLM("SUMMARY");
    std::cout
        << "[BENCH] Verbose mode enabled for connected_poses benchmark.\n";
  }
  // lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e10);

  std::cout << "pose error: " << EvaluatePoseError(gt, init_values) << "\n";

  // optimize soft constraints
  std::cout << "soft constraints:\n";
  auto soft_result = OptimizeE_SoftConstraints(problem, latex_os, lm_params,
                                               1e4, kConstraintUnitScale);
  std::cout << "pose error: " << EvaluatePoseError(gt, soft_result) << "\n";

  // optimize penalty method
  std::cout << "penalty method:\n";
  auto penalty_params = std::make_shared<gtsam::PenaltyOptimizerParams>();
  penalty_params->lmParams = lm_params;
  auto penalty_result = OptimizeE_Penalty(problem, latex_os, penalty_params,
                                          kConstraintUnitScale);
  std::cout << "pose error: " << EvaluatePoseError(gt, penalty_result) << "\n";

  // optimize augmented lagrangian
  std::cout << "augmented lagrangian:\n";
  auto almParams = std::make_shared<gtsam::AugmentedLagrangianParams>();
  almParams->lmParams = lm_params;
  auto almResult = OptimizeE_AugmentedLagrangian(problem, latex_os, almParams,
                                                 kConstraintUnitScale);
  std::cout << "pose error: " << EvaluatePoseError(gt, almResult) << "\n";

  // optimize constraint manifold specify variables (feasible)
  std::cout << "constraint manifold basis variables (feasible):\n";
  auto mopt_params = DefaultMoptParams();
  if (args.verbose_retractor) {
    mopt_params.cc_params->retractor_creator->params()
        ->lm_params.setVerbosityLM("SUMMARY");
    std::cout << "[BENCH] Retraction LM verbosity enabled.\n";
  }
  PrintCMComponentDebug(problem, mopt_params, lm_params,
                        args.debug_cm_components);
  // mopt_params.cc_params->basis_key_func = &FindBasisKeys;
  mopt_params.cc_params->retractor_creator->params()
      ->lm_params.linearSolverType =
      gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
  auto cm_basis_result =
      OptimizeE_CMOpt(problem, latex_os, mopt_params, lm_params,
                      "Constraint Manifold (F)", kConstraintUnitScale);
  std::cout << "pose error: " << EvaluatePoseError(gt, cm_basis_result) << "\n";

  // optimize constraint manifold specify variables (infeasible)
  std::cout << "constraint manifold basis variables (infeasible):\n";
  mopt_params.cc_params->retractor_creator->params()
      ->lm_params.setMaxIterations(1);
  auto cm_basis_infeasible_result =
      OptimizeE_CMOpt(problem, latex_os, mopt_params, lm_params,
                      "Constraint Manifold (I)", kConstraintUnitScale);
  std::cout << "pose error: "
            << EvaluatePoseError(gt, cm_basis_infeasible_result) << "\n";

  std::cout << latex_os.str();
}

int main(int argc, char** argv) {
  try {
    const ConnectedPosesArgs args = ParseArgs(argc, argv);
    kinematic_planning(args);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
