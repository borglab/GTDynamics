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
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

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
  ConstrainedOptBenchmark::ParsedCli benchmark_cli;
  bool debug_cm_components = false;
};

void PrintUsage(const char* program_name) {
  ConstrainedOptBenchmark::CliDefaults defaults;
  defaults.id = "connected_poses";
  ConstrainedOptBenchmark::PrintUsage(std::cout, program_name, defaults);
  std::cout
      << "Example-specific options:\n"
      << "  --debug-cm-components Print CM component/manifold diagnostics.\n";
}

ConnectedPosesArgs ParseArgs(int argc, char** argv) {
  ConstrainedOptBenchmark::CliDefaults defaults;
  defaults.id = "connected_poses";

  ConnectedPosesArgs args{
      ConstrainedOptBenchmark::ParseCli(argc, argv, defaults)};
  std::vector<std::string> remaining;
  for (const auto& arg : args.benchmark_cli.unknownArgs) {
    if (arg == "--debug-cm-components") {
      args.debug_cm_components = true;
    } else {
      remaining.push_back(arg);
    }
  }
  args.benchmark_cli.unknownArgs = remaining;
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

void ExportTrajectoryCsv(const Values& values, const std::string& file_path) {
  std::ofstream out(file_path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open trajectory file: " + file_path);
  }
  out << "t,ax,ay,ath,bx,by,bth\n";
  for (size_t k = 0; k <= kNumSteps; ++k) {
    const Pose2 a = values.at<Pose2>(A(k));
    const Pose2 b = values.at<Pose2>(B(k));
    out << k << "," << a.x() << "," << a.y() << "," << a.theta() << "," << b.x()
        << "," << b.y() << "," << b.theta() << "\n";
  }
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
  auto constraintsGraph = get_constraints_graph(gt);
  std::vector<std::vector<Pose2>> odoMeasurements = GetOdoMeasurements(gt);
  auto costs = get_costs(gt, odoMeasurements);
  auto initValues = get_init_values(gt, odoMeasurements);
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraintsGraph);
  auto problem = EConsOptProblem(costs, constraints, initValues);

  LevenbergMarquardtParams lmParams;
  lmParams.setlambdaUpperBound(1e10);

  std::cout << "pose error: " << EvaluatePoseError(gt, initValues) << "\n";

  auto runOptions = args.benchmark_cli.runOptions;
  runOptions.constraintUnitScale = kConstraintUnitScale;
  runOptions.softMu = 1e4;

  auto moptFactory = [](ConstrainedOptBenchmark::Method) {
    auto moptParams = ConstrainedOptBenchmark::DefaultMoptParams();
    moptParams.cc_params->retractor_creator->params()
        ->lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    return moptParams;
  };

  if (args.debug_cm_components &&
      (runOptions.methods.count(ConstrainedOptBenchmark::Method::CM_F) > 0 ||
       runOptions.methods.count(ConstrainedOptBenchmark::Method::CM_I) > 0)) {
    auto debugMopt = moptFactory(ConstrainedOptBenchmark::Method::CM_F);
    PrintCMComponentDebug(problem, debugMopt, lmParams, true);
  }

  ConstrainedOptBenchmark runner(runOptions);
  runner.setProblemFactory(
      [=]() { return EConsOptProblem(costs, constraints, initValues); });
  runner.setOuterLmBaseParams(lmParams);
  runner.setMoptFactory(moptFactory);
  runner.setResultCallback(
      [&](ConstrainedOptBenchmark::Method method, const Values& result) {
        std::cout << "pose error: " << EvaluatePoseError(gt, result) << "\n";
        ExportTrajectoryCsv(result, ConstrainedOptBenchmark::MethodDataPath(
                                        runOptions, method, "_traj.csv"));
      });

  std::ostringstream latexOs;
  runner.run(latexOs);
  std::cout << latexOs.str();
}

int main(int argc, char** argv) {
  try {
    const ConnectedPosesArgs args = ParseArgs(argc, argv);
    if (!args.benchmark_cli.unknownArgs.empty()) {
      throw std::invalid_argument("Unknown option: " +
                                  args.benchmark_cli.unknownArgs.front());
    }
    kinematic_planning(args);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
