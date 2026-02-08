/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main_range_constraint.cpp
 * @brief Two-vehicle state estimation with range constraint.
 * @author Yetong Zhang
 */

#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>
#include <gtdynamics/config.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gtsam;
using gtsam::symbol_shorthand::A, gtsam::symbol_shorthand::B;
using namespace gtdynamics;

namespace {
constexpr size_t kNumSteps = 100;
constexpr double kConstraintUnitScale = 1.0;
const auto kRangeNoise = noiseModel::Isotropic::Sigma(1, 1e0);
const auto kPriorNoise = noiseModel::Isotropic::Sigma(3, 1e-1);
const auto kOdoNoise = noiseModel::Isotropic::Sigma(3, 1e-1);
const Vector kInitValueSigma = (Vector(3) << 0.5, 0.5, 0.5).finished();
const Vector kOdoSigma = (Vector(3) << 0.1, 0.1, 0.1).finished();

Sampler& OdoSampler() {
  static Sampler sampler(noiseModel::Diagonal::Sigmas(kOdoSigma));
  return sampler;
}

Sampler& InitValueSampler() {
  static Sampler sampler(noiseModel::Diagonal::Sigmas(kInitValueSigma));
  return sampler;
}

struct RangeConstraintArgs {
  ParsedBenchmarkCli benchmark_cli;
};

void PrintUsage(const char* program_name) {
  BenchmarkCliDefaults defaults;
  defaults.id = "range_constraint";
  PrintBenchmarkUsage(std::cout, program_name, defaults);
}

RangeConstraintArgs ParseArgs(int argc, char** argv) {
  BenchmarkCliDefaults defaults;
  defaults.id = "range_constraint";
  return RangeConstraintArgs{ParseBenchmarkCli(argc, argv, defaults)};
}
}  // namespace

Pose2 add_noise(const Pose2 &pose, Sampler& sampler) {
  auto xi = sampler.sample();
  return pose.expmap(xi);
}

/** Build range constraints between trajectories A and B at each step. */
NonlinearFactorGraph get_constraints_graph(const Values &gt) {
  NonlinearFactorGraph constraints_graph;

  for (size_t k = 0; k <= kNumSteps; k++) {
    Pose2 pose_1 = gt.at<Pose2>(A(k));
    Pose2 pose_2 = gt.at<Pose2>(B(k));
    double range = pose_1.range(pose_2);
    constraints_graph.emplace_shared<RangeFactor<Pose2, Pose2>>(A(k), B(k), range,
                                                           kRangeNoise);
  }

  return constraints_graph;
}

/** Build priors and noisy odometry costs for both trajectories. */
NonlinearFactorGraph get_costs(const Values &gt) {
  NonlinearFactorGraph costs;

  costs.emplace_shared<PriorFactor<Pose2>>(A(0), gt.at<Pose2>(A(0)),
                                           kPriorNoise);
  costs.emplace_shared<PriorFactor<Pose2>>(B(0), gt.at<Pose2>(B(0)),
                                           kPriorNoise);

  for (size_t k = 0; k < kNumSteps; k++) {
    Pose2 pose1_curr = gt.at<Pose2>(A(k));
    Pose2 pose1_next = gt.at<Pose2>(A(k + 1));
    Pose2 rel_pose_1 = pose1_curr.inverse() * pose1_next;
    Pose2 pose2_curr = gt.at<Pose2>(B(k));
    Pose2 pose2_next = gt.at<Pose2>(B(k + 1));
    Pose2 rel_pose_2 = pose2_curr.inverse() * pose2_next;

    rel_pose_1 = add_noise(rel_pose_1, OdoSampler());
    rel_pose_2 = add_noise(rel_pose_2, OdoSampler());

    costs.emplace_shared<BetweenFactor<Pose2>>(A(k), A(k + 1), rel_pose_1,
                                               kOdoNoise);
    costs.emplace_shared<BetweenFactor<Pose2>>(B(k), B(k + 1), rel_pose_2,
                                               kOdoNoise);
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

/** Build initial values by adding noise to the ground-truth trajectories. */
Values get_init_values(const Values &gt) {
  Values init_values;
  for (size_t k = 0; k <= kNumSteps; k++) {
    Pose2 pose_1 = gt.at<Pose2>(A(k));
    Pose2 pose_2 = gt.at<Pose2>(B(k));
    pose_1 = add_noise(pose_1, InitValueSampler());
    pose_2 = add_noise(pose_2, InitValueSampler());
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


double EvaluatePoseError(const Values &gt, const Values &result) {
  double error1 = 0;
  double error2 = 0;
  for (size_t k=1; k<=kNumSteps; k++) {
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
  std::cout << sqrt(error1 / kNumSteps) << "\t" << sqrt(error2 / kNumSteps) << "\n";
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
    out << k << "," << a.x() << "," << a.y() << "," << a.theta() << ","
        << b.x() << "," << b.y() << "," << b.theta() << "\n";
  }
}

/** Benchmark constrained optimizers on range-constrained trajectory estimation. */
void kinematic_planning(const RangeConstraintArgs& args) {
  // problem
  auto gt = get_gt_values();
  auto constraintsGraph = get_constraints_graph(gt);
  auto costs = get_costs(gt);
  auto initValues = get_init_values(gt);
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraintsGraph);
  auto runOptions = args.benchmark_cli.runOptions;
  runOptions.constraintUnitScale = kConstraintUnitScale;
  runOptions.softMu = 1e4;
  runOptions.cmFRetractorMaxIterations = 10;
  runOptions.cmIRetractorMaxIterations = 1;

  std::cout << "pose error: " << EvaluatePoseError(gt, initValues) << "\n";

  ConstrainedOptBenchmark runner(runOptions);
  runner.setProblemFactory(
      [=]() { return EConsOptProblem(costs, constraints, initValues); });
  runner.setOuterLmBaseParams(LevenbergMarquardtParams());
  runner.setMoptFactory([](BenchmarkMethod) {
    auto moptParams = ConstrainedOptBenchmark::DefaultMoptParams();
    moptParams.cc_params->retractor_creator->params()
        ->lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    return moptParams;
  });
  runner.setResultCallback([&](BenchmarkMethod method, const Values& result) {
    std::cout << "pose error: " << EvaluatePoseError(gt, result) << "\n";
    ExportTrajectoryCsv(result,
                        BenchmarkMethodDataPath(runOptions, method, "_traj.csv"));
  });

  ExportTrajectoryCsv(
      initValues, std::string(kDataPath) + runOptions.id + "_init_traj.csv");

  std::ostringstream latexOs;
  runner.run(latexOs);
  std::cout << latexOs.str();
}

int main(int argc, char **argv) {
  try {
    const RangeConstraintArgs args = ParseArgs(argc, argv);
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
