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
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>

using namespace gtsam;
using gtsam::symbol_shorthand::A, gtsam::symbol_shorthand::B;

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

/** Benchmark constrained optimizers on range-constrained trajectory estimation. */
void kinematic_planning() {
  // problem
  auto gt = get_gt_values();
  auto constraints_graph = get_constraints_graph(gt);
  auto costs = get_costs(gt);
  auto init_values = get_init_values(gt);
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph);
  auto problem = gtdynamics::EConsOptProblem(costs, constraints, init_values);

  std::ostringstream latex_os;
  LevenbergMarquardtParams lm_params;


  std::cout << "pose error: " << EvaluatePoseError(gt, init_values) << "\n";

  // for (size_t i=0; i<10; i++) {
    // optimize soft constraints
    std::cout << "soft constraints:\n";
    auto soft_result =
        OptimizeE_SoftConstraints(problem, latex_os, lm_params, 1e4, kConstraintUnitScale);
    std::cout << "pose error: " << EvaluatePoseError(gt, soft_result) << "\n";
  // }
  


  // optimize penalty method
  std::cout << "penalty method:\n";
  auto penalty_params = std::make_shared<gtsam::PenaltyOptimizerParams>();
  penalty_params->lmParams = lm_params;
  // penalty_params.num_iterations=4;
  penalty_params->initialMuEq = 10000;
  auto penalty_result =
      OptimizeE_Penalty(problem, latex_os, penalty_params, kConstraintUnitScale);
  std::cout << "pose error: " << EvaluatePoseError(gt, penalty_result) << "\n";

  // optimize augmented lagrangian
  std::cout << "augmented lagrangian:\n";
  auto almParams = std::make_shared<gtsam::AugmentedLagrangianParams>();
  almParams->lmParams = lm_params;
  auto almResult =
      OptimizeE_AugmentedLagrangian(problem, latex_os, almParams, kConstraintUnitScale);
  std::cout << "pose error: " << EvaluatePoseError(gt, almResult) << "\n";

  // for (size_t i=0; i<10; i++) {
    // optimize constraint manifold specify variables (feasible)
    std::cout << "constraint manifold basis variables (feasible):\n";
    auto mopt_params = gtdynamics::DefaultMoptParams();
    mopt_params.cc_params->retractor_creator->params()->lm_params.linearSolverType = gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    auto cm_basis_result = OptimizeE_CMOpt(
        problem, latex_os, mopt_params, lm_params, "Constraint Manifold (F)", kConstraintUnitScale);
    std::cout << "pose error: " << EvaluatePoseError(gt, cm_basis_result) << "\n";
  // }

  // for (size_t i=0; i<10; i++) {
    // optimize constraint manifold specify variables (infeasible)
    std::cout << "constraint manifold basis variables (infeasible):\n";
    // auto mopt_params = DefaultMoptParams();
    mopt_params.cc_params->retractor_creator->params()->lm_params.setMaxIterations(1);
    auto cm_basis_infeasible_result = OptimizeE_CMOpt(
        problem, latex_os, mopt_params, lm_params, "Constraint Manifold (I)", kConstraintUnitScale);
    std::cout << "pose error: " << EvaluatePoseError(gt, cm_basis_infeasible_result) << "\n";
  // }

  std::cout << latex_os.str();
}

int main(int argc, char **argv) {
  kinematic_planning();
  return 0;
}
