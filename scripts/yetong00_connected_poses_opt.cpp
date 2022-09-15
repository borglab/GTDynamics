/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  kinematic_trajectory_planning.cpp
 * @brief Kinematic trajectory planning problem of reaching a target pose with a
 * kuka arm. Benchmarking dynamic factor graph, constraint manifold, manually
 * specified manifold.
 * @author Yetong Zhang
 */

#include <gtdynamics/optimizer/OptimizationBenchmark.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/slam/BetweenFactor.h>
#include <sstream>

using namespace gtsam;
using namespace gtdynamics;
using gtsam::symbol_shorthand::A, gtsam::symbol_shorthand::B;

// Kuka arm planning scenario setting.
const size_t num_steps = 100;
auto constraint_noise = noiseModel::Isotropic::Sigma(3, 1e-1);
auto prior_noise = noiseModel::Isotropic::Sigma(3, 1e-1);
auto odo_noise = noiseModel::Isotropic::Sigma(3, 1e-1);
Vector init_value_sigma = (Vector(3) << 0.1, 0.1, 0.1).finished();
Vector odo_sigma = (Vector(3) << 0.1, 0.1, 0.1).finished();

auto odo_noise_model = noiseModel::Diagonal::Sigmas(odo_sigma);
auto init_value_noise_model = noiseModel::Diagonal::Sigmas(init_value_sigma);
Sampler odo_sampler(odo_noise_model);
Sampler init_value_sampler(init_value_noise_model);

Pose2 add_noise(const Pose2 &pose, Sampler& sampler) {
  auto xi = sampler.sample();
  return pose.expmap(xi);
}

/** Factor graph of all kinematic constraints. Include kinematic constraints at
 * each time step, and the priors for the first step. */
NonlinearFactorGraph get_constraints_graph(const Values &gt) {
  NonlinearFactorGraph constraints_graph;

  for (size_t k = 0; k <= num_steps; k++) {
    Pose2 pose_1 = gt.at<Pose2>(A(k));
    Pose2 pose_2 = gt.at<Pose2>(B(k));
    Pose2 rel_pose = pose_1.inverse() * pose_2;
    constraints_graph.emplace_shared<BetweenFactor<Pose2>>(A(k), B(k), rel_pose,
                                                           constraint_noise);
  }

  // TODO: move friction cone from constraints to costs
  // create a Vector3 representing contact force, and select as base variables (for redundency elimination)

  return constraints_graph;
}

/** Cost function for planning, includes cost of rotation joints, joint limit
 * costs, and cost for reaching target pose at end-effector. */
NonlinearFactorGraph get_costs(const Values &gt) {
  NonlinearFactorGraph costs;

  costs.emplace_shared<PriorFactor<Pose2>>(A(0), gt.at<Pose2>(A(0)),
                                           prior_noise);
  costs.emplace_shared<PriorFactor<Pose2>>(B(0), gt.at<Pose2>(B(0)),
                                           prior_noise);

  for (size_t k = 0; k < num_steps; k++) {
    Pose2 pose1_curr = gt.at<Pose2>(A(k));
    Pose2 pose1_next = gt.at<Pose2>(A(k + 1));
    Pose2 rel_pose_1 = pose1_curr.inverse() * pose1_next;
    Pose2 pose2_curr = gt.at<Pose2>(B(k));
    Pose2 pose2_next = gt.at<Pose2>(B(k + 1));
    Pose2 rel_pose_2 = pose2_curr.inverse() * pose2_next;

    rel_pose_1 = add_noise(rel_pose_1, odo_sampler);
    rel_pose_2 = add_noise(rel_pose_2, odo_sampler);

    costs.emplace_shared<BetweenFactor<Pose2>>(A(k), A(k + 1), rel_pose_1,
                                               odo_noise);
    costs.emplace_shared<BetweenFactor<Pose2>>(B(k), B(k + 1), rel_pose_2,
                                               odo_noise);
  }
  return costs;
}

Values get_gt_values() {
  Values gt;
  for (size_t k = 0; k <= num_steps; k++) {
    gt.insert(A(k), Pose2(k * 0.1, 5.0, k * 0.1));
    gt.insert(B(k), Pose2(k * 0.1, -5.0, -k * 0.1));
  }
  return gt;
}

/** Initial values specifed by 0 for all joint angles. */
Values get_init_values(const Values &gt) {
  Values init_values;
  for (size_t k = 0; k <= num_steps; k++) {
    Pose2 pose_1 = gt.at<Pose2>(A(k));
    Pose2 pose_2 = gt.at<Pose2>(B(k));
    pose_1 = add_noise(pose_1, init_value_sampler);
    pose_2 = add_noise(pose_2, init_value_sampler);
    init_values.insert(A(k), pose_1);
    init_values.insert(B(k), pose_2);
  }
  return init_values;
}

/** Function to manually define the basis keys for each constraint manifold. */
KeyVector FindBasisKeys(const ConnectedComponent::shared_ptr& cc) {
  KeyVector basis_keys;
  for (const Key& key : cc->keys_) {
    auto symb = Symbol(key);
    if (symb.chr() == 'a') {
      basis_keys.push_back(key);
    }
  }
  return basis_keys;
}

/** Compare simple kinematic planning tasks of a robot arm using (1) dynamics
 * factor graph (2) constraint manifold (3) manually specifed serial chain
 * manifold. */
void kinematic_planning() {
  // Create constraiend optimization problem.
  auto gt = get_gt_values();
  auto constraints_graph = get_constraints_graph(gt);
  auto costs = get_costs(gt);
  auto init_values = get_init_values(gt);
  auto constraints = ConstraintsFromGraph(constraints_graph);
  auto problem = EqConsOptProblem(costs, constraints, init_values);

  std::ostringstream latex_os;
  LevenbergMarquardtParams lm_params;

  // optimize soft constraints
  std::cout << "soft constraints:\n";
  auto soft_result =
      OptimizeSoftConstraints(problem, latex_os, lm_params);

  // optimize penalty method
  std::cout << "penalty method:\n";
  PenaltyMethodParameters penalty_params;
  penalty_params.lm_parameters = lm_params;
  auto penalty_result =
      OptimizePenaltyMethod(problem, latex_os, penalty_params);

  // optimize augmented lagrangian
  std::cout << "augmented lagrangian:\n";
  AugmentedLagrangianParameters augl_params;
  augl_params.lm_parameters = lm_params;
  auto augl_result =
      OptimizeAugmentedLagrangian(problem, latex_os, augl_params);

  // optimize constraint manifold specify variables (feasbile)
  std::cout << "constraint manifold basis variables (feasible):\n";
  auto mopt_params = DefaultMoptParamsSV();
  mopt_params.cc_params->basis_key_func = &FindBasisKeys;
  auto cm_basis_result = OptimizeConstraintManifold(
      problem, latex_os, mopt_params, lm_params, "Constraint Manifold (F)");

  // optimize constraint manifold specify variables (infeasbile)
  std::cout << "constraint manifold basis variables (infeasible):\n";
  mopt_params.cc_params->retract_params->lm_params.setMaxIterations(1);
  auto cm_basis_infeasible_result = OptimizeConstraintManifold(
      problem, latex_os, mopt_params, lm_params, "Constraint Manifold (I)");

  std::cout << latex_os.str();
}

int main(int argc, char **argv) {
  kinematic_planning();
  return 0;
}
