/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testConstraintManifold.cpp
 * @brief Test constraint manifold.
 * @author Yetong Zhang
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/slam/BetweenFactor.h>
#include <sstream>

using namespace gtsam;
using namespace gtdynamics;

/** Simple example Pose3 with between constraints. */
TEST_UNSAFE(ConstraintManifold, connected_poses) {
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;

  // Constraints.
  auto constraints = std::make_shared<EqualityConstraints>();
  auto noise = noiseModel::Unit::Create(6);
  auto factor12 = std::make_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(0, 0, 1)), noise);
  auto factor23 = std::make_shared<BetweenFactor<Pose3>>(
      x2_key, x3_key, Pose3(Rot3(), Point3(0, 1, 0)), noise);
  constraints->emplace_shared<gtsam::ZeroCostConstraint>(factor12);
  constraints->emplace_shared<gtsam::ZeroCostConstraint>(factor23);

  // Create manifold values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  cm_base_values.insert(x2_key, Pose3(Rot3(), Point3(0, 0, 1)));
  cm_base_values.insert(x3_key, Pose3(Rot3(), Point3(0, 1, 1)));

  // Create constraint manifold with various tspacebasis and retractors
  BasisKeyFunc basis_key_func = [=](const KeyVector &keys) -> KeyVector {
    return KeyVector{x3_key};
  };
  std::vector<TspaceBasisCreator::shared_ptr> basis_creators{
    std::make_shared<OrthonormalBasisCreator>(),
    std::make_shared<EliminationBasisCreator>(basis_key_func)
  };
  std::vector<RetractorCreator::shared_ptr> retractor_creators{
    std::make_shared<UoptRetractorCreator>(),
    std::make_shared<BasisRetractorCreator>(basis_key_func)
  };

  for (const auto& basis_creator : basis_creators) {
    for (const auto& retractor_creator : retractor_creators) {
      auto params = std::make_shared<ConstraintManifold::Params>();
      params->basis_creator = basis_creator;
      params->retractor_creator = retractor_creator;
      ConstraintManifold manifold(constraints, cm_base_values, params, true);

      // Check recover
      Values values;
      Matrix H_recover_x2, H_recover_x3;
      EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)),
                          manifold.recover<Pose3>(x2_key, H_recover_x2)));
      EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 1, 1)),
                          manifold.recover<Pose3>(x3_key, H_recover_x3)));

      // Check recover jacobian
      std::function<Pose3(const ConstraintManifold&)> x2_recover_func =
          std::bind(&ConstraintManifold::recover<Pose3>, std::placeholders::_1,
                    x2_key, nullptr);
      EXPECT(assert_equal(numericalDerivative11<Pose3, ConstraintManifold, 6>(
                              x2_recover_func, manifold),
                          H_recover_x2));

      std::function<Pose3(const ConstraintManifold&)> x3_recover_func =
          std::bind(&ConstraintManifold::recover<Pose3>, std::placeholders::_1,
                    x3_key, nullptr);
      EXPECT(assert_equal(numericalDerivative11<Pose3, ConstraintManifold, 6>(
                              x3_recover_func, manifold),
                          H_recover_x3));

      // check retract
      Vector xi = (Vector(6) << 0, 0, 0, 0, 0, 1).finished();
      auto new_cm = manifold.retract(xi);
    }
  }
}

/** Minimal benchmark-method sequence regression for connected poses.
 * Runs Soft -> Penalty -> Augmented Lagrangian -> CM on a small problem.
 */
TEST(ConstraintManifold, benchmark_sequence_connected_poses_minimal) {
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;

  auto constraints = gtsam::NonlinearEqualityConstraints();
  auto equality_noise = noiseModel::Unit::Create(6);
  auto factor12 = std::make_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(0, 0, 1)), equality_noise);
  auto factor23 = std::make_shared<BetweenFactor<Pose3>>(
      x2_key, x3_key, Pose3(Rot3(), Point3(0, 1, 0)), equality_noise);
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(factor12);
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(factor23);

  NonlinearFactorGraph costs;
  auto prior_noise = noiseModel::Isotropic::Sigma(6, 1.0);
  costs.addPrior<Pose3>(x1_key, Pose3(Rot3(), Point3(0, 0, 0)), prior_noise);
  costs.addPrior<Pose3>(x3_key, Pose3(Rot3(), Point3(0, 1, 1)), prior_noise);

  Values init_values;
  init_values.insert(x1_key, Pose3(Rot3(), Point3(0.1, 0.0, 0.0)));
  init_values.insert(x2_key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  init_values.insert(x3_key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));

  EConsOptProblem problem(costs, constraints, init_values);

  LevenbergMarquardtParams lm_params;
  auto penalty_params = std::make_shared<gtsam::PenaltyOptimizerParams>();
  penalty_params->lmParams = lm_params;
  auto alm_params = std::make_shared<gtsam::AugmentedLagrangianParams>();
  alm_params->lmParams = lm_params;
  auto mopt_params = DefaultMoptParams();

  bool success = true;
  std::string error_msg;
  Values cm_result;
  std::ostringstream latex_os;

  try {
    OptimizeE_SoftConstraints(problem, latex_os, lm_params, 1.0);
    OptimizeE_Penalty(problem, latex_os, penalty_params);
    OptimizeE_AugmentedLagrangian(problem, latex_os, alm_params);
    cm_result =
        OptimizeE_CMOpt(problem, latex_os, mopt_params, lm_params, "CM");
  } catch (const std::exception& e) {
    success = false;
    error_msg = e.what();
  }

  if (!success) {
    std::cout << "benchmark sequence failure: " << error_msg << std::endl;
  }
  EXPECT(success);
  if (success) {
    EXPECT(problem.evaluateEConstraintViolationL2Norm(cm_result) < 1e-3);
  }
}

/** Multi-component benchmark regression to stress key-collision risks.
 * Uses two disconnected constrained components and one unconstrained variable,
 * and runs CM twice to catch state/key reuse issues.
 */
TEST(ConstraintManifold, benchmark_sequence_multi_component_no_key_collision) {
  Key x1_key = 1, x2_key = 2, x3_key = 3;
  Key y1_key = 4, y2_key = 5, y3_key = 6;
  Key u_key = 7;  // unconstrained variable

  auto constraints = gtsam::NonlinearEqualityConstraints();
  auto equality_noise = noiseModel::Unit::Create(6);
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(
      std::make_shared<BetweenFactor<Pose3>>(
          x1_key, x2_key, Pose3(Rot3(), Point3(0, 0, 1)), equality_noise));
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(
      std::make_shared<BetweenFactor<Pose3>>(
          x2_key, x3_key, Pose3(Rot3(), Point3(0, 1, 0)), equality_noise));
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(
      std::make_shared<BetweenFactor<Pose3>>(
          y1_key, y2_key, Pose3(Rot3(), Point3(1, 0, 0)), equality_noise));
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(
      std::make_shared<BetweenFactor<Pose3>>(
          y2_key, y3_key, Pose3(Rot3(), Point3(0, -1, 0)), equality_noise));

  NonlinearFactorGraph costs;
  auto pose_prior_noise = noiseModel::Isotropic::Sigma(6, 1.0);
  auto scalar_prior_noise = noiseModel::Isotropic::Sigma(1, 1.0);
  costs.addPrior<Pose3>(x1_key, Pose3(Rot3(), Point3(0, 0, 0)), pose_prior_noise);
  costs.addPrior<Pose3>(x3_key, Pose3(Rot3(), Point3(0, 1, 1)), pose_prior_noise);
  costs.addPrior<Pose3>(y1_key, Pose3(Rot3(), Point3(2, 0, 0)), pose_prior_noise);
  costs.addPrior<Pose3>(y3_key, Pose3(Rot3(), Point3(2, -1, 0)), pose_prior_noise);
  costs.addPrior<double>(u_key, 0.5, scalar_prior_noise);

  Values init_values;
  init_values.insert(x1_key, Pose3(Rot3(), Point3(0.3, 0.0, 0.0)));
  init_values.insert(x2_key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  init_values.insert(x3_key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  init_values.insert(y1_key, Pose3(Rot3(), Point3(2.2, 0.1, 0.0)));
  init_values.insert(y2_key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  init_values.insert(y3_key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  init_values.insert(u_key, -1.0);

  EConsOptProblem problem(costs, constraints, init_values);

  LevenbergMarquardtParams lm_params;
  auto penalty_params = std::make_shared<gtsam::PenaltyOptimizerParams>();
  penalty_params->lmParams = lm_params;
  auto alm_params = std::make_shared<gtsam::AugmentedLagrangianParams>();
  alm_params->lmParams = lm_params;
  auto mopt_params = DefaultMoptParams();

  bool success = true;
  std::string error_msg;
  Values cm_result_1, cm_result_2;
  std::ostringstream latex_os;

  try {
    OptimizeE_SoftConstraints(problem, latex_os, lm_params, 1.0);
    OptimizeE_Penalty(problem, latex_os, penalty_params);
    OptimizeE_AugmentedLagrangian(problem, latex_os, alm_params);
    cm_result_1 =
        OptimizeE_CMOpt(problem, latex_os, mopt_params, lm_params, "CM1");
    cm_result_2 =
        OptimizeE_CMOpt(problem, latex_os, mopt_params, lm_params, "CM2");
  } catch (const std::exception& e) {
    success = false;
    error_msg = e.what();
  }

  if (!success) {
    std::cout << "multi-component benchmark sequence failure: " << error_msg
              << std::endl;
  }
  EXPECT(success);
  if (success) {
    EXPECT(problem.evaluateEConstraintViolationL2Norm(cm_result_1) < 1e-3);
    EXPECT(problem.evaluateEConstraintViolationL2Norm(cm_result_2) < 1e-3);
    EXPECT(assert_equal(0.5, cm_result_2.atDouble(u_key), 1e-3));
  }
}

/** Regression: connected-poses benchmark has many disconnected constraint
 * components, and each manifold key should be stable and unique.
 */
TEST(ConstraintManifold, connected_poses_many_components_manifold_keys_stable) {
  using gtsam::symbol_shorthand::A;
  using gtsam::symbol_shorthand::B;

  constexpr size_t kNumSteps = 40;
  auto constraints = gtsam::NonlinearEqualityConstraints();
  auto costs = NonlinearFactorGraph();
  auto init_values = Values();

  auto equality_noise = noiseModel::Unit::Create(3);
  auto prior_noise = noiseModel::Isotropic::Sigma(3, 1.0);
  auto odo_noise = noiseModel::Isotropic::Sigma(3, 1.0);

  init_values.insert(A(0), Pose2(0.0, 0.0, 0.0));
  init_values.insert(B(0), Pose2(0.0, 1.0, 0.0));
  costs.addPrior<Pose2>(A(0), Pose2(0.0, 0.0, 0.0), prior_noise);
  costs.addPrior<Pose2>(B(0), Pose2(0.0, 1.0, 0.0), prior_noise);

  for (size_t k = 0; k <= kNumSteps; ++k) {
    constraints.emplace_shared<gtsam::ZeroCostConstraint>(
        std::make_shared<BetweenFactor<Pose2>>(A(k), B(k), Pose2(0.0, 1.0, 0.0),
                                               equality_noise));
    if (k > 0) {
      init_values.insert(A(k), Pose2(0.0, 0.0, 0.0));
      init_values.insert(B(k), Pose2(0.0, 1.0, 0.0));
      costs.emplace_shared<BetweenFactor<Pose2>>(A(k - 1), A(k),
                                                 Pose2(0.1, 0.0, 0.0), odo_noise);
      costs.emplace_shared<BetweenFactor<Pose2>>(B(k - 1), B(k),
                                                 Pose2(0.1, 0.0, 0.0), odo_noise);
    }
  }

  EConsOptProblem problem(costs, constraints, init_values);
  auto mopt_params = DefaultMoptParams();
  LevenbergMarquardtParams lm_params;
  NonlinearMOptimizer optimizer(mopt_params, lm_params);

  bool success = true;
  std::string error_msg;
  ManifoldOptProblem mopt_problem;
  try {
    mopt_problem = optimizer.initializeMoptProblem(problem.costs(),
                                                   problem.constraints(),
                                                   problem.initValues());
  } catch (const std::exception& e) {
    success = false;
    error_msg = e.what();
  }

  if (!success) {
    std::cout << "connected-poses manifold-key regression failure: " << error_msg
              << std::endl;
  }
  EXPECT(success);
  if (success) {
    EXPECT(mopt_problem.components_.size() == (kNumSteps + 1));
    EXPECT(mopt_problem.manifold_keys_.size() == (kNumSteps + 1));
    for (size_t k = 0; k <= kNumSteps; ++k) {
      EXPECT(mopt_problem.manifold_keys_.find(A(k)) !=
             mopt_problem.manifold_keys_.end());
    }
  }
}

/** Dynamics manifold for cart-pole robot. */
TEST(ConstraintManifold_retract, cart_pole_dynamics) {
  // cart-pole robot setting
  auto robot = CreateRobotFromFile(kUrdfPath + std::string("cart_pole.urdf"))
                   .fixLink("l0");
  int j0_id = robot.joint("j0")->id(), j1_id = robot.joint("j1")->id();
  const gtsam::Vector3 gravity(0, 0, -10);
  OptimizerSetting opt;
  auto graph_builder = DynamicsGraph(opt, gravity);

  // constraints graph
  NonlinearFactorGraph constraints_graph;
  constraints_graph.add(graph_builder.dynamicsFactorGraph(robot, 0));

  // initial values
  Initializer initializer;
  Values values0 = initializer.ZeroValues(robot, 0, 0.0);
  Values known_values;
  for (const auto& joint : robot.joints()) {
    InsertJointAngle(&known_values, joint->id(),
                     JointAngle(values0, joint->id()));
    InsertJointVel(&known_values, joint->id(), JointVel(values0, joint->id()));
    InsertTorque(&known_values, joint->id(), 0.0);
  }
  for (const auto& link : robot.links()) {
    InsertPose(&known_values, link->id(), Pose(values0, link->id()));
    InsertTwist(&known_values, link->id(), Twist(values0, link->id()));
  }
  values0 = graph_builder.linearSolveFD(robot, 0, known_values);
  Values init_values = values0;

  // basis keys
  KeyVector basis_keys;
  basis_keys.push_back(JointAngleKey(j0_id, 0));
  basis_keys.push_back(JointAngleKey(j1_id, 0));
  basis_keys.push_back(JointVelKey(j0_id, 0));
  basis_keys.push_back(JointVelKey(j1_id, 0));
  basis_keys.push_back(JointAccelKey(j0_id, 0));
  basis_keys.push_back(JointAccelKey(j1_id, 0));
  BasisKeyFunc basis_key_func = [=](const KeyVector &keys) -> KeyVector {
    return basis_keys;
  };

  // constraint manifold
  auto constraints = std::make_shared<EqualityConstraints>(
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph));
  auto cc_params = std::make_shared<ConstraintManifold::Params>();
  cc_params->retractor_creator = std::make_shared<BasisRetractorCreator>(basis_key_func);
  cc_params->basis_creator = std::make_shared<EliminationBasisCreator>(basis_key_func);
  auto cm = ConstraintManifold(constraints, init_values, cc_params, true);

  // retract
  Vector xi = (Vector(6) << 1, 0, 0, 0, 0, 0).finished();
  auto new_cm = cm.retract(xi);
  // Check basis variables shall get the exact update.
  EXPECT(assert_equal(1., new_cm.recover<double>(JointAngleKey(j0_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointAngleKey(j1_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointVelKey(j0_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointVelKey(j1_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointAccelKey(j0_id, 0))));
  EXPECT(assert_equal(0., new_cm.recover<double>(JointAccelKey(j1_id, 0))));

  // Check that all constraints shall be satisfied after retraction.
  EXPECT(assert_equal(0., constraints->violationNorm(new_cm.values())));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
