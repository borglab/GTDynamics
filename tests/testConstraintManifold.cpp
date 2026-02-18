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
#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
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

#include <cmath>
#include <map>
#include <sstream>

using namespace gtsam;
using namespace gtdynamics;

/** Simple example Pose3 with between constraints. */
TEST_UNSAFE(ConstraintManifold, connected_poses) {
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;

  // Constraints.
  auto constraints = std::make_shared<NonlinearEqualityConstraints>();
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
  BasisKeyFunc basis_key_func = [=](const KeyVector& keys) -> KeyVector {
    return KeyVector{x3_key};
  };
  std::vector<TspaceBasisCreator::shared_ptr> basis_creators{
      std::make_shared<OrthonormalBasisCreator>(),
      std::make_shared<EliminationBasisCreator>(basis_key_func)};
  std::vector<RetractorCreator::shared_ptr> retractor_creators{
      std::make_shared<UoptRetractorCreator>(),
      std::make_shared<BasisRetractorCreator>(basis_key_func)};

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
  const Key x1Key = 1;
  const Key x2Key = 2;
  const Key x3Key = 3;

  auto constraints = gtsam::NonlinearEqualityConstraints();
  const auto equalityNoise = noiseModel::Unit::Create(6);
  auto factor12 = std::make_shared<BetweenFactor<Pose3>>(
      x1Key, x2Key, Pose3(Rot3(), Point3(0, 0, 1)), equalityNoise);
  auto factor23 = std::make_shared<BetweenFactor<Pose3>>(
      x2Key, x3Key, Pose3(Rot3(), Point3(0, 1, 0)), equalityNoise);
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(factor12);
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(factor23);

  NonlinearFactorGraph costs;
  const auto priorNoise = noiseModel::Isotropic::Sigma(6, 1.0);
  costs.addPrior<Pose3>(x1Key, Pose3(Rot3(), Point3(0, 0, 0)), priorNoise);
  costs.addPrior<Pose3>(x3Key, Pose3(Rot3(), Point3(0, 1, 1)), priorNoise);

  Values initValues;
  initValues.insert(x1Key, Pose3(Rot3(), Point3(0.1, 0.0, 0.0)));
  initValues.insert(x2Key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  initValues.insert(x3Key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));

  const auto createProblem = [=]() {
    return EConsOptProblem(costs, constraints, initValues);
  };

  ConstrainedOptBenchmark::Options runOptions;
  runOptions.id = "test_connected_poses_minimal";
  runOptions.csvPath = "/tmp/test_connected_poses_minimal_benchmark.csv";
  runOptions.methods = {ConstrainedOptBenchmark::Method::SOFT,
                        ConstrainedOptBenchmark::Method::PENALTY,
                        ConstrainedOptBenchmark::Method::AUGMENTED_LAGRANGIAN,
                        ConstrainedOptBenchmark::Method::CM_F,
                        ConstrainedOptBenchmark::Method::CM_I};
  runOptions.softMu = 1.0;
  runOptions.constraintUnitScale = 1.0;

  ConstrainedOptBenchmark runner(runOptions);
  runner.setProblemFactory(createProblem);

  std::map<ConstrainedOptBenchmark::Method, Values> resultsByMethod;
  runner.setResultCallback(
      [&](ConstrainedOptBenchmark::Method method, const Values& result) {
        resultsByMethod[method] = result;
      });

  bool success = true;
  std::string errorMessage;
  std::ostringstream latexOs;

  try {
    runner.run(latexOs);
  } catch (const std::exception& e) {
    success = false;
    errorMessage = e.what();
  }

  if (!success) {
    std::cout << "benchmark sequence failure: " << errorMessage << std::endl;
  }
  EXPECT(success);
  if (success) {
    EXPECT(resultsByMethod.count(ConstrainedOptBenchmark::Method::SOFT) == 1);
    EXPECT(resultsByMethod.count(ConstrainedOptBenchmark::Method::PENALTY) ==
           1);
    EXPECT(resultsByMethod.count(
               ConstrainedOptBenchmark::Method::AUGMENTED_LAGRANGIAN) == 1);
    EXPECT(resultsByMethod.count(ConstrainedOptBenchmark::Method::CM_F) == 1);
    EXPECT(resultsByMethod.count(ConstrainedOptBenchmark::Method::CM_I) == 1);

    const auto problem = createProblem();
    EXPECT(problem.evaluateEConstraintViolationL2Norm(resultsByMethod.at(
               ConstrainedOptBenchmark::Method::CM_I)) < 1e-3);
  }
}

/** Multi-component benchmark regression to stress key-collision risks.
 * Uses two disconnected constrained components and one unconstrained variable,
 * and runs CM twice to catch state/key reuse issues.
 */
TEST(ConstraintManifold, benchmark_sequence_multi_component_no_key_collision) {
  const Key x1Key = 1, x2Key = 2, x3Key = 3;
  const Key y1Key = 4, y2Key = 5, y3Key = 6;
  const Key uKey = 7;  // unconstrained variable

  auto constraints = gtsam::NonlinearEqualityConstraints();
  const auto equalityNoise = noiseModel::Unit::Create(6);
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(
      std::make_shared<BetweenFactor<Pose3>>(
          x1Key, x2Key, Pose3(Rot3(), Point3(0, 0, 1)), equalityNoise));
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(
      std::make_shared<BetweenFactor<Pose3>>(
          x2Key, x3Key, Pose3(Rot3(), Point3(0, 1, 0)), equalityNoise));
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(
      std::make_shared<BetweenFactor<Pose3>>(
          y1Key, y2Key, Pose3(Rot3(), Point3(1, 0, 0)), equalityNoise));
  constraints.emplace_shared<gtsam::ZeroCostConstraint>(
      std::make_shared<BetweenFactor<Pose3>>(
          y2Key, y3Key, Pose3(Rot3(), Point3(0, -1, 0)), equalityNoise));

  NonlinearFactorGraph costs;
  const auto posePriorNoise = noiseModel::Isotropic::Sigma(6, 1.0);
  const auto scalarPriorNoise = noiseModel::Isotropic::Sigma(1, 1.0);
  costs.addPrior<Pose3>(x1Key, Pose3(Rot3(), Point3(0, 0, 0)), posePriorNoise);
  costs.addPrior<Pose3>(x3Key, Pose3(Rot3(), Point3(0, 1, 1)), posePriorNoise);
  costs.addPrior<Pose3>(y1Key, Pose3(Rot3(), Point3(2, 0, 0)), posePriorNoise);
  costs.addPrior<Pose3>(y3Key, Pose3(Rot3(), Point3(2, -1, 0)), posePriorNoise);
  costs.addPrior<double>(uKey, 0.5, scalarPriorNoise);

  Values initValues;
  initValues.insert(x1Key, Pose3(Rot3(), Point3(0.3, 0.0, 0.0)));
  initValues.insert(x2Key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  initValues.insert(x3Key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  initValues.insert(y1Key, Pose3(Rot3(), Point3(2.2, 0.1, 0.0)));
  initValues.insert(y2Key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  initValues.insert(y3Key, Pose3(Rot3(), Point3(0.0, 0.0, 0.0)));
  initValues.insert(uKey, -1.0);

  const auto createProblem = [=]() {
    return EConsOptProblem(costs, constraints, initValues);
  };

  ConstrainedOptBenchmark::Options runOptions;
  runOptions.id = "test_multi_component";
  runOptions.csvPath = "/tmp/test_multi_component_benchmark.csv";
  runOptions.methods = {ConstrainedOptBenchmark::Method::SOFT,
                        ConstrainedOptBenchmark::Method::PENALTY,
                        ConstrainedOptBenchmark::Method::AUGMENTED_LAGRANGIAN,
                        ConstrainedOptBenchmark::Method::CM_F,
                        ConstrainedOptBenchmark::Method::CM_I};
  runOptions.softMu = 1.0;
  runOptions.constraintUnitScale = 1.0;

  bool success = true;
  std::string errorMessage;
  std::map<ConstrainedOptBenchmark::Method, Values> firstRunResults;
  std::map<ConstrainedOptBenchmark::Method, Values> secondRunResults;
  std::ostringstream latexOs;

  try {
    ConstrainedOptBenchmark firstRunner(runOptions);
    firstRunner.setProblemFactory(createProblem);
    firstRunner.setResultCallback(
        [&](ConstrainedOptBenchmark::Method method, const Values& result) {
          firstRunResults[method] = result;
        });
    firstRunner.run(latexOs);

    ConstrainedOptBenchmark::Options secondRunOptions = runOptions;
    secondRunOptions.methods = {ConstrainedOptBenchmark::Method::CM_I};
    ConstrainedOptBenchmark secondRunner(secondRunOptions);
    secondRunner.setProblemFactory(createProblem);
    secondRunner.setResultCallback(
        [&](ConstrainedOptBenchmark::Method method, const Values& result) {
          secondRunResults[method] = result;
        });
    secondRunner.run(latexOs);
  } catch (const std::exception& e) {
    success = false;
    errorMessage = e.what();
  }

  if (!success) {
    std::cout << "multi-component benchmark sequence failure: " << errorMessage
              << std::endl;
  }
  EXPECT(success);
  if (success) {
    EXPECT(firstRunResults.count(ConstrainedOptBenchmark::Method::CM_I) == 1);
    EXPECT(secondRunResults.count(ConstrainedOptBenchmark::Method::CM_I) == 1);
    const auto problem = createProblem();
    EXPECT(problem.evaluateEConstraintViolationL2Norm(firstRunResults.at(
               ConstrainedOptBenchmark::Method::CM_I)) < 1e-3);
    EXPECT(problem.evaluateEConstraintViolationL2Norm(secondRunResults.at(
               ConstrainedOptBenchmark::Method::CM_I)) < 1e-3);
    EXPECT(
        assert_equal(0.5,
                     secondRunResults.at(ConstrainedOptBenchmark::Method::CM_I)
                         .atDouble(uKey),
                     1e-3));
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
  auto initValues = Values();

  auto equalityNoise = noiseModel::Unit::Create(3);
  auto priorNoise = noiseModel::Isotropic::Sigma(3, 1.0);
  auto odoNoise = noiseModel::Isotropic::Sigma(3, 1.0);

  initValues.insert(A(0), Pose2(0.0, 0.0, 0.0));
  initValues.insert(B(0), Pose2(0.0, 1.0, 0.0));
  costs.addPrior<Pose2>(A(0), Pose2(0.0, 0.0, 0.0), priorNoise);
  costs.addPrior<Pose2>(B(0), Pose2(0.0, 1.0, 0.0), priorNoise);

  for (size_t k = 0; k <= kNumSteps; ++k) {
    constraints.emplace_shared<gtsam::ZeroCostConstraint>(
        std::make_shared<BetweenFactor<Pose2>>(A(k), B(k), Pose2(0.0, 1.0, 0.0),
                                               equalityNoise));
    if (k > 0) {
      initValues.insert(A(k), Pose2(0.0, 0.0, 0.0));
      initValues.insert(B(k), Pose2(0.0, 1.0, 0.0));
      costs.emplace_shared<BetweenFactor<Pose2>>(
          A(k - 1), A(k), Pose2(0.1, 0.0, 0.0), odoNoise);
      costs.emplace_shared<BetweenFactor<Pose2>>(
          B(k - 1), B(k), Pose2(0.1, 0.0, 0.0), odoNoise);
    }
  }

  EConsOptProblem problem(costs, constraints, initValues);
  auto moptParams = ConstrainedOptBenchmark::DefaultMoptParams();
  LevenbergMarquardtParams lmParams;
  NonlinearMOptimizer optimizer(moptParams, lmParams);

  bool success = true;
  std::string errorMessage;
  ManifoldOptProblem moptProblem;
  try {
    moptProblem = optimizer.initializeMoptProblem(
        problem.costs(), problem.constraints(), problem.initValues());
  } catch (const std::exception& e) {
    success = false;
    errorMessage = e.what();
  }

  if (!success) {
    std::cout << "connected-poses manifold-key regression failure: "
              << errorMessage << std::endl;
  }
  EXPECT(success);
  if (success) {
    EXPECT(moptProblem.components_.size() == (kNumSteps + 1));
    EXPECT(moptProblem.manifold_keys_.size() == (kNumSteps + 1));
    for (size_t k = 0; k <= kNumSteps; ++k) {
      EXPECT(moptProblem.manifold_keys_.find(A(k)) !=
             moptProblem.manifold_keys_.end());
    }
  }
}

/** Regression for SV CM mode: fully constrained components should not fail
 * basis-key initialization.
 */
TEST(ConstraintManifold, sv_mode_fully_constrained_component_no_throw) {
  const Key x1Key = 1;
  const Key x2Key = 2;
  const auto noise = noiseModel::Unit::Create(6);

  NonlinearFactorGraph constraintsGraph;
  constraintsGraph.emplace_shared<BetweenFactor<Pose3>>(
      x1Key, x2Key, Pose3(Rot3(), Point3(0, 0, 1)), noise);
  constraintsGraph.addPrior<Pose3>(x1Key, Pose3(Rot3(), Point3(0, 0, 0)),
                                   noise);
  constraintsGraph.addPrior<Pose3>(x2Key, Pose3(Rot3(), Point3(0, 0, 1)),
                                   noise);
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraintsGraph);

  NonlinearFactorGraph costs;
  Values initValues;
  initValues.insert(x1Key, Pose3(Rot3(), Point3(0.1, 0.0, 0.0)));
  initValues.insert(x2Key, Pose3(Rot3(), Point3(0.0, 0.2, 0.8)));
  EConsOptProblem problem(costs, constraints, initValues);

  BasisKeyFunc basisKeyFunc = [](const KeyVector& keys) { return keys; };
  auto moptParams = ConstrainedOptBenchmark::DefaultMoptParamsSV(basisKeyFunc);
  LevenbergMarquardtParams lmParams;
  NonlinearMOptimizer optimizer(moptParams, lmParams);

  bool success = true;
  std::string errorMessage;
  ManifoldOptProblem moptProblem;
  try {
    moptProblem = optimizer.initializeMoptProblem(
        problem.costs(), problem.constraints(), problem.initValues());
  } catch (const std::exception& e) {
    success = false;
    errorMessage = e.what();
  }

  if (!success) {
    std::cout << "SV fully-constrained regression failure: " << errorMessage
              << std::endl;
  }
  EXPECT(success);
  if (success) {
    EXPECT(moptProblem.components_.size() == 1);
    EXPECT(moptProblem.manifold_keys_.size() == 0);
    EXPECT(moptProblem.fixed_manifolds_.size() == 1);
  }
}

/** Dynamics manifold for cart-pole robot. */
TEST(ConstraintManifold_retract, cart_pole_dynamics) {
  // cart-pole robot setting
  auto robot = CreateRobotFromFile(kUrdfPath + std::string("cart_pole.urdf"))
                   .fixLink("l0");
  int j0_id = robot.joint("j0")->id(), j1_id = robot.joint("j1")->id();
  const gtsam::Vector3 gravity(0, 0, -10);
  constexpr double ground_plane_height = 4.2;
  OptimizerSetting opt;
  auto graph_builder = DynamicsGraph(opt, gravity);

  // constraints graph
  NonlinearFactorGraph constraints_graph;
  constraints_graph.add(
      graph_builder.dynamicsFactorGraph(robot, 0, {}, {}, ground_plane_height));

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
  BasisKeyFunc basis_key_func = [=](const KeyVector& keys) -> KeyVector {
    return basis_keys;
  };

  // constraint manifold
  auto constraints = std::make_shared<NonlinearEqualityConstraints>(
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph));
  auto cc_params = std::make_shared<ConstraintManifold::Params>();
  cc_params->retractor_creator =
      std::make_shared<BasisRetractorCreator>(basis_key_func);
  cc_params->basis_creator =
      std::make_shared<EliminationBasisCreator>(basis_key_func);
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

namespace {

double Cos(const double& theta, gtsam::OptionalJacobian<1, 1> H_theta = {}) {
  if (H_theta) *H_theta << -std::sin(theta);
  return std::cos(theta);
}

double Sin(const double& theta, gtsam::OptionalJacobian<1, 1> H_theta = {}) {
  if (H_theta) *H_theta << std::cos(theta);
  return std::sin(theta);
}

double MinusTwoPi(const double& theta_sum,
                  gtsam::OptionalJacobian<1, 1> H_theta_sum = {}) {
  if (H_theta_sum) *H_theta_sum << 1.0;
  return theta_sum - 2.0 * M_PI;
}

}  // namespace

/** Lynch and Park (p.29) four-bar loop-closure equations in CM-Opt form.
 * Constraints:
 * 1) L1 cos(theta1) + L2 cos(theta1+theta2) + L3 cos(theta1+theta2+theta3) +
 *    L4 cos(theta1+theta2+theta3+theta4) = 0
 * 2) L1 sin(theta1) + L2 sin(theta1+theta2) + L3 sin(theta1+theta2+theta3) +
 *    L4 sin(theta1+theta2+theta3+theta4) = 0
 * 3) theta1 + theta2 + theta3 + theta4 - 2*pi = 0
 *
 * We use L1=L2=L3=L4=1.0. The feasible set is one-dimensional in R^4.
 */
TEST(ConstraintManifold, lynch_park_four_bar_loop_closure_cmopt) {
  using gtsam::symbol_shorthand::T;

  Double_ theta1(T(1)), theta2(T(2)), theta3(T(3)), theta4(T(4));

  Double_ theta12 = theta1 + theta2;
  Double_ theta123 = theta12 + theta3;
  Double_ theta1234 = theta123 + theta4;

  Double_ c1(Cos, theta1);
  Double_ c12(Cos, theta12);
  Double_ c123(Cos, theta123);
  Double_ c1234(Cos, theta1234);

  Double_ s1(Sin, theta1);
  Double_ s12(Sin, theta12);
  Double_ s123(Sin, theta123);
  Double_ s1234(Sin, theta1234);

  Double_ loop_x = c1 + c12 + c123 + c1234;
  Double_ loop_y = s1 + s12 + s123 + s1234;
  Double_ loop_angle(MinusTwoPi, theta1234);

  NonlinearEqualityConstraints constraints;
  const Vector1 tolerance = (Vector1() << 1e-9).finished();
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(loop_x, 0.0,
                                                                   tolerance);
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(loop_y, 0.0,
                                                                   tolerance);
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      loop_angle, 0.0, tolerance);

  NonlinearFactorGraph costs;
  const auto prior_noise = noiseModel::Isotropic::Sigma(1, 1e-1);
  costs.addPrior<double>(T(1), M_PI_2, prior_noise);
  costs.addPrior<double>(T(2), M_PI_2, prior_noise);
  costs.addPrior<double>(T(3), M_PI_2, prior_noise);
  costs.addPrior<double>(T(4), M_PI_2, prior_noise);

  Values init_values;
  init_values.insert(T(1), 1.2);
  init_values.insert(T(2), 1.8);
  init_values.insert(T(3), 1.1);
  init_values.insert(T(4), 2.0);

  EConsOptProblem problem(costs, constraints, init_values);
  auto mopt_params = ConstrainedOptBenchmark::DefaultMoptParams();
  LevenbergMarquardtParams lm_params;
  NonlinearMOptimizer optimizer(mopt_params, lm_params);

  const auto mopt_problem = optimizer.initializeMoptProblem(
      problem.costs(), problem.constraints(), problem.initValues());
  EXPECT_LONGS_EQUAL(1, mopt_problem.components_.size());
  EXPECT_LONGS_EQUAL(1, mopt_problem.manifold_keys_.size());
  EXPECT_LONGS_EQUAL(1, mopt_problem.manifolds().begin()->second.dim());

  const Values result =
      optimizer.optimize(problem.costs(), problem.constraints(), init_values);

  const double t1 = result.atDouble(T(1));
  const double t2 = result.atDouble(T(2));
  const double t3 = result.atDouble(T(3));
  const double t4 = result.atDouble(T(4));

  const double closure_x = std::cos(t1) + std::cos(t1 + t2) +
                           std::cos(t1 + t2 + t3) + std::cos(t1 + t2 + t3 + t4);
  const double closure_y = std::sin(t1) + std::sin(t1 + t2) +
                           std::sin(t1 + t2 + t3) + std::sin(t1 + t2 + t3 + t4);
  const double closure_angle = t1 + t2 + t3 + t4 - 2.0 * M_PI;

  EXPECT(problem.evaluateEConstraintViolationL2Norm(result) < 1e-6);
  EXPECT_DOUBLES_EQUAL(0.0, closure_x, 1e-6);
  EXPECT_DOUBLES_EQUAL(0.0, closure_y, 1e-6);
  EXPECT_DOUBLES_EQUAL(0.0, closure_angle, 1e-6);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
