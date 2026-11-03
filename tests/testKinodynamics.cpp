/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testKinodynamics.cpp
 * @brief Verify coupled mechanics, zero initial dynamics, and velocity frames.
 * @author GTDynamics contributors
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/TangentCone.h>
#include <gtdynamics/dynamics/Kinodynamics.h>
#include <gtdynamics/dynamics/WorldVelocityFactor.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/utils/PoseContactConstraintSpec.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/PriorFactor.h>
using namespace gtsam;
using namespace gtdynamics;

/* ************************************************************************* */
namespace velocity_frames {
// Checks the right-local pose Jacobian and the body/world velocity convention
// under a nonidentity rotation, with nonzero angular and linear body velocity.
TEST(Kinodynamics, WorldVelocityJacobians) {
  const Pose3 pose(Rot3::RzRyRx(.2, -.3, .5), Point3(2, 3, 4));
  const Vector6 twist{.2, .3, -.1, .8, -.4, .5};
  const Vector3 world = pose.rotation() * twist.tail<3>();
  const WorldVelocityFactor factor(1, 2, 3, noiseModel::Unit::Create(3));
  Matrix H1, H2, H3;
  EXPECT(assert_equal(Vector3::Zero(),
                      factor.evaluateError(pose, twist, world, &H1, &H2, &H3),
                      1e-12));
  std::function<Vector3(const Pose3&, const Vector6&, const Vector3&)> error =
      [&factor](const Pose3& p, const Vector6& v, const Vector3& w) {
        return Vector3(factor.evaluateError(p, v, w));
      };
  EXPECT(
      assert_equal(numericalDerivative31(error, pose, twist, world), H1, 1e-8));
  EXPECT(
      assert_equal(numericalDerivative32(error, pose, twist, world), H2, 1e-8));
  EXPECT(
      assert_equal(numericalDerivative33(error, pose, twist, world), H3, 1e-8));
}
// Large gradients and differently scaled rows describe the same homogeneous
// cone; projection must preserve both its solution and its blocking faces.
TEST(Kinodynamics, ScaledConeProjection) {
  LinearInequalityConstraints constraints;
  const Matrix22 A{{1e6, -2e6}, {-2e-6, 1e-6}};
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(1, A, Vector2::Zero()));
  const auto [active, result] =
      TangentCone(constraints).project(Vector2(1e9, 0));
  EXPECT(assert_equal(Vector2(.8e9, .4e9), result, 1e-4));
  EXPECT(result(0) - 2 * result(1) < 1e-4);
}
}  // namespace velocity_frames
/* ************************************************************************* */
namespace projection_retry {
class LimitedRetractor : public IERetractor {
 public:
  mutable size_t rejected = 0;
  IEConstraintManifold retract(
      const IEConstraintManifold* manifold, const VectorValues& delta,
      const std::optional<IndexSet>& blocking = {},
      IERetractionInfo* info = nullptr) const override {
    if (delta.at(1).norm() > .2) {
      ++rejected;
      throw RetractionFailure("Synthetic projection convergence failure");
    }
    return manifold->createWithNewValues(manifold->values().retract(delta));
  }
};
// Failed projections reject the trial and increase damping. Bounded details
// preserve all scalar callbacks while retaining at most one previous iterate.
TEST(Kinodynamics, RejectProjectionAndRetry) {
  Values values;
  values.insert(Key(1), .1);
  NonlinearFactorGraph cost;
  cost.emplace_shared<PriorFactor<double>>(1, 1., noiseModel::Unit::Create(1));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(ScalarExpressionInequalityConstraint::LeqZero(
      Double_(0.) - Double_(Key(1)), 1.));
  const auto retractor = std::make_shared<LimitedRetractor>();
  const auto manifoldParams = std::make_shared<IEConstraintManifold::Params>();
  manifoldParams->equalityBasisCreator =
      std::make_shared<OrthonormalBasisCreator>();
  manifoldParams->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(retractor);
  IELMParams params;
  params.historyWindow = 1;
  params.lmParams.maxIterations = 5;
  size_t observations = 0;
  params.iterationCallback = [&observations](const IELMState&) {
    ++observations;
  };
  const IELMOptimizer optimizer(params, manifoldParams);
  const auto result = optimizer.optimize(cost, {}, inequalities, values);
  EXPECT(retractor->rejected > 0);
  EXPECT(result.at<double>(1) > .1);
  EXPECT(result.at<double>(1) <= 1.);
  EXPECT(inequalities.violationNorm(result) < 1e-10);
  EXPECT_LONGS_EQUAL(optimizer.details().back().state.iterations + 1,
                     observations);
  EXPECT(optimizer.details().size() <= 2);
}
}  // namespace projection_retry
/* ************************************************************************* */
namespace coupled_support {
// A tiny articulated tree verifies actual CMC optimization from zero dynamics,
// gravity support, collocation and exact mechanics without a dataset
// dependency.
TEST(Kinodynamics, StaticSupportFromZero) {
  auto root = std::make_shared<Link>(0, "root", 8., Matrix3::Identity(),
                                     Pose3(), Pose3());
  auto foot = std::make_shared<Link>(1, "foot", 2., Matrix3::Identity(),
                                     Pose3(), Pose3());
  auto joint = std::make_shared<RevoluteJoint>(0, "joint", Pose3(), root, foot,
                                               Vector3::UnitZ());
  root->addJoint(joint);
  foot->addJoint(joint);
  const Robot robot(LinkMap{{"root", root}, {"foot", foot}},
                    JointMap{{"joint", joint}});
  const PoseOnLink sole(foot, Pose3(Rot3(), Point3(0, 0, -.5)));
  const Trajectory trajectory(
      WalkCycle({Phase(0, 2,
                       std::make_shared<PoseContactConstraintSpec>(
                           std::vector<PoseOnLink>{sole}))}),
      1);
  Values initial;
  NonlinearFactorGraph measurements;
  const Pose3 pose(Rot3(), Point3(0, 0, .5));
  const double poseRotationSigmaDegrees = .05;
  const double angularVelocitySigmaDegrees = .05;
  const double jointPositionSigmaDegrees = .5;
  const double radiansPerDegree = std::acos(-1.0) / 180.0;
  Vector6 poseSigmas = Vector6::Constant(.001);
  poseSigmas.head<3>().setConstant(poseRotationSigmaDegrees * radiansPerDegree);
  Vector6 twistSigmas = Vector6::Constant(.001);
  twistSigmas.head<3>().setConstant(angularVelocitySigmaDegrees *
                                    radiansPerDegree);
  for (size_t k = 0; k < 3; ++k) {
    initial.insert(PoseKey(0, k), pose);
    initial.insert(PoseKey(1, k), pose);
    initial.insert(JointAngleKey(0, k), 0.);
    measurements.emplace_shared<PriorFactor<Pose3>>(
        PoseKey(0, k), pose, noiseModel::Diagonal::Sigmas(poseSigmas));
    measurements.emplace_shared<PriorFactor<Vector6>>(
        TwistKey(0, k), Vector6::Zero(),
        noiseModel::Diagonal::Sigmas(twistSigmas));
    measurements.emplace_shared<PriorFactor<double>>(
        JointAngleKey(0, k), 0.,
        noiseModel::Isotropic::Sigma(
            1, jointPositionSigmaDegrees * radiansPerDegree));
  }
  const Values before = initial;
  kinodynamics::Options options;
  options.maxIterations = 10;
  options.gaugeSlice = 1;
  const auto result = solveKinodynamics(
      robot, trajectory, {Slice(0), Slice(1), Slice(2)}, {0., .02, .04},
      initial, {measurements}, {"Observations"}, {{sole, .1, .04, .8}},
      Vector(), options);
  options.retractionThreads = 1;
  const auto serial = solveKinodynamics(
      robot, trajectory, {Slice(0), Slice(1), Slice(2)}, {0., .02, .04},
      initial, {measurements}, {"Observations"}, {{sole, .1, .04, .8}},
      Vector(), options);
  EXPECT(assert_equal(serial.values, result.values, 1e-8));
  EXPECT(assert_equal(pose, result.values.at<Pose3>(PoseKey(0, 1)), 1e-12));
  EXPECT(result.feasible);
  EXPECT(result.objectiveFinal < result.objectiveInitial);
  EXPECT(result.initialEqualityL2 > 1);
  EXPECT(result.equalityL2 < 1e-7);
  EXPECT(result.inequalityL2 < 1e-7);
  EXPECT(assert_equal(before, initial));
  for (size_t k = 0; k < 3; ++k) {
    EXPECT(result.initialValues.at<Vector6>(TwistKey(0, k)).norm() == 0);
    EXPECT(result.initialValues.at<Vector6>(TwistAccelKey(0, k)).norm() == 0);
    EXPECT(result.initialValues.at<Vector6>(result.contactKeys[k]).norm() == 0);
    const auto wrench = result.values.at<Vector6>(result.contactKeys[k]);
    EXPECT_DOUBLES_EQUAL(98.1, wrench(5), .5);
    EXPECT(result.values.at<Vector6>(TwistKey(0, k)).norm() < .01);
    const auto velocity = result.values.at<Vector3>(LinearVelocityKey(0, k));
    EXPECT(assert_equal(velocity,
                        result.values.at<Pose3>(PoseKey(0, k)).rotation() *
                            result.values.at<Vector6>(TwistKey(0, k)).tail<3>(),
                        1e-10));
  }
}
}  // namespace coupled_support
/* ************************************************************************* */
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
