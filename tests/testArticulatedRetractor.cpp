/* GTDynamics Copyright 2026, Georgia Tech Research Corporation. See LICENSE. */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/ArticulatedRetractor.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/IndependentComponents.h>
#include <gtdynamics/dynamics/Dynamics.h>
#include <gtdynamics/dynamics/WorldVelocityFactor.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/mechanics/Mechanics.h>
#include <gtdynamics/mechanics/TorqueFactor.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/utils/ContactWrench.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/PriorFactor.h>
using namespace gtdynamics;
using namespace gtsam;
namespace {
struct Fixture {
  LinkSharedPtr root = std::make_shared<Link>(
      0, "body", 8., Matrix3::Identity(), Pose3(), Pose3());
  LinkSharedPtr tip = std::make_shared<Link>(1, "tip", 2., Matrix3::Identity(),
                                             Pose3(), Pose3());
  JointSharedPtr joint = std::make_shared<RevoluteJoint>(
      0, "hinge", Pose3(), root, tip, Vector3::UnitZ());
  std::shared_ptr<Robot> robot;
  Values values;
  NonlinearEqualityConstraints equalities;
  NonlinearInequalityConstraints inequalities;
  KeySet independent, ad;
  Fixture(size_t k = 0, bool worldVelocity = true) {
    root->addJoint(joint);
    tip->addJoint(joint);
    robot = std::make_shared<Robot>(LinkMap{{"body", root}, {"tip", tip}},
                                    JointMap{{"hinge", joint}});
    Values known;
    known.insert(PoseKey(0, k),
                 Pose3(Rot3::RzRyRx(.1, -.08, .2), Point3(1, 2, .5)));
    known.insert(TwistKey(0, k), Vector(Vector6{.1, .2, -.1, .3, .2, -.1}));
    known.insert(JointAngleKey(0, k), .2);
    known.insert(JointVelKey(0, k), .15);
    values = robot->forwardKinematics(known, k, "body");
    values.insert(TwistAccelKey(0, k), Vector(Vector6::Zero()));
    values.insert(TwistAccelKey(1, k), Vector(Vector6::Zero()));
    values.insert(JointAccelKey(0, k), .3);
    values.insert(TorqueKey(0, k), 0.);
    values.insert(WrenchKey(0, 0, k), Vector(Vector6::Zero()));
    values.insert(WrenchKey(1, 0, k), Vector(Vector6::Zero()));
    const Key contact = ContactWrenchKey(0, 0, k);
    values.insert(contact, Vector(Vector6::Zero()));
    KinematicsParameters kp;
    kp.p_cost_model = kp.v_cost_model = noiseModel::Unit::Create(6);
    Kinematics kin(kp);
    DynamicsParameters dp(1.);
    dp.gravity = Vector3(0, 0, -9.81);
    NonlinearFactorGraph exact =
        kin.constraints(Slice(k), *robot).penaltyGraph();
    exact.add(kin.twistObjectives(Slice(k), *robot));
    exact.add(Dynamics(dp).aFactors(Slice(k), *robot));
    exact.add(
        Dynamics(dp).wrenchBalanceFactors(Slice(k), *robot, {{0, {contact}}}));
    exact.add(Mechanics(dp).wrenchEquivalenceFactors(Slice(k), *robot));
    exact.add(Mechanics(dp).torqueFactors(Slice(k), *robot, Vector1(4.)));
    if (worldVelocity) {
      values.insert(LinearVelocityKey(0, k), Vector(Vector3::Zero()));
      exact.emplace_shared<WorldVelocityFactor>(PoseKey(0, k), TwistKey(0, k),
                                                LinearVelocityKey(0, k),
                                                noiseModel::Unit::Create(3));
    }
    equalities = NonlinearEqualityConstraints::FromCostGraph(exact);
    inequalities = RectangularContactWrench(.2, .1, .8).constraints(contact);
    inequalities.push_back(ScalarExpressionInequalityConstraint::LeqZero(
        Double_(TorqueKey(0, k)) - Double_(2.), 1.));
    inequalities.push_back(ScalarExpressionInequalityConstraint::LeqZero(
        (Double_(0.) - Double_(TorqueKey(0, k))) - Double_(2.), 1.));
    ad = {TwistAccelKey(0, k), JointAccelKey(0, k)};
    independent = ad;
    for (Key key : {PoseKey(0, k), TwistKey(0, k), JointAngleKey(0, k),
                    JointVelKey(0, k)})
      independent.insert(key);
  }
  IEConstraintManifold::Params::shared_ptr parameters() const {
    auto p = std::make_shared<IEConstraintManifold::Params>();
    const auto keys = independent;
    p->equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
        [keys](const KeyVector& available) {
          KeyVector selected;
          for (Key key : available)
            if (keys.exists(key)) selected.push_back(key);
          return selected;
        });
    auto rp = std::make_shared<IERetractorParams>();
    rp->feasibleThreshold = 1e-6;
    p->retractorCreator =
        std::make_shared<ArticulatedRetractorCreator>(robot, "body", ad, rp);
    return p;
  }
  IEConstraintManifold manifold() const {
    return IEConstraintManifold(
        parameters(),
        std::make_shared<NonlinearEqualityConstraints>(equalities),
        std::make_shared<NonlinearInequalityConstraints>(inequalities), values);
  }
};
TEST(ArticulatedRetractor, MovingTreeAndOptionalWorldVelocity) {
  for (bool world : {false, true}) {
    Fixture f(0, world);
    auto m = f.manifold();
    auto result = m.retract(m.values().zeroVectors());
    EXPECT(f.equalities.violationNorm(result.values()) < 1e-6);
    EXPECT(f.inequalities.violationNorm(result.values()) < 1e-6);
    EXPECT(assert_equal(f.values.at<Pose3>(PoseKey(0, 0)),
                        result.values().at<Pose3>(PoseKey(0, 0))));
    EXPECT_DOUBLES_EQUAL(.15, result.values().at<double>(JointVelKey(0, 0)),
                         1e-12);
    const auto w = result.values().at<Vector6>(WrenchKey(1, 0, 0));
    EXPECT_DOUBLES_EQUAL(
        f.joint->cScrewAxis().dot(w) +
            4. * result.values().at<double>(JointAccelKey(0, 0)),
        result.values().at<double>(TorqueKey(0, 0)), 1e-6);
    if (world)
      EXPECT(assert_equal(
          Vector3(result.values().at<Pose3>(PoseKey(0, 0)).rotation() *
                  result.values().at<Vector6>(TwistKey(0, 0)).tail<3>()),
          result.values().at<Vector3>(LinearVelocityKey(0, 0)), 1e-10));
  }
}
TEST(ArticulatedRetractor, IndependentCachesAndLifetime) {
  Fixture f;
  auto m = f.manifold();
  f.values.update(PoseKey(0, 0), Pose3(Rot3::Ry(.3), Point3(4, 2, 1)));
  auto other = f.manifold();
  f.robot.reset();
  auto expected = m.retract(m.values().zeroVectors());
  auto otherExpected = other.retract(other.values().zeroVectors());
  std::vector<std::optional<IEConstraintManifold>> results(2);
  gtdynamics::internal::forIndependentComponents(2, 2, [&](size_t i) {
    const auto& input = i ? other : m;
    results[i] = input.retract(input.values().zeroVectors());
  });
  EXPECT(assert_equal(expected.values(), results[0]->values(), 1e-8));
  EXPECT(assert_equal(otherExpected.values(), results[1]->values(), 1e-8));
  EXPECT(assert_equal(expected.values(),
                      m.retract(m.values().zeroVectors()).values(), 1e-8));
}
TEST(ArticulatedRetractor, ActiveTorqueAndWrenchBounds) {
  Fixture f;
  f.values.update(JointAccelKey(0, 0), 50.);
  auto m = f.manifold();
  auto result = m.retract(m.values().zeroVectors());
  EXPECT(f.equalities.violationNorm(result.values()) < 1e-6);
  EXPECT(f.inequalities.violationNorm(result.values()) < 1e-6);
  EXPECT_DOUBLES_EQUAL(2., result.values().at<double>(TorqueKey(0, 0)), 1e-5);
  EXPECT(!result.activeIndices().empty());
  // Explicitly select a friction face; the projected acceleration/wrenches
  // must satisfy both that face and the armature-aware mechanics equations.
  const auto boundary = result.moveToBoundary(IndexSet{1});
  EXPECT(f.equalities.violationNorm(boundary.values()) < 1e-6);
  EXPECT(f.inequalities.violationNorm(boundary.values()) < 1e-6);
  EXPECT_DOUBLES_EQUAL(
      0., f.inequalities[1]->unwhitenedExpr(boundary.values())(0), 1e-6);
}
TEST(ArticulatedRetractor, RejectMultipleSlices) {
  Fixture f;
  f.values.insert(JointAccelKey(0, 1), 0.);
  auto p = f.parameters();
  p->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  bool rejected = false;
  try {
    IEConstraintManifold m(
        p, std::make_shared<NonlinearEqualityConstraints>(f.equalities),
        std::make_shared<NonlinearInequalityConstraints>(f.inequalities),
        f.values);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
}
TEST(ArticulatedRetractor, RejectNonaffineMechanics) {
  Fixture f;
  NonlinearFactorGraph graph;
  Double_ a(JointAccelKey(0, 0));
  graph.emplace_shared<ExpressionFactor<double>>(noiseModel::Unit::Create(1),
                                                 1., a * a);
  f.equalities.add(NonlinearEqualityConstraints::FromCostGraph(graph));
  auto p = f.parameters();
  p->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  IEConstraintManifold m(
      p, std::make_shared<NonlinearEqualityConstraints>(f.equalities),
      std::make_shared<NonlinearInequalityConstraints>(f.inequalities),
      f.values);
  bool rejected = false;
  try {
    m.retract(m.values().zeroVectors());
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
}

TEST(ArticulatedRetractor, RejectConfigurationInequalities) {
  Fixture f;
  f.inequalities.push_back(ScalarExpressionInequalityConstraint::LeqZero(
      Double_(JointAngleKey(0, 0)) - Double_(1.), 1.));
  bool rejected = false;
  try {
    auto m = f.manifold();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
}
TEST(ArticulatedRetractor, RejectExtraConfigurationEquality) {
  Fixture f;
  NonlinearFactorGraph extra;
  extra.emplace_shared<PriorFactor<double>>(JointAngleKey(0, 0), .9,
                                            noiseModel::Unit::Create(1));
  f.equalities.add(NonlinearEqualityConstraints::FromCostGraph(extra));
  // Test the projection contract independently of a redundant elimination
  // basis.
  auto p = f.parameters();
  p->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  IEConstraintManifold m(
      p, std::make_shared<NonlinearEqualityConstraints>(f.equalities),
      std::make_shared<NonlinearInequalityConstraints>(f.inequalities),
      f.values);
  bool rejected = false;
  try {
    m.retract(m.values().zeroVectors());
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
}
TEST(ArticulatedRetractor, ArmatureOverloadPreservesZeroAndValidates) {
  Fixture f;
  const auto original = TorqueFactor(noiseModel::Unit::Create(1), f.joint, 0);
  const auto zero = TorqueFactor(noiseModel::Unit::Create(1), f.joint, 0, 0.);
  EXPECT(original->keys() == zero->keys());
  EXPECT_DOUBLES_EQUAL(original->error(f.values), zero->error(f.values), 0.);
  bool rejected = false;
  try {
    Mechanics().torqueFactors(Slice(0), *f.robot, Vector1(-1.));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
}
}  // namespace
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
