

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

using namespace gtdynamics;
using namespace gtsam;

namespace vision60_test {
IEVision60Robot::Params::shared_ptr params =
    std::make_shared<IEVision60Robot::Params>();
IEVision60Robot::PhaseInfo::shared_ptr phase_info =
    IEVision60Robot::PhaseInfo::Ground();
IEVision60Robot robot(params, phase_info);
size_t k = 0;
} // namespace vision60_test

TEST(frictionConeConstraint, error_jacobian) {
  using namespace vision60_test;
  params->mu = 0.8;
  auto contact_link_id = robot.robot.link("fl_lower")->id();
  auto fc_constraint = robot.frictionConeConstraint(contact_link_id, k);

  Values values1;
  values1.insert(ContactForceKey(contact_link_id, 0, k), Vector3(0, 0, 1));
  Values values2;
  values2.insert(ContactForceKey(contact_link_id, 0, k), Vector3(0, 1, 1));
  Values values3;
  values3.insert(ContactForceKey(contact_link_id, 0, k), Vector3(0.8, 0, 1));
  Values values4;
  values4.insert(ContactForceKey(contact_link_id, 0, k), Vector3(0.2, 0.2, -1));
  Values values5;
  values5.insert(ContactForceKey(contact_link_id, 0, k), Vector3(0, 0, 0));

  // Check feasible.
  EXPECT(fc_constraint->feasible(values1));
  EXPECT(!fc_constraint->feasible(values2));
  EXPECT(fc_constraint->feasible(values3));
  EXPECT(!fc_constraint->feasible(values4));
  EXPECT(fc_constraint->feasible(values5));

  // Check jacobian.
  auto factor = fc_constraint->createL2Factor(1.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values2, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values3, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values4, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values5, 1e-7, 1e-5);
}

TEST(groundCollisionFreeConstraint, feasible) {
  using namespace vision60_test;

  auto link_id = robot.robot.link("fl_lower")->id();
  Point3 p_l(0.2, 0, 0);
  auto constraint = robot.groundCollisionFreeConstraint(link_id, k, p_l);

  Values values1;
  values1.insert(PoseKey(link_id, k),
                 Pose3(Rot3::Identity(), Point3(0, 0, 0.2)));
  Values values2;
  values2.insert(PoseKey(link_id, k),
                 Pose3(Rot3::Ry(M_PI_2), Point3(0, 0, 0.2)));
  Values values3;
  values3.insert(PoseKey(link_id, k),
                 Pose3(Rot3::Ry(M_PI_2), Point3(0, 0, 0.1)));
  EXPECT(assert_equal(0.2, (*constraint)(values1)));
  EXPECT(assert_equal(0.0, (*constraint)(values2)));
  EXPECT(assert_equal(-0.1, (*constraint)(values3)));
}

TEST(obstacleCollisionFreeConstraint, feasible) {
  using namespace vision60_test;

  auto link_id = robot.robot.link("fl_lower")->id();
  Point3 p_l(0.2, 0, 0);
  Point3 center(0.5, 0, 0);
  double radius = 0.2;
  auto constraint =
      robot.obstacleCollisionFreeConstraint(link_id, k, p_l, center, radius);

  Values values1;
  values1.insert(PoseKey(link_id, k),
                 Pose3(Rot3::Identity(), Point3(0, 0, 0.4)));
  Values values2;
  values2.insert(PoseKey(link_id, k),
                 Pose3(Rot3::Ry(M_PI_2), Point3(0.3, 0.0, 0.2)));
  Values values3;
  values3.insert(PoseKey(link_id, k),
                 Pose3(Rot3::Ry(M_PI_2), Point3(0.5, 0.0, 0.2)));
  EXPECT(assert_equal(0.3, (*constraint)(values1)));
  EXPECT(assert_equal(0.0, (*constraint)(values2)));
  EXPECT(assert_equal(-0.2, (*constraint)(values3)));
}

TEST(statePointCostFactor, error_and_jacobian) {
  using namespace vision60_test;
  size_t link_id = 1;
  Point3 point_l(0.14, 0, 0);
  Point3 point_w(1.0, 0.0, 0.0);

  auto factor = robot.statePointCostFactor(link_id, point_l, point_w, k);

  Values values;
  Pose3 pose_l(Rot3::Ry(M_PI_2), Point3(1, 0, 0.14));
  values.insert(PoseKey(link_id, k), pose_l);

  Vector expected_error = Vector::Zero(3);
  EXPECT(assert_equal(expected_error, factor->unwhitenedError(values)));
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values, 1e-7, 1e-5);
}

TEST(statePointVelCostFactor, error_and_jacobian) {
  using namespace vision60_test;
  size_t link_id = 1;
  Point3 point_l(0.14, 0, 0);
  Vector3 vel_w(1.0 - 0.14, 0.2, -0.5);

  auto factor = robot.statePointVelCostFactor(link_id, point_l, vel_w, k);

  Values values;
  Pose3 pose_l(Rot3::Ry(M_PI_2), Point3(1, 0, 0.14));
  Vector6 twist_l = (Vector(6) << 0, 1, 0, 0.5, 0.2, 1.0).finished();
  values.insert(PoseKey(link_id, k), pose_l);
  values.insert(TwistKey(link_id, k), twist_l);

  Vector expected_error = Vector::Zero(3);
  EXPECT(assert_equal(expected_error, factor->unwhitenedError(values)));
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values, 1e-7, 1e-5);
}

TEST(TrajectoryValuesVerticalJump, constraints) {
  using namespace quadruped_vertical_jump;
  std::vector<size_t> phase_num_steps{10, 10};
  std::vector<double> phases_dt{0.02, 0.02};

  auto vision60_params = std::make_shared<IEVision60Robot::Params>();
  vision60_params->express_contact_force = true;
  vision60_params->express_redundancy = true;
  auto vision60_multi_phase =
      GetVision60MultiPhase(vision60_params, phase_num_steps);

  Values values = InitValuesTrajectory(vision60_multi_phase, phases_dt);
  auto e_constraints = vision60_multi_phase.eConstraints();
  EXPECT(
      assert_equal(0.0, e_constraints.evaluateViolationL2Norm(values), 1e-6));

  Values values1 =
      InitValuesTrajectoryInfeasible(vision60_multi_phase, phases_dt);
  EXPECT(
      assert_equal(0.0, e_constraints.evaluateViolationL2Norm(values1), 1e-6));
}

TEST(TrajectoryValuesForwardJump, constraints) {
  using namespace quadruped_forward_jump;
  std::vector<size_t> phase_num_steps{10, 10, 20};
  std::vector<double> phases_dt{0.02, 0.02, 0.05};

  auto vision60_params = std::make_shared<IEVision60Robot::Params>();
  vision60_params->express_contact_force = true;
  vision60_params->express_redundancy = true;
  auto vision60_multi_phase =
      GetVision60MultiPhase(vision60_params, phase_num_steps);

  Values values = InitValuesTrajectory(vision60_multi_phase, phases_dt);
  auto e_constraints = vision60_multi_phase.eConstraints();
  EXPECT(
      assert_equal(0.0, e_constraints.evaluateViolationL2Norm(values), 1e-6));
}

TEST(IEQuadrupedUtils, general) {
  using namespace vision60_test;
  std::string str1 = "fr_hip";
  std::string str2 = "fl_hip";
  std::string str3 = "rr_hip";
  std::string str4 = "rl_hip";

  EXPECT(!IEVision60Robot::isLeft(str1));
  EXPECT(IEVision60Robot::isLeft(str2));
  EXPECT(!IEVision60Robot::isLeft(str3));
  EXPECT(IEVision60Robot::isLeft(str4));
  EXPECT(IEVision60Robot::isRight(str1));
  EXPECT(!IEVision60Robot::isRight(str2));
  EXPECT(IEVision60Robot::isRight(str3));
  EXPECT(!IEVision60Robot::isRight(str4));

  EXPECT(IEVision60Robot::counterpart(str1) == str2);
  EXPECT(IEVision60Robot::counterpart(str2) == str1);
  EXPECT(IEVision60Robot::counterpart(str3) == str4);
  EXPECT(IEVision60Robot::counterpart(str4) == str3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
