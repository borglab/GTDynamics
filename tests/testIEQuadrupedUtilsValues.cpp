

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
Key phase_key = PhaseKey(0);
} // namespace vision60_test

TEST(trajectoryValuesNominal, constraints) {
  using namespace quadruped_forward_jump_land;
  std::vector<size_t> phase_num_steps{10, 10, 20, 10};
  std::vector<double> phases_dt{0.02, 0.02, 0.05, 0.02};
  double forward_distance = 1.5;

  auto vision60_params = std::make_shared<IEVision60Robot::Params>();
  vision60_params->express_contact_force = true;
  vision60_params->express_redundancy = true;
  auto vision60_multi_phase =
      GetVision60MultiPhase(vision60_params, phase_num_steps, forward_distance);

  Values values =
      vision60_multi_phase->trajectoryValuesNominal(phases_dt, true, true);
  auto e_constraints = vision60_multi_phase->eConstraints();
  auto i_constraints = vision60_multi_phase->iConstraints();
  EXPECT(
      assert_equal(0.0, e_constraints.evaluateViolationL2Norm(values), 1e-6));
  EXPECT(
      assert_equal(0.0, i_constraints.evaluateViolationL2Norm(values), 1e-6));
}

TEST(trajectoryValuesByInterpolation, constraints) {
  using namespace quadruped_forward_jump_land;
  std::vector<size_t> phase_num_steps{10, 10, 20, 10};
  std::vector<double> phases_dt{0.02, 0.02, 0.05, 0.02};
  double forward_distance = 1.5;

  auto vision60_params = std::make_shared<IEVision60Robot::Params>();
  vision60_params->express_contact_force = true;
  vision60_params->express_redundancy = true;
  auto vision60_multi_phase =
      GetVision60MultiPhase(vision60_params, phase_num_steps, forward_distance);

  std::vector<std::pair<size_t, Pose3>> prior_poses;

  Pose3 nominal_pose =
      Pose(vision60_multi_phase->phase_robots_.at(0).nominal_values,
           IEVision60Robot::base_id);
  Pose3 nominal_pose_forward =
      Pose(vision60_multi_phase->phase_robots_.back().nominal_values,
           IEVision60Robot::base_id);

  prior_poses.emplace_back(0, nominal_pose);
  prior_poses.emplace_back(10, nominal_pose *
                                   Pose3(Rot3::Ry(-0.2), Point3(0, 0, 0.2)));
  prior_poses.emplace_back(20, nominal_pose * Pose3(Rot3::Ry(-0.1), Point3(0, 0, 0.3)));
  prior_poses.emplace_back(40, nominal_pose_forward * Pose3(Rot3::Identity(), Point3(0, 0, 0.3)));
  prior_poses.emplace_back(50, nominal_pose_forward);

  Values values = vision60_multi_phase->trajectoryValuesByInterpolation(
      phases_dt, prior_poses, true, true);
  auto e_constraints = vision60_multi_phase->eConstraints();
  auto i_constraints = vision60_multi_phase->iConstraints();
  EXPECT(
      assert_equal(0.0, e_constraints.evaluateViolationL2Norm(values), 1e-6));
  EXPECT(
      assert_equal(0.0, i_constraints.evaluateViolationL2Norm(values), 1e-6));
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

  Values values = InitValuesTrajectory(*vision60_multi_phase, phases_dt);
  auto e_constraints = vision60_multi_phase->eConstraints();
  EXPECT(
      assert_equal(0.0, e_constraints.evaluateViolationL2Norm(values), 1e-6));

  Values values1 =
      InitValuesTrajectoryInfeasible(*vision60_multi_phase, phases_dt);
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

  Values values =
      InitValuesTrajectory(*vision60_multi_phase, phases_dt, false, false);
  auto e_constraints = vision60_multi_phase->eConstraints();
  EXPECT(
      assert_equal(0.0, e_constraints.evaluateViolationL2Norm(values), 1e-6));
}

TEST(TrajectoryValuesForwardJumpLand, constraints) {
  using namespace quadruped_forward_jump_land;
  std::vector<size_t> phase_num_steps{10, 10, 20, 10};
  std::vector<double> phases_dt{0.02, 0.02, 0.05, 0.02};
  double forward_distance = 1.5;

  auto vision60_params = std::make_shared<IEVision60Robot::Params>();
  vision60_params->express_contact_force = true;
  vision60_params->express_redundancy = true;
  auto vision60_multi_phase =
      GetVision60MultiPhase(vision60_params, phase_num_steps, forward_distance);

  Values values =
      InitValuesTrajectory(*vision60_multi_phase, phases_dt, false, false);
  auto e_constraints = vision60_multi_phase->eConstraints();
  EXPECT(
      assert_equal(0.0, e_constraints.evaluateViolationL2Norm(values), 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
