

#include "gtdynamics/imanifold/IERetractor.h"
#include "gtdynamics/manifold/ConnectedComponent.h"
#include "gtdynamics/manifold/TspaceBasis.h"
#include "gtdynamics/optimizer/EqualityConstraint.h"
#include "gtdynamics/optimizer/InequalityConstraint.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEQuadrupedUtils.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Values.h>
#include <memory>
#include <string>

using namespace gtdynamics;
using namespace gtsam;

namespace vision60_4c_single_step {

IEVision60Robot GetVision60() {
  IEVision60Robot::Params vision60_params;
  vision60_params.express_redundancy = true;
  std::map<std::string, double> joint_lower_limits;
  std::map<std::string, double> joint_upper_limits;
  double hip_joint_lower_limit = -M_PI_2;
  double hip_joint_upper_limit = M_PI_2;
  for (const auto& leg : IEVision60Robot::legs) {
    joint_lower_limits.insert({leg.hip_joint->name(), hip_joint_lower_limit});
    joint_upper_limits.insert({leg.hip_joint->name(), hip_joint_upper_limit});
  }
  vision60_params.joint_upper_limits = joint_upper_limits;
  vision60_params.joint_lower_limits = joint_lower_limits;

  std::map<std::string, double> torque_lower_limits;
  std::map<std::string, double> torque_upper_limits;
  double upper_torque_lower_limit = -10.0;
  double upper_torque_upper_limit = 10.0;
  for (const auto& leg : IEVision60Robot::legs) {
    torque_lower_limits.insert({leg.upper_joint->name(), upper_torque_lower_limit});
    torque_upper_limits.insert({leg.upper_joint->name(), upper_torque_upper_limit});
  }
  vision60_params.torque_upper_limits = torque_upper_limits;
  vision60_params.torque_lower_limits = torque_lower_limits;

  vision60_params.include_torque_limits = true;
  vision60_params.include_joint_limits = true;
  vision60_params.set4C();

  IEVision60Robot vision60_(vision60_params);
  return vision60_;
}

auto vision60 = GetVision60();
size_t k = 1;
Pose3 base_pose(Rot3::Ry(-0.2), Point3(0.2, 0, vision60.nominal_height + 0.1));
// Pose3 base_pose(Rot3::Ry(-0.0), Point3(0.0, 0, vision60.nominal_height + 0.0));
Vector6 base_twist = (Vector(6) << 0, 0, 0.1, 0, 0, 0.2).finished();
Vector6 base_accel = (Vector(6) << 0, 0, -0.1, 0, 0, 0.1).finished();
Values values =
    vision60.getInitValuesStep(k, base_pose, base_twist, base_accel);
EqualityConstraints e_constraints = vision60.eConstraints(k);
InequalityConstraints i_constraints = vision60.iConstraints(k);
}; // namespace vision60_4c_single_step

namespace vision60_back_on_ground_single_step {

IEVision60Robot GetVision60() {
  IEVision60Robot::Params vision60_params;
  std::map<std::string, double> joint_lower_limits;
  std::map<std::string, double> joint_upper_limits;
  double hip_joint_lower_limit = -M_PI_2;
  double hip_joint_upper_limit = M_PI_2;
  for (const auto& leg : IEVision60Robot::legs) {
    joint_lower_limits.insert({leg.hip_joint->name(), hip_joint_lower_limit});
    joint_upper_limits.insert({leg.hip_joint->name(), hip_joint_upper_limit});
  }
  vision60_params.joint_upper_limits = joint_upper_limits;
  vision60_params.joint_lower_limits = joint_lower_limits;

  std::map<std::string, double> torque_lower_limits;
  std::map<std::string, double> torque_upper_limits;
  double upper_torque_lower_limit = -20.0;
  double upper_torque_upper_limit = 20.0;
  for (const auto& leg : IEVision60Robot::legs) {
    torque_lower_limits.insert({leg.upper_joint->name(), upper_torque_lower_limit});
    torque_upper_limits.insert({leg.upper_joint->name(), upper_torque_upper_limit});
  }
  vision60_params.torque_upper_limits = torque_upper_limits;
  vision60_params.torque_lower_limits = torque_lower_limits;

  vision60_params.include_torque_limits = true;
  vision60_params.include_joint_limits = true;
  vision60_params.express_redundancy = false;
  vision60_params.basis_using_torques = true;
  vision60_params.setBackOnGround();
  IEVision60Robot vision60_(vision60_params);
  return vision60_;
}

auto vision60 = GetVision60();
size_t k = 1;
Pose3 base_pose(Rot3::Ry(0), Point3(0.0, 0, vision60.nominal_height+0.1));
// Pose3 base_pose(Rot3::Ry(-0.0), Point3(0.0, 0, vision60.nominal_height + 0.0));
Vector6 base_twist = (Vector(6) << 0, 0, 0.0, 0, 0, 0.0).finished();
Vector6 base_accel = (Vector(6) << 0, 0, -0.0, 0, 0, 0.0).finished();
Values values =
    vision60.getInitValuesStep(k, base_pose, base_twist, base_accel);
EqualityConstraints e_constraints = vision60.eConstraints(k);
InequalityConstraints i_constraints = vision60.iConstraints(k);
}; // namespace vision60_back_on_ground_single_step


namespace vision60_in_air_single_step {

IEVision60Robot GetVision60() {
  IEVision60Robot::Params vision60_params;
  std::map<std::string, double> joint_lower_limits;
  std::map<std::string, double> joint_upper_limits;
  double hip_joint_lower_limit = -M_PI_2;
  double hip_joint_upper_limit = M_PI_2;
  for (const auto& leg : IEVision60Robot::legs) {
    joint_lower_limits.insert({leg.hip_joint->name(), hip_joint_lower_limit});
    joint_upper_limits.insert({leg.hip_joint->name(), hip_joint_upper_limit});
  }
  vision60_params.joint_upper_limits = joint_upper_limits;
  vision60_params.joint_lower_limits = joint_lower_limits;

  std::map<std::string, double> torque_lower_limits;
  std::map<std::string, double> torque_upper_limits;
  double upper_torque_lower_limit = -10.0;
  double upper_torque_upper_limit = 10.0;
  for (const auto& leg : IEVision60Robot::legs) {
    torque_lower_limits.insert({leg.upper_joint->name(), upper_torque_lower_limit});
    torque_upper_limits.insert({leg.upper_joint->name(), upper_torque_upper_limit});
  }
  vision60_params.torque_upper_limits = torque_upper_limits;
  vision60_params.torque_lower_limits = torque_lower_limits;

  vision60_params.include_torque_limits = true;
  vision60_params.include_joint_limits = true;
  vision60_params.express_redundancy = false;
  vision60_params.basis_using_torques = true;
  vision60_params.setInAir();
  IEVision60Robot vision60_(vision60_params);
  return vision60_;
}

auto vision60 = GetVision60();
size_t k = 1;
Pose3 base_pose(Rot3::Ry(0), Point3(0.0, 0, vision60.nominal_height+0.1));
// Pose3 base_pose(Rot3::Ry(-0.0), Point3(0.0, 0, vision60.nominal_height + 0.0));
Vector6 base_twist = (Vector(6) << 0, 0, 0.0, 0, 0, 0.0).finished();
Vector6 base_accel = (Vector(6) << 0, 0, -0.0, 0, 0, 0.0).finished();
Values values =
    vision60.getInitValuesStep(k, base_pose, base_twist, base_accel);
EqualityConstraints e_constraints = vision60.eConstraints(k);
InequalityConstraints i_constraints = vision60.iConstraints(k);
}; // namespace vision60_back_on_ground_single_step


namespace vision60_ground_air_boundary_step {

IEVision60Robot GetVision60() {
  IEVision60Robot::Params vision60_params;
  std::map<std::string, double> joint_lower_limits;
  std::map<std::string, double> joint_upper_limits;
  double hip_joint_lower_limit = -M_PI_2;
  double hip_joint_upper_limit = M_PI_2;
  for (const auto& leg : IEVision60Robot::legs) {
    joint_lower_limits.insert({leg.hip_joint->name(), hip_joint_lower_limit});
    joint_upper_limits.insert({leg.hip_joint->name(), hip_joint_upper_limit});
  }
  vision60_params.joint_upper_limits = joint_upper_limits;
  vision60_params.joint_lower_limits = joint_lower_limits;

  std::map<std::string, double> torque_lower_limits;
  std::map<std::string, double> torque_upper_limits;
  double upper_torque_lower_limit = -10.0;
  double upper_torque_upper_limit = 10.0;
  for (const auto& leg : IEVision60Robot::legs) {
    torque_lower_limits.insert({leg.upper_joint->name(), upper_torque_lower_limit});
    torque_upper_limits.insert({leg.upper_joint->name(), upper_torque_upper_limit});
  }
  vision60_params.torque_upper_limits = torque_upper_limits;
  vision60_params.torque_lower_limits = torque_lower_limits;

  vision60_params.include_torque_limits = true;
  vision60_params.include_joint_limits = true;
  vision60_params.express_redundancy = true;
  vision60_params.basis_using_torques = true;

  auto vision60_params_ground = vision60_params;
  vision60_params_ground.set4C();
  auto vision60_params_air = vision60_params;
  vision60_params_air.setInAir();
  vision60_params.setBoundaryLeave(vision60_params_ground, vision60_params_air);
  IEVision60Robot vision60_(vision60_params);
  return vision60_;
}

auto vision60 = GetVision60();
size_t k = 1;
Pose3 base_pose(Rot3::Ry(0), Point3(0.0, 0, vision60.nominal_height+0.1));
// Pose3 base_pose(Rot3::Ry(-0.0), Point3(0.0, 0, vision60.nominal_height + 0.0));
Vector6 base_twist = (Vector(6) << 0, 0, 0.0, 0, 0, 0.0).finished();
Vector6 base_accel = (Vector(6) << 0, 0, -0.0, 0, 0, 0.0).finished();
Values values =
    vision60.getInitValuesStep(k, base_pose, base_twist, base_accel);
EqualityConstraints e_constraints = vision60.eConstraints(k);
InequalityConstraints i_constraints = vision60.iConstraints(k);
}; // namespace vision60_back_on_ground_single_step



/// Test that the init values satisfy the equality constraints.
TEST(IEVision60Robot_4c, constraints_and_values) {
  using namespace vision60_4c_single_step;

  // check base values are indeed the specified ones
  EXPECT(assert_equal(base_pose, Pose(values, vision60.base_id, k)));
  EXPECT(assert_equal(base_twist, Twist(values, vision60.base_id, k)));
  EXPECT(assert_equal(base_accel, TwistAccel(values, vision60.base_id, k)));

  // check all constraints are satisfied
  EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(values), 0.0));

  // check variable size and dimension
  size_t q_vars = 13 + 12;
  size_t v_vars = 13 + 12;
  size_t a_vars = 13 + 12;
  size_t d_vars = 12 * 2 + 12 + 4 + 1;
  size_t q_dim = 13 * 6 + 12 * 1;
  size_t v_var_dim = 13 * 6 + 12 * 1;
  size_t a_var_dim = 13 * 6 + 12 * 1;
  size_t d_var_dim = 12 * 2 * 6 + 12 * 1 + 4 * 6 + 1 * 6;
  EXPECT_LONGS_EQUAL(q_vars + v_vars + a_vars + d_vars, values.size());
  EXPECT_LONGS_EQUAL(q_dim + v_var_dim + a_var_dim + d_var_dim, values.dim());

  // check constraint size and dimension
  size_t q_cons = 12 + 4;
  size_t v_cons = 12 + 4;
  size_t a_cons = 12 + 4;
  size_t d_cons = 13 + 12 + 12 + 4 + 1;
  size_t q_cons_dim = 12 * 6 + 4 * 3;
  size_t v_cons_dim = 12 * 6 + 4 * 3;
  size_t a_cons_dim = 12 * 6 + 4 * 3;
  size_t d_cons_dim = 13 * 6 + 12 * 6 + 12 * 1 + 4 * 3 + 6;
  EXPECT_LONGS_EQUAL(q_cons + v_cons + a_cons + d_cons, e_constraints.size());
  EXPECT_LONGS_EQUAL(q_cons_dim + v_cons_dim + a_cons_dim + d_cons_dim,
                     e_constraints.dim());

  EXPECT_LONGS_EQUAL(16, i_constraints.size());
  EXPECT_LONGS_EQUAL(16, i_constraints.dim());
}

/// Test that the init values satisfy the equality constraints.
TEST(IEVision60Robot_back_on_ground, constraints_and_values) {
  using namespace vision60_back_on_ground_single_step;

  // check base values are indeed the specified ones
  EXPECT(assert_equal(base_pose, Pose(values, vision60.base_id, k)));
  EXPECT(assert_equal(base_twist, Twist(values, vision60.base_id, k)));
  EXPECT(assert_equal(base_accel, TwistAccel(values, vision60.base_id, k)));

  // check all constraints are satisfied
  EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(values), 0.0));

  // check variable size and dimension
  size_t q_vars = 13 + 12;
  size_t v_vars = 13 + 12;
  size_t a_vars = 13 + 12;
  size_t d_vars = 12 * 2 + 12 + 2;
  size_t q_dim = 13 * 6 + 12 * 1;
  size_t v_var_dim = 13 * 6 + 12 * 1;
  size_t a_var_dim = 13 * 6 + 12 * 1;
  size_t d_var_dim = 12 * 2 * 6 + 12 * 1 + 2 * 6;
  EXPECT_LONGS_EQUAL(q_vars + v_vars + a_vars + d_vars, values.size());
  EXPECT_LONGS_EQUAL(q_dim + v_var_dim + a_var_dim + d_var_dim, values.dim());

  // check constraint size and dimension
  size_t q_cons = 12 + 2;
  size_t v_cons = 12 + 2;
  size_t a_cons = 12 + 2;
  size_t d_cons = 13 + 12 + 12 + 2;
  size_t q_cons_dim = 12 * 6 + 2 * 3;
  size_t v_cons_dim = 12 * 6 + 2 * 3;
  size_t a_cons_dim = 12 * 6 + 2 * 3;
  size_t d_cons_dim = 13 * 6 + 12 * 6 + 12 * 1 + 2 * 3;
  EXPECT_LONGS_EQUAL(q_cons + v_cons + a_cons + d_cons, e_constraints.size());
  EXPECT_LONGS_EQUAL(q_cons_dim + v_cons_dim + a_cons_dim + d_cons_dim,
                     e_constraints.dim());

  EXPECT_LONGS_EQUAL(16, i_constraints.size());
  EXPECT_LONGS_EQUAL(16, i_constraints.dim());
}



/// Test that the init values satisfy the equality constraints.
TEST(IEVision60Robot_in_air, constraints_and_values) {
  using namespace vision60_in_air_single_step;

  // check base values are indeed the specified ones
  EXPECT(assert_equal(base_pose, Pose(values, vision60.base_id, k)));
  EXPECT(assert_equal(base_twist, Twist(values, vision60.base_id, k)));
  EXPECT(assert_equal(base_accel, TwistAccel(values, vision60.base_id, k)));

  // check all constraints are satisfied
  EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(values), 0.0));

  // check variable size and dimension
  size_t q_vars = 13 + 12;
  size_t v_vars = 13 + 12;
  size_t a_vars = 13 + 12;
  size_t d_vars = 12 * 2 + 12;
  size_t q_dim = 13 * 6 + 12 * 1;
  size_t v_var_dim = 13 * 6 + 12 * 1;
  size_t a_var_dim = 13 * 6 + 12 * 1;
  size_t d_var_dim = 12 * 2 * 6 + 12 * 1;
  EXPECT_LONGS_EQUAL(q_vars + v_vars + a_vars + d_vars, values.size());
  EXPECT_LONGS_EQUAL(q_dim + v_var_dim + a_var_dim + d_var_dim, values.dim());

  // check constraint size and dimension
  size_t q_cons = 12;
  size_t v_cons = 12;
  size_t a_cons = 12;
  size_t d_cons = 13 + 12 + 12;
  size_t q_cons_dim = 12 * 6;
  size_t v_cons_dim = 12 * 6;
  size_t a_cons_dim = 12 * 6;
  size_t d_cons_dim = 13 * 6 + 12 * 6 + 12 * 1;
  EXPECT_LONGS_EQUAL(q_cons + v_cons + a_cons + d_cons, e_constraints.size());
  EXPECT_LONGS_EQUAL(q_cons_dim + v_cons_dim + a_cons_dim + d_cons_dim,
                     e_constraints.dim());

  EXPECT_LONGS_EQUAL(16, i_constraints.size());
  EXPECT_LONGS_EQUAL(16, i_constraints.dim());
}



/// Test that the init values satisfy the equality constraints.
TEST(IEVision60Robot_ground_air_boundary, constraints_and_values) {
  using namespace vision60_ground_air_boundary_step;

  // check base values are indeed the specified ones
  EXPECT(assert_equal(base_pose, Pose(values, vision60.base_id, k)));
  EXPECT(assert_equal(base_twist, Twist(values, vision60.base_id, k)));
  // EXPECT(assert_equal(base_accel, TwistAccel(values, vision60.base_id, k)));

  // check all constraints are satisfied
  EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(values), 0.0));

  // check variable size and dimension
  size_t q_vars = 13 + 12;
  size_t v_vars = 13 + 12;
  size_t a_vars = 13 + 12;
  size_t d_vars = 12 * 2 + 12 + 4 + 1;
  size_t q_dim = 13 * 6 + 12 * 1;
  size_t v_var_dim = 13 * 6 + 12 * 1;
  size_t a_var_dim = 13 * 6 + 12 * 1;
  size_t d_var_dim = 12 * 2 * 6 + 12 * 1 + 4 * 6 + 1 * 6;
  EXPECT_LONGS_EQUAL(q_vars + v_vars + a_vars + d_vars, values.size());
  EXPECT_LONGS_EQUAL(q_dim + v_var_dim + a_var_dim + d_var_dim, values.dim());

  // check constraint size and dimension
  size_t q_cons = 12 + 4;
  size_t v_cons = 12 + 4;
  size_t a_cons = 12 + 4;
  size_t d_cons = 13 + 12 + 12 + 4 + 1;
  size_t q_cons_dim = 12 * 6 + 4 * 3;
  size_t v_cons_dim = 12 * 6 + 4 * 3;
  size_t a_cons_dim = 12 * 6 + 4 * 3;
  size_t d_cons_dim = 13 * 6 + 12 * 6 + 12 * 1 + 4 * 6 + 6;
  EXPECT_LONGS_EQUAL(q_cons + v_cons + a_cons + d_cons, e_constraints.size());
  EXPECT_LONGS_EQUAL(q_cons_dim + v_cons_dim + a_cons_dim + d_cons_dim,
                     e_constraints.dim());

  EXPECT_LONGS_EQUAL(16, i_constraints.size());
  EXPECT_LONGS_EQUAL(16, i_constraints.dim());
}


// /// Test create ie-manifold from IEVision60Robot
TEST(IEVision60Robot_4c, manifold) {
  using namespace vision60_4c_single_step;
  auto i_constraints_ptr = std::make_shared<InequalityConstraints>();
  i_constraints_ptr->add(i_constraints);
  auto e_cc = std::make_shared<ConnectedComponent>(e_constraints);
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->ecm_params->basis_params->setFixVars();
  iecm_params->ecm_params->basis_key_func = vision60.getBasisKeyFunc();

  Vision60Retractor::Params vision60_retractor_params;
  vision60_retractor_params.lm_params = LevenbergMarquardtParams();
  // vision60_retractor_params.lm_params.setVerbosityLM("SUMMARY");
  // vision60_retractor_params.lm_params.minModelFidelity = 0.5;
  vision60_retractor_params.check_feasible = true;
  vision60_retractor_params.feasible_threshold = 1e-3;
  vision60_retractor_params.use_basis_keys = true;
  vision60_retractor_params.prior_sigma = 0.1;
  iecm_params->retractor_creator =
      std::make_shared<Vision60RetractorCreator>(vision60, vision60_retractor_params);
  iecm_params->e_basis_creator = std::make_shared<TspaceBasisKeysCreator>(
      iecm_params->ecm_params->basis_params,
      iecm_params->ecm_params->basis_key_func);

  IEConstraintManifold manifold(iecm_params, e_cc, i_constraints_ptr, values);
  EXPECT_LONGS_EQUAL(24, manifold.dim());

  auto e_manifold = manifold.eConstraintManifold();
  EXPECT_LONGS_EQUAL(24, e_manifold.dim());

  /// Test retractor in the case that no constraints are violated
  {
    Vector6 delta_P = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_V = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_A = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_CR = (Vector(6) << 0, 20, 20, 0, 0, 0).finished();
    Vector xi = (Vector(24) << delta_P, delta_V, delta_A, delta_CR).finished();
    auto new_manifold = manifold.retract(xi);

    IndexSet expected_active_indices;
    EXPECT(assert_container_equality(expected_active_indices, new_manifold.activeIndices()));

    Pose3 new_base_pose = Pose(manifold.values(), vision60.base_id, k).retract(delta_P);
    Vector6 new_base_twist = Twist(manifold.values(), vision60.base_id, k) + delta_V;
    Vector6 new_base_accel = TwistAccel(manifold.values(), vision60.base_id, k) + delta_A;
    EXPECT(assert_equal(new_base_pose, Pose(new_manifold.values(), vision60.base_id, k), 1e-6));
    EXPECT(assert_equal(new_base_twist, Twist(new_manifold.values(), vision60.base_id, k), 1e-6));
    EXPECT(assert_equal(new_base_accel, TwistAccel(new_manifold.values(), vision60.base_id, k), 1e-6));

    // check all constraints are satisfied
    EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
    EXPECT(assert_equal(i_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
  }

  /// Test retractor in the case of violating torque limits
  {
    Vector6 delta_P = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_V = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_A = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_CR = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
    Vector xi = (Vector(24) << delta_P, delta_V, delta_A, delta_CR).finished();
    auto new_manifold = manifold.retract(xi);

    IndexSet expected_active_indices;
    expected_active_indices.insert(12);
    expected_active_indices.insert(13);
    EXPECT(assert_container_equality(expected_active_indices, new_manifold.activeIndices()));

    Pose3 new_base_pose = Pose(manifold.values(), vision60.base_id, k).retract(delta_P);
    Vector6 new_base_twist = Twist(manifold.values(), vision60.base_id, k) + delta_V;
    // Vector6 new_base_accel = TwistAccel(manifold.values(), vision60.base_id, k) + delta_A;
    EXPECT(assert_equal(new_base_pose, Pose(new_manifold.values(), vision60.base_id, k), 1e-6));
    EXPECT(assert_equal(new_base_twist, Twist(new_manifold.values(), vision60.base_id, k), 1e-6));
    // EXPECT(assert_equal(new_base_accel, TwistAccel(new_manifold.values(), vision60.base_id, k), 1e-6));

    // check all constraints are satisfied
    EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
    EXPECT(assert_equal(i_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
    // for (const auto& joint: vision60.robot.orderedJoints()) {
    //   int j = joint->id();
    //   // double q = JointAngle(new_manifold.values(), j, k);
    //   double torque = Torque(new_manifold.values(), j, k);
    //   std::cout << joint->name()  << "\t" << torque << "\n";
    // }
  }

}




// /// Test create ie-manifold from IEVision60Robot
TEST(IEVision60Robot_back_on_ground, manifold) {
  using namespace vision60_back_on_ground_single_step;
  auto i_constraints_ptr = std::make_shared<InequalityConstraints>();
  i_constraints_ptr->add(i_constraints);
  auto e_cc = std::make_shared<ConnectedComponent>(e_constraints);
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->ecm_params->basis_params->setFixVars();
  iecm_params->ecm_params->basis_key_func = vision60.getBasisKeyFunc();

  Vision60Retractor::Params vision60_retractor_params;
  vision60_retractor_params.lm_params = LevenbergMarquardtParams();
  // vision60_retractor_params.lm_params.setVerbosityLM("SUMMARY");
  // vision60_retractor_params.lm_params.minModelFidelity = 0.5;
  vision60_retractor_params.check_feasible = true;
  vision60_retractor_params.feasible_threshold = 1e-3;
  vision60_retractor_params.use_basis_keys = true;
  vision60_retractor_params.prior_sigma = 0.1;
  iecm_params->retractor_creator =
      std::make_shared<Vision60RetractorCreator>(vision60, vision60_retractor_params);
  iecm_params->e_basis_creator = std::make_shared<TspaceBasisKeysCreator>(
      iecm_params->ecm_params->basis_params,
      iecm_params->ecm_params->basis_key_func);

  // KeyVector basis_keys = iecm_params->ecm_params->basis_key_func(e_cc);
  // PrintKeyVector(basis_keys, "", GTDKeyFormatter);

  IEConstraintManifold manifold(iecm_params, e_cc, i_constraints_ptr, values);
  EXPECT_LONGS_EQUAL(36, manifold.dim());

  auto e_manifold = manifold.eConstraintManifold();
  EXPECT_LONGS_EQUAL(36, e_manifold.dim());



  /// Test retractor in the case that no constraints are violated
  {
    Vector6 delta_P = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_V = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_q = (Vector(6) << 0.1, 0, 0, 0.1, 0, 0).finished();
    Vector6 delta_v = (Vector(6) << 0, 0.1, 0, 0, 0.1, 0).finished();
    Vector12 delta_T = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
    Vector xi = (Vector(36) << delta_P, delta_V, delta_q, delta_v, delta_T).finished();
    auto new_manifold = manifold.retract(xi);

    IndexSet expected_active_indices;
    EXPECT(assert_container_equality(expected_active_indices, new_manifold.activeIndices()));

    Pose3 new_base_pose = Pose(manifold.values(), vision60.base_id, k).retract(delta_P);
    Vector6 new_base_twist = Twist(manifold.values(), vision60.base_id, k) + delta_V;
    EXPECT(assert_equal(new_base_pose, Pose(new_manifold.values(), vision60.base_id, k), 1e-6));
    EXPECT(assert_equal(new_base_twist, Twist(new_manifold.values(), vision60.base_id, k), 1e-6));

    // check all constraints are satisfied
    EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
    EXPECT(assert_equal(i_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
  }


  /// Test retractor in the case of violating torque limits
  {
    Vector6 delta_P = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_V = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_q = (Vector(6) << 0.1, 0, 0, 0.1, 0, 0).finished();
    Vector6 delta_v = (Vector(6) << 0, 0.1, 0, 0, 0.1, 0).finished();
    Vector12 delta_T = (Vector(12) << 0, 0, 0, 0, 10, 10, -10, -10, 0, 0, 0, 0).finished();
    Vector xi = (Vector(36) << delta_P, delta_V, delta_q, delta_v, delta_T).finished();
    auto new_manifold = manifold.retract(xi);

    IndexSet expected_active_indices;
    expected_active_indices.insert(10);
    expected_active_indices.insert(11);
    expected_active_indices.insert(12);
    expected_active_indices.insert(13);
    EXPECT(assert_container_equality(expected_active_indices, new_manifold.activeIndices()));

    Pose3 new_base_pose = Pose(manifold.values(), vision60.base_id, k).retract(delta_P);
    Vector6 new_base_twist = Twist(manifold.values(), vision60.base_id, k) + delta_V;
    EXPECT(assert_equal(new_base_pose, Pose(new_manifold.values(), vision60.base_id, k), 1e-6));
    EXPECT(assert_equal(new_base_twist, Twist(new_manifold.values(), vision60.base_id, k), 1e-6));

    // check all constraints are satisfied
    EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
    EXPECT(assert_equal(i_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));

    // for (const auto& joint: vision60.robot.orderedJoints()) {
    //   int j = joint->id();
    //   // double q = JointAngle(new_manifold.values(), j, k);
    //   double torque = Torque(new_manifold.values(), j, k);
    //   std::cout << joint->name()  << "\t" << torque << "\n";
    // }

  }

}



// /// Test create ie-manifold from IEVision60Robot
TEST(IEVision60Robot_in_air, manifold) {
  using namespace vision60_in_air_single_step;
  auto i_constraints_ptr = std::make_shared<InequalityConstraints>();
  i_constraints_ptr->add(i_constraints);
  auto e_cc = std::make_shared<ConnectedComponent>(e_constraints);
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->ecm_params->basis_params->setFixVars();
  iecm_params->ecm_params->basis_key_func = vision60.getBasisKeyFunc();

  Vision60Retractor::Params vision60_retractor_params;
  vision60_retractor_params.lm_params = LevenbergMarquardtParams();
  // vision60_retractor_params.lm_params.setVerbosityLM("SUMMARY");
  // vision60_retractor_params.lm_params.minModelFidelity = 0.5;
  vision60_retractor_params.check_feasible = true;
  vision60_retractor_params.feasible_threshold = 1e-3;
  vision60_retractor_params.use_basis_keys = true;
  vision60_retractor_params.prior_sigma = 0.1;
  iecm_params->retractor_creator =
      std::make_shared<Vision60RetractorCreator>(vision60, vision60_retractor_params);
  iecm_params->e_basis_creator = std::make_shared<TspaceBasisKeysCreator>(
      iecm_params->ecm_params->basis_params,
      iecm_params->ecm_params->basis_key_func);

  // KeyVector basis_keys = iecm_params->ecm_params->basis_key_func(e_cc);
  // PrintKeyVector(basis_keys, "", GTDKeyFormatter);

  IEConstraintManifold manifold(iecm_params, e_cc, i_constraints_ptr, values);
  EXPECT_LONGS_EQUAL(48, manifold.dim());

  auto e_manifold = manifold.eConstraintManifold();
  EXPECT_LONGS_EQUAL(48, e_manifold.dim());



  /// Test retractor in the case that no constraints are violated
  {
    Vector6 delta_P = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_V = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector12 delta_q = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
    Vector12 delta_v = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
    Vector12 delta_T = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
    Vector xi = (Vector(48) << delta_P, delta_V, delta_q, delta_v, delta_T).finished();
    auto new_manifold = manifold.retract(xi);

    IndexSet expected_active_indices;
    EXPECT(assert_container_equality(expected_active_indices, new_manifold.activeIndices()));

    Pose3 new_base_pose = Pose(manifold.values(), vision60.base_id, k).retract(delta_P);
    Vector6 new_base_twist = Twist(manifold.values(), vision60.base_id, k) + delta_V;
    EXPECT(assert_equal(new_base_pose, Pose(new_manifold.values(), vision60.base_id, k), 1e-6));
    EXPECT(assert_equal(new_base_twist, Twist(new_manifold.values(), vision60.base_id, k), 1e-6));

    // check all constraints are satisfied
    EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
    EXPECT(assert_equal(i_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
  }


  /// Test retractor in the case of violating torque limits
  {
    Vector6 delta_P = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_V = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector12 delta_q = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
    Vector12 delta_v = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
    Vector12 delta_T = (Vector(12) << 0, 0, 0, 0, 10, 10, -10, -10, 0, 0, 0, 0).finished();
    Vector xi = (Vector(48) << delta_P, delta_V, delta_q, delta_v, delta_T).finished();
    auto new_manifold = manifold.retract(xi);

    IndexSet expected_active_indices;
    expected_active_indices.insert(10);
    expected_active_indices.insert(11);
    expected_active_indices.insert(12);
    expected_active_indices.insert(13);
    EXPECT(assert_container_equality(expected_active_indices, new_manifold.activeIndices()));

    Pose3 new_base_pose = Pose(manifold.values(), vision60.base_id, k).retract(delta_P);
    Vector6 new_base_twist = Twist(manifold.values(), vision60.base_id, k) + delta_V;
    EXPECT(assert_equal(new_base_pose, Pose(new_manifold.values(), vision60.base_id, k), 1e-6));
    EXPECT(assert_equal(new_base_twist, Twist(new_manifold.values(), vision60.base_id, k), 1e-6));

    // check all constraints are satisfied
    EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
    EXPECT(assert_equal(i_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));

    // for (const auto& joint: vision60.robot.orderedJoints()) {
    //   int j = joint->id();
    //   // double q = JointAngle(new_manifold.values(), j, k);
    //   double torque = Torque(new_manifold.values(), j, k);
    //   std::cout << joint->name()  << "\t" << torque << "\n";
    // }
  }
}



// /// Test create ie-manifold from IEVision60Robot
TEST(IEVision60Robot_ground_air_boundary, manifold) {
  using namespace vision60_ground_air_boundary_step;
  auto i_constraints_ptr = std::make_shared<InequalityConstraints>();
  i_constraints_ptr->add(i_constraints);
  auto e_cc = std::make_shared<ConnectedComponent>(e_constraints);
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->ecm_params->basis_params->setFixVars();
  iecm_params->ecm_params->basis_key_func = vision60.getBasisKeyFunc();

  Vision60Retractor::Params vision60_retractor_params;
  vision60_retractor_params.lm_params = LevenbergMarquardtParams();
  // vision60_retractor_params.lm_params.setVerbosityLM("SUMMARY");
  // vision60_retractor_params.lm_params.minModelFidelity = 0.5;
  vision60_retractor_params.check_feasible = true;
  vision60_retractor_params.feasible_threshold = 1e-3;
  vision60_retractor_params.use_basis_keys = true;
  vision60_retractor_params.prior_sigma = 0.1;
  iecm_params->retractor_creator =
      std::make_shared<Vision60RetractorCreator>(vision60, vision60_retractor_params);
  iecm_params->e_basis_creator = std::make_shared<TspaceBasisKeysCreator>(
      iecm_params->ecm_params->basis_params,
      iecm_params->ecm_params->basis_key_func);

  // KeyVector basis_keys = iecm_params->ecm_params->basis_key_func(e_cc);
  // PrintKeyVector(basis_keys, "", GTDKeyFormatter);

  IEConstraintManifold manifold(iecm_params, e_cc, i_constraints_ptr, values);
  EXPECT_LONGS_EQUAL(12, manifold.dim());

  auto e_manifold = manifold.eConstraintManifold();
  EXPECT_LONGS_EQUAL(12, e_manifold.dim());



  /// Test retractor in the case that no constraints are violated
  {
    Vector6 delta_P = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector6 delta_V = (Vector(6) << 0, 0, 0, 0, 0, 0.1).finished();
    Vector12 delta_q = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
    Vector12 delta_v = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
    Vector12 delta_T = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
    Vector xi = (Vector(48) << delta_P, delta_V, delta_q, delta_v, delta_T).finished();
    auto new_manifold = manifold.retract(xi);

    IndexSet expected_active_indices;
    EXPECT(assert_container_equality(expected_active_indices, new_manifold.activeIndices()));

    Pose3 new_base_pose = Pose(manifold.values(), vision60.base_id, k).retract(delta_P);
    Vector6 new_base_twist = Twist(manifold.values(), vision60.base_id, k) + delta_V;
    EXPECT(assert_equal(new_base_pose, Pose(new_manifold.values(), vision60.base_id, k), 1e-6));
    EXPECT(assert_equal(new_base_twist, Twist(new_manifold.values(), vision60.base_id, k), 1e-6));

    // check all constraints are satisfied
    EXPECT(assert_equal(e_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
    EXPECT(assert_equal(i_constraints.evaluateViolationL2Norm(new_manifold.values()), 0.0));
  }

  // TODO: consider about handling cases that violate i-constriants.

}


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
