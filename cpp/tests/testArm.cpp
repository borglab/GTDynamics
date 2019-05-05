/**
 * @file  testArm.cpp
 * @brief test robotic arm
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <cmath>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

#include <Arm.h>
#include <CppUnitLite/TestHarness.h>
#include <DHLink.h>
#include <URDFLink.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

static const double HALF_PI = M_PI / 2;
static const Rot3 R90 = Rot3::Rz(HALF_PI);
static const Rot3 R180 = Rot3::Rz(M_PI);
static const Vector6 ZERO6 = Vector6::Zero();
static const Vector2 ZERO2 = Vector2::Zero();
static const double tol = 1e-9;
static const Vector2 QZ = Vector2(0, 0);        // at rest
static const Vector2 Q1 = Vector2(HALF_PI, 0);  // vertical
static const Vector2 Q2 = Vector2(0, M_PI);     // doubled back

// Expected link transform depending on angle.
Pose3 transform(double theta) {
  return Pose3(Rot3::Rz(theta), Point3(0, 0, 0)) * Pose3(Rot3(), Point3(2, 0, 0));
}

// Unit tests for dh link RR
TEST(Arm, DH_RR) {
  vector<DH_Link> dh_rr = {DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0),
                                   Vector3(0, 0, 0), -180, 180, 20),
                           DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0),
                                   Vector3(0, 0, 0), -180, 180, 20)};

  // The joint screw axis, in the COM frame, is the same for both joints
  auto AXIS = unit_twist(Vector3(0, 0, 1), Vector3(-1, 0, 0));
  auto robot = Arm<DH_Link>(dh_rr, Pose3());
  int N = robot.numLinks();

  vector<Pose3> frames;

  /* ===================== test link_transforms ====================== */
  // Check zero joint angles
  frames = robot.linkTransforms();
  EXPECT(assert_equal(frames.size(), 2));
  EXPECT(assert_equal(frames[0], transform(0)));
  EXPECT(assert_equal(frames[1], transform(0)));

  // Check vertical configuration
  frames = robot.linkTransforms(Q1);
  EXPECT(assert_equal(frames[0], transform(HALF_PI)));
  EXPECT(assert_equal(frames[1], transform(0)));

  // Check double back configuration
  frames = robot.linkTransforms(Q2);
  EXPECT(assert_equal(frames[0], transform(0)));
  EXPECT(assert_equal(frames[1], transform(M_PI)));
  /* ================================================================= */

  /* ======================= test link_frames ======================== */
  // Check zero joint angles
  frames = robot.linkFrames();
  EXPECT(assert_equal(frames.size(), 2));
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(2, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(Rot3(), Point3(4, 0, 0))));

  // Check vertical configuration
  frames = robot.linkFrames(Q1);
  EXPECT(assert_equal(frames[0], Pose3(R90, Point3(0, 2, 0))));
  EXPECT(assert_equal(frames[1], Pose3(R90, Point3(0, 4, 0))));

  // Check double back configuration
  frames = robot.linkFrames(Q2);
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(2, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(R180, Point3(0, 0, 0))));
  /* ================================================================= */

  /* ======================== test com_frames ======================== */
  // Check zero joint angles
  frames = robot.comFrames();
  EXPECT(assert_equal(frames.size(), 2));
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(1, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(Rot3(), Point3(3, 0, 0))));

  // Check vertical configuration
  frames = robot.comFrames(Q1);
  EXPECT(assert_equal(frames[0], Pose3(R90, Point3(0, 1, 0))));
  EXPECT(assert_equal(frames[1], Pose3(R90, Point3(0, 3, 0))));

  // Check double back configuration
  frames = robot.comFrames(Q2);
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(1, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(R180, Point3(1, 0, 0))));
  /* ================================================================= */

  /* ======================== test screw axes ======================== */
  // Check zero joint angles
  auto screw_axes = robot.screwAxes();
  EXPECT(assert_equal(screw_axes.size(), 2));
  EXPECT(assert_equal(screw_axes[0], AXIS));
  EXPECT(assert_equal(screw_axes[1], AXIS));
  /* ================================================================= */

  /* ==================== test spatial screw axes ==================== */
  // Check zero joint angles
  auto spatial_screw_axes = robot.spatialScrewAxes();
  Vector6 expected_spatial_screwAxes_1 =
      unit_twist(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Vector6 expected_spatial_screwAxes_2 =
      unit_twist(Vector3(0, 0, 1), Vector3(2, 0, 0));
  EXPECT(assert_equal(spatial_screw_axes.size(), 2));
  EXPECT(assert_equal(spatial_screw_axes[0], expected_spatial_screwAxes_1));
  EXPECT(assert_equal(spatial_screw_axes[1], expected_spatial_screwAxes_2));
  /* ================================================================= */

  /* ====================== test transform poe ======================= */
  // Check zero joint angles
  auto joint_frames = robot.transformPOE();
  EXPECT(assert_equal(joint_frames.size(), 2));
  EXPECT(assert_equal(joint_frames[0], Pose3(Rot3(), Point3(0, 0, 0))));
  EXPECT(assert_equal(joint_frames[1], Pose3(Rot3(), Point3(0, 0, 0))));

  // Check vertical configuration
  joint_frames = robot.transformPOE(Q1);
  EXPECT(assert_equal(joint_frames[0], Pose3(R90, Point3(0, 0, 0))));
  EXPECT(assert_equal(joint_frames[1], Pose3(R90, Point3(0, 0, 0))));

  // Check double back configuration
  joint_frames = robot.transformPOE(Q2);
  EXPECT(assert_equal(joint_frames[0], Pose3(Rot3(), Point3(0, 0, 0))));
  EXPECT(assert_equal(joint_frames[1], Pose3(R180, Point3(4, 0, 0))));
  /* ================================================================= */

  /* ===================== test spatial jacobian ===================== */
  // Check zero joint angles
  auto jacobian = robot.spatialManipulatorJacobian(QZ);
  auto jacobian_eef = jacobian.back();
  Matrix actual_col0 = (Matrix(6, 1) << 0, 0, 1, 0, 0, 0).finished();
  Matrix actual_col1 = (Matrix(6, 1) << 0, 0, 1, 0, -2, 0).finished();
  EXPECT(assert_equal(jacobian_eef.col(0), actual_col0));
  EXPECT(assert_equal(jacobian_eef.col(1), actual_col1));

  // Check vertical configuration
  jacobian = robot.spatialManipulatorJacobian(Q1);
  jacobian_eef = jacobian.back();
  actual_col0 = (Matrix(6, 1) << 0, 0, 1, 0, 0, 0).finished();
  actual_col1 = (Matrix(6, 1) << 0, 0, 1, 2, 0, 0).finished();
  EXPECT(assert_equal(jacobian_eef.col(0), actual_col0));
  EXPECT(assert_equal(jacobian_eef.col(1), actual_col1));

  // Check double back configuration
  jacobian = robot.spatialManipulatorJacobian(Q2);
  jacobian_eef = jacobian.back();
  actual_col0 = (Matrix(6, 1) << 0, 0, 1, 0, 0, 0).finished();
  actual_col1 = (Matrix(6, 1) << 0, 0, 1, 0, -2, 0).finished();
  EXPECT(assert_equal(jacobian_eef.col(0), actual_col0));
  EXPECT(assert_equal(jacobian_eef.col(1), actual_col1));
  /* ================================================================= */

  /* ======================= test body jacobian ====================== */
  // Check zero joint angles
  auto sTb = robot.forwardKinematics(QZ);
  jacobian = robot.bodyManipulatorJacobian(QZ, sTb);
  jacobian_eef = jacobian.back();
  actual_col0 = (Matrix(6, 1) << 0, 0, 1, 0, 4, 0).finished();
  actual_col1 = (Matrix(6, 1) << 0, 0, 1, 0, 2, 0).finished();
  EXPECT(assert_equal(jacobian_eef.col(0), actual_col0));
  EXPECT(assert_equal(jacobian_eef.col(1), actual_col1));
  /* ================================================================= */

  /* ==================== test inverse kinematics ==================== */
  Vector joint_coordinates_init = (Vector(2) << 0, 0).finished();
  Pose3 goal_pose =
      Pose3(Rot3::Rz(M_PI / 4), Point3(2.82842712, 2.82842712, 0));
  Vector actual_joint_coordinates =
      robot.inverseKinematics(goal_pose, joint_coordinates_init);
  Vector expected_joint_coordinates = (Vector(2) << M_PI / 4.0, 0).finished();
  EXPECT(
      assert_equal(actual_joint_coordinates, expected_joint_coordinates, 1e-6));
  /* ================================================================= */

  /* ========================= test jTi_list ========================= */
  // Check zero joint angles
  auto jTi_list = robot.jTi_list(QZ);
  EXPECT(assert_equal(jTi_list.size(), 3));
  EXPECT(assert_equal(jTi_list[0], Pose3(Rot3(), Point3(-1, 0, 0))));
  EXPECT(assert_equal(jTi_list[1], Pose3(Rot3(), Point3(-2, 0, 0))));
  EXPECT(assert_equal(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0))));

  // Check vertical configuration
  jTi_list = robot.jTi_list(Q1);
  EXPECT(assert_equal(jTi_list.size(), 3));
  EXPECT(assert_equal(jTi_list[0], Pose3(R90.inverse(), Point3(-1, 0, 0))));
  EXPECT(assert_equal(jTi_list[1], Pose3(Rot3(), Point3(-2, 0, 0))));
  EXPECT(assert_equal(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0))));

  // Check double back configuration
  jTi_list = robot.jTi_list(Q2);
  EXPECT(assert_equal(jTi_list.size(), 3));
  EXPECT(assert_equal(jTi_list[0], Pose3(Rot3(), Point3(-1, 0, 0))));
  EXPECT(assert_equal(jTi_list[1], Pose3(R180, Point3(0, 0, 0))));
  EXPECT(assert_equal(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0))));
  /* ================================================================= */

  /* ========================== test twists ========================== */
  // Check zero joint angles
  auto Ts = robot.comFrames();

  // Check zero joint velocities
  auto twists_test = robot.twists(Ts, Vector2(0, 0));
  EXPECT(assert_equal(twists_test.size(), 2));
  EXPECT(assert_equal(twists_test[0], ZERO6));
  EXPECT(assert_equal(twists_test[1], ZERO6));

  // Check rotating joint 1
  twists_test = robot.twists(Ts, Vector2(3, 0));
  auto expected = unit_twist(Vector3(0, 0, 3), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[0], expected));
  // second joint is also rotating around point (0,0,0), which is (-3,0,0) in
  // COM frame 2
  expected = unit_twist(Vector3(0, 0, 3), Vector3(-3, 0, 0));
  EXPECT(assert_equal(twists_test[1], expected));

  // Check rotating joint 2
  twists_test = robot.twists(Ts, Vector2(0, 2));
  EXPECT(assert_equal(twists_test[0], ZERO6));
  // second joint rotating around point (2,0,0), which is (-2,0,0) in COM frame
  // 2
  expected = unit_twist(Vector3(0, 0, 2), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[1], expected));

  // Check both rotating, should be linear combination
  twists_test = robot.twists(Ts, Vector2(3, 2));
  expected = unit_twist(Vector3(0, 0, 3), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[0], expected));
  // second joint rotating around point (2,0,0), which is (-2,0,0) in COM frame
  // 2
  expected = unit_twist(Vector3(0, 0, 3), Vector3(-3, 0, 0)) +
             unit_twist(Vector3(0, 0, 2), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[1], expected));

  // Check doubled back configuration
  Ts = robot.comFrames(Q2);

  // Check zero joint velocities
  twists_test = robot.twists(Ts, Vector2(3, 2));
  EXPECT(assert_equal(twists_test.size(), 2));
  expected = unit_twist(Vector3(0, 0, 3), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[0], expected));
  expected = unit_twist(Vector3(0, 0, 3), Vector3(1, 0, 0)) +
             unit_twist(Vector3(0, 0, 2), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[1], expected));
  /* ================================================================= */

  /* ====================== test stationary case ===================== */
  Vector joint_angles = ZERO2;
  Vector joint_velocities = ZERO2;
  Vector joint_accelerations = ZERO2;
  Vector torques = ZERO2;
  Vector expected_joint_accelerations = ZERO2;
  Vector expected_torques = ZERO2;
  Vector base_twist_accel = ZERO6;
  Vector external_wrench = ZERO6;

  auto factor_graph =
      robot.forwardDynamicsFactorGraph(joint_angles, joint_velocities, torques,
                                       base_twist_accel, external_wrench);
  EXPECT(assert_equal(factor_graph.size(), 1 + N * 3 + 1));

  auto result = robot.forwardDynamics(joint_angles, joint_velocities, torques,
                                      base_twist_accel, external_wrench);
  EXPECT(assert_equal(result, expected_joint_accelerations));

  result =
      robot.inverseDynamics(joint_angles, joint_velocities, joint_accelerations,
                            base_twist_accel, external_wrench);
  EXPECT(assert_equal(result, expected_torques));
  /* ================================================================= */

  /* ========= test case when an external wrench is applied ========== */
  joint_accelerations << 5, -20;
  external_wrench << 0, 0, 0, 0, -2.5, 0;
  expected_joint_accelerations << 5, -20;
  expected_torques << 0, 0;

  result = robot.forwardDynamics(joint_angles, joint_velocities, torques,
                                 base_twist_accel, external_wrench);
  EXPECT(assert_equal(result, expected_joint_accelerations));

  result =
      robot.inverseDynamics(joint_angles, joint_velocities, joint_accelerations,
                            base_twist_accel, external_wrench);
  EXPECT(assert_equal(result, expected_torques));
  /* ================================================================= */

  /* ======== gravity compensation case: assume Y-axis is up ========= */
  joint_accelerations << -9.8, 19.6;
  external_wrench << 0, 0, 0, 0, 0, 0;
  base_twist_accel << 0, 0, 0, 0, 0, 0;
  Vector3 gravity(0, -9.8, 0);
  expected_joint_accelerations << -9.8, 19.6;
  expected_torques << 0, 0;

  result = robot.forwardDynamics(joint_angles, joint_velocities, torques,
                                 base_twist_accel, external_wrench, gravity);
  EXPECT(assert_equal(result, expected_joint_accelerations));

  result =
      robot.inverseDynamics(joint_angles, joint_velocities, joint_accelerations,
                            base_twist_accel, external_wrench, gravity);
  EXPECT(assert_equal(result, expected_torques));
  /* ================================================================= */
}

// Unit tests for dh link Puma
TEST(Arm, DH_PUMA) {
  vector<DH_Link> dh_puma = {
      DH_Link(0, 0.0000, 0.0000, +90, 'R', 0, Point3(0, 0, 0),
              Vector3(0, 0.35, 0), -180, 180, 20),
      DH_Link(0, 0.4318, 0, 0.0, 'R', 17.40, Point3(-0.3638, 0.006, 0.2275),
              Vector3(0.130, 0.524, 0.539), -180, 180, 20),
      DH_Link(0, 0.0203, 0.15005, -90, 'R', 4.80,
              Point3(-0.0203, -0.0141, 0.0700), Vector3(0.066, 0.086, 0.0125),
              -180, 180, 20),
      DH_Link(0, 0, 0.4318, +90, 'R', 0.82, Point3(0, 0.19, 0),
              Vector3(0.0018, 0.0013, 0.0018), -180, 180, 20),
      DH_Link(0, 0.0000, 0.0000, -90, 'R', 0.34, Point3(0, 0, 0),
              Vector3(0.0003, 0.0004, 0.0003), -180, 180, 20),
      DH_Link(0, 0.0000, 0.0000, 0.0, 'R', 0.09, Point3(0, 0, 0.032),
              Vector3(0.00015, 0.00015, 0.00004), -180, 180, 20)};

  // Create Puma robot.
  auto robot = Arm<DH_Link>(dh_puma, Pose3(), Pose3());
  int N = robot.numLinks();
  vector<Pose3> frames;

  /* ==================== test forward kinematics ==================== */
  /* example from Corke 2017 page 203 */
  auto T = robot.forwardKinematics(ZERO6).back();
  EXPECT(assert_equal(T, Pose3(Rot3(), Point3(0.58185, -0.4521, 0))));
  /* ================================================================= */

  /* ======================= test link_frames ======================== */
  frames = robot.linkFrames();
  EXPECT(assert_equal(frames.size(), 6));
  EXPECT(assert_equal(frames[0], Pose3(Rot3::Rx(HALF_PI), Point3(0, 0, 0))));
  /* ================================================================= */

  /* ====================== test stationary case ===================== */
  Vector joint_angles = Vector6::Zero();
  Vector joint_velocities = Vector6::Zero();
  Vector joint_accelerations = Vector6::Zero();
  Vector torques = Vector6::Zero();
  Vector expected_joint_accelerations = Vector6::Zero();
  Vector expected_torques = Vector6::Zero();
  Vector base_twist_accel = ZERO6;
  Vector external_wrench = ZERO6;

  auto factor_graph =
      robot.forwardDynamicsFactorGraph(joint_angles, joint_velocities, torques,
                                       base_twist_accel, external_wrench);
  EXPECT(assert_equal(factor_graph.size(), 1 + N * 3 + 1));

  auto result = robot.forwardDynamics(joint_angles, joint_velocities, torques,
                                      base_twist_accel, external_wrench);
  EXPECT(assert_equal(result, expected_joint_accelerations));

  result =
      robot.inverseDynamics(joint_angles, joint_velocities, joint_accelerations,
                            base_twist_accel, external_wrench);
  EXPECT(assert_equal(result, expected_torques));
  /* ================================================================= */

  /* ================== test Mandy's MATLAB example ================== */
  joint_velocities << -5, -10, -15, -20, -25, -30;
  expected_torques << 2.14242560e+00, -4.72874900e+01, 1.37677604e+01,
      2.15162000e-01, 1.45261716e-03, 7.67944871e-05;
  expected_joint_accelerations << 0.174533, 0.349066, 0.523599, 0.698132,
      0.872665, 1.047198;
  torques << 2.14242560e+00, -4.72874900e+01, 1.37677604e+01, 2.15162000e-01,
      1.45261716e-03, 7.67944871e-05;
  joint_accelerations << 0.174533, 0.349066, 0.523599, 0.698132, 0.872665,
      1.047198;
  base_twist_accel << 0, 0, 0, 0, 0, 0;
  Vector3 gravity(0, 0, -9.8);
  factor_graph = robot.forwardDynamicsFactorGraph(
      joint_angles, radians(joint_velocities), torques, base_twist_accel,
      external_wrench, gravity);
  EXPECT(assert_equal(factor_graph.size(), 1 + N * 3 + 1));

  result =
      robot.forwardDynamics(joint_angles, radians(joint_velocities), torques,
                            base_twist_accel, external_wrench, gravity);
  EXPECT(assert_equal(result, expected_joint_accelerations, 1e-5));

  result = robot.inverseDynamics(joint_angles, radians(joint_velocities),
                                 joint_accelerations, base_twist_accel,
                                 external_wrench, gravity);
  EXPECT(assert_equal(result, expected_torques, 1e-5));
  /* ================================================================= */
}

// Unit tests for urdf link RR
TEST(Arm, URDF_RR) {
  vector<URDF_Link> urdf_rr = {
      URDF_Link(Pose3(Rot3(), Point3(2, 0, 0)), Vector3(0, 0, 1), 'R', 1,
                Pose3(Rot3(), Point3(1, 0, 0)), Vector3(0, 0, 0), -180, 180,
                20),
      URDF_Link(Pose3(Rot3(), Point3(2, 0, 0)), Vector3(0, 0, 1), 'R', 1,
                Pose3(Rot3(), Point3(1, 0, 0)), Vector3(0, 0, 0), -180, 180,
                20)};

  // The joint screw axis, in the COM frame, is the same for both joints
  auto AXIS = unit_twist(Vector3(0, 0, 1), Vector3(-1, 0, 0));
  auto robot = Arm<URDF_Link>(urdf_rr, Pose3(), Pose3(Rot3(), Point3(2, 0, 0)));
  int N = robot.numLinks();

  vector<Pose3> frames;

  /* ===================== test link_transforms ====================== */
  // Check zero joint angles
  frames = robot.linkTransforms();
  EXPECT(assert_equal(frames.size(), 2));
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(2, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(Rot3(), Point3(2, 0, 0))));

  // Check vertical configuration
  frames = robot.linkTransforms(Q1);
  EXPECT(assert_equal(frames[0], Pose3(R90, Point3(2, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(Rot3(), Point3(2, 0, 0))));

  // Check double back configuration
  frames = robot.linkTransforms(Q2);
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(2, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(R180, Point3(2, 0, 0))));
  /* ================================================================= */

  /* ======================= test link_frames ======================== */
  // Check zero joint angles
  frames = robot.linkFrames();
  EXPECT(assert_equal(frames.size(), 2));
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(2, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(Rot3(), Point3(4, 0, 0))));

  // Check vertical configuration
  frames = robot.linkFrames(Q1);
  EXPECT(assert_equal(frames[0], Pose3(R90, Point3(2, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(R90, Point3(2, 2, 0))));

  // Check double back configuration
  frames = robot.linkFrames(Q2);
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(2, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(R180, Point3(4, 0, 0))));
  /* ================================================================= */

  /* ======================== test com_frames ======================== */
  // Check zero joint angles
  frames = robot.comFrames();
  EXPECT(assert_equal(frames.size(), 2));
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(3, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(Rot3(), Point3(5, 0, 0))));

  // Check vertical configuration
  frames = robot.comFrames(Q1);
  EXPECT(assert_equal(frames[0], Pose3(R90, Point3(2, 1, 0))));
  EXPECT(assert_equal(frames[1], Pose3(R90, Point3(2, 3, 0))));

  // Check double back configuration
  frames = robot.comFrames(Q2);
  EXPECT(assert_equal(frames[0], Pose3(Rot3(), Point3(3, 0, 0))));
  EXPECT(assert_equal(frames[1], Pose3(R180, Point3(3, 0, 0))));
  /* ================================================================= */

  /* ======================== test screw axes ======================== */
  // Check zero joint angles
  auto screw_axes = robot.screwAxes();
  EXPECT(assert_equal(screw_axes.size(), 2));
  EXPECT(assert_equal(screw_axes[0], AXIS));
  EXPECT(assert_equal(screw_axes[1], AXIS));
  /* ================================================================= */

  /* ========================= test jTi_list ========================= */
  // Check zero joint angles
  auto jTi_list = robot.jTi_list(QZ);
  EXPECT(assert_equal(jTi_list.size(), 3));
  EXPECT(assert_equal(jTi_list[0], Pose3(Rot3(), Point3(-3, 0, 0))));
  EXPECT(assert_equal(jTi_list[1], Pose3(Rot3(), Point3(-2, 0, 0))));
  EXPECT(assert_equal(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0))));

  // Check vertical configuration
  jTi_list = robot.jTi_list(Q1);
  EXPECT(assert_equal(jTi_list.size(), 3));
  EXPECT(assert_equal(jTi_list[0], Pose3(R90.inverse(), Point3(-1, 2, 0))));
  EXPECT(assert_equal(jTi_list[1], Pose3(Rot3(), Point3(-2, 0, 0))));
  EXPECT(assert_equal(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0))));

  // Check double back configuration
  jTi_list = robot.jTi_list(Q2);
  EXPECT(assert_equal(jTi_list.size(), 3));
  EXPECT(assert_equal(jTi_list[0], Pose3(Rot3(), Point3(-3, 0, 0))));
  EXPECT(assert_equal(jTi_list[1], Pose3(R180, Point3(0, 0, 0))));
  EXPECT(assert_equal(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0))));
  /* ================================================================= */

  /* ========================== test twists ========================== */
  // Check zero joint angles
  auto Ts = robot.comFrames();

  // Check zero joint velocities
  auto twists_test = robot.twists(Ts, Vector2(0, 0));
  EXPECT(assert_equal(twists_test.size(), 2));
  EXPECT(assert_equal(twists_test[0], ZERO6));
  EXPECT(assert_equal(twists_test[1], ZERO6));

  // Check rotating joint 1
  twists_test = robot.twists(Ts, Vector2(3, 0));
  auto expected = unit_twist(Vector3(0, 0, 3), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[0], expected));
  // second joint is also rotating around point (0,0,0), which is (-3,0,0) in
  // COM frame 2
  expected = unit_twist(Vector3(0, 0, 3), Vector3(-3, 0, 0));
  EXPECT(assert_equal(twists_test[1], expected));

  // Check rotating joint 2
  twists_test = robot.twists(Ts, Vector2(0, 2));
  EXPECT(assert_equal(twists_test[0], ZERO6));
  // second joint rotating around point (2,0,0), which is (-2,0,0) in COM frame
  // 2
  expected = unit_twist(Vector3(0, 0, 2), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[1], expected));

  // Check both rotating, should be linear combination
  twists_test = robot.twists(Ts, Vector2(3, 2));
  expected = unit_twist(Vector3(0, 0, 3), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[0], expected));
  // second joint rotating around point (2,0,0), which is (-2,0,0) in COM frame
  // 2
  expected = unit_twist(Vector3(0, 0, 3), Vector3(-3, 0, 0)) +
             unit_twist(Vector3(0, 0, 2), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[1], expected));

  // Check doubled back configuration
  Ts = robot.comFrames(Q2);

  // Check zero joint velocities
  twists_test = robot.twists(Ts, Vector2(3, 2));
  EXPECT(assert_equal(twists_test.size(), 2));
  expected = unit_twist(Vector3(0, 0, 3), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[0], expected));
  expected = unit_twist(Vector3(0, 0, 3), Vector3(1, 0, 0)) +
             unit_twist(Vector3(0, 0, 2), Vector3(-1, 0, 0));
  EXPECT(assert_equal(twists_test[1], expected));
  /* ================================================================= */

  /* ====================== test stationary case ===================== */
  Vector joint_angles = ZERO2;
  Vector joint_velocities = ZERO2;
  Vector joint_accelerations = ZERO2;
  Vector torques = ZERO2;
  Vector expected_joint_accelerations = ZERO2;
  Vector expected_torques = ZERO2;
  Vector base_twist_accel = ZERO6;
  Vector external_wrench = ZERO6;

  auto factor_graph =
      robot.forwardDynamicsFactorGraph(joint_angles, joint_velocities, torques,
                                       base_twist_accel, external_wrench);
  EXPECT(assert_equal(factor_graph.size(), 1 + N * 3 + 1));

  auto result = robot.forwardDynamics(joint_angles, joint_velocities, torques,
                                      base_twist_accel, external_wrench);
  EXPECT(assert_equal(result, expected_joint_accelerations));

  result = robot.inverseDynamics(joint_angles, radians(joint_velocities),
                                 joint_accelerations, base_twist_accel,
                                 external_wrench);
  EXPECT(assert_equal(result, expected_torques));
  /* ================================================================= */

  /* ========= test case when an external wrench is applied ========== */
  joint_accelerations << 5, -20;
  external_wrench << 0, 0, 0, 0, -2.5, 0;
  expected_joint_accelerations << 5, -20;
  expected_torques << 0, 0;

  result = robot.forwardDynamics(joint_angles, joint_velocities, torques,
                                 base_twist_accel, external_wrench);
  EXPECT(assert_equal(result, expected_joint_accelerations));

  result =
      robot.inverseDynamics(joint_angles, joint_velocities, joint_accelerations,
                            base_twist_accel, external_wrench);
  EXPECT(assert_equal(result, expected_torques));
  /* ================================================================= */

  /* ======== gravity compensation case: assume Y-axis is up ========= */
  joint_accelerations << -9.8, 19.6;
  external_wrench << 0, 0, 0, 0, 0, 0;
  base_twist_accel << 0, 0, 0, 0, 0, 0;
  Vector3 gravity(0, -9.8, 0);
  expected_joint_accelerations << -9.8, 19.6;
  expected_torques << 0, 0;

  result = robot.forwardDynamics(joint_angles, joint_velocities, torques,
                                 base_twist_accel, external_wrench, gravity);
  EXPECT(assert_equal(result, expected_joint_accelerations));

  result =
      robot.inverseDynamics(joint_angles, joint_velocities, joint_accelerations,
                            base_twist_accel, external_wrench, gravity);
  EXPECT(assert_equal(result, expected_torques));
  /* ================================================================= */
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
