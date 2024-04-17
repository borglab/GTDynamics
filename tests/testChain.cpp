/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testChain.cpp
 * @brief Test Chain class.
 * @author: Dan Barladeanu, Frank Dellaert
 */

#define BOOST_BIND_NO_PLACEHOLDERS
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/Chain.h>
#include <gtdynamics/constraints/EqualityConstraint.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>

using namespace gtdynamics;
using gtsam::VectorExpressionEquality;
using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector3;
using gtsam::Vector6;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// Test Chain class functionality with a three-joint chain - compose method
TEST(Chain, ThreeLinksComposeMethod) {
  // Initialize pose and screw axis for chain instantiation
  Pose3 sMb = Pose3(Rot3(), Point3(5, 0, 0));
  Matrix screwAxis(6, 1);
  screwAxis << 0.0, 0.0, 1.0, 0.0, 5.0, 0.0;

  // Instantiate chains and create a vector
  Chain joint1(sMb, screwAxis), joint2(sMb, screwAxis), joint3(sMb, screwAxis);
  std::vector<Chain> chains{joint1, joint2, joint3};

  // Compose chains
  Chain composed = Chain::compose(chains);

  // Check result of composition
  Pose3 expected = Pose3(Rot3(), Point3(15, 0, 0));
  Matrix expected_J(6, 3);
  expected_J << 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 15, 10, 5, 0, 0, 0;
  EXPECT(assert_equal(composed.sMb(), expected, 1e-6));
  EXPECT(assert_equal(composed.axes(), expected_J, 1e-6));
}

// Test Chain class functionality with a three-joint chain - POE method with FK
// at rest
TEST(Chain, ThreeLinksPoeRest) {
  // Initialize pose and screw axis for chain instantiation
  Pose3 sMb = Pose3(Rot3(), Point3(5, 0, 0));
  Matrix screwAxis(6, 1);
  screwAxis << 0.0, 0.0, 1.0, 0.0, 5.0, 0.0;

  // Instantiate chains and create a vector
  Chain joint1(sMb, screwAxis), joint2(sMb, screwAxis), joint3(sMb, screwAxis);
  std::vector<Chain> chains{joint1, joint2, joint3};

  // Compose chains
  Chain composed = Chain::compose(chains);

  // Set expected values
  Pose3 expected = Pose3(Rot3(), Point3(15, 0, 0));
  Matrix expected_J(6, 3);
  expected_J << 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 15, 10, 5, 0, 0, 0;

  // Check poe for FK at rest (no jacobian)
  Vector joint_angles(3);
  joint_angles << 0, 0, 0;
  Pose3 POE = composed.poe(joint_angles);
  EXPECT(assert_equal(POE, expected, 1e-6));

  // Check poe for FK at rest (with jacobian)
  Matrix J0;
  POE = composed.poe(joint_angles, {}, J0);
  EXPECT(assert_equal(POE, expected, 1e-6));
  EXPECT(assert_equal(J0, expected_J, 1e-6));
}

// Test Chain class functionality with a three-joint chain - POE method with FK
// NOT at rest
TEST(Chain, ThreeLinksPoeNotRest) {
  // Initialize pose and screw axis for chain instantiation
  Pose3 sMb = Pose3(Rot3(), Point3(5, 0, 0));
  Matrix screwAxis(6, 1);
  screwAxis << 0.0, 0.0, 1.0, 0.0, 5.0, 0.0;

  // Instantiate chains and create a vector
  Chain joint1(sMb, screwAxis), joint2(sMb, screwAxis), joint3(sMb, screwAxis);
  std::vector<Chain> chains{joint1, joint2, joint3};

  // Compose chains
  Chain composed = Chain::compose(chains);

  // Set expected values
  Pose3 expected_not_rest = Pose3(Rot3::Rz(M_PI / 2), Point3(10, 5, 0));
  Matrix expected_J1(6, 3);
  expected_J1 << 0, 0, 0, 0, 0, 0, 1, 1, 1, 10, 5, 0, 5, 5, 5, 0, 0, 0;

  // Check poe for FK Not at rest (with jacobian)
  Vector joint_angles1(3);
  joint_angles1 << 0, 0, M_PI / 2;
  Matrix J1;
  Pose3 POE1 = composed.poe(joint_angles1, {}, J1);
  EXPECT(assert_equal(POE1, expected_not_rest, 1e-6));
  EXPECT(assert_equal(J1, expected_J1, 1e-6));
}

// Test Chain class functionality with no joints
TEST(Chain, ZeroLinks) {
  std::vector<Chain> chains;
  Chain composed = Chain::compose(chains);
  Matrix emptyMat(6, 0);
  EXPECT(assert_equal(composed.sMb(), Pose3(), 1e-6));
  EXPECT(assert_equal(composed.axes(), emptyMat, 1e-6));
}

// Test Exception on Chain Constructor
TEST(Chain, InitChain) {
  // Initialize arguments
  Pose3 sMb = Pose3(Rot3(), Point3(5, 0, 0));
  Matrix screwAxis0;
  Matrix screwAxis5(5, 1);
  screwAxis5 << 0.0, 0.0, 1.0, 0.0, 5.0;

  // screwAxis which does not have 6 rows will throw an exception
  THROWS_EXCEPTION(Chain(sMb, screwAxis0));
  THROWS_EXCEPTION(Chain(sMb, screwAxis5));
}

// Test Chain Constraint
TEST(Chain, ChainConstraint) {
  // Get three link robot
  Robot robot = CreateRobotFromFile(
      kSdfPath + std::string("test/simple_rrr.sdf"), "simple_rrr_sdf");

  // Create a single link chain using the robot joints
  std::vector<Chain> chains;
  for (auto&& joint : robot.joints()) {
    Chain single_link_chain(joint->pMc(), joint->cScrewAxis());
    chains.emplace_back(single_link_chain);
  }

  // Compose chains
  Chain composed = Chain::compose(chains);
  Pose3 expected_sMb = Pose3(Rot3(), Point3(0, 0, 1.6));
  Matrix expected_axes(6, 3);
  expected_axes << 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0.9, 0.3, 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(composed.sMb(), expected_sMb, 1e-6));
  EXPECT(assert_equal(composed.axes(), expected_axes, 1e-6));

  // Get key for wrench at joint 1 on link 0 at time 0
  const gtsam::Key wrench_key = gtdynamics::WrenchKey(0, 1, 0);

  // Get expression for chain constraint using 3 link chain at time 0
  auto expression = composed.ChainConstraint3(robot.joints(), wrench_key, 0);

  // Create initial values.
  gtsam::Values init_values;
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&init_values, joint->id(), 0, 0.0);
    InsertTorque(&init_values, joint->id(), 0, 0.0);
  }

  // Set initial values for wrench
  gtsam::Vector wrench(6);
  wrench << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  InsertWrench(&init_values, 0, 1, 0, wrench);

  // Set tolerance
  Vector3 tolerance(0.1, 0.1, 0.1);

  // Create VectorExpressionEquality Constraint
  auto constraint = VectorExpressionEquality<3>(expression, tolerance);
  Vector3 expected_values(1, 1.9, 1.3);
  bool constraint_violation = constraint.feasible(init_values);
  Vector values = constraint(init_values);
  EXPECT(!constraint_violation);
  EXPECT(assert_equal(values, expected_values, 1e-6));

  // Create Factor same as in Optimizer for SOFT_CONSTRAINT
  auto factor = constraint.createFactor(1.0);

  // Check error
  auto error = factor->unwhitenedError(init_values);
  EXPECT(assert_equal(error, expected_values, 1e-6));
}

// Test Chain Constraint Jacobians - MakeVector3
TEST(Chain, MakeVector3Jacobians) {
  Matrix J0, J1, J2;
  MakeVector3(17.0, 18.0, 19.0, J0, {}, {});
  MakeVector3(0.0, 0.0, 207.34567, {}, J1, {});
  MakeVector3(-9.0, -18.0, -1.0, {}, {}, J2);

  // binded function for numerical derivative
  auto f = std::bind(MakeVector3, _1, _2, _3, nullptr, nullptr, nullptr);

  auto numericalH0 =
      gtsam::numericalDerivative31<Vector3, double, double, double>(f, 17.0,
                                                                    18.0, 19.0);

  auto numericalH1 =
      gtsam::numericalDerivative32<Vector3, double, double, double>(f, 0.0, 0.0,
                                                                    207.34567);

  auto numericalH2 =
      gtsam::numericalDerivative33<Vector3, double, double, double>(
          f, -9.0, -18.0, -1.0);

  EXPECT(assert_equal(numericalH0, J0));
  EXPECT(assert_equal(numericalH1, J1));
  EXPECT(assert_equal(numericalH2, J2));
}

Matrix get_angles_test_cases() {
  Matrix angles_cases(10, 3);
  angles_cases << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, M_PI / 2, 0.0,
      0.0, 0.0, M_PI / 8, 0.0, 0.0, 0.0, M_PI / 14, M_PI / 14, M_PI / 7, 0.0,
      M_PI / 14, 0.0, M_PI / 7, 0.0, M_PI / 14, M_PI / 7, 0.0, M_PI / 14,
      M_PI / 7;
  return angles_cases;
}

Matrix get_torques_test_cases() {
  Matrix torques_cases(10, 3);
  torques_cases << 0.0, 0.0, 0.0, 100.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0;
  return torques_cases;
}

Matrix get_wrench_test_cases() {
  Matrix wrench_cases(10, 6);
  wrench_cases << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0,
      0.0, 0.0, 1.0;
  return wrench_cases;
}

// Test Chain Constraint Jacobians - DynamicalEquality3 - H_angles - first type
// of chain
TEST(Chain, DynamicalEquality3_H_angles_chain1) {
  // Initialize pose and screw axis for chain instantiation
  Pose3 sMb = Pose3(Rot3(), Point3(1, 1, 1));
  Matrix screwAxis(6, 1);
  screwAxis << 0.0, 0.0, 1.0, 0.0, 5.0, 0.0;

  // Instantiate chains and create a vector
  Chain joint1(sMb, screwAxis), joint2(sMb, screwAxis), joint3(sMb, screwAxis);
  std::vector<Chain> chains{joint1, joint2, joint3};

  // Compose chains
  Chain composed = Chain::compose(chains);

  Vector3 angles, torques;
  gtsam::Vector wrench(6);
  Matrix J1, J;

  // binded function for numerical derivative
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3, nullptr,
                     nullptr, nullptr);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative32<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, {}, J1, {});
    return J1;
  };

  Matrix anglesMat = get_angles_test_cases();
  Matrix torquesMat = get_torques_test_cases();
  Matrix wrenchMat = get_wrench_test_cases();

  for (int i = 0; i < anglesMat.rows(); ++i) {
    angles = anglesMat.row(i);
    torques = torquesMat.row(i);
    wrench = wrenchMat.row(i);

    auto numericalH_case = num_derivative(wrench, angles, torques);

    J = get_jacobian(wrench, angles, torques);

    EXPECT(assert_equal(J, numericalH_case, 1e-5));
  }
}

// Test Chain Constraint Jacobians - DynamicalEquality3 - H_angles - second type
// of chain
TEST(Chain, DynamicalEquality3_H_angles_chain2) {
  // Initialize pose and screw axis for chain instantiation
  Pose3 sMb = Pose3(Rot3(), Point3(17.0, -3.0, 99.5));
  Matrix screwAxis(6, 1);
  double one_over_sqrt_3 = 1 / sqrt(3);
  screwAxis << one_over_sqrt_3, one_over_sqrt_3, one_over_sqrt_3, 1.5, 1.5,
      0.0;  // size of omega should be 1

  // Instantiate chains and create a vector
  Chain joint1(sMb, screwAxis), joint2(sMb, screwAxis), joint3(sMb, screwAxis);
  std::vector<Chain> chains{joint1, joint2, joint3};

  // Compose chains
  Chain composed = Chain::compose(chains);

  Vector3 angles, torques;
  gtsam::Vector wrench(6);
  Matrix J1, J;

  // binded function for numerical derivative
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3, nullptr,
                     nullptr, nullptr);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative32<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, {}, J1, {});
    return J1;
  };

  Matrix anglesMat = get_angles_test_cases();
  Matrix torquesMat = get_torques_test_cases();
  Matrix wrenchMat = get_wrench_test_cases();

  for (int i = 0; i < anglesMat.rows(); ++i) {
    angles = anglesMat.row(i);
    torques = torquesMat.row(i);
    wrench = wrenchMat.row(i);

    auto numericalH_case = num_derivative(wrench, angles, torques);

    J = get_jacobian(wrench, angles, torques);

    EXPECT(assert_equal(J, numericalH_case, 1e-5));
  }
}

// Test Chain Constraint Jacobians - DynamicalEquality3 - H_angles - third type
// of chain
TEST(Chain, DynamicalEquality3_H_angles_chain3) {
  // Initialize pose and screw axis for chain instantiation
  Pose3 sMb = Pose3(Rot3(), Point3(0, 3, 8));
  Matrix screwAxis0(6, 1), screwAxis1(6, 1), screwAxis2(6, 1);
  screwAxis0 << 0.0, 0.0, 1.0, 0.0, 5.0, 0.0;
  screwAxis1 << 0.0, 1.0, 0.0, 3.0, 0.0, 0.0;
  screwAxis2 << 1.0, 0.0, 0.0, 0.0, 0.0, 29.0;

  // Instantiate chains and create a vector
  Chain joint1(sMb, screwAxis0), joint2(sMb, screwAxis1),
      joint3(sMb, screwAxis2);
  std::vector<Chain> chains{joint1, joint2, joint3};

  // Compose chains
  Chain composed = Chain::compose(chains);

  Vector3 angles, torques;
  gtsam::Vector wrench(6);
  Matrix J1, J;

  // binded function for numerical derivative
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3, nullptr,
                     nullptr, nullptr);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative32<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, {}, J1, {});
    return J1;
  };

  Matrix anglesMat = get_angles_test_cases();
  Matrix torquesMat = get_torques_test_cases();
  Matrix wrenchMat = get_wrench_test_cases();

  for (int i = 0; i < anglesMat.rows(); ++i) {
    angles = anglesMat.row(i);
    torques = torquesMat.row(i);
    wrench = wrenchMat.row(i);

    auto numericalH_case = num_derivative(wrench, angles, torques);

    J = get_jacobian(wrench, angles, torques);

    EXPECT(assert_equal(J, numericalH_case, 1e-5));
  }
}

// Test Chain Constraint Jacobians - DynamicalEquality3 - H_wrench
TEST(Chain, DynamicalEquality3_H_wrench) {
  // Initialize pose and screw axis for chain instantiation
  Pose3 sMb = Pose3(Rot3(), Point3(0, 3, 8));
  Matrix screwAxis0(6, 1), screwAxis1(6, 1), screwAxis2(6, 1);
  screwAxis0 << 0.0, 0.0, 1.0, 0.0, 5.0, 0.0;
  screwAxis1 << 0.0, 1.0, 0.0, 3.0, 0.0, 0.0;
  screwAxis2 << 1.0, 0.0, 0.0, 0.0, 0.0, 29.0;

  // Instantiate chains and create a vector
  Chain joint1(sMb, screwAxis0), joint2(sMb, screwAxis1),
      joint3(sMb, screwAxis2);
  std::vector<Chain> chains{joint1, joint2, joint3};

  // Compose chains
  Chain composed = Chain::compose(chains);

  Vector3 angles, torques;
  gtsam::Vector wrench(6);
  Matrix J0, J;

  // binded function for numerical derivative
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3, nullptr,
                     nullptr, nullptr);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative31<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, J0, {}, {});
    return J0;
  };

  Matrix anglesMat = get_angles_test_cases();
  Matrix torquesMat = get_torques_test_cases();
  Matrix wrenchMat = get_wrench_test_cases();

  for (int i = 0; i < anglesMat.rows(); ++i) {
    angles = anglesMat.row(i);
    torques = torquesMat.row(i);
    wrench = wrenchMat.row(i);

    auto numericalH_case = num_derivative(wrench, angles, torques);

    J = get_jacobian(wrench, angles, torques);

    EXPECT(assert_equal(J, numericalH_case, 1e-5));
  }
}

// Test Chain Constraint Jacobians - DynamicalEquality3 - H_torques
TEST(Chain, DynamicalEquality3_H_torques) {
  // Initialize pose and screw axis for chain instantiation
  Pose3 sMb = Pose3(Rot3(), Point3(0, 3, 8));
  Matrix screwAxis0(6, 1), screwAxis1(6, 1), screwAxis2(6, 1);
  screwAxis0 << 0.0, 0.0, 1.0, 0.0, 5.0, 0.0;
  screwAxis1 << 0.0, 1.0, 0.0, 3.0, 0.0, 0.0;
  screwAxis2 << 1.0, 0.0, 0.0, 0.0, 0.0, 29.0;

  // Instantiate chains and create a vector
  Chain joint1(sMb, screwAxis0), joint2(sMb, screwAxis1),
      joint3(sMb, screwAxis2);
  std::vector<Chain> chains{joint1, joint2, joint3};

  // Compose chains
  Chain composed = Chain::compose(chains);

  Vector3 angles, torques;
  gtsam::Vector wrench(6);
  Matrix J2, J;

  // binded function for numerical derivative
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3, nullptr,
                     nullptr, nullptr);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative33<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, {}, {}, J2);
    return J2;
  };

  Matrix anglesMat = get_angles_test_cases();
  Matrix torquesMat = get_torques_test_cases();
  Matrix wrenchMat = get_wrench_test_cases();

  for (int i = 0; i < anglesMat.rows(); ++i) {
    angles = anglesMat.row(i);
    torques = torquesMat.row(i);
    wrench = wrenchMat.row(i);

    auto numericalH_case = num_derivative(wrench, angles, torques);

    J = get_jacobian(wrench, angles, torques);

    EXPECT(assert_equal(J, numericalH_case, 1e-5));
  }
}

// Test Chain Constraint Factor Jacobians
TEST(Chain, ChainConstraintFactorJacobians) {
  // Get three link robot
  Robot robot = CreateRobotFromFile(
      kSdfPath + std::string("test/simple_rrr.sdf"), "simple_rrr_sdf");

  // Create a single link chain using the robot joints
  std::vector<Chain> chains;
  for (auto&& joint : robot.joints()) {
    Chain single_link_chain(joint->pMc(), joint->cScrewAxis());
    chains.emplace_back(single_link_chain);
  }

  // Compose chains
  Chain composed = Chain::compose(chains);
  Pose3 expected_sMb = Pose3(Rot3(), Point3(0, 0, 1.6));
  Matrix expected_axes(6, 3);
  expected_axes << 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0.9, 0.3, 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(composed.sMb(), expected_sMb, 1e-6));
  EXPECT(assert_equal(composed.axes(), expected_axes, 1e-6));

  // Get key for wrench at joint 1 on link 0 at time 0
  const gtsam::Key wrench_key = gtdynamics::WrenchKey(0, 1, 0);

  // Get expression for chain constraint using 3 link chain at time 0
  auto expression = composed.ChainConstraint3(robot.joints(), wrench_key, 0);

  // Create initial values.
  gtsam::Values init_values;
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&init_values, joint->id(), 0, 0.0);
    InsertTorque(&init_values, joint->id(), 0, 0.0);
  }

  // Set initial values for wrench
  gtsam::Vector wrench(6);
  wrench << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  InsertWrench(&init_values, 0, 1, 0, wrench);

  // Set tolerance
  Vector3 tolerance(0.1, 0.1, 0.1);

  // Create VectorExpressionEquality Constraint
  auto constraint = VectorExpressionEquality<3>(expression, tolerance);
  Vector3 expected_values(1, 1.9, 1.3);
  bool constraint_violation = constraint.feasible(init_values);
  Vector values = constraint(init_values);
  EXPECT(!constraint_violation);
  EXPECT(assert_equal(values, expected_values, 1e-6));

  // Create Factor same as in Optimizer for SOFT_CONSTRAINT
  auto factor = constraint.createFactor(1.0);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, init_values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
