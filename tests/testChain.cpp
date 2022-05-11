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
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/Optimizer.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>

using namespace gtdynamics;
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
  POE = composed.poe(joint_angles, boost::none, J0);
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
  Pose3 POE1 = composed.poe(joint_angles1, boost::none, J1);
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
  Vector3 expected_values(1, 0.46, 0.82);
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
  MakeVector3(17.0, 18.0, 19.0, J0, boost::none, boost::none);
  MakeVector3(0.0, 0.0, 207.34567, boost::none, J1, boost::none);
  MakeVector3(-9.0, -18.0, -1.0, boost::none, boost::none, J2);

  // binded function for numerical derivative
  auto f =
      std::bind(MakeVector3, _1, _2, _3, boost::none, boost::none, boost::none);

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
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3,
                     boost::none, boost::none, boost::none);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative32<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, boost::none, J1,
                                boost::none);
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
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3,
                     boost::none, boost::none, boost::none);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative32<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, boost::none, J1,
                                boost::none);
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
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3,
                     boost::none, boost::none, boost::none);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative32<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, boost::none, J1,
                                boost::none);
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
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3,
                     boost::none, boost::none, boost::none);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative31<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, J0, boost::none,
                                boost::none);
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
  auto f = std::bind(&Chain::DynamicalEquality3, composed, _1, _2, _3,
                     boost::none, boost::none, boost::none);

  // lambda function to get numerical derivative
  auto num_derivative = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    return gtsam::numericalDerivative33<Vector3, Vector6, Vector3, Vector3>(
        f, wrench, angles, torques);
  };

  // lambda function to get the Jacobian
  auto get_jacobian = [&](Vector6 wrench, Vector3 angles, Vector3 torques) {
    composed.DynamicalEquality3(wrench, angles, torques, boost::none,
                                boost::none, J2);
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
  Vector3 expected_values(1, 0.46, 0.82);
  bool constraint_violation = constraint.feasible(init_values);
  Vector values = constraint(init_values);
  EXPECT(!constraint_violation);
  EXPECT(assert_equal(values, expected_values, 1e-6));

  // Create Factor same as in Optimizer for SOFT_CONSTRAINT
  auto factor = constraint.createFactor(1.0);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, init_values, 1e-7, 1e-3);
}

TEST(Chain, A1QuadOneLegCompare) {
  // This test checks equality between torque calculation with and without
  // chains. This assumes one static leg of the a1 quadruped with zero mass for
  // the links besides the trunk.

  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

  // initialize joints of FL leg
  JointSharedPtr j0,j1,j2;

  for (auto&& joint : robot.joints()) {
    if (joint->name().find("FL") != std::string::npos) {
      if (joint->name().find("lower") != std::string::npos) {
        j2 = joint;
      }
      if (joint->name().find("upper") != std::string::npos) {
        j1 = joint;
      }
      if (joint->name().find("hip") != std::string::npos) {
        j0 = joint;
      }
    }
  }

  // Calculate all relevant relative poses.
  Pose3 M_T_H = j0->pMc();
  Pose3 M_H_T = M_T_H.inverse();
  Pose3 M_H_U = j1->pMc();
  Pose3 M_U_H = M_H_U.inverse();
  Pose3 M_U_L = j2->pMc();
  Pose3 M_L_U = M_U_L.inverse();
  Pose3 M_T_L = M_T_H * M_H_U * M_U_L;
  Pose3 M_L_T = M_T_L.inverse();

  // Set Gravity Wrench
  Matrix gravity(1, 6);
  gravity << 0.0, 0.0, 0.0, 0.0, 0.0, -10.0;

  // Calculate all wrenches and torques without chains.
  Matrix F_2_L = -gravity * M_T_L.AdjointMap();
  Matrix F_1_U = F_2_L * M_L_U.AdjointMap();
  Matrix F_0_H = F_1_U * M_U_H.AdjointMap();

  // joint 0
  Matrix tau0 = F_0_H * j0->cScrewAxis();
  EXPECT(assert_equal(tau0(0, 0), -0.448145, 1e-6));  // regression

  // joint 1
  Matrix tau1 = F_1_U * j1->cScrewAxis();
  EXPECT(assert_equal(tau1(0, 0), 1.70272, 1e-5));  // regression

  // joint 2
  Matrix tau2 = F_2_L * j2->cScrewAxis();
  EXPECT(assert_equal(tau2(0, 0), 1.70272, 1e-5));  // regression

  // Calculate all wrenches and torques with chains.
  Chain chain1(M_T_H, j0->cScrewAxis());
  Chain chain2(M_H_U, j1->cScrewAxis());
  Chain chain3(M_U_L, j2->cScrewAxis());

  std::vector<Chain> chains{chain1, chain2, chain3};

  // Compose Chains
  Chain composed = Chain::compose(chains);

  // test for same values
  Matrix torques = F_2_L * composed.axes();
  EXPECT(assert_equal(torques(0, 0), tau0(0, 0), 1e-6));
  EXPECT(assert_equal(torques(0, 1), tau1(0, 0), 1e-5));
  EXPECT(assert_equal(torques(0, 2), tau2(0, 0), 1e-5));
}

std::vector<std::vector<JointSharedPtr>> getChainJoints(const Robot& robot) {
  std::vector<JointSharedPtr> FR(3), FL(3), RR(3), RL(3);

  int loc;
  for (auto&& joint : robot.joints()) {
    if (joint->name().find("lower") != std::string::npos) {
      loc = 2;
    }
    if (joint->name().find("upper") != std::string::npos) {
      loc = 1;
    }
    if (joint->name().find("hip") != std::string::npos) {
      loc = 0;
    }
    if (joint->name().find("FR") != std::string::npos) {
      FR[loc] = joint;
    }
    if (joint->name().find("FL") != std::string::npos) {
      FL[loc] = joint;
    }
    if (joint->name().find("RR") != std::string::npos) {
      RR[loc] = joint;
    }
    if (joint->name().find("RL") != std::string::npos) {
      RL[loc] = joint;
    }
  }

  std::vector<std::vector<JointSharedPtr>> chain_joints{FL, FR, RL, RR};

  return chain_joints;
}

Chain BuildChain(std::vector<JointSharedPtr>& joints) {
  auto j0 = joints[0];
  auto j1 = joints[1];
  auto j2 = joints[2];

  // Calculate all relevant relative poses.
  Pose3 M_T_H = j0->pMc();
  Pose3 M_H_T = M_T_H.inverse();
  Pose3 M_H_U = j1->pMc();
  Pose3 M_U_H = M_H_U.inverse();
  Pose3 M_U_L = j2->pMc();
  Pose3 M_L_U = M_U_L.inverse();
  Pose3 M_T_L = M_T_H * M_H_U * M_U_L;
  Pose3 M_L_T = M_T_L.inverse();

  // Create chains
  Chain chain1(M_T_H, j0->cScrewAxis());
  Chain chain2(M_H_U, j1->cScrewAxis());
  Chain chain3(M_U_L, j2->cScrewAxis());

  std::vector<Chain> chains{chain1, chain2, chain3};

  Chain composed = Chain::compose(chains);

  return composed;
}

gtsam::Matrix16 get_joint3_screw_axis(std::vector<JointSharedPtr>& joints) {
  Point3 contact_in_com(0.0, 0.0, -0.07);
  Pose3 M_L_G = Pose3(Rot3(), contact_in_com);

  Pose3 M_T_G = joints[0]->pMc() * joints[1]->pMc() * joints[2]->pMc() * M_L_G;

  Vector6 A_G_3 = joints[2]->jMc().AdjointMap() * joints[2]->cScrewAxis();
  return (M_T_G.AdjointMap() * A_G_3).transpose();
}

gtsam::Double_ getContactConstraint(std::vector<JointSharedPtr>& joints) {

  gtsam::Matrix16 A_T_3 = get_joint3_screw_axis(joints);

  const int id = joints[0]->id();
  const gtsam::Key wrench_key =
      gtdynamics::WrenchKey(0, id, 0); 

  gtsam::Vector6_ wrench(wrench_key);

  // Create an expression of the wrench on the base multiplied by the adjoint
  // map
  const std::function<double(Vector6)> f =
      [A_T_3](const Vector6 &wrench) { return A_T_3 * wrench; };

  gtsam::Double_ tau3 = gtsam::linearExpression(f, wrench, A_T_3);

  return tau3;
}

std::vector<Chain> getComposedChains(
    std::vector<std::vector<JointSharedPtr>>& chain_joints) {
  Chain composed_fr, composed_fl, composed_rr, composed_rl;

  composed_fr = BuildChain(chain_joints[0]);
  composed_fl = BuildChain(chain_joints[1]);
  composed_rr = BuildChain(chain_joints[2]);
  composed_rl = BuildChain(chain_joints[3]);

  std::vector<Chain> composed_chains{composed_fr, composed_fl, composed_rr,
                                     composed_rl};

  return composed_chains;
}

TEST(Chain, A1QuadStaticChainGraph) {
  // This test uses chain constraints for each leg of the robot, a wrench
  // constraint on the trunk, zero angles at the joints constraints (robot
  // standing still) and minimum torque constraints.

  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

  // Get joint and composed chains for each leg
  auto chain_joints = getChainJoints(robot);
  auto composed_chains = getComposedChains(chain_joints);

  // Initialize Constraints
  EqualityConstraints constraints;

  // Set Gravity Wrench
  gtsam::Vector6 gravity;
  gravity << 0.0, 0.0, 0.0, 0.0, 0.0, -10.0;
  gtsam::Vector6_ gravity_wrench(gravity);

  // Create expression for wrench constraint on trunk
  gtsam::Vector6_ trunk_wrench_constraint = gravity_wrench;

  // Set torque tolerance
  double torqueTolerance = 1e-6;
  gtsam::Vector3 torque_tolerance;
  torque_tolerance << torqueTolerance, torqueTolerance, torqueTolerance;

  // Set Wrench tolerance
  gtsam::Vector6 wrench_tolerance;
  wrench_tolerance << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;

  //Set angle tolerance
  double angle_tolerance = 1e-30;

  for (int i=0; i < 4; ++i){
    // Get key for wrench at hip joints  on link 0 at time 0
    const gtsam::Key wrench_key = gtdynamics::WrenchKey(0, i * 3, 0);

    // create expressions for these wrenches
    gtsam::Vector6_ wrench(wrench_key);

    // add wrench to trunk constraint
    trunk_wrench_constraint += wrench;

    // Get expressions for chains on each leg
    auto expression_chain =
        composed_chains[i].ChainConstraint3(chain_joints[i], wrench_key, 0);

    // Add constraint for chain
    constraints.emplace_shared<VectorExpressionEquality<3>>(expression_chain,
                                                            torque_tolerance);

    // constraint on zero torque of contact point
    gtsam::Double_ tau = getContactConstraint(chain_joints[i]);
    constraints.emplace_shared<DoubleExpressionEquality>(tau,
                                                           torqueTolerance);

    // Hard constraint on zero angles
    for (int j = 0; j < 3; ++j) {
      const int joint_id = chain_joints[i][j]->id();
      gtsam::Double_ angle(JointAngleKey(joint_id, 0));
      constraints.emplace_shared<DoubleExpressionEquality>(angle,
                                                           angle_tolerance);
    }
  }

  // Add trunk wrench constraint to constraints
  constraints.emplace_shared<VectorExpressionEquality<6>>(trunk_wrench_constraint,
                                                          wrench_tolerance);

  // Constrain Minimum torque in actuators
  gtsam::NonlinearFactorGraph graph;
  auto cost_model = gtsam::noiseModel::Unit::Create(1);
  for (auto&& joint : robot.joints()) {
    MinTorqueFactor factor(TorqueKey(joint->id(), 0), cost_model);
    graph.add(factor);
  }

  // Create initial values.
  gtsam::Values init_values;
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&init_values, joint->id(), 0, 0.0);
    InsertTorque(&init_values, joint->id(), 0, 0.0);
  }

  gtsam::Vector6 wrench_zero;
  wrench_zero << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  for (int i=0; i < 4; ++i){
    init_values.insert(gtdynamics::WrenchKey(0, i * 3, 0), wrench_zero);
  }

  /// Solve the constraint problem with LM optimizer.
  Optimizer optimizer;
  gtsam::Values results = optimizer.optimize(graph, constraints, init_values);

  Matrix expected_wrenches(6, 4);
  expected_wrenches << -0.101417, 0.101411, -0.0928056, 0.0928114, -0.310963,
      -0.310967, 0.310963, 0.310967, -0.00503307, 0.00540063, 0.00365663,
      -0.00402418, -0.361902, -0.361892, 0.361903, 0.361891, -0.0384285,
      0.0384302, -0.034986, 0.0349843, 2.60931, 2.60931, 2.39069, 2.39069;

  for (int i=0; i < 4; ++i){
    // get hip wrench key
    const gtsam::Key wrench_key = gtdynamics::WrenchKey(0, i * 3, 0);

    // Get result wrench for hip joints
    gtsam::Vector6 result_wrench = results.at<gtsam::Vector6>(wrench_key);

    // We expect these wrenches to be close  to 2.5 N in Z direction, but not
    // exatly 2.5 since the robot is not symmetric
    gtsam::Vector6 expected_wrench = expected_wrenches.col(i);
    EXPECT(assert_equal(result_wrench, expected_wrench, 1e-1));

    // Check that the torque on the contact point is zero
    auto A_T_3 = get_joint3_screw_axis(chain_joints[i]);
    double tau3 = A_T_3 * result_wrench;
    EXPECT(assert_equal(tau3, 0.0, 1e-10));
  }

  Matrix expected_torques(12, 1);
  expected_torques << 0.115727, -0.261608, -0.168418, -0.11579, -0.261605,
      -0.168403, 0.105357, 0.238417, 0.145263, -0.10541, 0.238412, 0.145247;

  //check angles and torques
  int k = 0;
  for (auto&& joint : robot.joints()) {
    const int id = joint->id();
    EXPECT(assert_equal(results.at<double>(JointAngleKey(id)), 0.0, 1e-30));
    EXPECT(assert_equal(results.at<double>(TorqueKey(id)), expected_torques(k,0), 1e-6));
    ++k;
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
