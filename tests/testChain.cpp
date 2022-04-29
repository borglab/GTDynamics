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

TEST(Chain, A1QuadOneLegCompare) {
  // This test checks equality between torque calculation with and without chains.
  // This assumes one static leg of the a1 quadruped with zero mass for the links besides the trunk.

  auto robot =
  CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

  // create arranged vector of joints of FR leg
  std::vector<JointSharedPtr> FR(3);
  for (auto&& joint : robot.joints()) {
    if (joint->name().find("FR") < 100) {
      if (joint->name().find("lower") < 100) {
        FR[0] = joint;
      }
      if (joint->name().find("upper") < 100) {
        FR[1] = joint;
      }
      if (joint->name().find("hip") < 100) {
        FR[2] = joint;
      }
    }
  }

  JointSharedPtr j1 = FR[0];
  JointSharedPtr j2 = FR[1];
  JointSharedPtr j3 = FR[2];

  // Calculate all relevant relative poses.
  // B frame is at joint 1, C frame is at joint 2, T frame is at Trunk CoM, J3 frame is at joint 3.
  Pose3 M_B_C = j1->jMp() * (j2->jMc().inverse());
  Pose3 M_C_B = M_B_C.inverse();
  Pose3 M_C_T = j2->jMp() * j3->jMc().inverse() * j3->jMp();
  Pose3 M_T_C = M_C_T.inverse();
  Pose3 M_B_T = M_B_C * M_C_T;
  Pose3 M_T_B = M_B_T.inverse();
  Pose3 M_C_J3 = j2->jMp() * j3->jMc().inverse();
  Pose3 M_J3_C = M_C_J3.inverse();
  Pose3 M_J3_T = j3->jMp();
  Pose3 M_T_J3 = M_J3_T.inverse();
  Pose3 M_B_Bcom = j1->jMp();
  Pose3 M_C_Ccom = j2->jMp();

  // Set Gravity Wrench
  Matrix gravity(1, 6);
  gravity << 0.0, 0.0, 0.0, 0.0, 0.0, -10.0;
  
  // Calculate all wrenches and torques without chains.
  // joint pScrewAxis() method is in parent CoM frame so we need to adjoint to joint frame for torque calculation
  Matrix F_3_T = -gravity;
  Matrix F_3_J3 =  F_3_T * M_T_J3.AdjointMap();
  Matrix tau3 = F_3_J3 * M_J3_T.AdjointMap() * j3->pScrewAxis();
  EXPECT(assert_equal( tau3(0,0), -0.491855, 1e-6));

  Matrix F_2_C = F_3_T * M_T_C.AdjointMap();
  Matrix tau2 = F_2_C * M_C_Ccom.AdjointMap() * j2->pScrewAxis();
  EXPECT(assert_equal( tau2(0,0), -1.70272, 1e-5));
  
  Matrix F_1_B = F_2_C * M_C_B.AdjointMap(); 
  Matrix tau1 = F_1_B * M_B_Bcom.AdjointMap() * j1->pScrewAxis();
  EXPECT(assert_equal( tau1(0,0), -1.70272, 1e-5));
  
  // Calculate all wrenches and torques with chains.
  Chain chain1(M_B_C, (M_C_B * M_B_Bcom).AdjointMap()*j1->pScrewAxis());
  Chain chain2(M_C_J3, (M_J3_C * M_C_Ccom).AdjointMap()*j2->pScrewAxis());
  Chain chain3(M_J3_T, j3->pScrewAxis());

  std::vector<Chain> chains{chain1, chain2, chain3};

  // Compose Chains
  Chain composed = Chain::compose(chains);

  // test for same values
  Matrix torques = F_3_T * composed.axes();
  EXPECT(assert_equal( torques(0,2), -0.491855, 1e-5));
  EXPECT(assert_equal( torques(0,1), -1.70272, 1e-5));
  EXPECT(assert_equal( torques(0,0), -1.70272, 1e-5));
}

std::vector<std::vector<JointSharedPtr>> getChainJoints(const Robot& robot) {

  std::vector<JointSharedPtr> FR(3), FL(3), RR(3), RL(3);

  int loc;
  for (auto&& joint : robot.joints()) {
    if (joint->name().find("lower") < 100) {
      loc = 0;
    } 
    if (joint->name().find("upper") < 100) {
      loc = 1;
    } 
    if (joint->name().find("hip") < 100) {
      loc = 2;
    } 
    if (joint->name().find("FR") < 100) {
      FR[loc] = joint;
    }
    if (joint->name().find("FL") < 100) {
      FL[loc] = joint;
    }
    if (joint->name().find("RR") < 100) {
      RR[loc] = joint;
    }
    if (joint->name().find("RL") < 100) {
      RL[loc] = joint;
    }
  }
  
  //std::cout << RL[0] << RL[1] << RL[2] << std::endl;
  std::vector<std::vector<JointSharedPtr>> chain_joints{FR,FL,RR,RL};
  
  return chain_joints;
}

Chain BuildChain(std::vector<JointSharedPtr> &joints){

  auto j1 = joints[0];
  auto j2 = joints[1];
  auto j3 = joints[2];

  Pose3 M_B_C = j1->jMp() * (j2->jMc().inverse());
  Pose3 M_C_B = M_B_C.inverse();
  Pose3 M_C_J3 = j2->jMp() * j3->jMc().inverse();
  Pose3 M_J3_C = M_C_J3.inverse();
  Pose3 M_J3_T = j3->jMp();
  Pose3 M_T_J3 = M_J3_T.inverse();
  Pose3 M_B_Bcom = j1->jMp();
  Pose3 M_C_Ccom = j2->jMp();

  Chain chain1(M_B_C, (M_C_B * M_B_Bcom).AdjointMap()*j1->pScrewAxis());
  Chain chain2(M_C_J3, (M_J3_C * M_C_Ccom).AdjointMap()*j2->pScrewAxis());
  Chain chain3(M_J3_T, j3->pScrewAxis());

  std::vector<Chain> chains{chain1, chain2, chain3};

  Chain composed = Chain::compose(chains);

  return composed;
}

std::vector<Chain> getComposedChains(std::vector<std::vector<JointSharedPtr>> &chain_joints) {
  
  Chain composed_fr, composed_fl, composed_rr, composed_rl;

  composed_fr = BuildChain(chain_joints[0]);
  composed_fl = BuildChain(chain_joints[1]);
  composed_rr = BuildChain(chain_joints[2]);
  composed_rl = BuildChain(chain_joints[3]);

  std::vector<Chain> composed_chains{composed_fr, composed_fl, composed_rr, composed_rl};

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

  // Hard constraint on zero angles
  double angle_tolerance = 1e-30;
  for (int i = 0; i < 4 ; ++i){
    for (int j = 0; j < 3; ++j) {
      gtsam::Double_ angle(JointAngleKey(chain_joints[i][j]->id(), 0));
      constraints.emplace_shared<DoubleExpressionEquality>(angle, angle_tolerance);
    }
  }

  // Get key for wrench at hip joints  on link 0 at time 0
  const gtsam::Key wrench_key_fr = gtdynamics::WrenchKey(0, 3, 0); 
  const gtsam::Key wrench_key_fl = gtdynamics::WrenchKey(0, 0, 0);
  const gtsam::Key wrench_key_rr = gtdynamics::WrenchKey(0, 9, 0);
  const gtsam::Key wrench_key_rl = gtdynamics::WrenchKey(0, 6, 0);    

  // create expressions for these wrenches
  gtsam::Vector6_ wrench_fr(wrench_key_fr);
  gtsam::Vector6_ wrench_fl(wrench_key_fl);
  gtsam::Vector6_ wrench_rr(wrench_key_rr);
  gtsam::Vector6_ wrench_rl(wrench_key_rl);

  // Set Gravity Wrench
  gtsam::Vector6 gravity;
  gravity << 0.0, 0.0, 0.0, 0.0, 0.0, -10.0;
  gtsam::Vector6_ gravity_wrench(gravity);

  // Create expression for wrench constraint on trunk
  auto expression = wrench_fr + wrench_fl + wrench_rr + wrench_rl + gravity_wrench;
  gtsam::Vector6 wrench_tolerance;
  wrench_tolerance << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
  constraints.emplace_shared<VectorExpressionEquality<6>>(expression, wrench_tolerance);

  // Get expressions for chains on each leg
  auto expression_fr = composed_chains[0].ChainConstraint3(chain_joints[0], wrench_key_fr, 0);
  auto expression_fl = composed_chains[1].ChainConstraint3(chain_joints[1], wrench_key_fl, 0);
  auto expression_rr = composed_chains[2].ChainConstraint3(chain_joints[2], wrench_key_rr, 0);
  auto expression_rl = composed_chains[3].ChainConstraint3(chain_joints[3], wrench_key_rl, 0);

  gtsam::Vector3 torque_tolerance;
  torque_tolerance << 1e-6, 1e-6, 1e-6;

  constraints.emplace_shared<VectorExpressionEquality<3>>(expression_fr, torque_tolerance);
  constraints.emplace_shared<VectorExpressionEquality<3>>(expression_fl, torque_tolerance);
  constraints.emplace_shared<VectorExpressionEquality<3>>(expression_rr, torque_tolerance);
  constraints.emplace_shared<VectorExpressionEquality<3>>(expression_rl, torque_tolerance);

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

  gtsam::Vector6 zero_wrench;
  zero_wrench << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  init_values.insert(wrench_key_fr, zero_wrench);
  init_values.insert(wrench_key_fl, zero_wrench);
  init_values.insert(wrench_key_rr, zero_wrench);
  init_values.insert(wrench_key_rl, zero_wrench);
  
  /// Solve the constraint problem with LM optimizer.
  Optimizer optimizer;
  gtsam::Values results = optimizer.optimize(graph, constraints, init_values);

  gtsam::Vector6 result_wrench_fr = results.at<gtsam::Vector6>(wrench_key_fr);
  gtsam::Vector6 result_wrench_fl = results.at<gtsam::Vector6>(wrench_key_fl);
  gtsam::Vector6 result_wrench_rr = results.at<gtsam::Vector6>(wrench_key_rr);
  gtsam::Vector6 result_wrench_rl = results.at<gtsam::Vector6>(wrench_key_rl);

  // We expect these wrenches to be close  to 2.5 N in Z direction.
  // The robot is not symmetric so we don't get exactly 2.5 N.
  EXPECT(assert_equal( result_wrench_fr(5), 2.51413, 1e-5));
  EXPECT(assert_equal( result_wrench_fl(5), 2.51489, 1e-5));
  EXPECT(assert_equal( result_wrench_rr(5), 2.48511, 1e-5));
  EXPECT(assert_equal( result_wrench_rl(5), 2.48586, 1e-5));

  for (auto&& joint : robot.joints()) {
    const int id = joint->id();
    EXPECT(assert_equal( results.at<double>(JointAngleKey(id)), 0.0, 1e-30));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
