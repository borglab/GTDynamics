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
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/Optimizer.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/FootContactConstraintSpec.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>

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
  Vector3 expected_values(-1, -0.3, 0.3);  // regression
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
  Vector3 expected_values(-1, -0.3, 0.3);  // regression
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
  JointSharedPtr j0, j1, j2;

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

  int loc = 0;
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

std::vector<Chain> getComposedChains(
    std::vector<std::vector<JointSharedPtr>>& chain_joints) {
  Chain composed_fr, composed_fl, composed_rr, composed_rl;

  composed_fl = BuildChain(chain_joints[0]);
  composed_fr = BuildChain(chain_joints[1]);
  composed_rl = BuildChain(chain_joints[2]);
  composed_rr = BuildChain(chain_joints[3]);

  std::vector<Chain> composed_chains{composed_fl, composed_fr, composed_rl,
                                     composed_rr};

  return composed_chains;
}

gtsam::Values OldGraphOneLeg() {
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

  // Get trunk with one leg, only the trunk has mass
  for (auto&& link : robot.links()) {
    if (link->id() > 3) {
      robot.removeLink(link);
    } else if (link->name().find("trunk") == std::string::npos) {
      link->setMass(0.0);
      link->setInertia(gtsam::Matrix3::Zero());
    } else {
      link->setMass(1.0);
      link->setInertia(gtsam::Matrix3::Identity());
    }
  }

  // Create Contact Point
  std::vector<LinkSharedPtr> lower_feet = {robot.link("FL_lower")};
  const Point3 contact_in_com(0, 0, -0.07);
  auto stationary =
      boost::make_shared<FootContactConstraintSpec>(lower_feet, contact_in_com);
  auto contact_points = stationary->contactPoints();

  gtsam::Vector3 gravity(0, 0, -10.0);

  OptimizerSetting opt(1e-3, 1e-3, 1e-3, 1e-1);
  DynamicsGraph graph_builder(opt, gravity);

  gtsam::Vector6 wrench_zero = gtsam::Z_6x1;

  gtsam::NonlinearFactorGraph graph;

  // Need the following to use the wrench factor (Coriolis, Generalized
  // Momentum, Gravity)
  auto bp_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 1e-30));
  graph.addPrior(PoseKey(0, 0), robot.link("trunk")->bMcom(), bp_cost_model);
  graph.addPrior(TwistKey(0, 0), wrench_zero, bp_cost_model);
  graph.addPrior(TwistAccelKey(0, 0), wrench_zero, bp_cost_model);

  // Build dynamics factors
  graph.add(graph_builder.dynamicsFactors(robot, 0, contact_points, 1.0));

  // Initialize joint kinematics/dynamics to 0.
  gtsam::Values init_vals;
  for (auto&& joint : robot.joints()) {
    int j = joint->id();
    InsertWrench(&init_vals, joint->parent()->id(), j, 0, wrench_zero);
    InsertWrench(&init_vals, joint->child()->id(), j, 0, wrench_zero);
    InsertTorque(&init_vals, j, 0, 0.0);
    InsertJointAngle(&init_vals, j, 0, 0.0);
  }
  for (auto&& link : robot.links()) {
    // graph.addPrior(PoseKey(link->id(), 0), link->bMcom(), bp_cost_model);
    InsertTwist(&init_vals, link->id(), 0, wrench_zero);
    InsertTwistAccel(&init_vals, link->id(), 0, wrench_zero);
    InsertPose(&init_vals, link->id(), 0, link->bMcom());
  }
  init_vals.insert(ContactWrenchKey(3, 0, 0), wrench_zero);

  // Constraint angles to zero
  Optimizer optimizer;
  EqualityConstraints eqs;
  auto cost_model = gtsam::noiseModel::Unit::Create(1);
  for (auto&& joint : robot.joints()) {
    const int joint_id = joint->id();
    gtsam::Double_ angle(JointAngleKey(joint_id, 0));
    eqs.emplace_shared<DoubleExpressionEquality>(angle, 1e-1);
  }

  /// Solve the constraint problem with LM optimizer.
  gtsam::Values results = optimizer.optimize(graph, eqs, init_vals);

  return results;
}

gtsam::Values NewGraphOneLeg() {
  // This Graph uses chain constraints for one leg of the a1 robot, a wrench
  // constraint on the trunk and zero angles at the joints constraints (robot
  // standing still)

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

  // Set tolerances
  double angle_tolerance = 1e-1;
  double dynamicsTolerance = 1e-3;
  gtsam::Vector3 contact_tolerance = Vector3::Ones() * dynamicsTolerance;
  gtsam::Vector3 torque_tolerance = Vector3::Ones() * dynamicsTolerance;
  gtsam::Vector6 wrench_tolerance = Vector6::Ones() * dynamicsTolerance;

  // Get key for wrench at hip joint with id 0
  const gtsam::Key wrench_key_0_T = gtdynamics::WrenchKey(0, 0, 0);

  // create expression for the wrench
  gtsam::Vector6_ wrench_0_T(wrench_key_0_T);

  // add wrench to trunk constraint
  trunk_wrench_constraint += wrench_0_T;

  // Add trunk wrench constraint to constraints
  constraints.emplace_shared<VectorExpressionEquality<6>>(
      trunk_wrench_constraint, wrench_tolerance);

  // Get expression for chain on the leg
  auto expression_chain =
      composed_chains[0].ChainConstraint3(chain_joints[0], wrench_key_0_T, 0);

  // Add constraint for chain
  constraints.emplace_shared<VectorExpressionEquality<3>>(expression_chain,
                                                          torque_tolerance);

  // constraint on zero angles
  for (int j = 0; j < 3; ++j) {
    const int joint_id = chain_joints[0][j]->id();
    gtsam::Double_ angle(JointAngleKey(joint_id, 0));
    constraints.emplace_shared<DoubleExpressionEquality>(angle,
                                                         angle_tolerance);
  }

  // For contact constraint, calculate the expression for the wrench of joint 2
  // on the lower link
  gtsam::Vector6_ wrench_0_H =
      (-1) * chain_joints[0][0]->childAdjointWrench(wrench_0_T, 0);
  gtsam::Vector6_ wrench_1_U =
      (-1) * chain_joints[0][1]->childAdjointWrench(wrench_0_H, 0);
  gtsam::Vector6_ wrench_2_L =
      (-1) * chain_joints[0][2]->childAdjointWrench(wrench_1_U, 0);

  // Create contact constraint
  Point3 contact_in_com(0.0, 0.0, -0.07);
  gtsam::Vector3_ contact_constraint = ContactDynamicsMomentConstraint(
      wrench_2_L, gtsam::Pose3(gtsam::Rot3(), (-1) * contact_in_com));
  constraints.emplace_shared<VectorExpressionEquality<3>>(contact_constraint,
                                                          contact_tolerance);

  // Create initial values.
  gtsam::Values init_values;
  for (auto&& joint : robot.joints()) {
    if (joint->id() > 2) continue;
    InsertJointAngle(&init_values, joint->id(), 0, 0.0);
    InsertTorque(&init_values, joint->id(), 0, 0.0);
  }
  gtsam::Vector6 wrench_zero = gtsam::Z_6x1;
  init_values.insert(gtdynamics::WrenchKey(0, 0, 0), wrench_zero);

  /// Solve the constraint problem with LM optimizer.
  gtsam::NonlinearFactorGraph graph;
  Optimizer optimizer;
  gtsam::Values results = optimizer.optimize(graph, constraints, init_values);
  return results;
}

TEST(Chain, oneLegCompareGraphsA1) {
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

  // Get joint and composed chains for each leg
  auto chain_joints = getChainJoints(robot);

  // calculate both ways
  gtsam::Values new_graph_results = NewGraphOneLeg();
  gtsam::Values old_graph_results = OldGraphOneLeg();

  // Get torque and angle results and compare
  for ( int i = 0; i < 3 ; ++i) {
    double new_torque = new_graph_results.at<double>(gtdynamics::TorqueKey(i));
    double old_torque = old_graph_results.at<double>(gtdynamics::TorqueKey(i));
    double new_angle =
      new_graph_results.at<double>(gtdynamics::JointAngleKey(i));
    double old_angle =
      old_graph_results.at<double>(gtdynamics::JointAngleKey(i));
    EXPECT(assert_equal(new_torque, old_torque, 1e-4));
    EXPECT(assert_equal(new_angle, old_angle, 1e-4));
  }

  // Get new angles
  double new_angle0 =
      new_graph_results.at<double>(gtdynamics::JointAngleKey(0));
  double new_angle1 =
      new_graph_results.at<double>(gtdynamics::JointAngleKey(1));
  double new_angle2 =
      new_graph_results.at<double>(gtdynamics::JointAngleKey(2));

  // Get wrench keys
  const gtsam::Key wrench_key_0_T = gtdynamics::WrenchKey(0, 0, 0);
  const gtsam::Key contact_wrench_key = gtdynamics::ContactWrenchKey(3, 0, 0);

  // Get result wrench for hip joints and compare
  gtsam::Vector6 wrench_new_0_T =
      new_graph_results.at<gtsam::Vector6>(wrench_key_0_T);
  gtsam::Vector6 wrench_old_0_T =
      old_graph_results.at<gtsam::Vector6>(wrench_key_0_T);
  EXPECT(assert_equal(wrench_new_0_T, wrench_old_0_T, 1e-4));

  // Get result wrench for lower link and compare
  Pose3 trunkThip = chain_joints[0][0]->parentTchild(new_angle0);
  Pose3 hipTupper = chain_joints[0][1]->parentTchild(new_angle1);
  Pose3 upperTlower = chain_joints[0][2]->parentTchild(new_angle2);
  Pose3 trunkTlower = trunkThip * hipTupper * upperTlower;
  gtsam::Vector6 wrench_new_3_L =
      wrench_new_0_T.transpose() * trunkTlower.AdjointMap();
  gtsam::Vector6 wrench_old_3_L =
      old_graph_results.at<gtsam::Vector6>(contact_wrench_key);
  EXPECT(assert_equal(wrench_new_3_L, wrench_old_3_L, 1e-3));

  // calculate wrench on ground and compare
  Point3 contact_in_com(0.0, 0.0, -0.07);
  Pose3 M_L_G = Pose3(Rot3(), contact_in_com);
  gtsam::Vector6 wrench_new_3_G =
      wrench_new_3_L.transpose() * M_L_G.AdjointMap();
  gtsam::Vector6 wrench_old_3_G =
      wrench_old_3_L.transpose() * M_L_G.AdjointMap();
  EXPECT(assert_equal(wrench_new_3_G, wrench_old_3_G, 1e-3));

  // check torque zero on ground wrench
  gtsam::Matrix36 H_contact_wrench;
  H_contact_wrench << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
  gtsam::Vector3 zero_torque{0.0, 0.0, 0.0};
  gtsam::Vector3 contact_torque_new = H_contact_wrench * wrench_new_3_G;
  gtsam::Vector3 contact_torque_old = H_contact_wrench * wrench_old_3_G;
  EXPECT(assert_equal(contact_torque_new, zero_torque, 1e-4));
  EXPECT(assert_equal(contact_torque_old, zero_torque, 1e-4));
}

gtsam::Values OldGraphFourLegs() {
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

  // set masses and inertias
  for (auto&& link : robot.links()) {
    if (link->name().find("trunk") == std::string::npos) {
      link->setMass(0.0);
      link->setInertia(gtsam::Matrix3::Zero());
    }
  }

  // Create Contact Point
  std::vector<LinkSharedPtr> lower_feet = {robot.link("FL_lower"), robot.link("FR_lower"), robot.link("RL_lower"), robot.link("RR_lower")};
  const Point3 contact_in_com(0, 0, -0.07);
  auto stationary =
      boost::make_shared<FootContactConstraintSpec>(lower_feet, contact_in_com);
  auto contact_points = stationary->contactPoints();

  gtsam::Vector3 gravity(0, 0, -10.0);

  OptimizerSetting opt(1e-3, 1e-3, 1e-3, 1e-2);
  DynamicsGraph graph_builder(opt, gravity);

  gtsam::Vector6 wrench_zero = gtsam::Z_6x1;

  gtsam::NonlinearFactorGraph graph;

  // Need the following to use the wrench factor (Coriolis, Generalized
  // Momentum, Gravity)
  auto bp_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, 1e-30));
  graph.addPrior(PoseKey(0, 0), robot.link("trunk")->bMcom(), bp_cost_model);
  graph.addPrior(TwistKey(0, 0), wrench_zero, bp_cost_model);
  graph.addPrior(TwistAccelKey(0, 0), wrench_zero, bp_cost_model);

  // Build dynamics factors
  graph.add(graph_builder.dynamicsFactors(robot, 0, contact_points, 1.0));

  // Initialize joint kinematics/dynamics to 0.
  gtsam::Values init_vals;
  for (auto&& joint : robot.joints()) {
    const int j = joint->id(); 
    InsertWrench(&init_vals, joint->parent()->id(), j, 0, wrench_zero);
    InsertWrench(&init_vals, joint->child()->id(), j, 0, wrench_zero);
    InsertTorque(&init_vals, j, 0, 0.0);
    InsertJointAngle(&init_vals, j, 0, 0.0);
  }
  for (auto&& link : robot.links()) {
    InsertTwist(&init_vals, link->id(), 0, wrench_zero);
    InsertTwistAccel(&init_vals, link->id(), 0, wrench_zero);
    InsertPose(&init_vals, link->id(), 0, link->bMcom());
  }
  init_vals.insert(ContactWrenchKey(3, 0, 0), wrench_zero);
  init_vals.insert(ContactWrenchKey(6, 0, 0), wrench_zero);
  init_vals.insert(ContactWrenchKey(9, 0, 0), wrench_zero);
  init_vals.insert(ContactWrenchKey(12, 0, 0), wrench_zero);

  // Constraint angles to zero
  OptimizationParameters params;
  auto optimizer = Optimizer(params);
  EqualityConstraints constraints;
  auto cost_model = gtsam::noiseModel::Unit::Create(1);
  for (auto&& joint : robot.joints()) {
    const int joint_id = joint->id();
    gtsam::Double_ angle(JointAngleKey(joint_id, 0));
    gtsam::Double_ tor(TorqueKey(joint_id, 0));
    constraints.emplace_shared<DoubleExpressionEquality>(angle, 1e-2);
    if ((joint_id == 2) || (joint_id == 5) || (joint_id == 8) || (joint_id == 11)) continue;
    constraints.emplace_shared<DoubleExpressionEquality>(tor, 1e-1);
  }

  /// Solve the constraint problem with LM optimizer.
  gtsam::Values results = optimizer.optimize(graph, constraints, init_vals);

  return results;
}

gtsam::Values NewGraphFourLegs() {
  // This Graph uses chain constraints for four legs of the a1 robot, a wrench
  // constraint on the trunk and zero angles and torques at the joints constraints (robot
  // standing still)

  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

  // Get joint and composed chains for each leg
  auto chain_joints = getChainJoints(robot);
  auto composed_chains = getComposedChains(chain_joints);

  // Initialize Constraints
  EqualityConstraints constraints;

  // Set Gravity Wrench
  gtsam::Vector6 gravity;
  gravity << 0.0, 0.0, 0.0, 0.0, 0.0, -47.14;
  gtsam::Vector6_ gravity_wrench(gravity);

  // Create expression for wrench constraint on trunk
  gtsam::Vector6_ trunk_wrench_constraint = gravity_wrench;

  // Set tolerances
  double angle_tolerance = 1e-2;
  double dynamicsTolerance = 1e-3;
  gtsam::Vector3 contact_tolerance = Vector3::Ones() * dynamicsTolerance;
  gtsam::Vector3 torque_tolerance = Vector3::Ones() * dynamicsTolerance;
  gtsam::Vector6 wrench_tolerance = Vector6::Ones() * dynamicsTolerance;

  for (int i = 0 ; i < 4 ; ++i) {
    // Get key for wrench at hip joint with id 0
    const gtsam::Key wrench_key_3i_T = gtdynamics::WrenchKey(0, 3*i, 0);

    // create expression for the wrench
    gtsam::Vector6_ wrench_3i_T(wrench_key_3i_T);

    // add wrench to trunk constraint
    trunk_wrench_constraint += wrench_3i_T;

    // Get expression for chain on the leg
    auto expression_chain =
        composed_chains[i].ChainConstraint3(chain_joints[i], wrench_key_3i_T, 0);

    // Add constraint for chain
    constraints.emplace_shared<VectorExpressionEquality<3>>(expression_chain,
                                                            torque_tolerance);

    // constraint on zero angles
    auto cost_model = gtsam::noiseModel::Unit::Create(1);
    for (int j = 0; j < 3; ++j) {
      const int joint_id = chain_joints[i][j]->id();
      gtsam::Double_ angle(JointAngleKey(joint_id, 0));
      constraints.emplace_shared<DoubleExpressionEquality>(angle,
                                                          angle_tolerance);
      gtsam::Double_ tor(TorqueKey(joint_id, 0));
      if (j<2)
      constraints.emplace_shared<DoubleExpressionEquality>(tor,
                                                          angle_tolerance*10);
    }

    // For contact constraint, calculate the expression for the wrench of joint 2
    // on the lower link
    gtsam::Vector6_ wrench_0_H =
        (-1) * chain_joints[i][0]->childAdjointWrench(wrench_3i_T, 0);
    gtsam::Vector6_ wrench_1_U =
        (-1) * chain_joints[i][1]->childAdjointWrench(wrench_0_H, 0);
    gtsam::Vector6_ wrench_2_L =
        (-1) * chain_joints[i][2]->childAdjointWrench(wrench_1_U, 0);

    // Create contact constraint
    Point3 contact_in_com(0.0, 0.0, -0.07);
    gtsam::Vector3_ contact_constraint = ContactDynamicsMomentConstraint(
        wrench_2_L, gtsam::Pose3(gtsam::Rot3(), (-1) * contact_in_com));
    constraints.emplace_shared<VectorExpressionEquality<3>>(contact_constraint,
                                                           contact_tolerance);
  }

  // Add trunk wrench constraint to constraints
  constraints.emplace_shared<VectorExpressionEquality<6>>(
      trunk_wrench_constraint, wrench_tolerance);

  // Create initial values.
  gtsam::Values init_values;
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&init_values, joint->id(), 0, 0.0);
    InsertTorque(&init_values, joint->id(), 0, 0.0);
  }
  gtsam::Vector6 wrench_zero = gtsam::Z_6x1;
  for (int i = 0 ; i < 4; ++i) { 
    init_values.insert(gtdynamics::WrenchKey(0, 3*i, 0), wrench_zero);
  }

  /// Solve the constraint problem with LM optimizer.
  gtsam::NonlinearFactorGraph graph;
  OptimizationParameters params;
  auto optimizer = Optimizer(params);
  gtsam::Values results = optimizer.optimize(graph, constraints, init_values);
  return results;
}

TEST(Chain, fourLegsCompareGraphsA1) {
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

  // Get joint and composed chains for each leg
  auto chain_joints = getChainJoints(robot);

  // calculate both ways
  gtsam::Values new_graph_results = NewGraphFourLegs();
  gtsam::Values old_graph_results = OldGraphFourLegs();

  // Get torque and angle results and compare
  for ( int i = 0; i < 12 ; ++i) {
    double new_torque = new_graph_results.at<double>(gtdynamics::TorqueKey(i));
    double old_torque = old_graph_results.at<double>(gtdynamics::TorqueKey(i));
    double new_angle =
      new_graph_results.at<double>(gtdynamics::JointAngleKey(i));
    double old_angle =
      old_graph_results.at<double>(gtdynamics::JointAngleKey(i));
    EXPECT(assert_equal(new_torque, old_torque, 1e-3));
    EXPECT(assert_equal(new_angle, old_angle, 1e-3));
  }

  int hip_joint_id = 0;
  int hip_link_id = 1;
  for (int i = 0; i < 4; ++i) {
    // Get new angles
    double new_angle0 =
        new_graph_results.at<double>(gtdynamics::JointAngleKey(hip_joint_id));
    double new_angle1 =
        new_graph_results.at<double>(gtdynamics::JointAngleKey(hip_joint_id+1));
    double new_angle2 =
        new_graph_results.at<double>(gtdynamics::JointAngleKey(hip_joint_id+2));

    // Get wrench keys
    const gtsam::Key wrench_key_0_T = gtdynamics::WrenchKey(0, hip_joint_id, 0);
    const gtsam::Key contact_wrench_key = gtdynamics::ContactWrenchKey(hip_link_id+2, 0, 0);

    // Get result wrench for hip joints and compare
    gtsam::Vector6 wrench_new_0_T =
        new_graph_results.at<gtsam::Vector6>(wrench_key_0_T);
    gtsam::Vector6 wrench_old_0_T =
        old_graph_results.at<gtsam::Vector6>(wrench_key_0_T);
    EXPECT(assert_equal(wrench_new_0_T, wrench_old_0_T, 1e-3));

    // Get result wrench for lower link and compare
    Pose3 trunkThip = chain_joints[i][0]->parentTchild(new_angle0);
    Pose3 hipTupper = chain_joints[i][1]->parentTchild(new_angle1);
    Pose3 upperTlower = chain_joints[i][2]->parentTchild(new_angle2);
    Pose3 trunkTlower = trunkThip * hipTupper * upperTlower;
    gtsam::Vector6 wrench_new_3_L =
        wrench_new_0_T.transpose() * trunkTlower.AdjointMap();
    gtsam::Vector6 wrench_old_3_L =
        old_graph_results.at<gtsam::Vector6>(contact_wrench_key);
    EXPECT(assert_equal(wrench_new_3_L, wrench_old_3_L, 1e-3));

    // calculate wrench on ground and compare
    Point3 contact_in_com(0.0, 0.0, -0.07);
    Pose3 M_L_G = Pose3(Rot3(), contact_in_com);
    gtsam::Vector6 wrench_new_3_G =
        wrench_new_3_L.transpose() * M_L_G.AdjointMap();
    gtsam::Vector6 wrench_old_3_G =
        wrench_old_3_L.transpose() * M_L_G.AdjointMap();
    EXPECT(assert_equal(wrench_new_3_G, wrench_old_3_G, 1e-3));

    // check torque zero on ground wrench
    gtsam::Matrix36 H_contact_wrench;
    H_contact_wrench << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    gtsam::Vector3 zero_torque{0.0, 0.0, 0.0};
    gtsam::Vector3 contact_torque_new = H_contact_wrench * wrench_new_3_G;
    gtsam::Vector3 contact_torque_old = H_contact_wrench * wrench_old_3_G;
    EXPECT(assert_equal(contact_torque_new, zero_torque, 1e-4));
    EXPECT(assert_equal(contact_torque_old, zero_torque, 1e-4));
    hip_joint_id +=3;
    hip_link_id +=3;
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
