/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactStateEstimator.cpp
 * @brief Tests for contact based state estimation.
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <utility>
#include <vector>

#include "gtdynamics/control/ContactStateEstimator.h"
#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/factors/ContactKinematicsPoseFactor.h"
#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/values.h"

using namespace std;
using namespace gtdynamics;
using namespace gtsam;
using namespace gtdynamics::internal;

using gtsam::assert_equal;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

// Load the A1 robot
string robot_path = URDF_PATH + "/a1.urdf";
Robot robot = CreateRobotFromFile(robot_path);

const string base_link_name = "trunk";  //"pelvis";

// Foot contact links are FL_lower, FR_lower, RL_lower and RR_lower
// trunk_link -> hip_joint -> hip_link -> upper_joint -> upper_link ->
// lower_joint -> lower_link
vector<string> lower_links = {"FR_lower", "FL_lower", "RR_lower", "RL_lower"};

vector<function<gtsam::Key(uint64_t)>> foot_key_functions = {FR, FL, RR, RL};

Values initializeRobotJoints(const Robot& robot) {
  map<string, double> joint_angles_map = {
      {"FR_hip_joint", 0}, {"FR_upper_joint", 0}, {"FR_lower_joint", 0},
      {"FL_hip_joint", 0}, {"FL_upper_joint", 0}, {"FL_lower_joint", 0},
      {"RR_hip_joint", 0}, {"RR_upper_joint", 0}, {"RR_lower_joint", 0},
      {"RL_hip_joint", 0}, {"RL_upper_joint", 0}, {"RL_lower_joint", 0}};

  Values joint_angles_0;
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&joint_angles_0, joint->id(), 0, 0.0);
  }
  return joint_angles_0;
}

// Test one invocation of the estimator
TEST(ContactStateEstimator, Invoke) {
  ContactStateEstimator estimator(robot, lower_links, base_link_name, 2,
                                  foot_key_functions);

  CsvData csvData =
      readCsvData("/home/varun/borglab/motion_imitation/a1_data.csv");

  vector<string> header = csvData.first;
  vector<vector<double>> data = csvData.second;

  Values joint_angles_0 = getJointAngles(robot, header, data, 0);

  // Check the CoM of the fused calf and toe in the FR lower link frame
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0.004727, 0, -0.131975)),
                      robot.link("FR_lower")->lTcom(), 1e-5));

  // This is the pose of the contact point in the calf link frame after SDF
  // merges the calf and toe links connected by a fixed joint.
  Pose3 lTc_lower(Rot3(), Point3(0, 0, -0.22));

  // Create initial estimate given joint angles, assume all feet on the
  // ground. The constructor below will create the continuous and the discrete
  // states for the initial timestep 0. X is forward, Y is to the left, Z is
  // up.
  ContactStateEstimate estimate_0 =
      estimator.initialize(joint_angles_0, "RR_lower");

  // Set of base and leg states at initial time 0 for the fixed-lag smoother.
  Values expected_continuous_states_0;
  Pose3 p = getPose(data, 0) * robot.link(base_link_name)->lTcom();
  NavState state_0(p.rotation(), p.translation(), Point3::Zero());
  expected_continuous_states_0.insert(X(0), state_0);
  expected_continuous_states_0.insert(
      FR(0), Pose3(Rot3::Quaternion(8.96462182e-01, -1.93800697e-04,
                                    -4.43119974e-01, 8.13348341e-05),
                   Point3(0.03447161, -0.12761532, 0.1427124)) *
                 robot.link("FR_lower")->lTcom());

  expected_continuous_states_0.insert(
      FL(0), Pose3(Rot3::Quaternion(8.96044111e-01, -1.92925155e-04,
                                    -4.43964760e-01, 8.11309202e-05),
                   Point3(0.03461184, 0.13648479, 0.14241265)) *
                 robot.link("FL_lower")->lTcom());
  expected_continuous_states_0.insert(
      RR(0), Pose3(Rot3::Quaternion(8.81532617e-01, -2.05365878e-04,
                                    -4.72122859e-01, 9.51115317e-05),
                   Point3(-0.32569342, -0.12761457, 0.13209494)) *
                 robot.link("RR_lower")->lTcom());
  expected_continuous_states_0.insert(
      RL(0), Pose3(Rot3::Quaternion(8.81441187e-01, -2.06621107e-04,
                                    -4.72293535e-01, 9.58185374e-05),
                   Point3(-0.32574409, 0.13648506, 0.13203046)) *
                 robot.link("RL_lower")->lTcom());

  DiscreteFactor::Values expected_contact_sequence_0;
  expected_contact_sequence_0[C(0)] = 15;

  // Compare expected sequence with the actual sequence.
  EXPECT(assert_equal(expected_continuous_states_0.at<NavState>(X(0)),
                      estimate_0.continuousStates().at<NavState>(X(0)), 1e-2));
  EXPECT(assert_equal(expected_continuous_states_0,
                      estimate_0.continuousStates(), 1e-1));
  EXPECT(
      assert_equal(expected_contact_sequence_0, estimate_0.contactSequence()));

  // Measurement object containing IMU measurement between state 0 and 1, and
  // joint angles at timestep 1.
  auto imu_params = imuParams();
  imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias
  PreintegratedCombinedMeasurements imu_measurements(imu_params,
                                                     prior_imu_bias);
  size_t time_idx = 0;  // The time index
  double frequency = 500;
  double dt = 1 / frequency;
  for (size_t t = time_idx; t < time_idx + frequency; t++) {
    Vector3 measuredOmega(data[t][10], data[t][11], data[t][12]);
    Vector3 measuredAcc(data[t][13], data[t][14], data[t][15]);

    imu_measurements.integrateMeasurement(measuredAcc, measuredOmega, dt);
  }
  time_idx += frequency;

  Values joint_angles_1 = getJointAngles(robot, header, data, time_idx);
  ContactStateEstimator::Measurement measurements_01(imu_measurements,
                                                     joint_angles_1);

  // Invoke the fixed lag smoother
  // ContactStateEstimator::Estimate estimate_1 =
  //     estimator.run(estimate_0, measurements_01, 1);

  // NavState expected_state = getState(data, time_idx);
  // EXPECT(assert_equal(expected_state,
  //                     estimate_1.continuousStates().at<NavState>(X(1)),
  //                     1e-2));

  // // State sequence for fixed lag smoothing from previous invocation
  // Values expected_continuous_states_0_to_1;

  // // Sequence of discrete contact states
  DiscreteFactor::Values expected_contact_sequence;
  // expected_contact_sequence[C(0)] =
  //     15;  // All feet on the ground at initial time (1111)
  // expected_contact_sequence[C(1)] = 11;  // Foot 2 lifted off the ground
  // (1011)

  // // Compare expected sequence with the actual sequence.
  // EXPECT(assert_equal(expected_continuous_states_0_to_1,
  //                     estimate_1.continuousStates()));
  // EXPECT(assert_equal(expected_contact_sequence,
  // estimate_1.contactSequence()));
}

// Test forward kinematics
TEST(ContactStateEstimator, JointGraph) {
  ContactStateEstimator estimator(robot, lower_links, base_link_name, 2,
                                  foot_key_functions);

  Values joint_angles_0 = initializeRobotJoints(robot);

  size_t time = 0;
  auto graph = estimator.createJointsGraph(joint_angles_0, time);

  auto base_link_key =
      gtdynamics::internal::PoseKey(robot.link(base_link_name)->id(), time);

  // Assume the base link is at the origin
  graph.addPrior<Pose3>(base_link_key, Pose3());

  // Creating initial values below.
  Values initial;

  // Add all the initial link frame poses.
  for (auto&& link : robot.links()) {
    initial.insert<Pose3>(PoseKey(link->id(), time),
                          link->wTl() * link->lTcom());
  }

  // Add the initial values for the joint angles.
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&initial, joint->id(), time,
                     JointAngle(joint_angles_0, joint->id(), time));
  }

  // Create the optimizer and optimizer
  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Set of base and leg states at initial time 0 for the fixed-lag smoother.
  Pose3 expected_base_pose(Rot3(), Point3(0, 0, 0.0));
  Pose3 expected_FR_lower_pose =
      robot.link("FR_lower")->wTl() * robot.link("FR_lower")->lTcom();

  // Compare expected base pose to optimized one.
  EXPECT(
      assert_equal(expected_base_pose, result.at<Pose3>(base_link_key), 1e-2));
  EXPECT(
      assert_equal(expected_FR_lower_pose, result.at<Pose3>(FR(time)), 1e-1));
}

// Test zero joint angles state
TEST(ContactStateEstimator, ZeroAngles) {
  ContactStateEstimator estimator(robot, lower_links, base_link_name, 2,
                                  foot_key_functions);

  Values joint_angles_0 = initializeRobotJoints(robot);

  // Create initial estimate given joint angles, assume all feet on the
  // ground. The constructor below will create the continuous and the discrete
  // states for the initial timestep 0. X is forward, Y is to the left, Z is
  // up.
  ContactStateEstimate estimate_0 =
      estimator.initialize(joint_angles_0, "RR_lower");

  // Set of base and leg states at initial time 0 for the fixed-lag smoother.
  Values expected_continuous_states_0;
  Pose3 p =
      Pose3(Rot3(), Point3(0, 0, 0.42)) * robot.link(base_link_name)->lTcom();

  NavState state_0(p.rotation(), p.translation(), Point3::Zero());
  expected_continuous_states_0.insert(X(0), state_0);
  expected_continuous_states_0.insert(
      FR(0), Pose3(Rot3(), Point3(0.183, -0.13205, 0.22)) *
                 robot.link("FR_lower")->lTcom());
  expected_continuous_states_0.insert(
      FL(0), Pose3(Rot3(), Point3(0.183, 0.13205, 0.02)) *
                 robot.link("FL_lower")->lTcom());
  expected_continuous_states_0.insert(
      RR(0), Pose3(Rot3(), Point3(-0.183, -0.13205, 0.02)) *
                 robot.link("RR_lower")->lTcom());
  expected_continuous_states_0.insert(
      RL(0), Pose3(Rot3(), Point3(-0.183, 0.13205, 0.02)) *
                 robot.link("RL_lower")->lTcom());

  // Compare expected sequence with the actual sequence.
  EXPECT(assert_equal(expected_continuous_states_0.at<NavState>(X(0)),
                      estimate_0.continuousStates().at<NavState>(X(0)), 1e-2));
  // EXPECT(assert_equal(expected_continuous_states_0.at<Pose3>(FR(0)),
  //                     estimate_0.continuousStates().at<Pose3>(FR(0)),
  //                     1e-2));
}

TEST(ContactStateEstimator, LegKeys) {
  Robot robot = CreateRobotFromFile(robot_path);
  vector<string> lower_links = {"FR_lower", "FL_lower", "RR_lower", "RL_lower"};

  EXPECT(
      assert_equal(PoseKey(robot.link(lower_links[0])->id(), 0).key(), FR(0)));
  EXPECT(
      assert_equal(PoseKey(robot.link(lower_links[1])->id(), 0).key(), FL(0)));
  EXPECT(
      assert_equal(PoseKey(robot.link(lower_links[2])->id(), 0).key(), RR(0)));
  EXPECT(
      assert_equal(PoseKey(robot.link(lower_links[3])->id(), 0).key(), RL(0)));
}

// Test adding IMU chain
TEST(ContactStateEstimator, AddImuChain) {
  ContactStateEstimator estimator;
  auto state_prior = getStatePrior();
  auto imu_measurements = getImuMeasurements();
  NonlinearFactorGraph graph = estimator.addImuChain(1, imu_measurements);

  EXPECT(assert_equal(1, graph.size()));
}

// Test adding leg chain
TEST(ContactStateEstimator, AddLegChain) {
  ContactStateEstimator estimator;
  function<Key(uint64_t)> foot_key_fn = FL;  // Left-front
  vector<size_t> contact_sequence = {1, 1, 0};
  auto foot_priors = getFootPriors();
  NonlinearFactorGraph graph =
      estimator.addLegChain(1, foot_key_fn, contact_sequence);
  graph += estimator.addLegChain(2, foot_key_fn, contact_sequence);

  EXPECT(assert_equal(2, graph.size()));
}

// Test creating the smoothing factor graph given contact sequence.
TEST(ContactStateEstimator, FactorGraph) {
  vector<size_t> contact_sequence = {15, 15};
  ContactStateEstimator estimator;
  auto state_prior = getStatePrior();
  auto imu_measurements = getImuMeasurements();
  auto foot_priors = getFootPriors();

  Values joint_angles_0 = initializeRobotJoints(robot);

  auto graph = estimator.factorGraph(1, imu_measurements, joint_angles_0,
                                     contact_sequence);
  EXPECT(assert_equal(1 + 4, graph.size()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
