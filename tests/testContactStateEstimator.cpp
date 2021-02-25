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

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/factors/ContactKinematicsPoseFactor.h"
#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"

using namespace std;
using namespace gtdynamics;
using namespace gtsam;

using gtsam::assert_equal;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

inline Key FR(uint64_t j) { return PoseKey(6, j); }
inline Key FL(uint64_t j) { return PoseKey(3, j); }
inline Key RR(uint64_t j) { return PoseKey(12, j); }
inline Key RL(uint64_t j) { return PoseKey(9, j); }

// Load the A1 robot
Robot robot = CreateRobotFromFile(string(URDF_PATH) + "/a1.urdf");

const string base_link_name = "trunk";  //"pelvis";

// Foot contact links are FL_lower, FR_lower, RL_lower and RR_lower
// trunk_link -> hip_joint -> hip_link -> upper_joint -> upper_link ->
// lower_joint -> lower_link
vector<string> lower_links = {"FR_lower", "FL_lower", "RR_lower", "RL_lower"};

class ContactStateEstimator {
 private:
  Robot robot_;

  ///< Vector of links representing the feet names
  vector<string> lower_links_;

  ///< Name of the floating base link
  string base_link_;

  ///< Number of contact states
  size_t S;

  ///< Vector of functions for generating foot keys
  // Default value is for A1 robot.
  vector<function<Key(uint64_t)>> foot_key_fns = {FR, FL, RR, RL};

 public:
  /**
   * @brief
   *
   */
  struct Estimate {
    ///< Base and feet states for T time steps of a fixed-lag smoother.
    Values continuous_states;
    ///<
    DiscreteFactor::Values contact_sequence;

    Estimate() = default;

    Estimate(const Values& continuous, const DiscreteFactor::Values& discrete)
        : continuous_states(continuous), contact_sequence(discrete) {}

    Values continuousStates() { return continuous_states; }

    DiscreteFactor::Values contactSequence() { return contact_sequence; }
  };

  struct Measurement {
    PreintegratedCombinedMeasurements imu_measurements;
    JointValues joint_angles;

    Measurement(PreintegratedCombinedMeasurements imu_meas,
                JointValues j_angles)
        : imu_measurements(imu_meas), joint_angles(j_angles) {}
  };

  ContactStateEstimator() {}

  ContactStateEstimator(
      const Robot& robot, const vector<string>& lower_links,
      const string& base_link, size_t num_contact_states = 2,
      const vector<function<Key(uint64_t)>>& foot_key_functions = {FR, FL, RR,
                                                                   RL})
      : robot_(robot),
        lower_links_(lower_links),
        base_link_(base_link),
        S(num_contact_states),
        foot_key_fns(foot_key_functions) {}

  /**
   * @brief Initialize the estimate of the robot's base joint via forward and
   * contact kinematics.
   *
   * @param joint_angles The joint angles measured from the robot.
   * @param fixed_leg Name of the leg fixed to origin.
   * @return Estimate
   */
  Estimate initialize(const JointValues& joint_angles, string fixed_leg) {
    size_t time = 0;
    NonlinearFactorGraph graph = createJointsGraph(joint_angles, time);

    graph += createFeetPriorGraph();

    // Creating initial values below.
    Values initial;

    // Add all the initial link frame poses.
    // For the A1 we add 40 cm to Z.
    Pose3 iTw(Rot3(), Point3(0, 0, 0.4));
    for (auto&& link : robot_.links()) {
      initial.insert<Pose3>(PoseKey(link->getID(), time), iTw * link->wTl());
    }

    // Add the initial values for the joint angles.
    for (auto&& [joint_name, angle] : joint_angles) {
      JointSharedPtr joint = robot_.getJointByName(joint_name);
      initial.insert<JointTyped::JointCoordinate>(
          JointAngleKey(joint->getID(), time), angle);
    }

    // Create the optimizer and optimizer
    LevenbergMarquardtOptimizer optimizer(graph, initial);
    Values result = optimizer.optimize();

    std::cout << "Initial error: " << graph.error(initial) << std::endl;
    std::cout << "Final error: " << graph.error(result) << std::endl;
    Pose3 pose = result.at<Pose3>(
        PoseKey(robot_.getLinkByName(base_link_)->getID(), time));

    Point3 velocity(0, 0, 0);

    // Get the estimate for the current time slice from the optimizer result
    Values continuous_states;
    continuous_states.insert(X(time), NavState(pose, velocity));

    for (auto&& key_fn : foot_key_fns) {
      continuous_states.insert(key_fn(time), result.at<Pose3>(key_fn(time)));
    }

    // Initially all feet are on the ground.
    DiscreteFactor::Values contact_sequence;
    contact_sequence[C(time)] = 15;

    return Estimate(continuous_states, contact_sequence);
  }

  /**
   * @brief Add the forward kinematics factors.
   *
   * @param joint_angles The joint angle measurements.
   * @param time The time step at which the joint angles were measured.
   * @return NonlinearFactorGraph
   */
  NonlinearFactorGraph createJointsGraph(const JointValues& joint_angles,
                                         size_t time) const {
    NonlinearFactorGraph graph;
    // Add PoseFactors for forward kinematics for each joint in the robot.
    auto soft_constraint_model = noiseModel::Isotropic::Sigma(6, 0.1);
    for (auto&& joint : robot_.joints()) {
      graph.emplace_shared<PoseFactor>(soft_constraint_model, joint, time);
    }

    // Add unary measurements on joint angles
    auto joint_angle_model = noiseModel::Isotropic::Sigma(1, 0.01);
    for (auto&& joint : robot_.joints()) {
      // use .at since joint_angles is const ref
      double joint_angle = joint_angles.at(joint->name());
      graph.addPrior<double>(JointAngleKey(joint->getID(), time), joint_angle,
                             joint_angle_model);
    }

    return graph;
  }

  /**
   * Add the contact state factors and factors related to pose/foot position
   * priors. Initially, all feet are on the ground, thus default height is 0.
   *
   * @param height The height of the foot from the ground plane.
   */
  NonlinearFactorGraph createFeetPriorGraph(double height = 0.0,
                                            size_t time = 0.0) const {
    NonlinearFactorGraph graph;
    // Add prior on foot contacts.
    auto foot_prior_model = noiseModel::Isotropic::Sigma(1, 0.001);
    // This is contact when the leg is straight, and is only valid for the A1
    // robot.
    // 0.2 for lower link + 0.02 for toe sphere
    Pose3 lTc_lower(Rot3(), Point3(0, 0, -0.22));

    Vector3 gravity_n(0, 0, -9.81);
    for (auto&& lower_link_name : lower_links_) {
      auto lower_link = robot_.getLinkByName(lower_link_name);

      Pose3 cTcom = lTc_lower.inverse() * lower_link->lTcom();

      graph.emplace_shared<ContactKinematicsPoseFactor>(
          PoseKey(lower_link->getID(), time), foot_prior_model, cTcom,
          gravity_n, height);
    }

    // Add prior on base link CoM in spatial frame
    auto base = robot_.getLinkByName(base_link_);
    Pose3 sTbase = Pose3() * base->lTcom();
    Vector6 sigmas;
    sigmas << 0.001, 0.001, 0.001, 1, 1, 100;  // leaving height free
    auto base_model = noiseModel::Diagonal::Sigmas(sigmas);
    graph.addPrior<Pose3>(PoseKey(base->getID(), time), sTbase, base_model);

    return graph;
  }

  /**
   * Add a Combined IMU factor as a link for the chain of robot states.
   * Each state would correspond to a preintegrated IMU measurement.
   *
   * @param imu_measurements Preintegrated IMU measurements between the previous
   * state and the current state.
   * @param k The index of the current state.
   */
  NonlinearFactorGraph addImuChain(
      size_t t,
      const PreintegratedCombinedMeasurements& imu_measurements) const {
    NonlinearFactorGraph graph;

    // Add the IMU chain
    graph.emplace_shared<CombinedImuFactor>(X(t - 1), V(t - 1), X(t), V(t),
                                            B(t - 1), B(t), imu_measurements);

    return graph;
  }

  /**
   * Add a factor for the foot contact state transition.
   * Based on whether the foot is going from stance to swing, or swing to
   * stance, we decide whether the constraint is tight or loose.
   *
   * @param key0
   * @param state0
   * @param key1
   * @param state1
   */
  BetweenFactor<Point3> footFactor(const Key& key0, size_t state0,
                                   const Key& key1, size_t state1) const {
    Point3 origin_(0.0, 0.0, 0.0);
    SharedNoiseModel tight =
        noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
    SharedNoiseModel loose =
        noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.3));

    if (state0 > 0 && state1 > 0) {
      // Tight constraint in case of ground contact
      return BetweenFactor<Point3>(key0, key1, origin_, tight);
    } else {
      // Otherwise, loose BetweenFactor
      return BetweenFactor<Point3>(key0, key1, origin_, loose);
    }
  }

  /**
   * Add factor for contact state chain (prev_state -> new_state) for the leg
   * specified by `foot_key_fn`.
   */
  NonlinearFactorGraph addLegChain(
      size_t t, const function<Key(uint64_t)> foot_key_fn,
      const vector<size_t>& contact_sequence) const {
    NonlinearFactorGraph graph;
    // Add the foot evolution chain
    size_t prev_contact_state = contact_sequence.at(t - 1),
           contact_state = contact_sequence.at(t);
    graph.add(footFactor(foot_key_fn(t - 1), prev_contact_state, foot_key_fn(t),
                         contact_state));

    return graph;
  }

  /**
   * Method to create the complete factor graph using IMU factors and
   * Contact Transition factors.
   */
  NonlinearFactorGraph factorGraph(
      size_t t, const PreintegratedCombinedMeasurements& imu_measurements,
      const JointValues& joint_angles,
      const vector<size_t>& contact_sequence) const {
    // Create an empty graph
    NonlinearFactorGraph graph;

    // Create IMU chain
    graph += addImuChain(t, imu_measurements);

    // Add the forward kinematics factors
    graph += createJointsGraph(joint_angles, t);

    // Create leg plates
    for (size_t i = 0; i < foot_key_fns.size(); ++i) {
      vector<size_t> new_contact_sequence;
      for (auto&& state : contact_sequence) {
        new_contact_sequence.push_back(state & size_t(pow(S, i)));
      }
      graph += addLegChain(t, foot_key_fns.at(i), new_contact_sequence);
    }

    return graph;
  }

  /**
   * @brief Run the contact state estimator to return the state estimate for
   * the next state in the chain.
   *
   * @param estimate
   * @param measurement
   * @return Estimate containing the base pose, base velocity, pose of the
   * feet and the contact states over the last T time steps.
   */
  Estimate run(const Estimate& estimate, const Measurement& measurement,
               size_t k) {
    vector<size_t> contact_sequence;
    for (size_t i = 0; i < k; ++i) {
      contact_sequence.push_back(estimate.contact_sequence.at(C(i)));
    }
    auto graph = factorGraph(k, measurement.imu_measurements,
                             measurement.joint_angles, contact_sequence);

    Values initial;
    initial.update(estimate.continuous_states);

    initial.insert<NavState>(X(k), NavState());

    Values result = initial;
    DiscreteFactor::Values new_contact_sequence;
    Estimate new_estimate{result, new_contact_sequence};
    return new_estimate;
  }
};

using CsvData = pair<vector<string>, vector<vector<double>>>;

CsvData readCsvData(const string& filename, size_t num_row_elements = 28) {
  ifstream csv_file(filename);
  vector<double> row;
  vector<string> header;
  vector<vector<double>> data;
  string line, x;
  std::getline(csv_file, line);

  istringstream header_stream(line);
  for (size_t idx = 0; idx < num_row_elements; ++idx) {
    getline(header_stream, x, ',');
    header.push_back(x);
  }

  while (std::getline(csv_file, line)) {
    istringstream row_stream(line);
    row.clear();
    for (size_t idx = 0; idx < num_row_elements; ++idx) {
      getline(row_stream, x, ',');

      row.push_back(stod(x));
    }
    data.push_back(row);
  }

  return make_pair(header, data);
}

/**
 * @brief Get the joint angles at the specified index.
 *
 * @param header The CSV data header
 * @param data The 2 dimensional data matrix.
 * @param index The row index at which to get the joint angles.
 * @return JointValues
 */
JointValues getJointAngles(vector<string> header, vector<vector<double>> data,
                           size_t index) {
  JointValues joint_angles;
  for (size_t i = 16; i < 28; ++i) {
    joint_angles.emplace(header[i], data[index][i]);
  }
  return joint_angles;
}

Pose3 getPose(vector<vector<double>> data, size_t index) {
  Rot3 R = Rot3::Quaternion(data[index][6], data[index][3], data[index][4],
                            data[index][5]);
  Point3 t(data[index][0], data[index][1], data[index][2]);
  Pose3 pose(R, t);
  return pose;
}

/**
 * @brief Get the state at the specified index.
 *
 * @param data The CSV data including the ground truth position.
 * @param index The row index at which to get the pose and velocity.
 * @return NavState
 */
NavState getState(vector<vector<double>> data, size_t index) {
  Pose3 pose = getPose(data, index);
  Vector3 velocity(data[index][7], data[index][8], data[index][9]);
  return NavState(pose, velocity);
}

/**
 * @brief Get the IMU parameters.
 *
 * @param accel_noise_sigma
 * @param gyro_noise_sigma
 * @param accel_bias_rw_sigma
 * @param gyro_bias_rw_sigma
 * @return boost::shared_ptr<PreintegratedCombinedMeasurements::Params>
 */
boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams(
    double accel_noise_sigma = 0.0, double gyro_noise_sigma = 0.0,
    double accel_bias_rw_sigma = 0.0, double gyro_bias_rw_sigma = 0.0) {
  // We use the sensor specs to build the noise model for the IMU factor.
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_int =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  return p;
}

// Test one invocation of the estimator
TEST(ContactStateEstimator, Invoke) {
  ContactStateEstimator estimator(robot, lower_links, base_link_name, 2);

  CsvData csvData =
      readCsvData("/home/varun/borglab/motion_imitation/a1_data.csv");

  vector<string> header = csvData.first;
  vector<vector<double>> data = csvData.second;

  JointValues joint_angles_0 = getJointAngles(header, data, 0);

  // Check the CoM of the fused calf and toe in the FR lower link frame
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0.004727, 0, -0.131975)),
                      robot.getLinkByName("FR_lower")->lTcom(), 1e-5));

  // This is the pose of the contact point in the calf link frame after SDF
  // merges the calf and toe links connected by a fixed joint.
  Pose3 lTc_lower(Rot3(), Point3(0, 0, -0.22));

  // Create initial estimate given joint angles, assume all feet on the
  // ground. The constructor below will create the continuous and the discrete
  // states for the initial timestep 0. X is forward, Y is to the left, Z is
  // up.
  ContactStateEstimator::Estimate estimate_0 =
      estimator.initialize(joint_angles_0, "RR_lower");

  // Set of base and leg states at initial time 0 for the fixed-lag smoother.
  Values expected_continuous_states_0;
  Pose3 p = getPose(data, 0) * robot.getLinkByName(base_link_name)->lTcom();
  NavState state_0(p.rotation(), p.translation(), Point3::Zero());
  expected_continuous_states_0.insert(X(0), state_0);
  expected_continuous_states_0.insert(
      FR(0), Pose3(Rot3::Quaternion(8.96462182e-01, -1.93800697e-04,
                                    -4.43119974e-01, 8.13348341e-05),
                   Point3(0.03447161, -0.12761532, 0.1427124)) *
                 robot.getLinkByName("FR_lower")->lTcom());

  expected_continuous_states_0.insert(
      FL(0), Pose3(Rot3::Quaternion(8.96044111e-01, -1.92925155e-04,
                                    -4.43964760e-01, 8.11309202e-05),
                   Point3(0.03461184, 0.13648479, 0.14241265)) *
                 robot.getLinkByName("FL_lower")->lTcom());
  expected_continuous_states_0.insert(
      RR(0), Pose3(Rot3::Quaternion(8.81532617e-01, -2.05365878e-04,
                                    -4.72122859e-01, 9.51115317e-05),
                   Point3(-0.32569342, -0.12761457, 0.13209494)) *
                 robot.getLinkByName("RR_lower")->lTcom());
  expected_continuous_states_0.insert(
      RL(0), Pose3(Rot3::Quaternion(8.81441187e-01, -2.06621107e-04,
                                    -4.72293535e-01, 9.58185374e-05),
                   Point3(-0.32574409, 0.13648506, 0.13203046)) *
                 robot.getLinkByName("RL_lower")->lTcom());

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

  JointValues joint_angles_1 = getJointAngles(header, data, time_idx);
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

// Test zero joint angles state
TEST(ContactStateEstimator, JointGraph) {
  ContactStateEstimator estimator(robot, lower_links, base_link_name, 2);

  JointValues joint_angles_0 = {
      {"FR_hip_joint", 0}, {"FR_upper_joint", 0}, {"FR_lower_joint", 0},
      {"FL_hip_joint", 0}, {"FL_upper_joint", 0}, {"FL_lower_joint", 0},
      {"RR_hip_joint", 0}, {"RR_upper_joint", 0}, {"RR_lower_joint", 0},
      {"RL_hip_joint", 0}, {"RL_upper_joint", 0}, {"RL_lower_joint", 0}};

  size_t time = 0;
  auto graph = estimator.createJointsGraph(joint_angles_0, time);

  auto base_link_key =
      PoseKey(robot.getLinkByName(base_link_name)->getID(), time);

  // Assume the base link is at the origin
  graph.addPrior<Pose3>(base_link_key, Pose3());

  // Creating initial values below.
  Values initial;

  // Add all the initial link frame poses.
  for (auto&& link : robot.links()) {
    initial.insert<Pose3>(PoseKey(link->getID(), time),
                          link->wTl() * link->lTcom());
  }

  // Add the initial values for the joint angles.
  for (auto&& [joint_name, angle] : joint_angles_0) {
    JointSharedPtr joint = robot.getJointByName(joint_name);
    initial.insert<JointTyped::JointCoordinate>(
        JointAngleKey(joint->getID(), time), angle);
  }

  // Create the optimizer and optimizer
  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Set of base and leg states at initial time 0 for the fixed-lag smoother.
  Pose3 expected_base_pose(Rot3(), Point3(0, 0, 0.0));
  Pose3 expected_FR_lower_pose = robot.getLinkByName("FR_lower")->wTl() *
                                 robot.getLinkByName("FR_lower")->lTcom();

  // Compare expected base pose to optimized one.
  EXPECT(
      assert_equal(expected_base_pose, result.at<Pose3>(base_link_key), 1e-2));
  EXPECT(
      assert_equal(expected_FR_lower_pose, result.at<Pose3>(FR(time)), 1e-1));
}

// Test zero joint angles state
TEST(ContactStateEstimator, ZeroAngles) {
  ContactStateEstimator estimator(robot, lower_links, base_link_name, 2);

  JointValues joint_angles_0 = {
      {"FR_hip_joint", 0}, {"FR_upper_joint", 0}, {"FR_lower_joint", 0},
      {"FL_hip_joint", 0}, {"FL_upper_joint", 0}, {"FL_lower_joint", 0},
      {"RR_hip_joint", 0}, {"RR_upper_joint", 0}, {"RR_lower_joint", 0},
      {"RL_hip_joint", 0}, {"RL_upper_joint", 0}, {"RL_lower_joint", 0}};

  // Create initial estimate given joint angles, assume all feet on the
  // ground. The constructor below will create the continuous and the discrete
  // states for the initial timestep 0. X is forward, Y is to the left, Z is
  // up.
  ContactStateEstimator::Estimate estimate_0 =
      estimator.initialize(joint_angles_0, "RR_lower");

  // Set of base and leg states at initial time 0 for the fixed-lag smoother.
  Values expected_continuous_states_0;
  Pose3 p = Pose3(Rot3(), Point3(0, 0, 0.42)) *
            robot.getLinkByName(base_link_name)->lTcom();

  NavState state_0(p.rotation(), p.translation(), Point3::Zero());
  expected_continuous_states_0.insert(X(0), state_0);
  expected_continuous_states_0.insert(
      FR(0), Pose3(Rot3(), Point3(0.183, -0.13205, 0.22)) *
                 robot.getLinkByName("FR_lower")->lTcom());
  expected_continuous_states_0.insert(
      FL(0), Pose3(Rot3(), Point3(0.183, 0.13205, 0.02)) *
                 robot.getLinkByName("FL_lower")->lTcom());
  expected_continuous_states_0.insert(
      RR(0), Pose3(Rot3(), Point3(-0.183, -0.13205, 0.02)) *
                 robot.getLinkByName("RR_lower")->lTcom());
  expected_continuous_states_0.insert(
      RL(0), Pose3(Rot3(), Point3(-0.183, 0.13205, 0.02)) *
                 robot.getLinkByName("RL_lower")->lTcom());

  // Compare expected sequence with the actual sequence.
  EXPECT(assert_equal(expected_continuous_states_0.at<NavState>(X(0)),
                      estimate_0.continuousStates().at<NavState>(X(0)), 1e-2));
  // EXPECT(assert_equal(expected_continuous_states_0.at<Pose3>(FR(0)),
  //                     estimate_0.continuousStates().at<Pose3>(FR(0)),
  //                     1e-2));
}

TEST(ContactStateEstimator, LegKeys) {
  Robot robot = CreateRobotFromFile(string(URDF_PATH) + "/a1.urdf");
  vector<string> lower_links = {"FR_lower", "FL_lower", "RR_lower", "RL_lower"};

  EXPECT(assert_equal(
      PoseKey(robot.getLinkByName(lower_links[0])->getID(), 0).key(), FR(0)));
  EXPECT(assert_equal(
      PoseKey(robot.getLinkByName(lower_links[1])->getID(), 0).key(), FL(0)));
  EXPECT(assert_equal(
      PoseKey(robot.getLinkByName(lower_links[2])->getID(), 0).key(), RR(0)));
  EXPECT(assert_equal(
      PoseKey(robot.getLinkByName(lower_links[3])->getID(), 0).key(), RL(0)));
}

pair<Pose3, SharedNoiseModel> getStatePrior() {
  return make_pair<Pose3, SharedNoiseModel>(
      Pose3(), noiseModel::Diagonal::Sigmas(Vector6::Ones()));
}

PreintegratedCombinedMeasurements getImuMeasurements() {
  auto params = PreintegrationCombinedParams::MakeSharedU(9.81);
  PreintegratedCombinedMeasurements imu_measurements(params);
  return imu_measurements;
}

vector<pair<Point3, SharedNoiseModel>> getFootPriors() {
  vector<pair<Point3, SharedNoiseModel>> foot_priors;
  vector<Point3> feet = {Point3(0.5, -0.2, 0), Point3(0.5, 0.2, 0),
                         Point3(0.0, -0.2, 0), Point3(0.0, 0.2, 0)};
  for (size_t index = 0; index < feet.size(); ++index) {
    foot_priors.emplace_back(
        feet.at(index), noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.01)));
  }
  return foot_priors;
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
  JointValues joint_angles_0 = {
      {"FR_hip_joint", 0}, {"FR_upper_joint", 0}, {"FR_lower_joint", 0},
      {"FL_hip_joint", 0}, {"FL_upper_joint", 0}, {"FL_lower_joint", 0},
      {"RR_hip_joint", 0}, {"RR_upper_joint", 0}, {"RR_lower_joint", 0},
      {"RL_hip_joint", 0}, {"RL_upper_joint", 0}, {"RL_lower_joint", 0}};

  auto graph = estimator.factorGraph(1, imu_measurements, joint_angles_0,
                                     contact_sequence);
  EXPECT(assert_equal(1 + 4, graph.size()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
