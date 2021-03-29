/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactStateEstimator.h
 * @brief Classes for estimating the hybrid state of robots.
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

#include <fstream>
#include <utility>
#include <vector>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/factors/ContactKinematicsPoseFactor.h"
#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/values.h"

using std::pair;
using std::vector;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace gtdynamics {

inline DynamicsSymbol FR(uint64_t j) { return internal::PoseKey(6, j); }
inline DynamicsSymbol FL(uint64_t j) { return internal::PoseKey(3, j); }
inline DynamicsSymbol RR(uint64_t j) { return internal::PoseKey(12, j); }
inline DynamicsSymbol RL(uint64_t j) { return internal::PoseKey(9, j); }

/**
 * @brief Encapsulate the hybrid state of a robot. The continuous states are the
 * 9-dimensional NavStates, and the discrete states are the contact states of
 * each foot (e.g. 0 for no-contact, 1 for contact).
 *
 */
struct ContactStateEstimate {
  ///< Base and feet states for T time steps of a fixed-lag smoother.
  gtsam::Values continuous_states;
  ///<
  gtsam::DiscreteFactor::Values contact_sequence;

  ContactStateEstimate() = default;

  ContactStateEstimate(const gtsam::Values& continuous,
                       const gtsam::DiscreteFactor::Values& discrete)
      : continuous_states(continuous), contact_sequence(discrete) {}

  gtsam::Values continuousStates() { return continuous_states; }

  gtsam::DiscreteFactor::Values contactSequence() { return contact_sequence; }
};


/**
 * @brief Estimator for estimating the hybrid state of the robot.
 *
 */
class ContactStateEstimator {
 private:
  Robot robot_;

  ///< Vector of links representing the feet names
  vector<std::string> lower_links_;

  ///< Name of the floating base link
  std::string base_link_;

  ///< Number of contact states
  size_t S;

  ///< Vector of functions for generating foot keys
  // Default value is for A1 robot.
  vector<std::function<gtsam::Key(uint64_t)>> foot_key_fns = {FR, FL, RR, RL};

 public:
  /**
   * @brief Struct to hold IMU and joint angle measurements.
   *
   */
  struct Measurement {
    gtsam::PreintegratedCombinedMeasurements imu_measurements;
    gtsam::Values joint_angles;

    Measurement(gtsam::PreintegratedCombinedMeasurements imu_meas,
                gtsam::Values j_angles)
        : imu_measurements(imu_meas), joint_angles(j_angles) {}
  };

  ContactStateEstimator() {}

  ContactStateEstimator(const Robot& robot,
                        const vector<std::string>& lower_links,
                        const std::string& base_link,
                        size_t num_contact_states = 2,
                        const vector<std::function<gtsam::Key(uint64_t)>>&
                            foot_key_functions = {FR, FL, RR, RL})
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
  ContactStateEstimate initialize(const gtsam::Values& joint_angles,
                                  std::string fixed_leg) {
    size_t time = 0;
    gtsam::NonlinearFactorGraph graph = createJointsGraph(joint_angles, time);

    graph += createFeetPriorGraph();

    // Creating initial values below.
    gtsam::Values initial;

    // Add all the initial link frame poses.
    // For the A1 we add 40 cm to Z.
    gtsam::Pose3 iTw(gtsam::Rot3(), gtsam::Point3(0, 0, 0.4));
    for (auto&& link : robot_.links()) {
      initial.insert<gtsam::Pose3>(internal::PoseKey(link->id(), time),
                                   iTw * link->wTl());
    }

    // Add the initial values for the joint angles.
    initial.insert(joint_angles);

    // Create the optimizer and optimizer
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    gtsam::Values result = optimizer.optimize();

    std::cout << "Initial error: " << graph.error(initial) << std::endl;
    std::cout << "Final error: " << graph.error(result) << std::endl;
    gtsam::Pose3 pose = result.at<gtsam::Pose3>(
        internal::PoseKey(robot_.link(base_link_)->id(), time));

    gtsam::Point3 velocity(0, 0, 0);

    // Get the estimate for the current time slice from the optimizer result
    gtsam::Values continuous_states;
    continuous_states.insert(X(time), gtsam::NavState(pose, velocity));

    for (auto&& key_fn : foot_key_fns) {
      continuous_states.insert(key_fn(time),
                               result.at<gtsam::Pose3>(key_fn(time)));
    }

    // Initially all feet are on the ground.
    gtsam::DiscreteFactor::Values contact_sequence;
    contact_sequence[C(time)] = 15;

    return ContactStateEstimate(continuous_states, contact_sequence);
  }

  /**
   * @brief Add the forward kinematics factors.
   *
   * @param joint_angles The joint angle measurements.
   * @param time The time step at which the joint angles were measured.
   * @return NonlinearFactorGraph
   */
  gtsam::NonlinearFactorGraph createJointsGraph(
      const gtsam::Values& joint_angles, size_t time) const {
    gtsam::NonlinearFactorGraph graph;
    // Add PoseFactors for forward kinematics for each joint in the robot.
    auto soft_constraint_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
    for (auto&& joint : robot_.joints()) {
      graph.emplace_shared<PoseFactor>(soft_constraint_model, joint, time);
    }

    // Add unary measurements on joint angles
    auto joint_angle_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.01);
    for (auto&& joint : robot_.joints()) {
      graph.addPrior<double>(internal::JointAngleKey(joint->id(), time),
                             JointAngle(joint_angles, joint->id(), time),
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
  gtsam::NonlinearFactorGraph createFeetPriorGraph(
      double height = 0.0, size_t time = 0.0,
      const gtsam::Pose3& wTb = gtsam::Pose3()) const {
    gtsam::NonlinearFactorGraph graph;
    // Add prior on foot contacts.
    auto foot_prior_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
    // This is contact when the leg is straight, and is only valid for the A1
    // robot.
    // 0.2 for lower link + 0.02 for toe sphere
    gtsam::Pose3 lTc_lower(gtsam::Rot3(), gtsam::Point3(0, 0, -0.22));

    gtsam::Vector3 gravity_n(0, 0, -9.81);
    for (auto&& lower_link_name : lower_links_) {
      auto lower_link = robot_.link(lower_link_name);

      gtsam::Pose3 cTcom = lTc_lower.inverse() * lower_link->lTcom();

      graph.emplace_shared<ContactKinematicsPoseFactor>(
          internal::PoseKey(lower_link->id(), time), foot_prior_model, cTcom,
          gravity_n, height);
    }

    // Add prior on base link CoM in spatial frame
    auto base = robot_.link(base_link_);
    gtsam::Pose3 sTbase = wTb * base->lTcom();
    gtsam::Vector6 sigmas;
    sigmas << 0.001, 0.001, 0.001, 1, 1, 100;  // leaving height free
    auto base_model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    graph.addPrior<gtsam::Pose3>(internal::PoseKey(base->id(), time), sTbase,
                                 base_model);

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
  gtsam::NonlinearFactorGraph addImuChain(
      size_t t,
      const gtsam::PreintegratedCombinedMeasurements& imu_measurements) const {
    gtsam::NonlinearFactorGraph graph;

    // Add the IMU chain
    graph.emplace_shared<gtsam::CombinedImuFactor>(
        X(t - 1), V(t - 1), X(t), V(t), B(t - 1), B(t), imu_measurements);

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
  gtsam::BetweenFactor<gtsam::Point3> footFactor(const gtsam::Key& key0,
                                                 size_t state0,
                                                 const gtsam::Key& key1,
                                                 size_t state1) const {
    gtsam::Point3 origin_(0.0, 0.0, 0.0);
    gtsam::SharedNoiseModel tight = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.001, 0.001, 0.001));
    gtsam::SharedNoiseModel loose =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.3));

    if (state0 > 0 && state1 > 0) {
      // Tight constraint in case of ground contact
      return gtsam::BetweenFactor<gtsam::Point3>(key0, key1, origin_, tight);
    } else {
      // Otherwise, loose BetweenFactor
      return gtsam::BetweenFactor<gtsam::Point3>(key0, key1, origin_, loose);
    }
  }

  /**
   * Add factor for contact state chain (prev_state -> new_state) for the leg
   * specified by `foot_key_fn`.
   */
  gtsam::NonlinearFactorGraph addLegChain(
      size_t t, const std::function<gtsam::Key(uint64_t)> foot_key_fn,
      const vector<size_t>& contact_sequence) const {
    gtsam::NonlinearFactorGraph graph;
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
  gtsam::NonlinearFactorGraph factorGraph(
      size_t t,
      const gtsam::PreintegratedCombinedMeasurements& imu_measurements,
      const gtsam::Values& joint_angles,
      const vector<size_t>& contact_sequence) const {
    // Create an empty graph
    gtsam::NonlinearFactorGraph graph;

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
  ContactStateEstimate run(const ContactStateEstimate& estimate,
                           const Measurement& measurement, size_t k) {
    vector<size_t> contact_sequence;
    for (size_t i = 0; i < k; ++i) {
      contact_sequence.push_back(estimate.contact_sequence.at(C(i)));
    }
    auto graph = factorGraph(k, measurement.imu_measurements,
                             measurement.joint_angles, contact_sequence);

    gtsam::Values initial;
    initial.update(estimate.continuous_states);

    initial.insert<gtsam::NavState>(X(k), gtsam::NavState());

    gtsam::Values result = initial;
    gtsam::DiscreteFactor::Values new_contact_sequence;
    ContactStateEstimate new_estimate{result, new_contact_sequence};
    return new_estimate;
  }
};

using CsvData = pair<vector<std::string>, vector<vector<double>>>;

CsvData readCsvData(const std::string& filename, size_t num_row_elements = 28) {
  std::ifstream csv_file(filename);
  vector<double> row;
  vector<std::string> header;
  vector<vector<double>> data;
  std::string line, x;
  std::getline(csv_file, line);

  std::istringstream header_stream(line);
  for (size_t idx = 0; idx < num_row_elements; ++idx) {
    std::getline(header_stream, x, ',');
    header.push_back(x);
  }

  while (std::getline(csv_file, line)) {
    std::istringstream row_stream(line);
    row.clear();
    for (size_t idx = 0; idx < num_row_elements; ++idx) {
      std::getline(row_stream, x, ',');

      row.push_back(stod(x));
    }
    data.push_back(row);
  }

  return std::make_pair(header, data);
}

/**
 * @brief Get the joint angles at the specified index.
 *
 * @param header The CSV data header
 * @param data The 2 dimensional data matrix.
 * @param index The row index at which to get the joint angles.
 * @return Values
 */
gtsam::Values getJointAngles(const Robot& robot,
                             const vector<std::string>& header,
                             const vector<vector<double>>& data, size_t index) {
  gtsam::Values joint_angles;
  for (size_t i = 16; i < 28; ++i) {
    InsertJointAngle(&joint_angles, robot.joint(header[i])->id(), index,
                     data[index][i]);
  }
  return joint_angles;
}

gtsam::Pose3 getPose(vector<vector<double>> data, size_t index) {
  gtsam::Rot3 R = gtsam::Rot3::Quaternion(data[index][6], data[index][3],
                                          data[index][4], data[index][5]);
  gtsam::Point3 t(data[index][0], data[index][1], data[index][2]);
  return gtsam::Pose3(R, t);
}

/**
 * @brief Get the state at the specified index.
 *
 * @param data The CSV data including the ground truth position.
 * @param index The row index at which to get the pose and velocity.
 * @return NavState
 */
gtsam::NavState getState(vector<vector<double>> data, size_t index) {
  gtsam::Pose3 pose = getPose(data, index);
  gtsam::Vector3 velocity(data[index][7], data[index][8], data[index][9]);
  return gtsam::NavState(pose, velocity);
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
boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams(
    double accel_noise_sigma = 0.0, double gyro_noise_sigma = 0.0,
    double accel_bias_rw_sigma = 0.0, double gyro_bias_rw_sigma = 0.0) {
  // We use the sensor specs to build the noise model for the IMU factor.
  gtsam::Matrix3 measured_acc_cov = gtsam::I_3x3 * pow(accel_noise_sigma, 2);
  gtsam::Matrix3 measured_omega_cov = gtsam::I_3x3 * pow(gyro_noise_sigma, 2);
  // error committed in integrating position from velocities
  gtsam::Matrix3 integration_error_cov = gtsam::I_3x3 * 1e-8;
  gtsam::Matrix3 bias_acc_cov = gtsam::I_3x3 * pow(accel_bias_rw_sigma, 2);
  gtsam::Matrix3 bias_omega_cov = gtsam::I_3x3 * pow(gyro_bias_rw_sigma, 2);
  // error in the bias used for preintegration
  gtsam::Matrix6 bias_acc_omega_int = gtsam::I_6x6 * 1e-5;

  auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
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

pair<gtsam::Pose3, gtsam::SharedNoiseModel> getStatePrior() {
  return std::make_pair<gtsam::Pose3, gtsam::SharedNoiseModel>(
      gtsam::Pose3(),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones()));
}

gtsam::PreintegratedCombinedMeasurements getImuMeasurements() {
  auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
  gtsam::PreintegratedCombinedMeasurements imu_measurements(params);
  return imu_measurements;
}

vector<pair<gtsam::Point3, gtsam::SharedNoiseModel>> getFootPriors() {
  vector<pair<gtsam::Point3, gtsam::SharedNoiseModel>> foot_priors;
  vector<gtsam::Point3> feet = {
      gtsam::Point3(0.5, -0.2, 0), gtsam::Point3(0.5, 0.2, 0),
      gtsam::Point3(0.0, -0.2, 0), gtsam::Point3(0.0, 0.2, 0)};
  for (size_t index = 0; index < feet.size(); ++index) {
    foot_priors.emplace_back(
        feet.at(index),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.01)));
  }
  return foot_priors;
}

}  // namespace gtdynamics
