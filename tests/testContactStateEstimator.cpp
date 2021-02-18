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
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <utility>
#include <vector>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"

using namespace std;
using namespace gtdynamics;
using namespace gtsam;

using gtsam::assert_equal;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
inline gtsam::Key LF(uint64_t j) { return gtsam::symbol_shorthand::L(j); }
inline gtsam::Key RF(uint64_t j) { return gtsam::Symbol('r', j); }
inline gtsam::Key LH(uint64_t j) { return gtsam::Symbol('m', j); }
inline gtsam::Key RH(uint64_t j) { return gtsam::Symbol('s', j); }

class ContactStateEstimator {
 private:
  Robot robot_;

  // Vector of functions for generating foot keys
  vector<function<gtsam::Key(uint64_t)>> foot_key_fns = {LF, RF, LH, RH};

  // Number of contact states
  size_t S;

 public:
  struct Estimate {
    gtsam::Point3 t = Vector3(0, 0, 0);   /// Translation in inertial frame
    gtsam::Rot3 R;                        /// Rotation in inertial frame
    gtsam::Vector3 v = Vector3(0, 0, 0);  /// Velocity in inertial frame

    Estimate() = default;
    Estimate(const Robot& robot, const JointValues& joint_angles) {}
  };

  struct Measurement {
    PreintegratedCombinedMeasurements imu_measurements_;
    JointValues joint_angles_;

    Measurement(PreintegratedCombinedMeasurements imu_measurements,
                JointValues joint_angles)
        : imu_measurements_(imu_measurements), joint_angles_(joint_angles) {}
  };

  ContactStateEstimator() {}

  ContactStateEstimator(const Robot& robot, size_t num_contact_states = 2)
      : robot_(robot), S(num_contact_states) {}

  Estimate initialize(const JointValues& joint_angles) {
    return Estimate(robot_, joint_angles);
  }

  /**
   * Add Combined IMU factors for the chain of robot states.
   * Each state would correspond to a preintegrated IMU measurement.
   */
  void addImuChain(gtsam::NonlinearFactorGraph& graph,
                   const gtsam::Pose3& initial_state,
                   const gtsam::SharedNoiseModel& noise_model,
                   const vector<gtsam::PreintegratedCombinedMeasurements>&
                       imu_measurements) const {
    // Add prior on initial state and constraint on velocity
    graph.addPrior<gtsam::Pose3>(X(0), initial_state, noise_model);
    graph.add(
        gtsam::NonlinearEquality<gtsam::Point3>(V(0), gtsam::Point3(0, 0, 0)));

    // Add the rest of the IMU chain
    for (size_t k = 0; k < imu_measurements.size(); k++) {
      graph.emplace_shared<gtsam::CombinedImuFactor>(X(k), V(k), X(k + 1),
                                                     V(k + 1), B(k), B(k + 1),
                                                     imu_measurements.at(k));
    }
  }

  /**
   * Add a factor for the foot contact state transition.
   * Based on whether the foot is going from stance to swing, or swing to
   * stance, we decide whether the constraint is tight or loose.
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
  void addLegChain(gtsam::NonlinearFactorGraph& graph,
                   const function<gtsam::Key(uint64_t)> foot_key_fn,
                   const gtsam::Point3& initial_foot_position,
                   const gtsam::SharedNoiseModel& noise_model,
                   const vector<size_t>& contact_sequence) const {
    // Add prior on initial foot position
    graph.addPrior<gtsam::Point3>(foot_key_fn(0), initial_foot_position,
                                  noise_model);
    // Add the rest of the foot evolution chain
    for (size_t k = 0; k < contact_sequence.size() - 1; ++k) {
      size_t prev_contact_state = contact_sequence.at(k),
             contact_state = contact_sequence.at(k + 1);
      graph.add(footFactor(foot_key_fn(k), prev_contact_state,
                           foot_key_fn(k + 1), contact_state));
    }
  }

  /**
   * Method to create the complete factor graph using IMU factors and
   * Contact Transition factors.
   */
  gtsam::NonlinearFactorGraph factorGraph(
      const gtsam::Pose3& initial_state,
      const gtsam::SharedNoiseModel& noise_model,
      const vector<gtsam::PreintegratedCombinedMeasurements>& imu_measurements,
      const vector<pair<gtsam::Point3, gtsam::SharedNoiseModel>>& foot_priors,
      const vector<size_t>& contact_sequence) const {
    // Create an empty graph
    gtsam::NonlinearFactorGraph graph;

    // Create IMU chain
    addImuChain(graph, initial_state, noise_model, imu_measurements);

    // Create leg plates
    for (size_t i = 0; i < foot_key_fns.size(); ++i) {
      vector<size_t> new_contact_sequence;
      for (auto&& state : contact_sequence) {
        new_contact_sequence.push_back(state & size_t(pow(S, i)));
      }
      addLegChain(graph, foot_key_fns.at(i), foot_priors.at(i).first,
                  foot_priors.at(i).second, new_contact_sequence);
    }

    return graph;
  }
};

// Test one invocation of the estimator
TEST(ContactStateEstimator, Invoke) {
  Robot robot = CreateRobotFromFile(string(URDF_PATH) + "/a1.urdf");
  ContactStateEstimator estimator(robot);

  for (size_t idx = 0; idx < robot.numLinks(); idx++) {
    cout << idx << ": " << robot.links().at(idx)->name() << endl;
  }

  // for (size_t idx = 0; idx < robot.numJoints(); idx++) {
  //   cout << idx << ": " << robot.joints().at(idx)->name() << endl;
  // }

  const string base_name = "trunk";  //"pelvis";

  JointValues joint_angles_0;

  Point3 base_translation = robot.getLinkByName(base_name)->wTl().translation();
  EXPECT(assert_equal(Point3(0, 0, 0.9), base_translation));

  // Create initial estimate given joint angles, assume all feet on the ground.
  // The constructor below will create the continuous and the discrete states
  // for the initial timestep 0.
  ContactStateEstimator::Estimate estimate_0 =
      estimator.initialize(joint_angles_0);

  EXPECT(assert_equal(Point3(0, 0, 0.9), estimate_0.t));

  // Measurement object containing IMU measurement between state 0 and 1, and
  // joint angles at timestep 1.
  // PreintegratedCombinedMeasurements imu_measurements;
  // JointValues joint_angles_1;
  // ContactStateEstimator::Measurement measurements_01(imu_measurements,
  //                                                    joint_angles_1);

  // Invoke the fixed lag smoother
  // ContactStateEstimator::Estimate estimate_1 =
  //     estimator.run(estimate_0, measurements_01);

  // // State sequence for fixed lag smoothing from previous invocation
  // gtsam::Values expected_continuous_states_0_to_1;

  // // Sequence of discrete contact states
  // gtsam::DiscreteFactor::Values expected_contact_sequence;
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

pair<gtsam::Pose3, gtsam::SharedNoiseModel> getStatePrior() {
  return make_pair<gtsam::Pose3, gtsam::SharedNoiseModel>(
      gtsam::Pose3(),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones()));
}

vector<gtsam::PreintegratedCombinedMeasurements> getImuMeasurements() {
  auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
  gtsam::PreintegratedCombinedMeasurements imu(params);
  vector<gtsam::PreintegratedCombinedMeasurements> imu_measurements = {imu,
                                                                       imu};
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

// Test adding IMU chain
TEST(ContactStateEstimator, AddImuChain) {
  gtsam::NonlinearFactorGraph graph;
  ContactStateEstimator estimator;
  auto state_prior = getStatePrior();
  auto imu_measurements = getImuMeasurements();
  estimator.addImuChain(graph, state_prior.first, state_prior.second,
                        imu_measurements);

  EXPECT(assert_equal(4, graph.size()));
}

// Test adding leg chain
TEST(ContactStateEstimator, AddLegChain) {
  gtsam::NonlinearFactorGraph graph;
  ContactStateEstimator estimator;
  function<gtsam::Key(uint64_t)> foot_key_fn = LF;  // Left-front
  vector<size_t> contact_sequence = {1, 1, 0};
  auto foot_priors = getFootPriors();
  estimator.addLegChain(graph, foot_key_fn, foot_priors[0].first,
                        foot_priors[0].second, contact_sequence);

  EXPECT(assert_equal(3, graph.size()));
}

// Test creating the smoothing factor graph given contact sequence.
TEST(ContactStateEstimator, FactorGraph) {
  vector<size_t> contact_sequence = {15, 15, 15};
  ContactStateEstimator estimator;
  auto state_prior = getStatePrior();
  auto imu_measurements = getImuMeasurements();
  auto foot_priors = getFootPriors();
  auto graph =
      estimator.factorGraph(state_prior.first, state_prior.second,
                            imu_measurements, foot_priors, contact_sequence);
  // self.assertIsInstance(graph, gtsam.NonlinearFactorGraph)
  EXPECT(assert_equal(4 + 4 * 3, graph.size()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
