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
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
inline gtsam::Key LF(uint64_t j) { return gtsam::symbol_shorthand::L(j); }
inline gtsam::Key RF(uint64_t j) { return gtsam::Symbol('r', j); }
inline gtsam::Key LH(uint64_t j) { return gtsam::Symbol('m', j); }
inline gtsam::Key RH(uint64_t j) { return gtsam::Symbol('s', j); }

class ContactStateEstimator {
 private:
  Robot robot_;

  ///< Vector of functions for generating foot keys
  // TODO(Varun) make this a constructor argument
  vector<function<gtsam::Key(uint64_t)>> foot_key_fns = {LF, RF, LH, RH};

  ///< Vector of links representing the feet names
  vector<string> foot_links_;

  ///< Name of the floating base link
  string base_link_;

  ///< Number of contact states
  size_t S;

 public:
  struct Estimate {
    Point3 t = Vector3(0, 0, 0);   /// Translation in inertial frame
    Rot3 R;                        /// Rotation in inertial frame
    Vector3 v = Vector3(0, 0, 0);  /// Velocity in inertial frame

    Estimate() = default;

    Estimate(const Robot& robot, const JointValues& joint_angles,
             const vector<string>& foot_links, const string& base_link = "base",
             const string& fixed_leg = "RR_lower", double time = 0) {
      NonlinearFactorGraph graph;

      // Add PoseFactors for each joint in the robot.
      auto soft_constraint_model = noiseModel::Isotropic::Sigma(6, 0.1);
      for (auto&& joint : robot.joints()) {
        graph.emplace_shared<PoseFactor>(soft_constraint_model, joint, time);
      }

      // Add unary measurements on joint angles
      auto joint_angle_model = noiseModel::Isotropic::Sigma(1, 0.01);
      for (auto&& joint : robot.joints()) {
        // use .at since joint_angles is const ref
        double joint_angle = joint_angles.at(joint->name());
        graph.addPrior<double>(JointAngleKey(joint->getID(), time), joint_angle,
                               joint_angle_model);
      }

      // Add prior on foot contacts.
      // Initially, all feet are on the ground.
      double height = 0.0;
      auto foot_prior_model = noiseModel::Isotropic::Sigma(1, 0.001);
      // This is contact when the leg is straight, and is only valid for the A1
      // robot.
      Pose3 lTc_lower(Rot3(), Point3(0, 0, -0.22));
      Vector3 gravity_n(0, 0, -9.81);
      for (auto&& foot_link_name : foot_links) {
        auto foot_link = robot.getLinkByName(foot_link_name);

        Pose3 cTcom = lTc_lower.inverse() * foot_link->lTcom();

        graph.emplace_shared<ContactKinematicsPoseFactor>(
            PoseKey(foot_link->getID(), time), foot_prior_model, cTcom,
            gravity_n, height);
      }

      // Add prior on base link in spatial frame
      auto base = robot.getLinkByName(base_link);
      Pose3 sTbase;
      Vector6 sigmas;
      sigmas << 0.1, 0.1, 0.1, 0.1, 0.1, 100;  // leaving height free
      auto base_model = noiseModel::Diagonal::Sigmas(sigmas);
      graph.addPrior<Pose3>(PoseKey(base->getID(), time), sTbase, base_model);

      // Creating initial values below.
      Values initial;

      // Add all the initial link frame poses.
      // For the A1 we add 40 cm to Z.
      Pose3 iTw(Rot3(), Point3(0, 0, 0.4));
      for (auto&& link : robot.links()) {
        std::cout << "Link " << link->name() << " wTl " << std::endl
                  << link->wTl() << std::endl;
        initial.insert<Pose3>(PoseKey(link->getID(), time), iTw * link->wTl());
      }

      // Add the initial values for the joint angles.
      for (auto&& [joint_name, angle] : joint_angles) {
        JointSharedPtr joint = robot.getJointByName(joint_name);
        initial.insert<JointTyped::JointCoordinate>(
            JointAngleKey(joint->getID(), time), angle);
      }

      // Create the optimizer and optimizer
      GaussNewtonOptimizer optimizer(graph, initial);
      Values result = optimizer.optimize();

      std::cout << "Initial error: " << graph.error(initial) << std::endl;
      std::cout << "Final error: " << graph.error(result) << std::endl;
      Pose3 pose = result.at<Pose3>(
          PoseKey(robot.getLinkByName(base_link)->getID(), time));

      t = pose.translation();
      R = pose.rotation();

      for (auto&& link : robot.links()) {
        if (std::find(foot_links.begin(), foot_links.end(), link->name()) !=
            foot_links.end()) {
          std::cout << "Pose for foot " << link->name() << " with ID "
                    << link->getID() << "\n"
                    << result.at<Pose3>(PoseKey(link->getID(), time))
                    << std::endl;
        }
      }
    }
  };

  struct Measurement {
    PreintegratedCombinedMeasurements imu_measurements_;
    JointValues joint_angles_;

    Measurement(PreintegratedCombinedMeasurements imu_measurements,
                JointValues joint_angles)
        : imu_measurements_(imu_measurements), joint_angles_(joint_angles) {}
  };

  ContactStateEstimator() {}

  ContactStateEstimator(const Robot& robot, vector<string> foot_links,
                        string base_link, size_t num_contact_states = 2)
      : robot_(robot),
        foot_links_(foot_links),
        base_link_(base_link),
        S(num_contact_states) {}

  /**
   * @brief Initialize the estimate of the robot's base joint via forward and
   * contact kinematics.
   *
   * @param joint_angles The joint angles measured from the robot.
   * @param fixed_leg Name of the leg fixed to origin.
   * @return Estimate
   */
  Estimate initialize(const JointValues& joint_angles, string fixed_leg) {
    return Estimate(robot_, joint_angles, foot_links_, base_link_, fixed_leg,
                    0);
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
  // Load the A1 robot
  Robot robot = CreateRobotFromFile(string(URDF_PATH) + "/a1.urdf");

  const string base_name = "trunk";  //"pelvis";

  // Foot contact links are FL_lower, FR_lower, RL_lower and RR_lower
  // trunk_link -> hip_joint -> hip_link -> upper_joint -> upper_link ->
  // lower_joint -> lower_link
  vector<string> foot_links = {"FL_lower", "FR_lower", "RL_lower", "RR_lower"};

  ContactStateEstimator estimator(robot, foot_links, base_name, 2);

  JointValues joint_angles_0 = {
      {"FR_hip_joint", 1.71831757e-04},    {"FR_upper_joint", 9.24168539e-01},
      {"FR_lower_joint", -1.83299289e+00},  // front right
      {"FL_hip_joint", 1.73695544e-04},    {"FL_upper_joint", 9.23078979e-01},
      {"FL_lower_joint", -1.83378528e+00},  // front left
      {"RR_hip_joint", 1.39569334e-04},    {"RR_upper_joint", 8.77893931e-01},
      {"RR_lower_joint", -1.85187479e+00},  // rear right
      {"RL_hip_joint", 1.36664882e-04},    {"RL_upper_joint", 8.78340180e-01},
      {"RL_lower_joint", -1.85270761e+00}  // rear left
  };

  // Check the CoM of the fused calf and toe in the FR lower link frame
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0.004727, 0, -0.131975)),
                      robot.getLinkByName("FR_lower")->lTcom(), 1e-5));

  // This is the pose of the contact point in the calf link frame after SDF
  // merges the calf and toe links connected by a fixed joint.
  Pose3 lTc_lower(Rot3(), Point3(0, 0, -0.22));

  // Create initial estimate given joint angles, assume all feet on the ground.
  // The constructor below will create the continuous and the discrete states
  // for the initial timestep 0.
  // X is forward, Y is to the left, Z is up.
  ContactStateEstimator::Estimate estimate_0 =
      estimator.initialize(joint_angles_0, "RR_lower");

  EXPECT(
      assert_equal(Point3(-0.00274313, 0.0023002, 0.26228938), estimate_0.t));

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
