/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSpiderWalking.cpp
 * @brief Test robot trajectory optimization with Phases.
 * @author: Alejandro Escontrela, Stephanie McCormick
 * @author: Disha Das, Tarushree Gandhi
 * @author: Frank Dellaert, Varun Agrawal, Stefanos Charalambous
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector3;
using gtsam::noiseModel::Isotropic;
using gtsam::noiseModel::Unit;
using std::string;
using std::vector;

using namespace gtdynamics;

#define EXPECT_DOUBLES_SIMILAR(expected, actual, rtol) \
  EXPECT_DOUBLES_EQUAL(expected, actual, rtol* expected)

// Returns a Trajectory object for a single robot walk cycle.
Trajectory getTrajectory(const Robot &robot, size_t repeat) {
  // Label which feet are what...
  vector<LinkSharedPtr> odd_feet = {
      robot.link("tarsus_1_L1"), robot.link("tarsus_3_L3"),
      robot.link("tarsus_5_R4"), robot.link("tarsus_7_R2")};
  vector<LinkSharedPtr> even_feet = {
      robot.link("tarsus_2_L2"), robot.link("tarsus_4_L4"),
      robot.link("tarsus_6_R3"), robot.link("tarsus_8_R1")};
  auto all_feet = odd_feet;
  all_feet.insert(all_feet.end(), even_feet.begin(), even_feet.end());

  // Create three different FootContactConstraintSpecs, one for all the feet on the
  // ground, one with even feet on the ground, one with odd feet in contact..
  const Point3 contact_in_com(0, 0.19, 0);
  auto stationary = boost::make_shared<FootContactConstraintSpec>(all_feet, contact_in_com);
  auto odd = boost::make_shared<FootContactConstraintSpec>(odd_feet, contact_in_com);
  auto even = boost::make_shared<FootContactConstraintSpec>(even_feet, contact_in_com);
  
  FootContactVector states = {stationary, even, stationary, odd};
  std::vector<size_t> phase_lengths = {1,2,1,2};

  WalkCycle walk_cycle(states, phase_lengths);

  // TODO(issue #257): Trajectory should *be* a vector of phases, so rather that
  // store the walkcycle and repeat, it should store the phases.
  return Trajectory(walk_cycle, repeat);
}

TEST(testSpiderWalking, WholeEnchilada) {
  // Load Stephanie's robot robot (alt version, created by Tarushree/Disha).
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("spider_alt.sdf"), "spider");

  double sigma_dynamics = 1e-5;    // std of dynamics constraints.
  double sigma_objectives = 1e-6;  // std of additional objectives.

  // Noise models.
  auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
       dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
       objectives_model_6 = Isotropic::Sigma(6, sigma_objectives),
       objectives_model_1 = Isotropic::Sigma(1, sigma_objectives);

  // Env parameters.
  Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;

  OptimizerSetting opt(sigma_dynamics);
  DynamicsGraph graph_builder(opt, gravity);

  // Create the trajectory, consisting of 2 walk phases, each consisting of 4
  // phases: [stationary, odd, stationary, even].
  auto trajectory = getTrajectory(robot, 2);

  // Create multi-phase trajectory factor graph
  auto collocation = CollocationScheme::Euler;
  auto graph =
      trajectory.multiPhaseFactorGraph(robot, graph_builder, collocation, mu);
  EXPECT_LONGS_EQUAL(3557, graph.size());
  EXPECT_LONGS_EQUAL(3847, graph.keys().size());

  // Build the objective factors.
  const Point3 step(0, 0.4, 0);
  NonlinearFactorGraph objectives =
      trajectory.contactPointObjectives(robot, Isotropic::Sigma(3, 1e-7), step, 0);
  // per walk cycle: 1*8 + 2*8 + 1*8 + 2*8 = 48
  // 2 repeats, hence:
  EXPECT_LONGS_EQUAL(48 * 2, objectives.size());
  EXPECT_LONGS_EQUAL(48 * 2, objectives.keys().size());

  // Get final time step.
  int K = trajectory.getEndTimeStep(trajectory.numPhases() - 1);
  EXPECT_LONGS_EQUAL(12, K);  // TODO(frank): why not 11?

  // Add base goal objectives to the factor graph.
  auto base_link = robot.link("body");
  for (int k = 0; k <= K; k++) {
    objectives.add(
        LinkObjectives(base_link->id(), k)
            .pose(Pose3(Rot3(), Point3(0, 0.0, 0.5)), Isotropic::Sigma(6, 5e-5))
            .twist(gtsam::Z_6x1, Isotropic::Sigma(6, 5e-5)));
  }

  // Add link and joint boundary conditions to FG.
  trajectory.addBoundaryConditions(&objectives, robot, dynamics_model_6,
                                   dynamics_model_6, objectives_model_6,
                                   objectives_model_1, objectives_model_1);

  // Constrain all Phase keys to have duration of 1 /240.
  const double desired_dt = 1. / 240;
  trajectory.addIntegrationTimeFactors(&objectives, desired_dt, 1e-30);

  // Add min torque objectives.
  trajectory.addMinimumTorqueFactors(&objectives, robot, Unit::Create(1));

  // Add prior on hip joint angles (spider specific)
  auto prior_model = Isotropic::Sigma(1, 1.85e-4);
  for (auto&& joint : robot.joints())
    if (joint->name().find("hip2") == 0)
      for (int k = 0; k <= K; k++)
        objectives.add(JointObjectives(joint->id(), k).angle(2.5, prior_model));

  // Regression test on objective factors
  EXPECT_LONGS_EQUAL(910, objectives.size());
  EXPECT_LONGS_EQUAL(899, objectives.keys().size());

  // Add objective factors to the graph
  graph.add(objectives);
  EXPECT_LONGS_EQUAL(3557 + 910, graph.size());
  EXPECT_LONGS_EQUAL(3847, graph.keys().size());

  // Initialize solution.
  double gaussian_noise = 0.0;
  Initializer initializer;
  Values init_vals =
      trajectory.multiPhaseInitialValues(robot, initializer, gaussian_noise, desired_dt);
  EXPECT_LONGS_EQUAL(3847, init_vals.size());

  // Compare error for all factors with expected values in file.
  // Note, expects space after comma in csv or won't work.
  std::string key, comm;
  double expected;
  std::string filename = kTestPath + std::string("testSpiderWalking.csv");
  std::ifstream is(filename.c_str());
  is >> key >> expected;
  double actual = graph.error(init_vals);
  const double tol = 1.0, rtol = 0.01;
  EXPECT_DOUBLES_SIMILAR(expected, actual, rtol);

  // If there is an error, create a file errors.csv with all error comparisons.
  if (fabs(actual - expected) > tol) {
    std::ofstream os("errors.csv");
    os << "total, " << actual << "\n";
    for (size_t i = 0; i < graph.size(); i++) {
      is >> key >> expected;
      const auto& factor = graph[i];
      const double actual = factor->error(init_vals);
      bool equal = fabs(actual - expected) < std::max(tol, rtol * expected);
      auto bare_ptr = factor.get();
      if (!equal) {
        // Print to stdout for CI.
        std::cout << typeid(*bare_ptr).name() << ", " << actual << ", "
                  << expected << ", " << equal << "\n";
      }
      os << typeid(*bare_ptr).name() << ", " << actual << ", " << expected
         << ", " << equal << "\n";
    }
  }

  // Optimize!
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals);
  auto results = optimizer.optimize();

  // Regression!
  EXPECT_DOUBLES_SIMILAR(986944306277409, graph.error(init_vals), rtol);
  EXPECT_DOUBLES_SIMILAR(349267962507918, graph.error(results), rtol);

  // Add regressions on initial values.
  auto body = robot.link("body");
  EXPECT(gtsam::assert_equal(Pose3(), Pose(init_vals, body->id(), 0), 1e-3));

  // Still at the identity as this is a zero values initialization.
  EXPECT(gtsam::assert_equal(Pose3(), Pose(init_vals, body->id(), K), 1e-3));

  auto foot = robot.link("tarsus_1_L1");
  EXPECT(gtsam::assert_equal(Point3(-1.08497, 1.27372, 0),
                             Pose(init_vals, foot->id(), 0).translation(),
                             1e-3));

  // TODO(frank): why did this not move?
  EXPECT(gtsam::assert_equal(Point3(-1.08497, 1.27372, 0),
                             Pose(init_vals, foot->id(), K).translation(),
                             1e-3));

  EXPECT_DOUBLES_EQUAL(0, JointAngle(init_vals, 0, 0), 1e-7);
  EXPECT_DOUBLES_EQUAL(0, JointAngle(init_vals, 31, 0), 1e-7);
  EXPECT_DOUBLES_EQUAL(0, JointAngle(init_vals, 0, K), 1e-7);
  EXPECT_DOUBLES_EQUAL(0, JointAngle(init_vals, 31, K), 1e-7);

  EXPECT_DOUBLES_EQUAL(0, JointVel(init_vals, 0, 0), 1e-7);
  EXPECT_DOUBLES_EQUAL(0, JointVel(init_vals, 31, 0), 1e-7);
  EXPECT_DOUBLES_EQUAL(0, JointVel(init_vals, 0, K), 1e-7);
  EXPECT_DOUBLES_EQUAL(0, JointVel(init_vals, 31, K), 1e-7);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
