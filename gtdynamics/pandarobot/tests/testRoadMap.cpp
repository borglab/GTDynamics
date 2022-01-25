/**
 * @file  testRoadMap.cpp
 * @brief test Roadmap
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/pandarobot/roadmap/RoadMap.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <iostream>
#include <vector>

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;

// Probably can put all these together into a single test

TEST(RoadMap, addPoseNodes) {
  RoadMap roadmap;

  std::vector<Pose3> poses(3);
  poses[0] = Pose3(Rot3::identity(), (Point3() << 0, 0, 0.25).finished());
  poses[1] = Pose3(Rot3::identity(), (Point3() << 0, 0.1, 0.25).finished());
  poses[2] = Pose3(Rot3::identity(), (Point3() << 0, -0.1, 0.25).finished());

  // Add 3 poses
  std::vector<Pose3> result_poses = roadmap.addPoseNodes(poses).getposes();

  // Check if poses match
  EXPECT(assert_equal(poses, result_poses, 1e-5))
}

TEST(RoadMap, addStateNodes) {
  RoadMap roadmap;

  // Create graph from nodes and check if Dijsktra works
  std::vector<Vector7> states(6);
  states[0] << 2, 0.5, 1, -0.5, 0.0, 1.00, 0.00;
  states[1] << 2, 0.5, 1, -0.5, 0.3, 1.00, 0.10;
  states[2] << 2, 0.5, 1, -0.5, 0.6, 1.05, 0.00;
  states[3] << 2, 0.5, 1, -0.5, 1.2, 1.00, 0.00;
  states[4] << 2, 0.5, 1, -0.5, 0.3, 1.00, -0.1;
  states[5] << 2, 0.5, 1, -0.5, 0.9, 1.00, 0.0;

  std::vector<Vector7> actualstates =
      roadmap.addStateNodes(states).getstatenodes();
  EXPECT(assert_equal(states, actualstates))
}

TEST(RoadMap, checkJointLimits) {
  std::vector<Vector7> states(8);
  states[0] << 4.00, 1.50, 2.00, -0.5, 2.00, 3.5, 2.00;
  states[1] << 2.00, -2.0, 1.00, -1.0, 1.00, 3.0, 1.00;
  states[2] << 1.00, 1.00, 3.00, -1.5, 0.00, 2.5, 0.00;
  states[3] << 0.00, 0.50, 0.00, 0.00, -1.0, 2.0, -1.0;
  states[4] << -1.0, 0.00, -1.0, -2.0, -3.0, 1.0, -2.0;
  states[5] << -2.0, -0.5, -2.0, -2.5, -2.0, 4.0, 0.00;
  states[6] << 0.00, 1.50, 2.00, -1.5, 0.50, 2.0, -3.5;
  states[7] << 1.25, -0.5, 0.00, -1.0, -1.0, 1.5, 0.25;
  // using size_t instead of bool to be able to use the assert_equal fun
  std::vector<size_t> expected_results(8, 0);
  expected_results[7] = true;
  std::vector<size_t> actual_results(8);
  for (size_t i = 0; i < 8; i++) {
    actual_results[i] = size_t(RoadMap::checkJointLimits(states[i]));
  }

  EXPECT(assert_equal(expected_results, actual_results))
}

TEST(RoadMap, computeStateSolutions) {
  RoadMap roadmap;

  std::vector<Pose3> poses(3);
  Rot3 rotation(0, 0, 1, 1, 0, 0, 0, 1, 0);
  poses[0] = Pose3(rotation, (Point3() << 0.4, -0.300, 0.4)
                                 .finished());  // 6 solutions (2 theta_{2,3,4})
  poses[1] = Pose3(rotation, (Point3() << 0.4, -0.225, 0.4)
                                 .finished());  // 4 solutions (2 theta_{2,4})
  poses[2] = Pose3(rotation, (Point3() << 0.4, -0.150, 0.4)
                                 .finished());  // 2 solutions (2 theta_{4, })

  // Compute solutions with a discretization on theta (uniform atm)
  size_t theta7_samples = 4;
  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);

  // Get list of nodes
  std::vector<Vector7> statenodes = roadmap.getstatenodes();

  EXPECT(assert_equal(12, statenodes.size()))

  std::vector<Vector7> expected_statenodes(7);
  expected_statenodes[0] << -1.79295, 1.59592, 1.62524, -1.88506, -2.57194,
      3.03272, -0.57946;
  expected_statenodes[1] << 1.34326, -1.26355, -1.51391, -1.83391, 2.87389,
      3.0935, 0.57946;
  expected_statenodes[2] << -1.84029, 0.61681, 1.46604, -2.02171, 2.27525,
      2.71283, 1.73838;
  expected_statenodes[3] << 1.35437, -1.73332, -1.45725, -2.0822, -2.67492,
      2.81303, -0.57946;
  expected_statenodes[4] << -1.89951, 0.37131, 1.54981, -2.16696, 2.50181,
      2.57467, 1.73838;
  expected_statenodes[5] << 1.00791, -0.191668, -1.2719, -2.26973, 2.7066,
      2.50179, 1.73838;

  std::vector<size_t> indicestotest = {0, 3, 4, 7, 8, 11};
  std::vector<size_t> expectedposeindex = {0, 0, 0, 1, 1, 2};
  std::vector<size_t> posestatelist_index = {0, 3, 4, 1, 2, 1};

  for (size_t i = 0; i < 6; ++i) {
    int state_index = indicestotest[i], pose_index = expectedposeindex[i];

    EXPECT(assert_equal(expected_statenodes[i], statenodes[state_index],
                        1e-5))  // check in full list

    size_t a = roadmap.getPoseFromState(state_index);
    // Check if stored indices are the correct ones:
    // check state_conf->pose vector
    EXPECT(assert_equal(pose_index, a))
    // check pose->state_confs list
    EXPECT(assert_equal(state_index, roadmap.getStatesFromPose(
                                         pose_index)[posestatelist_index[i]]))
  }
}

// test optional parameter: relationships
TEST(RoadMap, createGraph) {
  RoadMap roadmap;

  std::vector<Pose3> poses(3);
  Rot3 rotation(0, 0, 1, 1, 0, 0, 0, 1, 0);
  poses[0] = Pose3(rotation, (Point3() << 0.4, -0.300, 0.4)
                                 .finished());  // 6 solutions (2 theta_{2,3,4})
  poses[1] = Pose3(rotation, (Point3() << 0.4, -0.225, 0.4)
                                 .finished());  // 4 solutions (2 theta_{2,4})
  poses[2] = Pose3(rotation, (Point3() << 0.4, -0.150, 0.4)
                                 .finished());  // 2 solutions (2 theta_{4, })

  // Compute solutions with a discretization on theta (uniform atm)
  size_t theta7_samples = 4;
  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);

  double distance_threshold = 0.7;
  roadmap.set_threshold(distance_threshold);
  roadmap.createGraph();
  std::vector<std::vector<RoadMap::Edge>> adjacencylist =
      roadmap.getadjacencylist();

  EXPECT(assert_equal(12, adjacencylist.size()))

  std::vector<std::vector<RoadMap::Edge>> expected_adjacencylist(12);
  {
    expected_adjacencylist[0] = {{0.34660791176774935, 6}};
    expected_adjacencylist[1] = {{0.34660807693993506, 7}};
    expected_adjacencylist[4] = {{0.40287697005413425, 8}};
    expected_adjacencylist[5] = {{0.4028769700541343, 9}};
    expected_adjacencylist[6] = {{0.34660791176774935, 0}};
    expected_adjacencylist[7] = {{0.34660807693993506, 1}};
    expected_adjacencylist[8] = {{0.40287697005413425, 4},
                                 {0.4972345149564741, 10}};
    expected_adjacencylist[9] = {{0.4028769700541343, 5},
                                 {0.49723451495647397, 11}};
    expected_adjacencylist[10] = {{0.4972345149564741, 8}};
    expected_adjacencylist[11] = {{0.49723451495647397, 9}};
  }

  for (size_t node = 0; node < 12; node++) {
    EXPECT(assert_equal(expected_adjacencylist[node].size(),
                        adjacencylist[node].size()))
    for (size_t edge = 0; edge < adjacencylist[node].size(); ++edge) {
      RoadMap::Edge actual_edge = adjacencylist[node][edge],
                    expected_edge = expected_adjacencylist[node][edge];
      EXPECT(assert_equal(expected_edge.first, actual_edge.first, 1e-5))
      EXPECT(assert_equal(expected_edge.second, actual_edge.second))
    }
  }

  // Test the threshold, small enough value there is nothing
}

// A 3x2 canvas, with 4 theta7 samples, check the relationships
TEST(RoadMap, computeStateLocality) {
  RoadMap roadmap;

  Rot3 rotation(0, 0, 1, 1, 0, 0, 0, 1, 0);
  std::vector<Pose3> poses(6);
  poses[0] = Pose3(rotation, (Point3() << 0.4, -0.300, 0.3)
                                 .finished());  // 4 solutions (2 theta_{3,4})
  poses[1] = Pose3(rotation, (Point3() << 0.4, -0.225, 0.3)
                                 .finished());  // 4 solutions (2 theta_{3,4})
  poses[2] = Pose3(rotation, (Point3() << 0.4, -0.150, 0.3)
                                 .finished());  // 4 solutions (2 theta_{3,4})
  poses[3] = Pose3(rotation, (Point3() << 0.4, -0.300, 0.4)
                                 .finished());  // 6 solutions (2 theta_{2,3,4})
  poses[4] = Pose3(rotation, (Point3() << 0.4, -0.225, 0.4)
                                 .finished());  // 4 solutions (2 theta_{2,4})
  poses[5] = Pose3(rotation, (Point3() << 0.4, -0.150, 0.4)
                                 .finished());  // 2 solutions (2 theta_{4, })

  // Compute solutions with a discretization on theta (uniform atm)
  size_t theta7_samples = 4;
  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);

  std::vector<std::vector<size_t>> pose_locality(6);
  {
    pose_locality[0] = {0, 1, 3, 4};
    pose_locality[1] = {0, 1, 2, 3, 4, 5};
    pose_locality[2] = {1, 2, 4, 5};
    pose_locality[3] = {0, 1, 3, 4};
    pose_locality[4] = {0, 1, 2, 3, 4, 5};
    pose_locality[5] = {1, 2, 4, 5};
  }

  size_t theta_kernel_size = 1;
  std::vector<std::vector<std::pair<size_t, size_t>>> actual_state_locality =
      roadmap.computeStateLocality(pose_locality, theta_kernel_size);

  std::vector<std::vector<std::pair<size_t, size_t>>> expected_state_locality(
      24);

  {
    expected_state_locality[0] = {{0, 4}, {4, 8}, {12, 18}, {18, 22}};
    expected_state_locality[1] = expected_state_locality[0];
    expected_state_locality[2] = {{0, 4}, {4, 8}, {14, 18}, {20, 22}};
    expected_state_locality[3] = expected_state_locality[2];
    expected_state_locality[4] = {{0, 4},   {4, 8},   {8, 12},
                                   {12, 18}, {18, 22}, {22, 24}};
    expected_state_locality[5] = expected_state_locality[4];
    expected_state_locality[6] = {{0, 4},   {4, 8},   {8, 12},
                                   {14, 18}, {20, 22}, {22, 24}};
    expected_state_locality[7] = expected_state_locality[6];
    expected_state_locality[8] = {{4, 8}, {8, 12}, {18, 22}, {22, 24}};
    expected_state_locality[9] = expected_state_locality[8];
    expected_state_locality[10] = {{4, 8}, {8, 12}, {20, 22}, {22, 24}};
    expected_state_locality[11] = expected_state_locality[10];
    expected_state_locality[12] = {{0, 2}, {4, 6}, {12, 16}, {18, 20}};
    expected_state_locality[13] = expected_state_locality[12];
    expected_state_locality[14] = {{0, 4}, {4, 8}, {12, 18}, {18, 22}};
    expected_state_locality[15] = expected_state_locality[14];
    expected_state_locality[16] = {{0, 4}, {4, 8}, {14, 18}, {20, 22}};
    expected_state_locality[17] = expected_state_locality[16];
    expected_state_locality[18] = {
        {0, 2}, {4, 6}, {8, 10}, {12, 16}, {18, 20}};
    expected_state_locality[19] = expected_state_locality[18];
    expected_state_locality[20] = {{0, 4},   {4, 8},   {8, 12},
                                    {14, 18}, {20, 22}, {22, 24}};
    expected_state_locality[21] = expected_state_locality[20];
    expected_state_locality[22] = {{4, 8}, {8, 12}, {20, 22}, {22, 24}};
    expected_state_locality[23] = expected_state_locality[22];
  }

  for (size_t node = 0; node < 24; node++) {
    std::cout << "node: " << node << std::endl;
    EXPECT(assert_equal(expected_state_locality[node].size(),
                        actual_state_locality[node].size()))

    {
      size_t minsize = (expected_state_locality[node].size() <
                                actual_state_locality[node].size()
                            ? expected_state_locality[node].size()
                            : actual_state_locality[node].size());

      for (size_t pair = 0; pair < minsize; ++pair) {
        std::pair<size_t, size_t> actual_pair =
                                      actual_state_locality[node][pair],
                                  expected_pair =
                                      expected_state_locality[node][pair];
        std::cout << "expected pair: " << expected_pair.first << " "
                  << expected_pair.second << "\t\t"
                  << "actual pair: " << actual_pair.first << " "
                  << actual_pair.second << std::endl;
      }
      for (size_t pair = minsize; pair < expected_state_locality[node].size();
           ++pair) {
        std::pair<size_t, size_t> expected_pair =
            expected_state_locality[node][pair];
        std::cout << "expected pair: " << expected_pair.first << " "
                  << expected_pair.second << std::endl;
      }
      for (size_t pair = minsize; pair < actual_state_locality[node].size();
           ++pair) {
        std::pair<size_t, size_t> actual_pair =
            actual_state_locality[node][pair];
        std::cout << "\t\t\t\t"
                  << "actual pair: " << actual_pair.first << " "
                  << actual_pair.second << std::endl;
      }
    }

    for (size_t pair = 0; pair < actual_state_locality[node].size(); ++pair) {
      std::pair<size_t, size_t> actual_pair =
                                    actual_state_locality[node][pair],
                                expected_pair =
                                    expected_state_locality[node][pair];
      EXPECT(assert_equal(expected_pair.first, actual_pair.first))
      EXPECT(assert_equal(expected_pair.second, actual_pair.second))
    }
  }
}
// States created arbitrarily with an arbitrary reference, check if it's what is
// expected
TEST(RoadMap, createGraphFromReference) {
  RoadMap roadmap;

  std::vector<Pose3> poses(3);
  Rot3 rotation(0, 0, 1, 1, 0, 0, 0, 1, 0);
  poses[0] = Pose3(rotation, (Point3() << 0.4, -0.300, 0.4)
                                 .finished());  // 6 solutions (2 theta_{2,3,4})
  poses[1] = Pose3(rotation, (Point3() << 0.4, -0.225, 0.4)
                                 .finished());  // 4 solutions (2 theta_{2,4})
  poses[2] = Pose3(rotation, (Point3() << 0.4, -0.150, 0.4)
                                 .finished());  // 2 solutions (2 theta_{4, })

  // Compute solutions with a discretization on theta (uniform atm)
  size_t theta7_samples = 4;
  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);

  std::vector<std::vector<std::pair<size_t, size_t>>> state_locality(
      12);

  {
    state_locality[0] = {{0, 4}, {6, 8}};
    state_locality[1] = state_locality[0];
    state_locality[2] = {{0, 6}, {6, 10}};
    state_locality[3] = state_locality[2];
    state_locality[4] = {{2, 6}, {8, 10}};
    state_locality[5] = state_locality[4];
    state_locality[6] = {{0, 4}, {6, 8}};
    state_locality[7] = state_locality[6];
    state_locality[8] = {{2, 6}, {8, 10}, {10, 12}};
    state_locality[9] = state_locality[8];
    state_locality[10] = {{8, 10}, {10, 12}};
    state_locality[11] = state_locality[10];
  }

  double distance_threshold = 0.7;
  roadmap.set_threshold(distance_threshold);
  roadmap.createGraphFromReference(state_locality);
  std::vector<std::vector<RoadMap::Edge>> adjacencylist =
      roadmap.getadjacencylist();

  EXPECT(assert_equal(12, adjacencylist.size()))

  std::vector<std::vector<RoadMap::Edge>> expected_adjacencylist(12);
  {
    expected_adjacencylist[0] = {{0.34660791176774935, 6}};
    expected_adjacencylist[1] = {{0.34660807693993506, 7}};
    expected_adjacencylist[4] = {{0.40287697005413425, 8}};
    expected_adjacencylist[5] = {{0.4028769700541343, 9}};
    expected_adjacencylist[6] = {{0.34660791176774935, 0}};
    expected_adjacencylist[7] = {{0.34660807693993506, 1}};
    expected_adjacencylist[8] = {{0.40287697005413425, 4},
                                 {0.4972345149564741, 10}};
    expected_adjacencylist[9] = {{0.4028769700541343, 5},
                                 {0.49723451495647397, 11}};
    expected_adjacencylist[10] = {{0.4972345149564741, 8}};
    expected_adjacencylist[11] = {{0.49723451495647397, 9}};
  }

  for (size_t node = 0; node < 12; node++) {
    EXPECT(assert_equal(expected_adjacencylist[node].size(),
                        adjacencylist[node].size()))
    for (size_t edge = 0; edge < adjacencylist[node].size(); ++edge) {
      RoadMap::Edge actual_edge = adjacencylist[node][edge],
                    expected_edge = expected_adjacencylist[node][edge];
      EXPECT(assert_equal(expected_edge.first, actual_edge.first, 1e-5))
      EXPECT(assert_equal(expected_edge.second, actual_edge.second))
    }
  }

  // Test the threshold, small enough value there is nothing
}

// maybe a function that does all these previous tasks?
TEST(RoadMap, findClosestNodeState) {
  RoadMap roadmap;

  // For now no weighted distance but may be interesting?
  std::vector<Pose3> poses(3);
  Rot3 rotation(0, 0, 1, 1, 0, 0, 0, 1, 0);
  poses[0] = Pose3(rotation, (Point3() << 0.4, -0.300, 0.4)
                                 .finished());  // 6 solutions (2 theta_{2,3,4})
  poses[1] = Pose3(rotation, (Point3() << 0.4, -0.225, 0.4)
                                 .finished());  // 4 solutions (2 theta_{2,4})
  poses[2] = Pose3(rotation, (Point3() << 0.4, -0.150, 0.4)
                                 .finished());  // 2 solutions (2 theta_{4, })

  // Add 3 poses
  roadmap.addPoseNodes(poses);

  size_t theta7_samples = 4;
  roadmap.computeStateSolutions(theta7_samples);

  // First one example with just little tweaks from a stored solution
  Vector7 simple_configuration;
  simple_configuration << -1.84029, 0.61681, 1.46604, -2.02171, 2.27525,
      2.71283, 1.73838 + 0.05;

  size_t simple_node = roadmap.findClosestNodeState(simple_configuration);

  EXPECT(assert_equal(4, simple_node))

  // Second: another one "random" one
  Vector7 complex_configuration;
  complex_configuration << 0.5, 0, 0.3, -0.4, 0.2, 2, 0.7;

  size_t complex_node = roadmap.findClosestNodeState(complex_configuration);

  EXPECT(assert_equal(5, complex_node))
}

TEST(RoadMap, findClosestNodesPose) {
  RoadMap roadmap;

  std::vector<Pose3> poses(3);
  Rot3 rotation(0, 0, 1, 1, 0, 0, 0, 1, 0);
  poses[0] = Pose3(rotation, (Point3() << 0.4, -0.300, 0.4)
                                 .finished());  // 6 solutions (2 theta_{2,3,4})
  poses[1] = Pose3(rotation, (Point3() << 0.4, -0.225, 0.4)
                                 .finished());  // 4 solutions (2 theta_{2,4})
  poses[2] = Pose3(rotation, (Point3() << 0.4, -0.150, 0.4)
                                 .finished());  // 2 solutions (2 theta_{4, })

  // Add 3 poses
  roadmap.addPoseNodes(poses);

  size_t theta7_samples = 4;
  roadmap.computeStateSolutions(theta7_samples);

  // Case with very similar values
  Pose3 simple_pose =
      Pose3(Rot3::identity(), (Point3() << 0.4, -0.150+0.01, 0.4).finished());
  std::vector<size_t> simple_actualnodes =
      roadmap.findClosestNodesPose(simple_pose);

  std::vector<size_t> simple_expectednodes = {10,11};

  EXPECT(assert_equal(simple_expectednodes, simple_actualnodes))

  // Case with more random values
  Pose3 pose = Pose3(Rot3::identity(), (Point3() << 0.05, -0.4, 0.3).finished());
  std::vector<size_t> actualnodes = roadmap.findClosestNodesPose(pose);

  std::vector<size_t> expectednodes(6);
  for (int i = 0; i < 6; ++i) expectednodes[i] = i;

  EXPECT(assert_equal(expectednodes, actualnodes))
}
/*
// need to check the differentpossible paths that don't end up in the same place
// need to create tests for the case with waypoints and levels and stuff
// need to create tests for A* with heuristics
TEST(RoadMap, findWaypoints) {  // find shortest path to some pose (meaning many
                                // possible end points)

  // Probably will have to build different graph than previous one? A bit more
  // complex?

  RoadMap roadmap;

  std::vector<Pose3> poses(5);
  poses[0] = Pose3(Rot3::identity(), (Point3() << 0, 0.050, 0.25).finished());
  poses[1] = Pose3(Rot3::identity(), (Point3() << 0, 0.075, 0.25).finished());
  poses[2] = Pose3(Rot3::identity(), (Point3() << 0, 0.100, 0.25).finished());
  poses[3] = Pose3(Rot3::identity(), (Point3() << 0, 0.125, 0.25).finished());
  poses[4] = Pose3(Rot3::identity(), (Point3() << 0, 0.150, 0.25).finished());

  // Add 3 poses
  roadmap.addPoseNodes(poses);

  size_t theta7_samples = 2;
  roadmap.computeStateSolutions(theta7_samples);

  roadmap.set_num_maxpaths(1);
  size_t start_node = 8;
  size_t end_pose_idx = 4;
  {
    roadmap.set_threshold(0.7);
    roadmap.createGraph();
    std::vector<std::vector<size_t>> actual_waypoints =
        roadmap.findWaypoints(start_node, end_pose_idx);
    std::vector<std::vector<size_t>> expected_waypoints = {{8, 60}};
    EXPECT(assert_equal(expected_waypoints.size(), actual_waypoints.size()))
    for (size_t i = 0; i < expected_waypoints.size(); ++i) {
      EXPECT(assert_equal(expected_waypoints[i], actual_waypoints[i]))
    }
  }
  {
    roadmap.set_threshold(0.5);
    roadmap.createGraph();
    std::vector<std::vector<size_t>> actual_waypoints =
        roadmap.findWaypoints(start_node, end_pose_idx);
    std::vector<std::vector<size_t>> expected_waypoints = {{8, 24, 60}};
    EXPECT(assert_equal(expected_waypoints.size(), actual_waypoints.size()))
    for (size_t i = 0; i < expected_waypoints.size(); ++i) {
      EXPECT(assert_equal(expected_waypoints[i], actual_waypoints[i]))
    }
  }
  {
    roadmap.set_threshold(0.3);
    roadmap.createGraph();
    std::vector<std::vector<size_t>> actual_waypoints =
        roadmap.findWaypoints(start_node, end_pose_idx);
    std::vector<std::vector<size_t>> expected_waypoints = {{8, 24, 36, 60}};
    EXPECT(assert_equal(expected_waypoints.size(), actual_waypoints.size()))
    for (size_t i = 0; i < expected_waypoints.size(); ++i) {
      EXPECT(assert_equal(expected_waypoints[i], actual_waypoints[i]))
    }
  }
  {
    roadmap.set_threshold(0.2);
    roadmap.createGraph();
    std::vector<std::vector<size_t>> actual_waypoints =
        roadmap.findWaypoints(start_node, end_pose_idx);
    std::vector<std::vector<size_t>> expected_waypoints = {{8, 24, 36, 48, 60}};
    EXPECT(assert_equal(expected_waypoints.size(), actual_waypoints.size()))
    for (size_t i = 0; i < expected_waypoints.size(); ++i) {
      EXPECT(assert_equal(expected_waypoints[i], actual_waypoints[i]))
    }
  }
}
*/


TEST(RoadMap, findPath) {
  RoadMap roadmap;

  // Create graph from nodes and check if Dijsktra works
  std::vector<Vector7> states(6);
  states[0] << 2, 0.5, 1, -0.5, 0, 0, 0;
  states[1] << 2, 0.5, 1, -0.5, 0.3, 0, 0.1;
  states[2] << 2, 0.5, 1, -0.5, 0.6, 0.05, 0;
  states[3] << 2, 0.5, 1, -0.5, 1.2, 0, 0;
  states[4] << 2, 0.5, 1, -0.5, 0.3, 0, -0.15;
  states[5] << 2, 0.5, 1, -0.5, 0.9, 0, 0;

  roadmap.set_threshold(0.5);
  roadmap.addStateNodes(states).createGraph();
  std::vector<std::vector<size_t>> stagepointset(2);
  stagepointset[0] = {0};
  stagepointset[1] = {3};

  roadmap.set_num_maxpaths(1);
  Heuristic h;
  std::vector<std::vector<size_t>> paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(1, paths.size()))
  std::vector<size_t> expected_path = {0, 1, 2, 5, 3};
  EXPECT(assert_equal(expected_path, paths[0]))

  roadmap.set_num_maxpaths(2);
  h = Heuristic();
  paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(1, paths.size()))
  EXPECT(assert_equal(expected_path, paths[0]))

  // Add more nodes and another possible end node
  std::vector<Vector7> more_states(6);
  more_states[0] << 2, 0.5, 1, -0.5, 0.3, 0, 0.5;
  more_states[1] << 2, 0.5, 1, -0.5, 0.7, 0, 0.5;
  more_states[2] << 2, 0.5, 1, -0.5, 1.0, 0, 0.5;
  more_states[3] << 2, 0.5, 1, -0.5, 1.2, 0, 0.4;
  more_states[4] << 2, 0.5, 1, -0.5, 1.0, 0.5, 0.5;
  more_states[5] << 2, 0.5, 1, -0.5, 1.2, 0.4, 0.5;

  roadmap.set_threshold(0.5);
  roadmap.addStateNodes(more_states).createGraph();

  stagepointset = std::vector<std::vector<size_t>>(2);
  stagepointset[0] = {0};
  stagepointset[1] = {3, 11};

  roadmap.set_num_maxpaths(1);
  h = Heuristic();
  paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(1, paths.size()))

  std::vector<std::vector<size_t>> expected_paths(2);
  expected_paths[0] = {0, 1, 2, 5, 3};
  expected_paths[1] = {0, 1, 2, 5, 9, 11};
  EXPECT(assert_equal(expected_paths[0], paths[0]))

  roadmap.set_num_maxpaths(2);
  h = Heuristic();
  paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(2, paths.size()))
  for (size_t i = 0; i < 2; i++) {
    EXPECT(assert_equal(expected_paths[i], paths[i]))
  }

  // Add an intermediate stage set
  std::vector<Vector7> some_more_states(3);
  some_more_states[0] << 2, 0.5, 1, -1.0, 1.0, 0.5, 0.5;
  some_more_states[1] << 2, 0.5, 1, -1.3, 1.0, 0.5, 0.5;
  some_more_states[2] << 2, 0.5, 1, -1.6, 1.0, 0.5, 0.5;

  roadmap.set_threshold(0.5);
  roadmap.addStateNodes(some_more_states).createGraph();

  stagepointset = std::vector<std::vector<size_t>>(3);
  stagepointset[0] = {0};
  stagepointset[1] = {3, 7};
  stagepointset[2] = {11, 14};

  roadmap.set_num_maxpaths(1);
  h = Heuristic();
  paths = roadmap.findPath(stagepointset, &h);

  expected_paths = std::vector<std::vector<size_t>>(2);
  expected_paths[0] = {0, 1, 6, 7, 8, 11};
  expected_paths[1] = {0, 1, 6, 7, 8, 10, 12, 13, 14};

  EXPECT(assert_equal(1, paths.size()))
  EXPECT(assert_equal(expected_paths[0], paths[0]))

  roadmap.set_num_maxpaths(2);
  h = Heuristic();
  paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(2, paths.size()))
  for (size_t i = 0; i < 2; i++) {
    EXPECT(assert_equal(expected_paths[i], paths[i]))
  }

  // Add a new possible starting node
  std::vector<Vector7> even_more_states(3);
  even_more_states[0] << 2, 0.0, 1, -0.5, 1.2, 0, 0;
  even_more_states[1] << 2.2, 0.0, 1, -0.5, 1.2, 0, 0;
  even_more_states[2] << 2.1, 0.0, 1.4, -0.5, 1.2, 0, 0;

  roadmap.set_threshold(0.5);
  roadmap.addStateNodes(even_more_states).createGraph();

  stagepointset = std::vector<std::vector<size_t>>(3);
  stagepointset[0] = {0, 17};
  stagepointset[1] = {3, 7};
  stagepointset[2] = {11, 14};

  roadmap.set_num_maxpaths(1);
  h = Heuristic();
  paths = roadmap.findPath(stagepointset, &h);

  expected_paths = std::vector<std::vector<size_t>>(2);
  expected_paths[0] = {17, 15, 3, 9, 11};
  expected_paths[1] = {0, 1, 6, 7, 8, 10, 12, 13, 14};

  EXPECT(assert_equal(1, paths.size()))
  EXPECT(assert_equal(expected_paths[0], paths[0]))

  roadmap.set_num_maxpaths(2);
  h = Heuristic();
  paths = roadmap.findPath(stagepointset, &h, "~/files/dijsktra.txt");
  EXPECT(assert_equal(2, paths.size()))
  for (size_t i = 0; i < 2; i++) {
    EXPECT(assert_equal(expected_paths[i], paths[i]))
  }
}
/*
TEST(RoadMap, DirectDistance) {
  RoadMap roadmap;

  // Create graph from nodes and check if Dijsktra works
  std::vector<Vector7> states(6);
  states[0] << 2, 0.5, 1, 0, 0, 0, 0;
  states[1] << 2, 0.5, 1, 0, 0.3, 0, 0.1;
  states[2] << 2, 0.5, 1, 0, 0.6, 0.05, 0;
  states[3] << 2, 0.5, 1, 0, 1.2, 0, 0;
  states[4] << 2, 0.5, 1, 0, 0.3, 0, -0.15;
  states[5] << 2, 0.5, 1, 0, 0.9, 0, 0;

  roadmap.set_threshold(0.5);
  roadmap.addStateNodes(states).createGraph();
  std::vector<std::vector<size_t>> stagepointset(2);
  stagepointset[0] = {0};
  stagepointset[1] = {3};

  roadmap.set_num_maxpaths(1);
  DirectDistance h;
  std::vector<std::vector<size_t>> paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(1, paths.size()))
  std::vector<size_t> expected_path = {0, 1, 2, 5, 3};
  EXPECT(assert_equal(expected_path, paths[0]))

  roadmap.set_num_maxpaths(2);
  h = DirectDistance();
  paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(1, paths.size()))
  EXPECT(assert_equal(expected_path, paths[0]))

  // Add more nodes and another possible end node
  std::vector<Vector7> more_states(6);
  more_states[0] << 2, 0.5, 1, 0, 0.3, 0, 0.5;
  more_states[1] << 2, 0.5, 1, 0, 0.7, 0, 0.5;
  more_states[2] << 2, 0.5, 1, 0, 1.0, 0, 0.5;
  more_states[3] << 2, 0.5, 1, 0, 1.2, 0, 0.4;
  more_states[4] << 2, 0.5, 1, 0, 1.0, 0.5, 0.5;
  more_states[5] << 2, 0.5, 1, 0, 1.2, 0.4, 0.5;

  roadmap.set_threshold(0.5);
  roadmap.addStateNodes(more_states).createGraph();

  stagepointset = std::vector<std::vector<size_t>>(2);
  stagepointset[0] = {0};
  stagepointset[1] = {3, 11};

  roadmap.set_num_maxpaths(1);
  h = DirectDistance();
  paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(1, paths.size()))

  std::vector<std::vector<size_t>> expected_paths(2);
  expected_paths[0] = {0, 1, 2, 5, 3};
  expected_paths[1] = {0, 1, 2, 5, 9, 11};
  EXPECT(assert_equal(expected_paths[0], paths[0]))

  roadmap.set_num_maxpaths(2);
  h = DirectDistance();
  paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(2, paths.size()))
  for (size_t i = 0; i < 2; i++) {
    EXPECT(assert_equal(expected_paths[i], paths[i]))
  }

  // Add an intermediate stage set
  std::vector<Vector7> some_more_states(3);
  some_more_states[0] << 2, 0.5, 1, 0.5, 1.0, 0.5, 0.5;
  some_more_states[1] << 2, 0.5, 1, 0.8, 1.0, 0.5, 0.5;
  some_more_states[2] << 2, 0.5, 1, 1.1, 1.0, 0.5, 0.5;

  roadmap.set_threshold(0.5);
  roadmap.addStateNodes(some_more_states).createGraph();

  stagepointset = std::vector<std::vector<size_t>>(3);
  stagepointset[0] = {0};
  stagepointset[1] = {3, 7};
  stagepointset[2] = {11, 14};

  roadmap.set_num_maxpaths(1);
  h = DirectDistance();
  paths = roadmap.findPath(stagepointset, &h);

  expected_paths = std::vector<std::vector<size_t>>(2);
  expected_paths[0] = {0, 1, 6, 7, 8, 11};
  expected_paths[1] = {0, 1, 6, 7, 8, 10, 12, 13, 14};

  EXPECT(assert_equal(1, paths.size()))
  EXPECT(assert_equal(expected_paths[0], paths[0]))

  roadmap.set_num_maxpaths(2);
  h = DirectDistance();
  paths = roadmap.findPath(stagepointset, &h);
  EXPECT(assert_equal(2, paths.size()))
  for (size_t i = 0; i < 2; i++) {
    EXPECT(assert_equal(expected_paths[i], paths[i]))
  }

  // Add a new possible starting node
  std::vector<Vector7> even_more_states(3);
  even_more_states[0] << 2, 0.0, 1, 0, 1.2, 0, 0;
  even_more_states[1] << 2.2, 0.0, 1, 0, 1.2, 0, 0;
  even_more_states[2] << 2.1, 0.0, 1.4, 0, 1.2, 0, 0;

  roadmap.set_threshold(0.5);
  roadmap.addStateNodes(even_more_states).createGraph();

  stagepointset = std::vector<std::vector<size_t>>(3);
  stagepointset[0] = {0, 17};
  stagepointset[1] = {3, 7};
  stagepointset[2] = {11, 14};

  roadmap.set_num_maxpaths(1);
  h = DirectDistance();
  paths = roadmap.findPath(stagepointset, &h);

  expected_paths = std::vector<std::vector<size_t>>(2);
  expected_paths[0] = {17, 15, 3, 9, 11};
  expected_paths[1] = {0, 1, 6, 7, 8, 10, 12, 13, 14};

  EXPECT(assert_equal(1, paths.size()))
  EXPECT(assert_equal(expected_paths[0], paths[0]))

  roadmap.set_num_maxpaths(2);
  h = DirectDistance();
  paths = roadmap.findPath(stagepointset, &h, "~/files/directdist.txt");
  EXPECT(assert_equal(2, paths.size()))
  for (size_t i = 0; i < 2; i++) {
    EXPECT(assert_equal(expected_paths[i], paths[i]))
  }
}
*/
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}