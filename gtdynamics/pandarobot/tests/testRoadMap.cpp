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

TEST(RoadMap, createNodes) {
  RoadMap roadmap;

  std::vector<Pose3> poses(3);
  poses[0] = Pose3(Rot3::identity(), (Point3() << 0, 0, 0.25).finished());
  poses[1] = Pose3(Rot3::identity(), (Point3() << 0, 0.1, 0.25).finished());
  poses[2] = Pose3(Rot3::identity(), (Point3() << 0, -0.1, 0.25).finished());

  // Add 3 poses
  std::vector<Pose3> result_poses = roadmap.addPoseNodes(poses).getposes();

  // Check if poses match
  EXPECT(assert_equal(poses, result_poses, 1e-5))

  // Compute solutions with a discretization on theta (uniform atm)
  size_t theta7_samples = 2;
  roadmap.computeStateSolutions(theta7_samples);

  // Get list of nodes
  std::vector<Vector7> statenodes = roadmap.getstatenodes();

  EXPECT(assert_equal(40, statenodes.size()))

  std::vector<Vector7> expected_statenodes(7);
  expected_statenodes[0] << -2.17582, -2.2104, -7.10543e-15, 2.3909, 0,
      -1.45971, -0.96577;
  expected_statenodes[1] << 2.17582, 0.581145, 3.14159, 2.3909, -3.14159,
      -0.169549, 0.96577;
  expected_statenodes[2] << -0.96577, -0.581145, -3.55271e-15, 2.3909, -3.14159,
      -0.169549, 0.96577;
  expected_statenodes[3] << 1.28794, -0.577196, 2.12827, 2.88724, -0.756875,
      0.740109, -0.96577;
  expected_statenodes[4] << 1.56129, 0.646345, -2.8722, 2.14819, 2.71426,
      -0.397121, 0.96577;
  expected_statenodes[5] << 1.1291, -0.113325, -2.98946, -3.0822, 0.314957,
      0.0553512, -0.96577;
  expected_statenodes[6] << -0.697389, 0.709638, -0.946245, 2.88724, 2.38472,
      -0.878473, 0.96577;

  std::vector<size_t> indicestotest = {3, 8, 9, 17, 20, 33, 38};
  std::vector<size_t> expectedposeindex = {0, 0, 0, 1, 1, 2, 2};
  std::vector<size_t> posestatelist_index = {3, 8, 9, 1, 4, 5, 10};

  for (size_t i = 0; i < 7; ++i) {
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
  poses[0] = Pose3(Rot3::identity(), (Point3() << 0, 0.05, 0.25).finished());
  poses[1] = Pose3(Rot3::identity(), (Point3() << 0, 0.1, 0.25).finished());
  poses[2] = Pose3(Rot3::identity(), (Point3() << 0, 0.15, 0.25).finished());

  // Add 3 poses
  size_t theta7_samples = 2;
  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);

  double distance_threshold = 0.7;
  roadmap.set_threshold(distance_threshold);
  roadmap.createGraph();
  std::vector<std::vector<RoadMap::Edge>> adjacencylist =
      roadmap.getadjacencylist();

  EXPECT(assert_equal(40, adjacencylist.size()))

  std::vector<std::vector<RoadMap::Edge>> expected_adjacencylist(40);
  {
    expected_adjacencylist[4] = {{0.6932301580305058, 16}};
    expected_adjacencylist[5] = {{0.6932344698541757, 17}};
    expected_adjacencylist[8] = {{0.35573285669586385, 20},
                                 {0.637921630322252, 32}};
    expected_adjacencylist[9] = {{0.35574164483371934, 21},
                                 {0.6379242686259868, 33}};
    expected_adjacencylist[16] = {{0.6932301580305058, 4}};
    expected_adjacencylist[17] = {{0.6932344698541757, 5}};
    expected_adjacencylist[18] = {{0.6347320717775335, 30}};
    expected_adjacencylist[19] = {{0.634735643881766, 31}};
    expected_adjacencylist[20] = {{0.35573285669586385, 8},
                                  {0.28634031584113334, 32}};
    expected_adjacencylist[21] = {{0.35574164483371934, 9},
                                  {0.28633541701473114, 33}};
    expected_adjacencylist[22] = {{0.6321761854546563, 34}};
    expected_adjacencylist[24] = {{0.42491481683751625, 36}};
    expected_adjacencylist[26] = {{0.31833958849788024, 38}};
    expected_adjacencylist[27] = {{0.31833843547394636, 39}};
    expected_adjacencylist[30] = {{0.6347320717775335, 18}};
    expected_adjacencylist[31] = {{0.634735643881766, 19}};
    expected_adjacencylist[32] = {{0.637921630322252, 8},
                                  {0.28634031584113334, 20}};
    expected_adjacencylist[33] = {{0.6379242686259868, 9},
                                  {0.28633541701473114, 21}};
    expected_adjacencylist[34] = {{0.6321761854546563, 22}};
    expected_adjacencylist[36] = {{0.42491481683751625, 24}};
    expected_adjacencylist[38] = {{0.31833958849788024, 26}};
    expected_adjacencylist[39] = {{0.31833843547394636, 27}};
  }

  for (size_t node = 0; node < 40; node++) {
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
  poses[0] = Pose3(Rot3::identity(), (Point3() << 0, 0.05, 0.25).finished());
  poses[1] = Pose3(Rot3::identity(), (Point3() << 0, 0.1, 0.25).finished());
  poses[2] = Pose3(Rot3::identity(), (Point3() << 0, 0.15, 0.25).finished());

  // Add 3 poses
  roadmap.addPoseNodes(poses);

  size_t theta7_samples = 2;
  roadmap.computeStateSolutions(theta7_samples);

  // First one example with just little tweaks from a stored solution
  Vector7 simple_configuration;
  simple_configuration << 2.30654, -2.53725, -2.65202, -2.94484, -2.753,
      -2.35883, 0.96577 + 0.05;

  size_t simple_node = roadmap.findClosestNodeState(simple_configuration);

  EXPECT(assert_equal(39, simple_node))

  // Second: another one "random" one
  Vector7 complex_configuration;
  complex_configuration << 0.5, 0, 0.3, -0.4, 0.2, 2, 0.7;

  size_t complex_node = roadmap.findClosestNodeState(complex_configuration);

  EXPECT(assert_equal(36, complex_node))
}

TEST(RoadMap, findClosestNodesPose) {
  RoadMap roadmap;

  std::vector<Pose3> poses(3);
  poses[0] = Pose3(Rot3::identity(), (Point3() << 0, 0.05, 0.25).finished());
  poses[1] = Pose3(Rot3::identity(), (Point3() << 0, 0.1, 0.25).finished());
  poses[2] = Pose3(Rot3::identity(), (Point3() << 0, 0.15, 0.25).finished());

  // Add 3 poses
  roadmap.addPoseNodes(poses);

  size_t theta7_samples = 2;
  roadmap.computeStateSolutions(theta7_samples);

  // Case with very similar values
  Pose3 simple_pose =
      Pose3(Rot3::identity(), (Point3() << 0, 0.11, 0.25).finished());
  std::vector<size_t> simple_actualnodes =
      roadmap.findClosestNodesPose(simple_pose);

  std::vector<size_t> simple_expectednodes(12);
  for (int i = 0; i < 12; ++i) simple_expectednodes[i] = i + 16;

  EXPECT(assert_equal(simple_expectednodes, simple_actualnodes))

  // Case with more random values
  Pose3 pose = Pose3(Rot3::identity(), (Point3() << 0.05, 0, 0.3).finished());
  std::vector<size_t> actualnodes = roadmap.findClosestNodesPose(pose);

  std::vector<size_t> expectednodes(16);
  for (int i = 0; i < 16; ++i) expectednodes[i] = i;

  EXPECT(assert_equal(expectednodes, actualnodes))
}

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

TEST(RoadMap, findPath) {
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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}