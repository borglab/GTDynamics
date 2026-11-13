/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testScateLive.cpp
 * @brief Tests of the live SCATE app's solver (live_scenario.h): requests,
 * robot descriptions the browser poses arms from, UR5 motion capture and a
 * live solve.
 * @author Harrison Jenkins
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <Eigen/SVD>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>

#include "live_scenario.h"

using namespace scate;

namespace {

Vector joints(double a, double b, double c, double d, double e, double f) {
  Vector q(6);
  q << a, b, c, d, e, f;
  return q;
}

/// Link poses from a description's screw axes and rest frames, as the browser computes them.
std::vector<gtsam::Pose3> productOfExponentials(const std::vector<gtsam::Vector6>& axes,
                                                const std::vector<gtsam::Pose3>& rest,
                                                const Vector& q) {
  std::vector<gtsam::Pose3> poses{rest[0]};
  gtsam::Pose3 motion;
  for (size_t j = 0; j < axes.size(); ++j) {
    motion = motion * gtsam::Pose3::Expmap(axes[j] * q(j));
    poses.push_back(motion * rest[j + 1]);
  }
  return poses;
}

const char* kRequest =
    "# a comment\n"
    "robot ur5\n"
    "q_start 0 -1.5708 1.5708 -1.5708 -1.5708 0\n"
    "q_goal 1.6 -1.2 1.3 -1.9 -1.3 0.4  # trailing comment\n"
    "obstacle 0.3 0.9 0.44 -0.01 -0.03 0 0.08\n"
    "obstacle 0.2 0.1 0.65 0 0 -0.02 0.06\n"
    "sigmas 0 3\n"
    "seed 4\n";

}  // namespace

// A request is read item by item.
TEST(LiveRequest, parses) {
  std::istringstream in(kRequest);
  const LiveRequest request = parseLiveRequest(in);
  EXPECT(request.robot == "ur5");
  EXPECT(gtsam::assert_equal(joints(1.6, -1.2, 1.3, -1.9, -1.3, 0.4), request.q_goal));
  EXPECT_LONGS_EQUAL(2, request.obstacles.size());
  EXPECT(gtsam::assert_equal(gtsam::Vector3(0, 0, -0.02), request.obstacles[1].velocity));
  EXPECT_DOUBLES_EQUAL(0.06, request.obstacles[1].radius, 1e-12);
  EXPECT_LONGS_EQUAL(2, request.sigmas.size());
  EXPECT_LONGS_EQUAL(4, request.seed);
  EXPECT(request.environment.empty());
}

// Walls and floor are optional "plane" lines; their normals must have unit length.
TEST(LiveRequest, parsesPlanes) {
  std::istringstream in(std::string(kRequest) +
                        "plane wall 0 0 1 -0.05\nplane floor 0 -1 0 -0.975\n");
  const LiveRequest request = parseLiveRequest(in);
  EXPECT_LONGS_EQUAL(2, request.environment.size());
  EXPECT(request.environment[1].name == "floor");
  EXPECT(gtsam::assert_equal(gtsam::Vector3(0, -1, 0), request.environment[1].normal));
  EXPECT_DOUBLES_EQUAL(-0.975, request.environment[1].offset, 1e-12);
  for (const std::string bad : {"plane wall 0 0 2 0\n", "plane wall 0 0 1\n", "plane\n",
                                "plane wall 0 0 1 x\n"}) {
    std::istringstream bad_in(std::string(kRequest) + bad);
    CHECK_EXCEPTION(parseLiveRequest(bad_in), std::invalid_argument);
  }
}

// Malformed requests are refused with a message, not solved.
TEST(LiveRequest, refusesMalformedRequests) {
  for (const std::string bad :
       {"robot ur5\nq_start 0 0 0 0 0 0\nq_goal 0 0 0 0 0 0\n",                       // no obstacle
        "q_start 0 0 0 0 0 0\nq_goal 0 0 0\nobstacle 0 0 1 0 0 0 0.1\n",              // lengths
        "q_start 0 0 0 0 0 0\nq_goal 0 0 0 0 0 0\nobstacle 0 0 1 0 0 0\n",            // 6 values
        "q_start 0 0 0 0 0 0\nq_goal 0 0 0 0 0 0\nobstacle 0 0 1 0 0 0 0.1x\n",       // not a number
        "q_start 0 0 0 0 0 0\nq_goal 0 0 0 0 0 0\nobstacle 0 0 1 0 0 0 0.1\nsigmas -1\n",
        "q_start 0 0 0 0 0 0\nq_goal 0 0 0 0 0 0\nobstacle 0 0 1 0 0 0 0.1\nspeed 3\n"}) {
    std::istringstream in(bad);
    CHECK_EXCEPTION(parseLiveRequest(in), std::invalid_argument);
  }
  CHECK_EXCEPTION(liveRobot("ur3"), std::invalid_argument);
}

// The browser's product of exponentials over the description's screw axes
// reproduces the link frames the replay records, for both robots.
TEST(LiveDescription, screwAxesReproduceTheLinkFrames) {
  for (const std::string key : {"ur5", "ur10e"}) {
    const LiveRobot live = liveRobot(key);
    const LiveSetup setup = makeLiveSetup(live, startJoints(), sweepGoalJoints());
    const auto axes = spaceScrewAxes(*setup.model, 6);
    std::vector<gtsam::Pose3> rest;
    for (const auto& frame : setup.model->linkFrames(Vector::Zero(6))) rest.push_back(frame.second);
    for (const Vector& q : {startJoints(), sweepGoalJoints(), joints(0.7, -2.0, 2.4, 0.3, 1.9, -2.8)}) {
      const auto expected = setup.model->linkFrames(q);
      const auto actual = productOfExponentials(axes, rest, q);
      for (size_t l = 0; l < expected.size(); ++l) {
        EXPECT(gtsam::assert_equal(expected[l].second, actual[l], 1e-7));
      }
    }
    for (const auto& axis : axes) EXPECT_DOUBLES_EQUAL(1.0, axis.head<3>().norm(), 1e-9);
  }
}

// The description holds what the app needs, and its default UR10e experiment
// is the recorded one.
TEST(LiveDescription, writesTheDefaultExperiment) {
  const LiveRobot live = liveRobot("ur10e");
  const LiveDefaults defaults = liveDefaults(live);
  EXPECT_LONGS_EQUAL(2, defaults.obstacles.size());
  EXPECT(gtsam::assert_equal(startJoints(), defaults.q_start));
  std::ostringstream out;
  writeLiveDescription(out, live, defaults);
  const std::string json = out.str();
  for (const std::string key : {"\"links\":[", "\"joints\":[", "\"screw\":[", "\"spheres\":[",
                                "\"tool_sphere\":", "\"workspace\":", "\"defaults\":",
                                "\"name\":\"UR10e\"", "\"rig\":{\"name\":\"MK 111 wall and floor\"",
                                "{\"name\":\"floor\",\"normal\":[0,-1,0],\"offset\":-0.975}"}) {
    EXPECT(json.find(key) != std::string::npos);
  }
  // The UR5 has no rig.
  std::ostringstream ur5;
  const LiveRobot live5 = liveRobot("ur5");
  writeLiveDescription(ur5, live5, liveDefaults(live5));
  EXPECT(ur5.str().find("\"rig\":null") != std::string::npos);
}

// Every UR5 joint angle moves a marker, at the start, the goal and a bent pose.
TEST(LiveRobot, ur5MarkersObserveEveryJoint) {
  const LiveRobot live = liveRobot("ur5");
  EXPECT_LONGS_EQUAL(8, live.sphere_model.markers.size());
  const LiveSetup setup = makeLiveSetup(live, startJoints(), sweepGoalJoints());
  for (const Vector& q : {startJoints(), sweepGoalJoints(), joints(0.4, -0.9, 1.0, -1.1, 0.8, 1.2)}) {
    gtsam::Matrix H;
    setup.model->markerPositions(q, &H);
    const Eigen::JacobiSVD<gtsam::Matrix> svd(H);
    const double smallest = svd.singularValues()(5);
    std::cout << "  [diag] UR5 marker Jacobian smallest singular value " << smallest << " m/rad\n";
    EXPECT(smallest > 0.05);
  }
}

// A live UR5 solve with the default experiment avoids the obstacles and
// writes a replay naming the robot, with the default sigma as the primary run.
TEST(LiveSolve, ur5DefaultExperiment) {
  const LiveRobot live = liveRobot("ur5");
  const LiveDefaults defaults = liveDefaults(live);
  LiveRequest request;
  request.robot = "ur5";
  request.q_start = defaults.q_start;
  request.q_goal = defaults.q_goal;
  request.obstacles = defaults.obstacles;
  request.sigmas = {0.0, 3.0};
  std::ostringstream progress;
  const auto runs = runLiveRequest(request, "/tmp/testScateLive_replay.json", &progress);
  EXPECT_LONGS_EQUAL(2, runs.size());
  for (const auto& run : runs) {
    EXPECT(!run.second.collided());
    EXPECT(run.second.goal_error < 0.1);
  }
  EXPECT(progress.str().find("PROGRESS 2 2") != std::string::npos);
  std::ifstream file("/tmp/testScateLive_replay.json");
  const std::string replay((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT(replay.find("\"robot\":\"UR5\"") != std::string::npos);
  EXPECT(replay.find("\"name\":\"online\"") < replay.find("\"name\":\"cost_off\""));
  EXPECT(replay.find("scate_live (GTDynamics UR5)") != std::string::npos);
}

/**
 * The MK 111 rig on the UR10e: a sweep whose straight joint-space path runs
 * the stretched-out arm through the floor (pan 0 -> pi with the arm level,
 * passing over base +y, where the floor is 0.975 m away) is planned and
 * executed clear of the wall and floor, and the replay records both planes.
 */
TEST(LiveSolve, ur10eKeepsClearOfTheRig) {
  const LiveRobot live = liveRobot("ur10e");
  LiveRequest request;
  request.robot = "ur10e";
  request.q_start = joints(0.0, -0.3, 0.0, -1.5708, -1.5708, 0.0);
  request.q_goal = joints(3.1, -0.3, 0.0, -1.5708, -1.5708, 0.0);
  MovingObstacle far_away;  // a request needs one; this one stays out of reach
  far_away.start = gtsam::Point3(-1.4, -1.4, 1.6);
  far_away.radius = 0.05;
  request.obstacles = {far_away};
  request.sigmas = {0.0};
  const LiveSetup free_space = makeLiveSetup(live, request.q_start, request.q_goal);
  // The straight-line initial guess crosses the floor and the ends do not.
  double guess_min = std::numeric_limits<double>::infinity();
  SphereCollisionParams with_rig = live.params;
  with_rig.environment = live.rig;
  const SphereCollisionModel rig_model(free_space.robot, live.sphere_model, with_rig);
  for (int k = 0; k < free_space.spec.num_poses; ++k) {
    guess_min = std::min(guess_min, rig_model.environmentClearance(
                                        free_space.initial.at<Vector>(QKey(k))));
  }
  std::cout << "  [diag] straight guess: wall/floor clearance " << 100 * guess_min << " cm; ends "
            << 100 * rig_model.environmentClearance(request.q_start) << " / "
            << 100 * rig_model.environmentClearance(request.q_goal) << " cm\n";
  EXPECT(guess_min < 0.0);
  EXPECT(rig_model.environmentClearance(request.q_start) > with_rig.obstacle_epsilon);
  EXPECT(rig_model.environmentClearance(request.q_goal) > with_rig.obstacle_epsilon);

  // The initial plan, before any obstacle is seen, already avoids the floor.
  const LiveSetup rig_setup = makeLiveSetup(live, request.q_start, request.q_goal, live.rig);
  ReactivePlanner offline(rig_setup.spec, liveConfig(live, request.obstacles), rig_setup.model,
                          rig_setup.initial);
  offline.planOffline();
  double offline_min = std::numeric_limits<double>::infinity();
  for (int k = 0; k < rig_setup.spec.num_poses; ++k) {
    offline_min = std::min(offline_min, rig_setup.model->environmentClearance(
                                            offline.values().at<Vector>(QKey(k))));
  }
  std::cout << "  [diag] initial plan: wall/floor clearance " << 100 * offline_min << " cm\n";
  EXPECT(offline_min > 0.0);

  request.environment = live.rig;
  const auto runs = runLiveRequest(request, "/tmp/testScateLive_rig.json", nullptr);
  const ClosedLoopResult& result = runs.at(0).second;
  std::cout << "  [diag] executed: wall/floor clearance " << 100 * result.min_environment_clearance
            << " cm, goal error " << result.goal_error << " rad\n";
  EXPECT(result.min_environment_clearance > 0.0);
  EXPECT(!result.collided());
  EXPECT(result.goal_error < 0.1);
  std::ifstream file("/tmp/testScateLive_rig.json");
  const std::string replay((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT(replay.find("\"environment\":[{\"name\":\"wall\"") != std::string::npos);
  EXPECT(replay.find("\"min_environment_clearance\":") != std::string::npos);
}

// A start and goal of the wrong length are refused.
TEST(LiveSolve, refusesWrongJointCounts) {
  CHECK_EXCEPTION(makeLiveSetup(liveRobot("ur10e"), Vector::Zero(5), Vector::Zero(5)),
                  std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
