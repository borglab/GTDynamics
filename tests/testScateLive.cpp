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
                                "\"name\":\"UR10e\""}) {
    EXPECT(json.find(key) != std::string::npos);
  }
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

// A start and goal of the wrong length are refused.
TEST(LiveSolve, refusesWrongJointCounts) {
  CHECK_EXCEPTION(makeLiveSetup(liveRobot("ur10e"), Vector::Zero(5), Vector::Zero(5)),
                  std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
