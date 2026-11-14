/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testScateHardware.cpp
 * @brief Hardware mode of SCATE: request limits and sensing, assessRun, and
 * the HardwareSession driven like the executor drives it.
 * @author Harrison Jenkins
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include "hardware_session.h"

using namespace scate;

namespace {

/// The default UR10e experiment on the MK 111 rig, with joint encoders only.
const char* kHardwareRequest =
    "robot ur10e\n"
    "q_start 0 -1.5708 1.5708 -1.5708 -1.5708 0\n"
    "q_goal 1.6 -1.2 1.3 -1.9 -1.3 0.4\n"
    "obstacle 0.375494351 1.17508458 0.673015389 -0.0091315195 -0.0285764826 0 0.08\n"
    "obstacle 0.295916119 0.190900169 0.922017319 0 0 -0.02 0.08\n"
    "sigmas 0 3\n"
    "plane wall 0 0 1 -0.05\n"
    "plane floor 0 -1 0 -0.975\n"
    "sensing encoders\n"
    "hardware_sigma 3\n";

LiveRequest parse(const std::string& text) {
  std::istringstream in(text);
  return parseLiveRequest(in);
}

std::string readFile(const std::string& path) {
  std::ifstream file(path);
  return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

size_t count(const std::string& text, const std::string& what) {
  size_t found = 0;
  for (size_t at = text.find(what); at != std::string::npos; at = text.find(what, at + 1)) ++found;
  return found;
}

}  // namespace

// The hardware items of a request: limits, sensing, executed-control sigma, executed run.
TEST(HardwareRequest, parsesLimitsAndSensing) {
  const LiveRequest request = parse(std::string(kHardwareRequest) +
                                    "v_limit 0.3\nu_limit 0.6\nsigma_executed_control 0.02\n");
  EXPECT(request.sensing == "encoders");
  EXPECT_DOUBLES_EQUAL(0.3, *request.v_limit, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.6, *request.u_limit, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.02, request.sigma_executed_control, 1e-12);
  EXPECT_DOUBLES_EQUAL(3.0, *request.hardware_sigma, 1e-12);
  const LiveSetup setup = makeLiveSetup(liveRobot("ur10e"), request);
  EXPECT(gtsam::assert_equal(Vector(Vector::Constant(6, 0.3)), setup.spec.v_limit));
  EXPECT(gtsam::assert_equal(Vector(Vector::Constant(6, 0.6)), setup.spec.u_limit));
  // Defaults: markers, the planner's own limits.
  const LiveRequest plain = parse("q_start 0 0 0 0 0 0\nq_goal 0 0 0 0 0 1\nobstacle 1 1 1 0 0 0 0.1\n");
  EXPECT(plain.sensing == "markers");
  EXPECT(!plain.v_limit && !plain.u_limit && !plain.hardware_sigma);
  for (const std::string bad : {"v_limit 0\n", "v_limit 1.5\n", "u_limit 2.5\n", "u_limit -1\n",
                                "v_limit 0.3 0.4\n", "sensing vicon\n", "hardware_sigma 0.3\n",
                                "sigma_executed_control 0\n"}) {
    CHECK_EXCEPTION(parse(std::string(kHardwareRequest) + bad), std::invalid_argument);
  }
}

// assessRun flags each limit it checks, and passes a clean run.
TEST(AssessRun, flagsEachLimit) {
  GraphSpec spec;
  spec.q_start = Vector::Zero(2);
  spec.q_goal = Vector::Zero(2);
  spec.q_lower = Vector::Constant(2, -1.0);
  spec.q_upper = Vector::Constant(2, 1.0);
  spec.v_limit = Vector::Constant(2, 0.5);
  spec.u_limit = Vector::Constant(2, 1.0);
  spec.num_poses = 3;
  spec.dt = 1.0;
  ClosedLoopResult result;
  result.q = {Vector::Zero(2), Vector::Zero(2), Vector::Zero(2)};
  result.v = result.q;
  result.u = {Vector::Zero(2), Vector::Zero(2)};
  result.min_environment_clearance = 0.05;
  result.min_obstacle_clearance = 0.04;
  result.min_self_clearance = 0.03;
  EXPECT(assessRun(result, spec).safe);

  ClosedLoopResult fast = result;  // 0.6 rad/s reached at the end of interval 0
  fast.u[0] << 0.6, 0.0;
  fast.v[1] << 0.6, 0.0;
  fast.q[1] << 0.3, 0.0;
  fast.u[1] << -0.6, 0.0;
  fast.q[2] << 0.6, 0.0;
  const RunSafety too_fast = assessRun(fast, spec);
  EXPECT(!too_fast.safe);
  EXPECT_DOUBLES_EQUAL(1.2, too_fast.max_velocity_ratio, 1e-9);
  EXPECT_LONGS_EQUAL(1, too_fast.reasons.size());

  ClosedLoopResult hard = result;
  hard.u[1] << 0.0, -1.2;
  EXPECT_DOUBLES_EQUAL(1.2, assessRun(hard, spec).max_acceleration_ratio, 1e-12);
  EXPECT(!assessRun(hard, spec).safe);

  ClosedLoopResult close = result;
  close.min_environment_clearance = 0.005;
  EXPECT(!assessRun(close, spec).safe);
  ClosedLoopResult hit = result;
  hit.min_obstacle_clearance = -0.01;
  EXPECT(!assessRun(hit, spec).safe);

  ClosedLoopResult out = result;  // the measured trace leaves the joint range
  out.trace_t = {0.0, 1.0};
  out.trace_q = {Vector::Zero(2), Vector::Constant(2, 1.1)};
  out.trace_v = {Vector::Zero(2), Vector::Zero(2)};
  const RunSafety outside = assessRun(out, spec);
  EXPECT(!outside.safe);
  EXPECT_DOUBLES_EQUAL(-0.1, outside.joint_limit_margin, 1e-12);
}

// A lower velocity limit is honored by the planned (ghost) motion.
TEST(HardwareSession, velocityLimitIsHonored) {
  HardwareSession session(parse(std::string(kHardwareRequest) + "v_limit 0.3\n"));
  const ClosedLoopResult& ghost = session.runGhost();
  double fastest = 0.0;
  for (const Vector& v : ghost.v) fastest = std::max(fastest, v.cwiseAbs().maxCoeff());
  std::cout << "  [diag] v_limit 0.3: fastest knot speed " << fastest << " rad/s, ratio "
            << ghost.safety->max_velocity_ratio << ", goal error " << ghost.goal_error << '\n';
  EXPECT(ghost.safety->max_velocity_ratio <= 1.05);
  EXPECT(fastest <= 0.3 * 1.05);
}

/**
 * The session driven like the executor drives it, with a perfect arm: the
 * ghost run matches the live solve of the same run, and executing each new
 * plan's control exactly reaches the goal clear of the rig, with a replay
 * holding the hardware run and the ghost run.
 */
TEST(HardwareSession, executorLoopWithPerfectArm) {
  const LiveRequest request = parse(kHardwareRequest);
  HardwareSession session(request);
  EXPECT_DOUBLES_EQUAL(3.0, session.sigma(), 0.0);
  const ClosedLoopResult& ghost = session.runGhost();
  EXPECT(ghost.safety && ghost.safety->safe);

  // The live app's solve of the same request gives the same run.
  const auto live = runLiveRequest(request, "/tmp/testScateHardware_live.json", nullptr);
  const ClosedLoopResult& same = live.at(1).second;  // sigmas 0 3
  EXPECT_DOUBLES_EQUAL(same.min_obstacle_clearance, ghost.min_obstacle_clearance, 1e-12);
  EXPECT_DOUBLES_EQUAL(same.goal_error, ghost.goal_error, 1e-12);
  EXPECT(gtsam::assert_equal(same.q.back(), ghost.q.back(), 1e-12));

  session.start();
  const GraphSpec& spec = session.spec();
  const int last = session.numIntervals();
  const double dt = spec.dt;
  Vector q = spec.q_start, v = Vector::Zero(6);
  for (int k = 0; k <= last; ++k) {
    const double t = k * dt;
    std::vector<gtsam::Point3> centers;
    for (const MovingObstacle& obstacle : request.obstacles) centers.push_back(obstacle.center(t));
    session.measure(k, t, q, v, centers);
    if (k == last) break;
    const Vector u = session.planner().control(k);
    for (int s = 0; s < 10; ++s) {  // the executor's trace at 10 Hz
      const double tau = dt * s / 10;
      session.addTrace(t + tau, q + tau * v + 0.5 * tau * tau * u, v + tau * u);
    }
    q = q + dt * v + 0.5 * dt * dt * u;
    v = v + dt * u;
  }
  session.addTrace(last * dt, q, v);
  CHECK_EXCEPTION(session.addTrace(0.0, q, v), std::invalid_argument);

  const ClosedLoopResult hardware = session.hardwareResult();
  std::cout << "  [diag] perfect arm: goal error " << hardware.goal_error << " rad, wall/floor "
            << 100 * hardware.min_environment_clearance << " cm, obstacles "
            << 100 * hardware.min_obstacle_clearance << " cm, slowest step "
            << hardware.max_solve_ms << " ms\n";
  EXPECT(!hardware.collided());
  EXPECT(hardware.safety->safe);
  EXPECT(hardware.goal_error < 0.05);
  EXPECT(hardware.min_environment_clearance > 0.01);
  EXPECT_LONGS_EQUAL(last + 1, hardware.steps.size());

  session.writeReplay("/tmp/testScateHardware.json", hardware, "Hardware run");
  const std::string replay = readFile("/tmp/testScateHardware.json");
  EXPECT(replay.find("\"name\":\"online\",\"label\":\"Hardware run\"") <
         replay.find("\"name\":\"ghost\""));
  EXPECT(replay.find("\"name\":\"ghost\"") != std::string::npos);
  EXPECT_LONGS_EQUAL(2, count(replay, "\"safety\":{\"safe\":true"));
  EXPECT(replay.find("\"v_limit\":[0.5,0.5,0.5,0.5,0.5,0.5]") != std::string::npos);
  // Both runs have N * substeps + 1 frames.
  EXPECT_LONGS_EQUAL(2 * (last * 10 + 1), count(replay, "{\"t\":"));
}

// A run stopped early still makes a replay; bad readings are refused.
TEST(HardwareSession, stoppedEarlyAndBadInput) {
  CHECK_EXCEPTION(HardwareSession(parse(std::string(kHardwareRequest) + "sensing markers\n")),
                  std::invalid_argument);
  HardwareSession session(parse(kHardwareRequest));
  CHECK_EXCEPTION(session.measure(0, 0.0, Vector::Zero(6), Vector::Zero(6), {}), std::logic_error);
  session.start();
  const Vector q = session.spec().q_start, v = Vector::Zero(6);
  CHECK_EXCEPTION(session.measure(0, 0.0, q, v, {gtsam::Point3(1, 1, 1)}), std::invalid_argument);
  CHECK_EXCEPTION(session.measure(0, 0.0, Vector::Zero(5), v,
                                  {gtsam::Point3(1, 1, 1), gtsam::Point3(1, 1, 2)}),
                  std::invalid_argument);
  session.measure(0, 0.0, q, v, {gtsam::Point3(1, 1, 1), gtsam::Point3(1, 1, 2)});
  session.addTrace(0.0, q, v);
  session.addTrace(0.4, q, v);
  const ClosedLoopResult stopped = session.hardwareResult();
  EXPECT_LONGS_EQUAL(session.numIntervals() + 1, stopped.q.size());
  EXPECT_LONGS_EQUAL(session.numIntervals(), stopped.u.size());
  EXPECT(stopped.goal_error > 0.5);  // it never left the start
  session.writeReplay("/tmp/testScateHardware_stopped.json", stopped, "Hardware run (stopped early)");
  EXPECT(readFile("/tmp/testScateHardware_stopped.json").find("stopped early") != std::string::npos);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
