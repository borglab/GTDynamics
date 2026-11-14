/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file hardware_session.h
 * @author Harrison Jenkins
 *
 * Reactive SCATE on a real arm. The planner runs in its own process
 * (scate_hardware) and an executor drives it: at each state time k dt the
 * executor reads the joint encoders and the obstacles and sends them here;
 * the planner adds the reading, re-plans and answers with the new plan,
 * which the executor turns into joint velocity commands until the next
 * state. Between states the arm follows the latest plan.
 *
 * Before the run, the session simulates the same experiment (the "ghost"
 * run: the planner's closed loop on a simulated double integrator, with the
 * same settings, sensing and noise seed as the live app's solve of that
 * run) and checks it with assessRun. The executor refuses to start unless
 * the ghost run is safe.
 *
 * Sensing is joint encoders only: the hardware has no motion capture yet.
 * After the run, the measured trace becomes a "hardware" run beside the
 * ghost run in one replay, for the app.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "live_scenario.h"

namespace scate {

/// One hardware experiment: the ghost run, then the online steps, then the replay.
class HardwareSession {
 public:
  /// The request must use joint encoders (sensing encoders), as the hardware does.
  explicit HardwareSession(LiveRequest request)
      : request_(std::move(request)), live_(liveRobot(request_.robot)) {
    if (request_.sensing != "encoders") {
      throw std::invalid_argument("a hardware run needs 'sensing encoders' (no motion capture)");
    }
    sigma_ = request_.hardware_sigma ? *request_.hardware_sigma : request_.sigmas.at(0);
    setup_ = makeLiveSetup(live_, request_);
    config_ = runConfig(liveConfig(live_, request_), setup_, sigma_);
    for (const MovingObstacle& obstacle : request_.obstacles) {
      if (!(obstacle.radius > 0.0)) throw std::invalid_argument("obstacle radius must be > 0");
    }
  }

  const LiveRequest& request() const { return request_; }
  const GraphSpec& spec() const { return setup_.spec; }
  const SphereCollisionModel& model() const { return *setup_.model; }
  /// Base-cost sigma of the executed run (0 = off).
  double sigma() const { return sigma_; }
  /// Number of intervals N.
  int numIntervals() const { return setup_.spec.num_poses - 1; }

  /// Simulate the experiment (same as the live app's run of this sigma) and assess it.
  const ClosedLoopResult& runGhost() {
    ReactivePlanner planner(setup_.spec, config_, setup_.model, setup_.initial);
    ghost_ = runReactiveClosedLoop(planner, *setup_.model, liveSimulation(live_, request_));
    ghost_->safety = assessRun(*ghost_, setup_.spec);
    return *ghost_;
  }
  const std::optional<ClosedLoopResult>& ghost() const { return ghost_; }

  /// The initial plan (paper Algorithm 1, no obstacle known yet).
  StepInfo start() {
    if (planner_) throw std::logic_error("start: already started");
    planner_.emplace(setup_.spec, config_, setup_.model, setup_.initial);
    return planner_->planOffline();
  }

  /**
   * The reading at state k (time t from the start, s): measured joints and
   * the obstacle centers. Adds it, re-plans and returns the step.
   */
  StepInfo measure(int k, double t, const Vector& q, const Vector& v,
                   const std::vector<gtsam::Point3>& obstacles) {
    if (!planner_) throw std::logic_error("measure: call start first");
    const long n = setup_.spec.q_start.size();
    if (q.size() != n || v.size() != n) {
      throw std::invalid_argument("measure: q and v need " + std::to_string(n) + " values");
    }
    if (obstacles.size() != request_.obstacles.size()) {
      throw std::invalid_argument("measure: need " + std::to_string(request_.obstacles.size()) +
                                  " obstacle centers");
    }
    ViconMeasurement reading;
    reading.q_encoder = q;
    reading.v_encoder = v;
    for (const gtsam::Point3& center : obstacles) reading.obstacles.emplace_back(center);
    StepInfo info = planner_->update(k, reading);
    knot_t_.push_back(t);
    knot_q_.push_back(q);
    knot_v_.push_back(v);
    knot_obstacles_.push_back(obstacles);
    if (k < numIntervals()) executed_u_.push_back(planner_->control(k));
    steps_.push_back(info);
    return info;
  }

  /// The current plan, for the executor.
  const ReactivePlanner& planner() const {
    if (!planner_) throw std::logic_error("planner: call start first");
    return *planner_;
  }

  /// One measured sample of the arm (t from the start, s), for the replay.
  void addTrace(double t, const Vector& q, const Vector& v) {
    if (!trace_t_.empty() && t < trace_t_.back()) {
      throw std::invalid_argument("addTrace: samples must be in time order");
    }
    trace_t_.push_back(t);
    trace_q_.push_back(q);
    trace_v_.push_back(v);
  }

  /**
   * The executed run as a ClosedLoopResult: the measured states at the knots
   * (padded with the last one if the run stopped early), the executed
   * controls, the planner steps and the measured trace, scored against the
   * obstacles, walls, floor and limits.
   */
  ClosedLoopResult hardwareResult() const {
    ClosedLoopResult result;
    const int last = numIntervals();
    const long n = setup_.spec.q_start.size();
    result.q = knot_q_;
    result.v = knot_v_;
    result.u = executed_u_;
    result.steps = steps_;
    result.obstacle_centers = knot_obstacles_;
    if (result.q.empty()) {
      result.q.push_back(setup_.spec.q_start);
      result.v.push_back(Vector::Zero(n));
    }
    while (static_cast<int>(result.q.size()) < last + 1) {
      result.q.push_back(trace_q_.empty() ? result.q.back() : trace_q_.back());
      result.v.push_back(Vector::Zero(n));
    }
    while (static_cast<int>(result.u.size()) < last) result.u.push_back(Vector::Zero(n));
    result.trace_t = trace_t_;
    result.trace_q = trace_q_;
    result.trace_v = trace_v_;
    result.first_detection = steps_.empty() ? -1 : 0;
    result.min_clearance_per_obstacle.assign(request_.obstacles.size(),
                                             std::numeric_limits<double>::infinity());
    const SphereCollisionModel& truth = *setup_.model;
    for (size_t i = 0; i < trace_t_.size(); ++i) {
      const double gap = movingObstacleClearance(truth, request_.obstacles, trace_q_[i],
                                                 trace_t_[i], &result.min_clearance_per_obstacle);
      result.min_obstacle_clearance = std::min(result.min_obstacle_clearance, gap);
      result.min_self_clearance = std::min(result.min_self_clearance, truth.selfClearance(trace_q_[i]));
      result.min_environment_clearance =
          std::min(result.min_environment_clearance, truth.environmentClearance(trace_q_[i]));
    }
    const Vector& q_end = trace_q_.empty() ? result.q.back() : trace_q_.back();
    const Vector& v_end = trace_v_.empty() ? result.v.back() : trace_v_.back();
    result.goal_error = (q_end - setup_.spec.q_goal).cwiseAbs().maxCoeff();
    result.final_speed = v_end.cwiseAbs().maxCoeff();
    for (const StepInfo& step : steps_) result.max_solve_ms = std::max(result.max_solve_ms, step.solve_ms);
    result.safety = assessRun(result, setup_.spec);
    return result;
  }

  /// Write the ghost and hardware runs as one replay for the app.
  void writeReplay(const std::string& path, const ClosedLoopResult& hardware,
                   const std::string& hardware_label) const {
    // The hardware run is the replay's primary ("online") run, which the viewer
    // draws first and whose planner steps it shows; the ghost run is beside it.
    std::vector<ReplayRun> runs{{"online", hardware_label, &hardware, 0.0}};
    if (ghost_) runs.push_back({"ghost", "Ghost run (simulated)", &*ghost_, 0.0});
    std::ostringstream source;
    source << "scate_hardware (GTDynamics " << live_.scenario.robot_name
           << "), hardware run beside its ghost run, base cost " << baseWrenchRunLabel(sigma_)
           << ", robot " << live_.urdf;
    writeBaseWrenchReplayRuns(path, live_.scenario, setup_.spec, *setup_.model,
                              setup_.base_wrench, request_.obstacles, runs, source.str());
  }

 private:
  LiveRequest request_;
  LiveRobot live_;
  LiveSetup setup_;
  ReactiveConfig config_;
  double sigma_ = 0.0;
  std::optional<ClosedLoopResult> ghost_;
  std::optional<ReactivePlanner> planner_;
  std::vector<double> knot_t_;
  std::vector<Vector> knot_q_, knot_v_, executed_u_;
  std::vector<std::vector<gtsam::Point3>> knot_obstacles_;
  std::vector<StepInfo> steps_;
  std::vector<double> trace_t_;
  std::vector<Vector> trace_q_, trace_v_;
};

}  // namespace scate
