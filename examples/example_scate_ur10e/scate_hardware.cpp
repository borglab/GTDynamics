/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_hardware.cpp
 * @author Harrison Jenkins
 *
 * The planner process of a hardware run (hardware_session.h), driven by the
 * executor (examples/example_scate_ur10e/hardware/scate_hw) over stdin and
 * stdout, one line per message. Diagnostics go to stderr.
 *
 *   scate_hardware serve request.txt
 *
 * The request is the live app's (live_scenario.h) and must say
 * "sensing encoders". On start the program simulates the ghost run and
 * prints it; then it reads commands:
 *
 *   start                                   initial plan
 *   measure k t q1..qn v1..vn x y z ...     reading at state k (one x y z per obstacle)
 *   trace t q1..qn v1..vn                   a measured sample, for the replay
 *   finish completed|aborted path           write the replay (hardware + ghost run)
 *   quit
 *
 * and answers with JSON lines: {"type":"ghost",...}, {"type":"plan",...},
 * {"type":"replay",...} or {"type":"error","message":...}. A plan holds the
 * whole horizon: q and v at every state 0..N and u on every interval, with
 * the executed states replaced by their estimates.
 */

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_session.h"

using namespace scate;

namespace {

/// A JSON number (null if not finite: JSON has no infinity).
std::string num(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream out;
  out << std::setprecision(9) << value;
  return out.str();
}

std::string str(const std::string& text) {
  std::string out = "\"";
  for (char c : text) {
    if (c == '"' || c == '\\') out += '\\';
    if (c == '\n') { out += "\\n"; continue; }
    out += c;
  }
  return out + '"';
}

std::string vec(const Vector& values) {
  std::string out = "[";
  for (long i = 0; i < values.size(); ++i) out += (i ? "," : "") + num(values(i));
  return out + ']';
}

std::string safetyJson(const RunSafety& safety) {
  std::string reasons = "[";
  for (size_t r = 0; r < safety.reasons.size(); ++r) reasons += (r ? "," : "") + str(safety.reasons[r]);
  reasons += ']';
  return "{\"safe\":" + std::string(safety.safe ? "true" : "false") + ",\"reasons\":" + reasons +
         ",\"max_velocity_ratio\":" + num(safety.max_velocity_ratio) +
         ",\"max_acceleration_ratio\":" + num(safety.max_acceleration_ratio) +
         ",\"joint_limit_margin\":" + num(safety.joint_limit_margin) + '}';
}

std::string summaryJson(const ClosedLoopResult& result) {
  return "{\"min_obstacle_clearance\":" + num(result.min_obstacle_clearance) +
         ",\"min_self_clearance\":" + num(result.min_self_clearance) +
         ",\"min_environment_clearance\":" + num(result.min_environment_clearance) +
         ",\"goal_error\":" + num(result.goal_error) + ",\"final_speed\":" + num(result.final_speed) +
         ",\"max_solve_ms\":" + num(result.max_solve_ms) +
         ",\"collided\":" + (result.collided() ? "true" : "false") + '}';
}

/// The current plan over the whole horizon.
std::string planJson(const HardwareSession& session, const StepInfo& info, bool offline) {
  const ReactivePlanner& planner = session.planner();
  const gtsam::Values& values = planner.values();
  const int last = session.numIntervals();
  std::string q = "[", v = "[", u = "[";
  for (int i = 0; i <= last; ++i) {
    q += (i ? "," : "") + vec(values.at<Vector>(QKey(i)));
    v += (i ? "," : "") + vec(values.at<Vector>(VKey(i)));
    if (i < last) u += (i ? "," : "") + vec(values.at<Vector>(UKey(i)));
  }
  return "{\"type\":\"plan\",\"k\":" + std::to_string(offline ? -1 : info.k) +
         ",\"dt\":" + num(session.spec().dt) + ",\"solve_ms\":" + num(info.solve_ms) +
         ",\"graph_error\":" + num(info.graph_error) + ",\"q\":" + q + "],\"v\":" + v +
         "],\"u\":" + u + "]}";
}

/// n numbers from a stream, or throw naming what was expected.
Vector readVector(std::istringstream& fields, long n, const std::string& what) {
  Vector values(n);
  for (long i = 0; i < n; ++i) {
    if (!(fields >> values(i))) throw std::invalid_argument(what + " needs " + std::to_string(n) + " numbers");
  }
  return values;
}

/// The protocol stream: the real stdout. std::cout itself is pointed at
/// stderr in main, so library printing cannot corrupt the protocol.
std::ostream* protocol = &std::cout;

void emit(const std::string& line) { *protocol << line << std::endl; }

int serve(const std::string& request_path) {
  std::ifstream file(request_path);
  if (!file) throw std::invalid_argument("cannot read " + request_path);
  HardwareSession session(parseLiveRequest(file));
  const long n = session.spec().q_start.size();
  const size_t num_obstacles = session.request().obstacles.size();

  const ClosedLoopResult& ghost = session.runGhost();
  emit("{\"type\":\"ghost\",\"sigma\":" + num(session.sigma()) + ",\"num_intervals\":" +
       std::to_string(session.numIntervals()) + ",\"dt\":" + num(session.spec().dt) +
       ",\"v_limit\":" + vec(session.spec().v_limit) + ",\"u_limit\":" + vec(session.spec().u_limit) +
       ",\"q_lower\":" + vec(session.spec().q_lower) + ",\"q_upper\":" + vec(session.spec().q_upper) +
       ",\"safety\":" + safetyJson(*ghost.safety) + ",\"summary\":" + summaryJson(ghost) + '}');

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream fields(line);
    std::string command;
    if (!(fields >> command)) continue;
    try {
      if (command == "quit") {
        break;
      } else if (command == "start") {
        const StepInfo info = session.start();
        emit(planJson(session, info, true));
      } else if (command == "measure") {
        int k = 0;
        double t = 0.0;
        if (!(fields >> k >> t)) throw std::invalid_argument("measure needs k t");
        const Vector q = readVector(fields, n, "measure q");
        const Vector v = readVector(fields, n, "measure v");
        std::vector<gtsam::Point3> obstacles;
        for (size_t j = 0; j < num_obstacles; ++j) {
          obstacles.emplace_back(readVector(fields, 3, "each obstacle center"));
        }
        const StepInfo info = session.measure(k, t, q, v, obstacles);
        emit(planJson(session, info, false));
      } else if (command == "trace") {
        double t = 0.0;
        if (!(fields >> t)) throw std::invalid_argument("trace needs t");
        const Vector q = readVector(fields, n, "trace q");
        const Vector v = readVector(fields, n, "trace v");
        session.addTrace(t, q, v);
      } else if (command == "finish") {
        std::string status, path;
        if (!(fields >> status >> path) || (status != "completed" && status != "aborted")) {
          throw std::invalid_argument("finish needs completed|aborted and a path");
        }
        const ClosedLoopResult hardware = session.hardwareResult();
        session.writeReplay(path, hardware,
                            status == "completed" ? "Hardware run" : "Hardware run (stopped early)");
        emit("{\"type\":\"replay\",\"path\":" + str(path) + ",\"status\":" + str(status) +
             ",\"safety\":" + safetyJson(*hardware.safety) + ",\"summary\":" +
             summaryJson(hardware) + '}');
      } else {
        throw std::invalid_argument("unknown command '" + command + "'");
      }
    } catch (const std::exception& error) {
      emit("{\"type\":\"error\",\"message\":" + str(error.what()) + '}');
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  static std::ostream out(std::cout.rdbuf());
  protocol = &out;
  std::cout.rdbuf(std::cerr.rdbuf());
  if (argc != 3 || std::string(argv[1]) != "serve") {
    std::cerr << "usage: scate_hardware serve request.txt\n";
    return 2;
  }
  try {
    return serve(argv[2]);
  } catch (const std::exception& error) {
    emit("{\"type\":\"error\",\"fatal\":true,\"message\":" + str(error.what()) + '}');
    std::cerr << "scate_hardware: " << error.what() << '\n';
    return 1;
  }
}
