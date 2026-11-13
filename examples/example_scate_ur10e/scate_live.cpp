/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_live.cpp
 * @author Harrison Jenkins
 *
 * The native solver of the SCATE UR10e Base app's live simulation:
 *
 *   scate_live describe <ur5|ur10e> <robot.json>
 *     writes the robot description and default experiment the app drags;
 *   scate_live solve <request.txt> <replay.json>
 *     solves a start, goal and obstacles chosen in the app (live_scenario.h)
 *     once per base-wrench sigma and writes the replay the app plays.
 *
 * Progress goes to stdout as "PROGRESS done total label" lines; errors to
 * stderr with a nonzero exit code.
 */

#include <chrono>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>

#include "live_scenario.h"

namespace {

int usage() {
  std::cerr << "usage: scate_live describe <ur5|ur10e> <robot.json>\n"
               "       scate_live solve <request.txt> <replay.json>\n";
  return 2;
}

/// Write the robot description of key to path.
int describe(const std::string& key, const std::string& path) {
  const scate::LiveRobot live = scate::liveRobot(key);
  const scate::LiveDefaults defaults = scate::liveDefaults(live);
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot open '" + path + "'");
  scate::writeLiveDescription(out, live, defaults);
  std::cout << "Wrote " << path << "\n";
  return 0;
}

/// Solve the request in request_path and write the replay to replay_path.
int solve(const std::string& request_path, const std::string& replay_path) {
  std::ifstream in(request_path);
  if (!in) throw std::runtime_error("cannot open '" + request_path + "'");
  const scate::LiveRequest request = scate::parseLiveRequest(in);
  const auto start = std::chrono::steady_clock::now();
  const auto runs = scate::runLiveRequest(request, replay_path, &std::cout);
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  for (const auto& [sigma, result] : runs) {
    std::cout << scate::baseWrenchRunLabel(sigma) << ": clearance "
              << 100.0 * result.min_obstacle_clearance << " cm"
              << (result.collided() ? " (collision)" : "") << ", goal error " << result.goal_error
              << " rad\n";
  }
  std::cout << "Solved in " << seconds << " s; wrote " << replay_path << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) return usage();
  const std::string command = argv[1];
  try {
    if (command == "describe") return describe(argv[2], argv[3]);
    if (command == "solve") return solve(argv[2], argv[3]);
  } catch (const std::exception& error) {
    std::cerr << "scate_live: " << error.what() << "\n";
    return 1;
  }
  return usage();
}
