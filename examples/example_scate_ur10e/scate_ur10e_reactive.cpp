/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_ur10e_reactive.cpp
 * @author Harrison Jenkins
 *
 * Reactive SCATE (paper eq. 7, Algorithm 1) on the UR10e with motion-capture
 * sensing and moving obstacles (runReactiveExample in scate_reactive_scenario.h).
 * The arm makes the same shoulder-pan sweep as scate_ur10e_online. Two
 * spheres cross its offline plan: one moves toward the base and crosses the
 * tool's path at t = 12 s; one descends onto the forearm at t = 7 s. A
 * VICON-style system reads 8 markers on the arm and both obstacle centers
 * every second. Frozen (obstacles seen at the start only) and blind runs on
 * the same noise are the negative controls.
 *
 * Usage:  ./scate_ur10e_reactive [/path/to/ur10e.urdf] [seed] [encoders]
 * Without a path it loads the UR10e model that ships with GTDynamics. With
 * "encoders" as the third argument the joint encoders are read as well as
 * the markers (by default the arm's state comes from the markers alone).
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "scate_reactive_scenario.h"
#include "ur10e_collision.h"

/// The two crossing obstacles of the UR10e scenario.
scate::ReactiveScenario ur10eReactiveScenario() {
  scate::ReactiveScenario scenario;
  scenario.program = "scate_ur10e_reactive";
  scenario.robot_name = "UR10e";
  scate::ObstacleCrossing tool_crossing;  // toward the base, across the tool path
  tool_crossing.state = 12;
  tool_crossing.direction = "inward";
  tool_crossing.speed = 0.03;
  scate::ObstacleCrossing forearm_crossing;  // down onto the middle of the forearm
  forearm_crossing.state = 7;
  forearm_crossing.sphere = 8;
  forearm_crossing.offset = gtsam::Vector3(0, 0, 0.0);
  forearm_crossing.direction = "down";
  forearm_crossing.speed = 0.02;
  scenario.crossings = {tool_crossing, forearm_crossing};
  return scenario;
}

int main(int argc, char** argv) {
  try {
    // Robot file: the first argument, else the UR10e model that ships with GTDynamics.
    const std::string path = argc >= 2 ? std::string(argv[1]) : scate::ur10eUrdfPath();
    const unsigned seed = argc >= 3 ? static_cast<unsigned>(std::atoi(argv[2])) : 0u;
    scate::ReactiveScenario scenario = ur10eReactiveScenario();
    scenario.use_encoders = argc >= 4 && std::string(argv[3]) == "encoders";
    return scate::runReactiveExample(path, seed, scenario,
                                     scate::ur10eSphereModel(), scate::ur10eCollisionParams());
  } catch (const std::exception& e) {
    std::cerr << "scate_ur10e_reactive failed: " << e.what() << "\n";
    return 1;
  }
}
