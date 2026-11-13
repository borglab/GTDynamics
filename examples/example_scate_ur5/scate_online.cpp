/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_online.cpp
 * @author Harrison Jenkins
 *
 * Manipulator SCATE, online (paper Algorithm 1) on the UR5:
 *   1. initialize: start pose, IK for the goal, initial trajectory  (scate_init.h)
 *   2. offline plan with self-collision factors; the obstacle is unknown
 *   3. place a spherical obstacle on that plan's tool path, out of sensor
 *      range at the start
 *   4. run the closed loop: execute one control with process noise, measure
 *      joints (and the obstacle once in range), update the graph, re-solve
 *   5. repeat with a blind sensor as a negative control, and write CSVs
 * The steps are in runOnlineExample (scate_scenario.h).
 *
 * Usage:  ./scate_online [/path/to/ur5.urdf] [seed]
 * Without a path it loads the UR5 model that ships with GTDynamics.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "scate_scenario.h"
#include "ur5_collision.h"

int main(int argc, char** argv) {
  try {
    // Robot file: the first argument, else the UR5 model that ships with GTDynamics.
    const std::string path = argc >= 2 ? std::string(argv[1]) : scate::ur5UrdfPath();
    const unsigned seed = argc >= 3 ? static_cast<unsigned>(std::atoi(argv[2])) : 0u;

    scate::OnlineScenario scenario;
    scenario.program = "scate_online";
    scenario.robot_name = "UR5";
    return scate::runOnlineExample(path, seed, scenario, scate::ur5SphereModel(),
                                   scate::Ur5CollisionParams());
  } catch (const std::exception& e) {
    std::cerr << "scate_online failed: " << e.what() << "\n";
    return 1;
  }
}
