/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_init.cpp
 * @author Harrison Jenkins
 *
 * Manipulator SCATE, offline plan (paper Algorithm 1, "Generate Initial Plan"):
 *   1. load the robot and solve IK for the goal          (scate_init.h)
 *   2. build the initial trajectory guess                 (scate_init.h)
 *   3. build the factor graph from a GraphSpec            (build_scate_graph.h)
 *   4. solve with Levenberg-Marquardt                     (build_scate_graph.h)
 *   5. check the solution and write both trajectories to CSV
 * The steps are in runOfflineExample (scate_scenario.h).
 *
 * Usage:  ./scate_init [/path/to/ur5.urdf]
 * Without an argument it loads the UR5 model that ships with GTDynamics.
 */

#include <iostream>
#include <string>

#include "scate_init.h"
#include "scate_scenario.h"

int main(int argc, char** argv) {
  try {
    // Robot file: the first argument, else the UR5 model that ships with GTDynamics.
    const std::string path = argc >= 2 ? std::string(argv[1]) : scate::ur5UrdfPath();
    return scate::runOfflineExample(path);
  } catch (const std::exception& e) {
    std::cerr << "scate_init failed: " << e.what() << "\n";
    return 1;
  }
}
