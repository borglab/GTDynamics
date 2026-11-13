/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_ur10e_init.cpp
 * @author Harrison Jenkins
 *
 * Manipulator SCATE, offline plan, on the UR10e: the same program as the
 * UR5's scate_init (runOfflineExample in scate_scenario.h), with the same
 * start and goal joint angles.
 *
 * Usage:  ./scate_ur10e_init [/path/to/ur10e.urdf]
 * Without an argument it loads the UR10e model that ships with GTDynamics.
 */

#include <iostream>
#include <string>

#include "scate_scenario.h"
#include "ur10e_collision.h"

int main(int argc, char** argv) {
  try {
    // Robot file: the first argument, else the UR10e model that ships with GTDynamics.
    const std::string path = argc >= 2 ? std::string(argv[1]) : scate::ur10eUrdfPath();
    return scate::runOfflineExample(path);
  } catch (const std::exception& e) {
    std::cerr << "scate_ur10e_init failed: " << e.what() << "\n";
    return 1;
  }
}
