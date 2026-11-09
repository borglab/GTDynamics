/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_ur10e_online.cpp
 * @author Harrison Jenkins
 *
 * Manipulator SCATE, online (paper Algorithm 1), on the UR10e: the same
 * scenario as the UR5's scate_online (runOnlineExample in scate_scenario.h:
 * shoulder-pan sweep, obstacle on the offline plan's tool path at state 12,
 * 0.4 m sensor range, blind negative control), with the UR10e's spheres and
 * SDF grid.
 *
 * Usage:  ./scate_ur10e_online [/path/to/ur10e.urdf] [seed]
 * Without a path it loads the UR10e model that ships with GTDynamics.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "scate_scenario.h"
#include "ur10e_collision.h"

int main(int argc, char** argv) {
  try {
    // Robot file: the first argument, else the UR10e model that ships with GTDynamics.
    const std::string path = argc >= 2 ? std::string(argv[1]) : scate::ur10eUrdfPath();
    const unsigned seed = argc >= 3 ? static_cast<unsigned>(std::atoi(argv[2])) : 0u;

    scate::OnlineScenario scenario;
    scenario.program = "scate_ur10e_online";
    scenario.robot_name = "UR10e";
    return scate::runOnlineExample(path, seed, scenario, scate::ur10eSphereModel(),
                                   scate::ur10eCollisionParams());
  } catch (const std::exception& e) {
    std::cerr << "scate_ur10e_online failed: " << e.what() << "\n";
    return 1;
  }
}
