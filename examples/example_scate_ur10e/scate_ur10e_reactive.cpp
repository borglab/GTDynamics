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
 * Usage:  ./scate_ur10e_reactive [/path/to/ur10e.urdf] [seed] [encoders] [base]
 * Without a path it loads the UR10e model that ships with GTDynamics. Options
 * after the seed, in any order: "encoders" reads the joint encoders as well
 * as the markers (by default the arm's state comes from the markers alone);
 * "base" adds the base-disturbance cost (base_wrench.h).
 */

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "scate_reactive_scenario.h"
#include "ur10e_collision.h"
#include "ur10e_scenarios.h"

int main(int argc, char** argv) {
  try {
    // Robot file: the first argument, else the UR10e model that ships with GTDynamics.
    const std::string path = argc >= 2 ? std::string(argv[1]) : scate::ur10eUrdfPath();
    const unsigned seed = argc >= 3 ? static_cast<unsigned>(std::atoi(argv[2])) : 0u;
    scate::ReactiveScenario scenario = scate::ur10eReactiveScenario();
    for (int i = 3; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "encoders") scenario.use_encoders = true;
      else if (option == "base") scenario.minimize_base_wrench = true;
      else throw std::invalid_argument("unknown option '" + option + "'");
    }
    return scate::runReactiveExample(path, seed, scenario,
                                     scate::ur10eSphereModel(), scate::ur10eCollisionParams());
  } catch (const std::exception& e) {
    std::cerr << "scate_ur10e_reactive failed: " << e.what() << "\n";
    return 1;
  }
}
