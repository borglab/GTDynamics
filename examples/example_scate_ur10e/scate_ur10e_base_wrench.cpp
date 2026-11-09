/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_ur10e_base_wrench.cpp
 * @author Harrison Jenkins
 *
 * How much the base-disturbance cost (base_wrench.h) reduces the wrench the
 * UR10e puts on its mount, and what it costs: reactive SCATE with motion
 * capture, the cost off and at several sigmas, over several noise seeds, on
 * two scenarios: the sweep alone, and the sweep with the two moving
 * obstacles of scate_ur10e_reactive. Prints a table per scenario and writes
 * seed 0's executed base wrench to base_wrench_sweep.csv and
 * base_wrench_obstacles.csv.
 *
 * Usage:  ./scate_ur10e_base_wrench [/path/to/ur10e.urdf] [number of seeds]
 */

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "scate_reactive_scenario.h"
#include "ur10e_collision.h"
#include "ur10e_scenarios.h"

namespace {

/// Print one scenario's study as a table.
void printStudy(const std::string& title, const std::vector<scate::BaseWrenchStudyRow>& rows,
                bool with_obstacles) {
  std::cout << "\n--- " << title << " (means over seeds; worst clearance and goal error) ---\n";
  std::printf("%-9s %9s %9s %11s %9s %9s %10s %10s %10s %8s %5s\n", "sigma", "rms tau",
              "peak tau", "int|tau|dt", "rms f", "peak f", "clearance", "goal err",
              "slowest", "", "hits");
  std::printf("%-9s %9s %9s %11s %9s %9s %10s %10s %10s\n", "(Nm, N)", "(Nm)", "(Nm)", "(Nms)",
              "(N)", "(N)", "(cm)", "(rad)", "(ms)");
  for (const auto& row : rows) {
    const std::string sigma = row.sigma > 0.0 ? std::to_string(row.sigma).substr(0, 5) : "off";
    std::printf("%-9s %9.4f %9.4f %11.4f %9.4f %9.4f %10s %10.4f %10.0f %8s %5d\n",
                sigma.c_str(), row.mean.rms_torque, row.mean.peak_torque,
                row.mean.torque_impulse, row.mean.rms_force, row.mean.peak_force,
                with_obstacles ? std::to_string(100 * row.worst_clearance).substr(0, 5).c_str()
                               : "-",
                row.worst_goal_error, row.slowest_step_ms, "", row.collisions);
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string path = argc >= 2 ? std::string(argv[1]) : scate::ur10eUrdfPath();
    const int seeds = argc >= 3 ? std::atoi(argv[2]) : 3;
    const std::vector<double> sigmas = {0.0, 3.0, 1.0, 0.3, 0.1};

    scate::ReactiveScenario sweep = scate::ur10eReactiveScenario();
    sweep.crossings.clear();
    printStudy("Sweep, no obstacles",
               scate::runBaseWrenchStudy(path, seeds, sweep, scate::ur10eSphereModel(),
                                         scate::ur10eCollisionParams(), sigmas,
                                         "base_wrench_sweep.csv"),
               false);
    printStudy("Sweep with two moving obstacles",
               scate::runBaseWrenchStudy(path, seeds, scate::ur10eReactiveScenario(),
                                         scate::ur10eSphereModel(),
                                         scate::ur10eCollisionParams(), sigmas,
                                         "base_wrench_obstacles.csv"),
               true);
    std::cout << "\nWrote base_wrench_sweep.csv and base_wrench_obstacles.csv\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "scate_ur10e_base_wrench failed: " << e.what() << "\n";
    return 1;
  }
}
