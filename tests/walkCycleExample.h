/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WalkCycleExample.h
 * @brief example re-used in several tests.
 * @author: Disha Das, Varun Agrawal
 */

#pragma once

#include <gtdynamics/utils/WalkCycle.h>

namespace gtdynamics {
namespace walk_cycle_example {

Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("/spider.sdf"), "spider");

// First phase
constexpr size_t num_time_steps = 2;
const gtsam::Point3 contact_in_com(0, 0.19, 0);
const Phase phase_1(robot, num_time_steps,
                    {"tarsus_1_L1", "tarsus_2_L2", "tarsus_3_L3"},
                    contact_in_com);

// Second phase
constexpr size_t num_time_steps_2 = 3;
const Phase phase_2(robot, num_time_steps_2,
                    {"tarsus_2_L2", "tarsus_3_L3", "tarsus_4_L4",
                     "tarsus_5_R4"},
                    contact_in_com);

// Initialize walk cycle
const WalkCycle walk_cycle({phase_1, phase_2});

}  // namespace walk_cycle_example
}  // namespace gtdynamics