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
const std::vector<LinkSharedPtr> links_1 = {robot.link("tarsus_1_L1"),
                                            robot.link("tarsus_2_L2"),
                                            robot.link("tarsus_3_L3")};
/////const Phase phase_1(num_time_steps, links_1, contact_in_com);
const auto phase_1 = boost::make_shared<FootContactConstraintSpec>(links_1, contact_in_com);

// Second phase
constexpr size_t num_time_steps_2 = 3;
const std::vector<LinkSharedPtr> links_2 = {robot.link("tarsus_2_L2"),
                                            robot.link("tarsus_3_L3"),
                                            robot.link("tarsus_4_L4"),
                                            robot.link("tarsus_5_R4")};
/////const Phase phase_2(num_time_steps_2, links_2, contact_in_com);
const auto phase_2 = boost::make_shared<FootContactConstraintSpec>(links_2, contact_in_com);

// Initialize walk cycle
const WalkCycle walk_cycle({phase_1, phase_2}, {num_time_steps, num_time_steps_2});

}  // namespace walk_cycle_example
}  // namespace gtdynamics