/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ur10e_scenarios.h
 * @author Harrison Jenkins
 *
 * The UR10e's reactive SCATE scenario, shared by scate_ur10e_reactive and
 * scate_ur10e_base_wrench: the shoulder-pan sweep of scate_ur10e_online,
 * crossed by two moving spheres.
 */

#pragma once

#include "scate_reactive_scenario.h"

namespace scate {

/**
 * Two spheres cross the offline plan: one moves toward the base at 3 cm/s and
 * crosses the tool's path at t = 12 s (3 cm above it); one descends at 2 cm/s
 * onto the middle of the forearm (sphere 8) at t = 7 s.
 */
inline ReactiveScenario ur10eReactiveScenario() {
  ReactiveScenario scenario;
  scenario.program = "scate_ur10e_reactive";
  scenario.robot_name = "UR10e";
  ObstacleCrossing tool_crossing;  // toward the base, across the tool path
  tool_crossing.state = 12;
  tool_crossing.direction = "inward";
  tool_crossing.speed = 0.03;
  ObstacleCrossing forearm_crossing;  // down onto the middle of the forearm
  forearm_crossing.state = 7;
  forearm_crossing.sphere = 8;
  forearm_crossing.offset = gtsam::Vector3(0, 0, 0.0);
  forearm_crossing.direction = "down";
  forearm_crossing.speed = 0.02;
  scenario.crossings = {tool_crossing, forearm_crossing};
  return scenario;
}

}  // namespace scate
