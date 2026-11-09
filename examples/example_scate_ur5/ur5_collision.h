/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ur5_collision.h
 * @author Harrison Jenkins
 *
 * The UR5's collision spheres and self-collision pairs, and Ur5CollisionModel:
 * the SphereCollisionModel (sphere_collision.h) built from them. The obstacle
 * SDF uses the default SdfGrid, which covers the UR5's workspace.
 */

#pragma once

#include <utility>
#include <vector>

#include "sphere_collision.h"

namespace scate {

/**
 * UR5 spheres placed along the URDF geometry: the upper arm and forearm run
 * along their link z axes (0.425 m and 0.392 m), the wrists sit at their
 * link origins, and the last sphere is the tool flange (0.0823 m along y of
 * wrist_3, where the URDF puts tool0). Radii are rough fits to the UR5
 * housings; check them against the collision meshes before relying on
 * clearances of a centimeter or two.
 */
inline std::vector<CollisionSphere> ur5Spheres() {
  return {
      {"shoulder_link", {0, 0, 0}, 0.075},        //  0
      {"upper_arm_link", {0, 0, 0}, 0.065},       //  1
      {"upper_arm_link", {0, 0, 0.106}, 0.06},    //  2
      {"upper_arm_link", {0, 0, 0.2125}, 0.06},   //  3
      {"upper_arm_link", {0, 0, 0.319}, 0.06},    //  4
      {"upper_arm_link", {0, 0, 0.425}, 0.06},    //  5
      {"forearm_link", {0, 0, 0.0}, 0.05},        //  6
      {"forearm_link", {0, 0, 0.1}, 0.045},       //  7
      {"forearm_link", {0, 0, 0.2}, 0.045},       //  8
      {"forearm_link", {0, 0, 0.3}, 0.045},       //  9
      {"forearm_link", {0, 0, 0.392}, 0.045},     // 10
      {"wrist_1_link", {0, 0, 0}, 0.045},         // 11
      {"wrist_2_link", {0, 0, 0}, 0.045},         // 12
      {"wrist_3_link", {0, 0, 0}, 0.045},         // 13: sensor
      {"wrist_3_link", {0, 0.0823, 0}, 0.04},     // 14: tool flange
  };
}
inline constexpr size_t kUr5SensorSphere = 13;
inline constexpr size_t kUr5ToolSphere = 14;

/**
 * Self-collision pairs, as indices into ur5Spheres(). Chosen offline with an
 * FK of the URDF over 20,000 random configurations: spheres on links at
 * least two joints apart whose distance actually changes (rigidly connected
 * pairs only cost time) and can come within 5 cm. All start 7 cm or more
 * apart at the home pose.
 */
inline std::vector<std::pair<size_t, size_t>> ur5SelfCollisionPairs() {
  return {{0, 9},  {0, 10}, {0, 11}, {0, 12}, {0, 13}, {0, 14}, {1, 11}, {1, 12},
          {1, 13}, {1, 14}, {2, 11}, {2, 12}, {2, 13}, {2, 14}, {3, 13}, {3, 14},
          {8, 13}, {8, 14}, {9, 13}, {9, 14}, {10, 14}, {11, 14}};
}

/// Spheres, self-collision pairs, sensor and tool of the UR5 (no motion-capture markers).
inline SphereModel ur5SphereModel() {
  return {ur5Spheres(), ur5SelfCollisionPairs(), kUr5SensorSphere, kUr5ToolSphere,
          /*markers=*/{}};
}

/// Factor settings of the UR5 collision model (default grid: the UR5 workspace).
using Ur5CollisionParams = SphereCollisionParams;

/// SphereCollisionModel of the UR5.
class Ur5CollisionModel : public SphereCollisionModel {
 public:
  /// Build the UR5's sphere, sensor and drawing query points.
  explicit Ur5CollisionModel(const gtdynamics::Robot& robot,
                             const Ur5CollisionParams& params = {})
      : SphereCollisionModel(robot, ur5SphereModel(), params) {}
};

}  // namespace scate
