/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ur10e_collision.h
 * @author Harrison Jenkins
 *
 * The UR10e for SCATE: its robot file, collision spheres, self-collision
 * pairs and obstacle SDF grid, and Ur10eCollisionModel, the
 * SphereCollisionModel (examples/example_scate_ur5/sphere_collision.h) built
 * from them. Everything else (graph, online loop, scenario) is the UR5 code.
 *
 * The UR10e URDF (Universal_Robots_ROS2_Description) uses different link
 * frames from the UR5 one: the upper arm and forearm run along -x of their
 * link frames, the tubes are offset along z, and the wrist_3 link frame sits
 * at the tool flange. Joint angles follow the same convention as the UR5 (and
 * the robot controller), so the same joint vectors give the same posture.
 */

#pragma once

#include <gtdynamics/config.h>

#include <string>
#include <utility>
#include <vector>

#include "sphere_collision.h"

namespace scate {

/// The UR10e model that ships with GTDynamics (models/urdfs/ur10e/ur10e.urdf).
inline std::string ur10eUrdfPath() {
  return std::string(gtdynamics::kUrdfPath) + "ur10e/ur10e.urdf";
}

inline constexpr double kUr10eUpperArmLength = 0.6127;  ///< m, shoulder to elbow (URDF)
inline constexpr double kUr10eForearmLength = 0.57155;  ///< m, elbow to wrist 1 (URDF)

/**
 * UR10e spheres placed along the URDF geometry: five on the upper arm tube
 * (link -x axis, 0.176 m along link z) and five on the forearm tube (link -x
 * axis, 0.043 m along z), the wrists at their link origins, the wrist_3
 * housing 5 cm behind the flange, and the tool flange at the wrist_3 link
 * origin. Radii are rough fits to the collision meshes: sampling the mesh
 * surfaces, 95% of each link's surface lies within about 3 cm of its
 * spheres (a similar fit to the UR5 spheres). Check them against the meshes
 * before relying on clearances of a centimeter or two.
 */
inline std::vector<CollisionSphere> ur10eSpheres() {
  const double upper_z = 0.176, forearm_z = 0.043;
  std::vector<CollisionSphere> spheres = {{"shoulder_link", {0, 0, 0}, 0.09}};  // 0
  for (int i = 0; i <= 4; ++i) {                                                // 1-5
    const double radius = (i == 0 || i == 4) ? 0.08 : 0.075;
    spheres.push_back({"upper_arm_link", {-kUr10eUpperArmLength * i / 4, 0, upper_z}, radius});
  }
  for (int i = 0; i <= 4; ++i) {  // 6-10
    const double radius = (i == 0) ? 0.065 : 0.06;
    spheres.push_back({"forearm_link", {-kUr10eForearmLength * i / 4, 0, forearm_z}, radius});
  }
  spheres.push_back({"wrist_1_link", {0, 0, 0}, 0.06});      // 11
  spheres.push_back({"wrist_2_link", {0, 0, 0}, 0.06});      // 12
  spheres.push_back({"wrist_3_link", {0, 0, -0.05}, 0.05});  // 13: sensor
  spheres.push_back({"wrist_3_link", {0, 0, 0}, 0.045});     // 14: tool flange
  return spheres;
}
inline constexpr size_t kUr10eSensorSphere = 13;  ///< wrist_3 housing; its link carries the sensor
inline constexpr size_t kUr10eToolSphere = 14;    ///< tool flange

/**
 * Self-collision pairs, as indices into ur10eSpheres(), chosen as for the
 * UR5: FK of the URDF over 20,000 random configurations within the joint
 * limits, keeping spheres on links at least two joints apart whose distance
 * changes and can come within 5 cm. All are 7.9 cm or more apart at the
 * home pose.
 */
inline std::vector<std::pair<size_t, size_t>> ur10eSelfCollisionPairs() {
  return {{0, 9},  {0, 10}, {0, 11}, {0, 12}, {0, 13}, {0, 14}, {1, 11}, {1, 12},
          {1, 13}, {1, 14}, {2, 11}, {2, 12}, {2, 13}, {2, 14}, {3, 12}, {3, 13},
          {3, 14}, {8, 14}, {9, 12}, {9, 13}, {9, 14}, {10, 13}, {10, 14}};
}

/**
 * Motion-capture (VICON-style) markers on the UR10e, for scate_reactive.h:
 * two on the surface of the upper arm, two on the forearm, and four on
 * 10 cm posts (one on each of wrist 1 and 2, two on a tree on the tool
 * flange). Each sits off its own link's joint axis (the link z axis), so
 * every joint angle moves at least one marker and q is observable from the
 * markers alone. The wrist angles are the least observed: with 1 mm marker
 * noise one reading pins them to about 0.006-0.009 rad (about 0.02 rad for
 * wrist 3 with markers 4.5 cm from its axis instead of 10 cm).
 */
inline std::vector<Marker> ur10eMarkers() {
  return {
      {"upper_arm_link", {-0.15, 0.0, 0.26}},     //  0
      {"upper_arm_link", {-0.46, 0.075, 0.176}},  //  1
      {"forearm_link", {-0.15, 0.0, 0.11}},       //  2
      {"forearm_link", {-0.42, 0.065, 0.043}},    //  3
      {"wrist_1_link", {0.10, 0.0, 0.0}},         //  4: post
      {"wrist_2_link", {0.10, 0.0, 0.0}},         //  5: post
      {"wrist_3_link", {0.10, 0.0, 0.02}},        //  6: flange tree
      {"wrist_3_link", {0.0, 0.10, 0.02}},        //  7: flange tree
  };
}

/// Spheres, self-collision pairs, sensor, tool and markers of the UR10e.
inline SphereModel ur10eSphereModel() {
  return {ur10eSpheres(), ur10eSelfCollisionPairs(), kUr10eSensorSphere, kUr10eToolSphere,
          ur10eMarkers()};
}

/**
 * SDF grid for the UR10e (reach about 1.3 m): x, y in [-1.5, 1.5] and z in
 * [-0.45, 1.65] m around the base, at the UR5's 3 cm resolution.
 */
inline SdfGrid ur10eSdfGrid() {
  SdfGrid grid;
  grid.origin = gtsam::Point3(-1.5, -1.5, -0.45);
  grid.cell_size = 0.03;
  grid.num_x = 101;
  grid.num_y = 101;
  grid.num_z = 71;
  return grid;
}

/// Factor settings of the UR10e collision model: the UR5's, on the UR10e grid.
inline SphereCollisionParams ur10eCollisionParams() {
  SphereCollisionParams params;
  params.grid = ur10eSdfGrid();
  return params;
}

/// SphereCollisionModel of the UR10e.
class Ur10eCollisionModel : public SphereCollisionModel {
 public:
  /// Build the UR10e's sphere, sensor and drawing query points.
  explicit Ur10eCollisionModel(const gtdynamics::Robot& robot,
                               const SphereCollisionParams& params = ur10eCollisionParams())
      : SphereCollisionModel(robot, ur10eSphereModel(), params) {}
};

}  // namespace scate
