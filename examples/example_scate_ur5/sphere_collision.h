/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file sphere_collision.h
 * @author Harrison Jenkins
 *
 * CollisionModel (scate_online.h) for a serial arm approximated by spheres,
 * built on GTDynamics:
 *   - RobotQueryPoints: FK of collision-sphere centers, with Jacobians;
 *   - ObstacleSDFFactorGP: robot spheres vs. a signed distance field of the
 *     obstacle, checked at time tau inside each interval;
 *   - SelfCollisionSphereFactorGP: robot spheres vs. each other;
 *   - the sensor pose (the link of one chosen sphere) from
 *     RobotQueryPoints::queryPoses.
 *
 * The robot-specific part is a SphereModel: the spheres, the self-collision
 * pairs, and which sphere is the sensor and which the tool. ur5_collision.h
 * and ur10e_collision.h (examples/example_scate_ur10e) each supply one.
 *
 * The obstacle is a sphere. Its SDF is sampled on a fixed grid covering the
 * robot's workspace and rebuilt whenever the planner's estimate of its center
 * changes (setObstacle). The grid must contain every robot sphere: GTDynamics
 * treats points outside it as "fail closed" and pushes them back inside.
 */

#pragma once

#include <gtdynamics/factors/ObstacleSDFFactorGP.h>
#include <gtdynamics/factors/SelfCollisionSphereFactorGP.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SelfCollisionCost.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "scate_init.h"
#include "scate_online.h"

namespace scate {

// ============================================================
// 1. Sphere model
// ============================================================

/**
 * A collision sphere, its center given in the LINK frame of the URDF (the
 * frame the joint origins are written in). The model converts it to the
 * link's CoM frame, which is what RobotQueryPoints expects.
 */
struct CollisionSphere {
  std::string link;              ///< URDF link name
  gtsam::Point3 center_in_link;  ///< center in the URDF link frame (m)
  double radius;                 ///< m
};

/// The robot-specific geometry of a SphereCollisionModel.
struct SphereModel {
  std::vector<CollisionSphere> spheres;  ///< collision spheres
  /// Sphere index pairs checked for self collision.
  std::vector<std::pair<size_t, size_t>> self_collision_pairs;
  size_t sensor_sphere = 0;  ///< sphere whose link's CoM frame is the sensor frame
  size_t tool_sphere = 0;    ///< sphere drawn and reported as the tool
};

// ============================================================
// 2. Obstacle field
// ============================================================

/**
 * Uniform grid for the obstacle SDF. The default covers x, y in [-1.0, 0.98]
 * and z in [-0.3, 1.2] m around the base, enough for the UR5 (reach about
 * 0.85 m); larger arms pass their own grid.
 */
struct SdfGrid {
  gtsam::Point3 origin{-1.0, -1.0, -0.3};  ///< corner of the grid (m)
  double cell_size = 0.03;                 ///< m
  size_t num_x = 67, num_y = 67, num_z = 51;  ///< grid points per axis
};

/// Exact signed distance to a sphere, sampled on the grid (positive outside).
inline std::shared_ptr<const gtdynamics::SignedDistanceField> makeSphereObstacleSDF(
    const gtsam::Point3& center, double radius, const SdfGrid& grid) {
  // Flat layout of the fast constructor: x fastest, then y (rows), then z.
  gtsam::Vector values(grid.num_x * grid.num_y * grid.num_z);
  for (size_t z = 0; z < grid.num_z; ++z) {
    for (size_t row = 0; row < grid.num_y; ++row) {
      for (size_t col = 0; col < grid.num_x; ++col) {
        const gtsam::Point3 point =
            grid.origin + gtsam::Point3(col * grid.cell_size, row * grid.cell_size,
                                        z * grid.cell_size);
        values((z * grid.num_y + row) * grid.num_x + col) = (point - center).norm() - radius;
      }
    }
  }
  return std::make_shared<const gtdynamics::SignedDistanceField>(
      grid.origin, grid.cell_size, grid.num_y, grid.num_x, grid.num_z, values);
}

// ============================================================
// 3. The model
// ============================================================

/// Factor settings of a sphere collision model.
struct SphereCollisionParams {
  double dt = kDt;                 ///< must match the graph's dt
  double obstacle_sigma = 0.01;    ///< m: hinge cost sigma per sphere
  double obstacle_epsilon = 0.05;  ///< m: safety distance kept from the obstacle
  double self_sigma = 0.01;        ///< m
  double self_epsilon = 0.02;      ///< m: gap kept between paired spheres
  SdfGrid grid;                    ///< obstacle SDF grid
};

/// The robot's joints, in the chain order that indexes q (scate_init.h).
inline std::vector<gtdynamics::JointSharedPtr> jointsInChainOrder(
    const gtdynamics::Robot& robot) {
  std::vector<gtdynamics::JointSharedPtr> joints;
  for (size_t id : jointIdsInChainOrder(robot)) {
    for (auto&& joint : robot.joints()) {
      if (joint->id() == id) joints.push_back(joint);
    }
  }
  return joints;
}

/// CollisionModel of a sphere-approximated arm on GTDynamics' GPMP2 factors.
class SphereCollisionModel : public CollisionModel {
 public:
  /**
   * Build the sphere, sensor and drawing query points for this robot.
   * @param robot  fixed-base serial arm (scate_init.h loadRobot)
   * @param model  spheres, self-collision pairs, sensor and tool sphere
   * @param params factor settings and the obstacle SDF grid
   */
  SphereCollisionModel(const gtdynamics::Robot& robot, const SphereModel& model,
                       const SphereCollisionParams& params)
      : params_(params), model_(model) {
    validateModel();
    // RobotQueryPoints takes the joints in the order q indexes them.
    const std::vector<gtdynamics::JointSharedPtr> joints = jointsInChainOrder(robot);
    dof_ = joints.size();
    const std::string base = baseLinkName(robot);
    const gtsam::Pose3 wTbase = robot.link(base)->bMcom();

    const gtdynamics::PointOnLinks points = sphereQueryPoints(robot);
    points_ = std::make_shared<const gtdynamics::RobotQueryPoints>(robot, base, joints,
                                                                   points, wTbase);
    sensor_ = std::make_shared<const gtdynamics::RobotQueryPoints>(
        robot, base, joints, gtdynamics::PointOnLinks{points[model_.sensor_sphere]}, wTbase);

    for (const auto& pair : model_.self_collision_pairs) {
      pairs_.emplace_back(pair.first, pair.second, params_.self_epsilon);
    }
    qc_ = gtsam::noiseModel::Isotropic::Sigma(dof_, 1.0);  // only its size is used

    frames_ = std::make_shared<const gtdynamics::RobotQueryPoints>(
        robot, base, joints, linkFrameQueryPoints(robot, base, joints), wTbase);
  }

  /// Number of joints.
  size_t dof() const override { return dof_; }

  /**
   * Sensor frame = CoM frame of the link that carries the sensor sphere (on
   * the UR5 its origin is the wrist_3 joint; on the UR10e it is 2.6 cm
   * behind the tool flange).
   */
  gtsam::Pose3 sensorPose(const Vector& q, gtsam::Matrix* H = nullptr) const override {
    std::vector<gtsam::Pose3> poses;
    std::vector<gtsam::Matrix> jacobians;
    sensor_->queryPoses(q, &poses, H ? &jacobians : nullptr);
    if (H) *H = jacobians.at(0);
    return poses.at(0);
  }

  /// Rebuild the obstacle SDF for a sphere at center with this radius.
  void setObstacle(const gtsam::Point3& center, double radius) override {
    sdf_ = makeSphereObstacleSDF(center, radius, params_.grid);
  }

  /// ObstacleSDFFactorGP on interval i at time tau; needs setObstacle first.
  gtsam::NonlinearFactor::shared_ptr obstacleFactor(int i, double tau) const override {
    if (!sdf_) {
      throw std::logic_error("SphereCollisionModel: obstacleFactor before setObstacle");
    }
    return std::make_shared<gtdynamics::ObstacleSDFFactorGP>(
        QKey(i), VKey(i), QKey(i + 1), VKey(i + 1), points_, sdf_, params_.obstacle_sigma,
        params_.obstacle_epsilon, radii_, qc_, params_.dt, tau);
  }

  /// SelfCollisionSphereFactorGP on interval i at time tau.
  gtsam::NonlinearFactor::shared_ptr selfCollisionFactor(int i, double tau) const override {
    return std::make_shared<gtdynamics::SelfCollisionSphereFactorGP>(
        QKey(i), VKey(i), QKey(i + 1), VKey(i + 1), points_, pairs_, radii_,
        params_.self_sigma, qc_, params_.dt, tau);
  }

  /// Smallest gap between any robot sphere and the obstacle sphere surface.
  double obstacleClearance(const Vector& q, const gtsam::Point3& center,
                           double radius) const override {
    const gtsam::Matrix world = points_->worldPoints(q);
    double best = std::numeric_limits<double>::infinity();
    for (long i = 0; i < world.cols(); ++i) {
      best = std::min(best,
                      (gtsam::Point3(world.col(i)) - center).norm() - radius - radii_(i));
    }
    return best;
  }

  /// Smallest gap over the self-collision pairs.
  double selfClearance(const Vector& q) const override {
    const gtsam::Matrix world = points_->worldPoints(q);
    double best = std::numeric_limits<double>::infinity();
    for (const auto& pair : pairs_) {
      best = std::min(best, (world.col(pair.a) - world.col(pair.b)).norm() -
                                radii_(pair.a) - radii_(pair.b));
    }
    return best;
  }

  /// World position of one sphere center.
  gtsam::Point3 spherePosition(const Vector& q, size_t index) const {
    return gtsam::Point3(points_->worldPoints(q).col(index));
  }

  /**
   * Smallest gap between any sphere and the faces of the SDF grid (negative
   * = a sphere center is outside it, which the obstacle factors penalize).
   */
  double gridMargin(const Vector& q) const {
    const gtsam::Matrix world = points_->worldPoints(q);
    const SdfGrid& grid = params_.grid;
    const gtsam::Point3 far_corner =
        grid.origin + gtsam::Point3((grid.num_x - 1) * grid.cell_size,
                                    (grid.num_y - 1) * grid.cell_size,
                                    (grid.num_z - 1) * grid.cell_size);
    double best = std::numeric_limits<double>::infinity();
    for (long i = 0; i < world.cols(); ++i) {
      for (int axis = 0; axis < 3; ++axis) {
        best = std::min({best, world(axis, i) - grid.origin(axis),
                         far_corner(axis) - world(axis, i)});
      }
    }
    return best;
  }

  /// Number of collision spheres.
  size_t numSpheres() const { return model_.spheres.size(); }
  /// Number of self-collision pairs.
  size_t numPairs() const { return pairs_.size(); }
  /// Index of the sensor sphere.
  size_t sensorSphere() const { return model_.sensor_sphere; }
  /// Factor settings and SDF grid.
  const SphereCollisionParams& params() const { return params_; }

  // ---- Drawing hooks (scate_replay.h) ----

  /// World positions of all sphere centers (3 x M).
  gtsam::Matrix sphereCenters(const Vector& q) const override { return points_->worldPoints(q); }
  /// Sphere radii.
  Vector sphereRadii() const override { return radii_; }
  /// The tool sphere.
  int toolSphere() const override { return static_cast<int>(model_.tool_sphere); }

  /// Self-collision pairs as sphere index pairs.
  std::vector<std::pair<size_t, size_t>> selfCollisionPairs() const override {
    std::vector<std::pair<size_t, size_t>> index_pairs;
    for (const auto& pair : pairs_) index_pairs.emplace_back(pair.a, pair.b);
    return index_pairs;
  }

  /// World poses of the base and chain links in their URDF link frames.
  std::vector<std::pair<std::string, gtsam::Pose3>> linkFrames(const Vector& q) const override {
    std::vector<gtsam::Pose3> wTcom;
    frames_->queryPoses(q, &wTcom);
    std::vector<std::pair<std::string, gtsam::Pose3>> frames;
    for (size_t l = 0; l < wTcom.size(); ++l) {
      frames.emplace_back(link_names_[l], wTcom[l].compose(comTlink_[l]));
    }
    return frames;
  }

 private:
  SphereCollisionParams params_;
  SphereModel model_;
  size_t dof_ = 0;
  std::shared_ptr<const gtdynamics::RobotQueryPoints> points_, sensor_, frames_;
  std::vector<std::string> link_names_;
  std::vector<gtsam::Pose3> comTlink_;
  gtsam::Vector radii_;
  gtdynamics::SelfCollisionPairs pairs_;
  gtsam::SharedNoiseModel qc_;
  std::shared_ptr<const gtdynamics::SignedDistanceField> sdf_;

  /// Throws if the sensor, tool or a self-collision pair names a missing sphere.
  void validateModel() const {
    const size_t count = model_.spheres.size();
    if (model_.sensor_sphere >= count || model_.tool_sphere >= count) {
      throw std::invalid_argument("SphereCollisionModel: sensor or tool sphere index out of range");
    }
    for (const auto& pair : model_.self_collision_pairs) {
      if (pair.first >= count || pair.second >= count || pair.first == pair.second) {
        throw std::invalid_argument("SphereCollisionModel: bad self-collision pair (" +
                                    std::to_string(pair.first) + ", " +
                                    std::to_string(pair.second) + ")");
      }
    }
  }

  /**
   * One query point per sphere, converted from the link frame to the link's
   * CoM frame (link frame -> base frame at rest -> CoM frame); also fills radii_.
   */
  gtdynamics::PointOnLinks sphereQueryPoints(const gtdynamics::Robot& robot) {
    const std::vector<CollisionSphere>& spheres = model_.spheres;
    gtdynamics::PointOnLinks points;
    radii_ = gtsam::Vector(spheres.size());
    for (size_t i = 0; i < spheres.size(); ++i) {
      gtdynamics::LinkSharedPtr link;
      try {
        link = robot.link(spheres[i].link);
      } catch (const std::exception& e) {
        throw std::runtime_error("SphereCollisionModel: sphere " + std::to_string(i) +
                                 " is on link '" + spheres[i].link +
                                 "', which this robot does not have (" + e.what() + ")");
      }
      const gtsam::Point3 center_in_com = link->bMcom().transformTo(
          link->bMlink().transformFrom(spheres[i].center_in_link));
      points.emplace_back(link, center_in_com);
      radii_(i) = spheres[i].radius;
    }
    return points;
  }

  /**
   * For drawing: one query point at the CoM-frame origin of the base and of
   * every link along the chain. queryPoses then returns each wTcom, and
   * wTlink = wTcom * comTlink with comTlink = bMcom^-1 * bMlink (at rest).
   * Also fills link_names_ and comTlink_.
   */
  gtdynamics::PointOnLinks linkFrameQueryPoints(
      const gtdynamics::Robot& robot, const std::string& base,
      const std::vector<gtdynamics::JointSharedPtr>& joints) {
    gtdynamics::PointOnLinks frame_points;
    std::vector<gtdynamics::LinkSharedPtr> drawn = {robot.link(base)};
    for (auto&& joint : joints) drawn.push_back(joint->child());
    for (auto&& link : drawn) {
      frame_points.emplace_back(link, gtsam::Point3(0, 0, 0));
      link_names_.push_back(link->name());
      comTlink_.push_back(link->bMcom().between(link->bMlink()));
    }
    return frame_points;
  }
};

}  // namespace scate
