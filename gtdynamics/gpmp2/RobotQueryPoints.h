/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RobotQueryPoints.h
 * @brief Forward kinematics of query points on a robot, with Jacobians.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <memory>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * Maps a stacked joint angle vector q to the world positions of a fixed set of
 * query points on the robot, with Jacobians with respect to q. The joints
 * given to the constructor define the ordering of q; the kinematic tree is
 * traversed once at construction. Trees only, no loops.
 * Immutable after construction, so one instance can be evaluated concurrently.
 */
class GTSAM_EXPORT RobotQueryPoints {
 private:
  gtsam::Pose3 wTbase_;
  PointOnLinks points_;
  size_t dof_;  ///< number of joints spanned by q

  /// Place the link at childSlot by applying q(qCol) to the parentSlot link.
  struct TraversalStep {
    JointSharedPtr joint;
    LinkSharedPtr childLink;
    size_t parentSlot, childSlot, qCol;
  };
  std::vector<TraversalStep> steps_;  ///< the tree, in topological order
  std::vector<size_t> pointSlots_;    ///< slot of each query point's link
  size_t nrLinks_;                    ///< number of slots (base + reached links)

  /// Run the traversal, filling *poses and, if non-null, *linkJacobians.
  void computeForwardKinematics(const gtsam::Vector &q,
                                std::vector<gtsam::Pose3> *poses,
                                gtsam::Matrix *linkJacobians) const;

 public:
  /**
   * Constructor.
   * @param robot the robot model
   * @param baseLinkName the link the kinematic tree is rooted at
   * @param joints the joints spanned by q, in the order q indexes them
   * @param points the query points, each in its link's CoM frame
   * @param wTbase pose of the base link in the world frame
   * @throw std::invalid_argument on a null, repeated or unreachable joint, or
   *        a point whose link is null or not reached by the joints
   */
  RobotQueryPoints(const Robot &robot, const std::string &baseLinkName,
                   const std::vector<JointSharedPtr> &joints,
                   const PointOnLinks &points,
                   const gtsam::Pose3 &wTbase = gtsam::Pose3());

  /**
   * Construct a model restricted to a subset of another model's query points,
   * for factors that only reference a few of a larger model's points (e.g.
   * sparse self collision pairs). Replays only the traversal steps needed to
   * reach the requested points, instead of the whole tree.
   * @param model the model to draw a subset of points from
   * @param pointIndices indices into model->points(), in the order this
   *        model's own query points (and queryPoints/worldPoints output) will
   *        be returned
   * @throw std::invalid_argument if model is null or an index is out of range
   */
  RobotQueryPoints(const std::shared_ptr<const RobotQueryPoints> &model,
                   const std::vector<size_t> &pointIndices);

  /// Return the number of joints spanned by q.
  size_t dof() const { return dof_; }

  /// Return the number of query points.
  size_t nrPoints() const { return points_.size(); }

  /// Return the query points, each in its link's CoM frame.
  const PointOnLinks &points() const { return points_; }

  /**
   * World positions of the query points.
   * @param q stacked joint angles
   * @param wPts filled with the world position of each query point
   * @param ptJacobians if non-null, filled with a 3 x dof matrix per point
   */
  void queryPoints(const gtsam::Vector &q, std::vector<gtsam::Point3> *wPts,
                   std::vector<gtsam::Matrix> *ptJacobians = nullptr) const;

  /**
   * World positions of the query points, one per column.
   * @param q stacked joint angles
   * @returns a 3 x nrPoints matrix of world positions
   */
  gtsam::Matrix worldPoints(const gtsam::Vector &q) const;

  /**
   * World poses of the query points' links, with Jacobians.
   * @param q stacked joint angles
   * @param wTls filled with the world pose of each query point's link
   * @param poseJacobians if non-null, filled with a 6 x dof matrix per point,
   *        in the gtsam Pose3 tangent convention (body-frame twist, rows 0-2
   *        rotation, rows 3-5 translation)
   */
  void queryPoses(const gtsam::Vector &q, std::vector<gtsam::Pose3> *wTls,
                  std::vector<gtsam::Matrix> *poseJacobians = nullptr) const;
};  // \class RobotQueryPoints

}  // namespace gtdynamics
