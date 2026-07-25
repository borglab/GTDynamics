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

#include <map>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * Maps a stacked joint angle vector q to the world positions of a fixed set of
 * query points on the robot, together with their Jacobians with respect to q.
 * This is the sphere free counterpart of gpmp2's robot model: a query point
 * carries no radius, since the standoff distance lives in the obstacle cost.
 *
 * The joints given to the constructor define the ordering of q. Any joint of
 * the robot left out of that list is not traversed, so the subtree beyond it is
 * absent from the model; a query point on such a link raises an exception.
 * Loops are not supported, only trees.
 */
class RobotQueryPoints {
 private:
  Robot robot_;
  std::string baseLinkName_;
  gtsam::Pose3 wTbase_;
  std::vector<JointSharedPtr> joints_;
  PointOnLinks points_;
  std::map<uint8_t, size_t> jointColumn_;

 public:
  /**
   * Constructor.
   * @param robot the robot model
   * @param baseLinkName the link the kinematic tree is rooted at
   * @param joints the joints spanned by q, in the order q indexes them
   * @param points the query points, each in its link's CoM frame
   * @param wTbase pose of the base link in the world frame
   */
  RobotQueryPoints(const Robot &robot, const std::string &baseLinkName,
                   const std::vector<JointSharedPtr> &joints,
                   const PointOnLinks &points,
                   const gtsam::Pose3 &wTbase = gtsam::Pose3());

  /// Return the number of joints spanned by q.
  size_t dof() const { return joints_.size(); }

  /// Return the number of query points.
  size_t nrPoints() const { return points_.size(); }

  /// Return the query points, each in its link's CoM frame.
  const PointOnLinks &points() const { return points_; }

  /**
   * Forward kinematics over the tree rooted at the base link.
   * @param q stacked joint angles, ordered as the constructor's joints
   * @param wTl filled with the world pose of every reachable link, by link id
   * @param linkJacobians if non-null, filled with d(wTl)/dq, a 6 x dof matrix
   *                      per link
   */
  void forwardKinematics(
      const gtsam::Vector &q, std::map<uint8_t, gtsam::Pose3> *wTl,
      std::map<uint8_t, gtsam::Matrix> *linkJacobians = nullptr) const;

  /**
   * World positions of the query points.
   * @param q stacked joint angles
   * @param wPts filled with the world position of each query point
   * @param ptJacobians if non-null, filled with d(wPt)/dq, a 3 x dof matrix
   *                    per point
   */
  void queryPoints(const gtsam::Vector &q, std::vector<gtsam::Point3> *wPts,
                   std::vector<gtsam::Matrix> *ptJacobians = nullptr) const;

  /**
   * World positions of the query points, one per column.
   * @param q stacked joint angles
   * @returns a 3 x nrPoints matrix of world positions
   */
  gtsam::Matrix worldPoints(const gtsam::Vector &q) const;
};  // \class RobotQueryPoints

}  // namespace gtdynamics
