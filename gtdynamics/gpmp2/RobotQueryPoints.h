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
#include <queue>
#include <set>
#include <stdexcept>
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
  std::vector<PointOnLink> points_;
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
                   const std::vector<PointOnLink> &points,
                   const gtsam::Pose3 &wTbase = gtsam::Pose3())
      : robot_(robot),
        baseLinkName_(baseLinkName),
        wTbase_(wTbase),
        joints_(joints),
        points_(points) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      jointColumn_[joints_[i]->id()] = i;
    }
  }

  /// Return the number of joints spanned by q.
  size_t dof() const { return joints_.size(); }

  /// Return the number of query points.
  size_t nrPoints() const { return points_.size(); }

  /// Return the query points, each in its link's CoM frame.
  const std::vector<PointOnLink> &points() const { return points_; }

  /**
   * Forward kinematics over the tree rooted at the base link.
   * @param q stacked joint angles, ordered as the constructor's joints
   * @param wTl filled with the world pose of every reachable link, by link id
   * @param linkJacobians if non-null, filled with d(wTl)/dq, a 6 x dof matrix
   *                      per link
   */
  void forwardKinematics(
      const gtsam::Vector &q, std::map<uint8_t, gtsam::Pose3> *wTl,
      std::map<uint8_t, gtsam::Matrix> *linkJacobians = nullptr) const {
    if (static_cast<size_t>(q.size()) != dof()) {
      throw std::invalid_argument(
          "RobotQueryPoints: q size must equal the number of joints.");
    }
    const LinkSharedPtr base = robot_.link(baseLinkName_);
    (*wTl)[base->id()] = wTbase_;
    if (linkJacobians) {
      (*linkJacobians)[base->id()] = gtsam::Matrix::Zero(6, dof());
    }

    std::set<uint8_t> visited{base->id()};
    std::queue<LinkSharedPtr> frontier;
    frontier.push(base);

    while (!frontier.empty()) {
      const LinkSharedPtr link = frontier.front();
      frontier.pop();
      for (auto &&joint : link->joints()) {
        auto it = jointColumn_.find(joint->id());
        if (it == jointColumn_.end()) continue;  // subtree excluded from q
        const LinkSharedPtr other = joint->otherLink(link);
        if (visited.count(other->id())) continue;
        visited.insert(other->id());

        const size_t col = it->second;
        const gtsam::Pose3 &wTparent = wTl->at(link->id());
        if (linkJacobians) {
          gtsam::Matrix6 HparentPose;
          gtsam::Vector6 HjointAngle;
          (*wTl)[other->id()] =
              joint->poseOf(other, wTparent, q(col), HparentPose, HjointAngle);
          gtsam::Matrix jacobian = HparentPose * linkJacobians->at(link->id());
          jacobian.col(col) += HjointAngle;
          (*linkJacobians)[other->id()] = jacobian;
        } else {
          (*wTl)[other->id()] = joint->poseOf(other, wTparent, q(col));
        }
        frontier.push(other);
      }
    }
  }

  /**
   * World positions of the query points.
   * @param q stacked joint angles
   * @param wPts filled with the world position of each query point
   * @param ptJacobians if non-null, filled with d(wPt)/dq, a 3 x dof matrix
   *                    per point
   */
  void queryPoints(const gtsam::Vector &q, std::vector<gtsam::Point3> *wPts,
                   std::vector<gtsam::Matrix> *ptJacobians = nullptr) const {
    std::map<uint8_t, gtsam::Pose3> poses;
    std::map<uint8_t, gtsam::Matrix> linkJacobians;
    forwardKinematics(q, &poses, ptJacobians ? &linkJacobians : nullptr);

    wPts->resize(nrPoints());
    if (ptJacobians) ptJacobians->resize(nrPoints());
    for (size_t i = 0; i < nrPoints(); ++i) {
      const uint8_t id = points_[i].link->id();
      auto it = poses.find(id);
      if (it == poses.end()) {
        throw std::runtime_error(
            "RobotQueryPoints: query point on a link not reachable from the "
            "base through the given joints.");
      }
      if (ptJacobians) {
        gtsam::Matrix36 Hpose;
        (*wPts)[i] = it->second.transformFrom(points_[i].point, Hpose);
        (*ptJacobians)[i] = Hpose * linkJacobians.at(id);
      } else {
        (*wPts)[i] = it->second.transformFrom(points_[i].point);
      }
    }
  }

  /**
   * World positions of the query points, one per column.
   * @param q stacked joint angles
   * @returns a 3 x nrPoints matrix of world positions
   */
  gtsam::Matrix worldPoints(const gtsam::Vector &q) const {
    std::vector<gtsam::Point3> wPts;
    queryPoints(q, &wPts);
    gtsam::Matrix pts(3, nrPoints());
    for (size_t i = 0; i < nrPoints(); ++i) pts.col(i) = wPts[i];
    return pts;
  }
};  // \class RobotQueryPoints

}  // namespace gtdynamics
