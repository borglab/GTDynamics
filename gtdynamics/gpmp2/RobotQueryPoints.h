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
  std::string base_link_name_;
  gtsam::Pose3 wTbase_;
  std::vector<JointSharedPtr> joints_;
  std::vector<PointOnLink> points_;
  std::map<uint8_t, size_t> joint_column_;

 public:
  /**
   * Constructor.
   * @param robot the robot model
   * @param base_link_name the link the kinematic tree is rooted at
   * @param joints the joints spanned by q, in the order q indexes them
   * @param points the query points, each in its link's CoM frame
   * @param wTbase pose of the base link in the world frame
   */
  RobotQueryPoints(const Robot &robot, const std::string &base_link_name,
                   const std::vector<JointSharedPtr> &joints,
                   const std::vector<PointOnLink> &points,
                   const gtsam::Pose3 &wTbase = gtsam::Pose3())
      : robot_(robot),
        base_link_name_(base_link_name),
        wTbase_(wTbase),
        joints_(joints),
        points_(points) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      joint_column_[joints_[i]->id()] = i;
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
   * @param Jl if non-null, filled with d(wTl)/dq, a 6 x dof matrix per link
   */
  void forwardKinematics(const gtsam::Vector &q,
                         std::map<uint8_t, gtsam::Pose3> *wTl,
                         std::map<uint8_t, gtsam::Matrix> *Jl = nullptr) const {
    if (static_cast<size_t>(q.size()) != dof()) {
      throw std::invalid_argument(
          "RobotQueryPoints: q size must equal the number of joints.");
    }
    const LinkSharedPtr base = robot_.link(base_link_name_);
    (*wTl)[base->id()] = wTbase_;
    if (Jl) (*Jl)[base->id()] = gtsam::Matrix::Zero(6, dof());

    std::set<uint8_t> visited{base->id()};
    std::queue<LinkSharedPtr> frontier;
    frontier.push(base);

    while (!frontier.empty()) {
      const LinkSharedPtr link = frontier.front();
      frontier.pop();
      for (auto &&joint : link->joints()) {
        auto it = joint_column_.find(joint->id());
        if (it == joint_column_.end()) continue;  // subtree excluded from q
        const LinkSharedPtr other = joint->otherLink(link);
        if (visited.count(other->id())) continue;
        visited.insert(other->id());

        const size_t col = it->second;
        const gtsam::Pose3 &wTp = wTl->at(link->id());
        if (Jl) {
          gtsam::Matrix6 H_wTp;
          gtsam::Vector6 H_qj;
          (*wTl)[other->id()] = joint->poseOf(other, wTp, q(col), H_wTp, H_qj);
          gtsam::Matrix J = H_wTp * Jl->at(link->id());
          J.col(col) += H_qj;
          (*Jl)[other->id()] = J;
        } else {
          (*wTl)[other->id()] = joint->poseOf(other, wTp, q(col));
        }
        frontier.push(other);
      }
    }
  }

  /**
   * World positions of the query points.
   * @param q stacked joint angles
   * @param wPs filled with the world position of each query point
   * @param J if non-null, filled with d(wP)/dq, a 3 x dof matrix per point
   */
  void queryPoints(const gtsam::Vector &q, std::vector<gtsam::Point3> *wPs,
                   std::vector<gtsam::Matrix> *J = nullptr) const {
    std::map<uint8_t, gtsam::Pose3> poses;
    std::map<uint8_t, gtsam::Matrix> link_jacobians;
    forwardKinematics(q, &poses, J ? &link_jacobians : nullptr);

    wPs->resize(nrPoints());
    if (J) J->resize(nrPoints());
    for (size_t i = 0; i < nrPoints(); ++i) {
      const uint8_t id = points_[i].link->id();
      auto it = poses.find(id);
      if (it == poses.end()) {
        throw std::runtime_error(
            "RobotQueryPoints: query point on a link not reachable from the "
            "base through the given joints.");
      }
      if (J) {
        gtsam::Matrix36 H_pose;
        (*wPs)[i] = it->second.transformFrom(points_[i].point, H_pose);
        (*J)[i] = H_pose * link_jacobians.at(id);
      } else {
        (*wPs)[i] = it->second.transformFrom(points_[i].point);
      }
    }
  }
};  // \class RobotQueryPoints

}  // namespace gtdynamics
