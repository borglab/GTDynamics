/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RobotQueryPoints.cpp
 * @brief Forward kinematics of query points on a robot, with Jacobians.
 * @author Karthik Shaji
 */

#include <gtdynamics/gpmp2/RobotQueryPoints.h>

#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
RobotQueryPoints::RobotQueryPoints(const Robot &robot,
                                   const std::string &baseLinkName,
                                   const std::vector<JointSharedPtr> &joints,
                                   const PointOnLinks &points,
                                   const gtsam::Pose3 &wTbase)
    : robot_(robot),
      baseLinkName_(baseLinkName),
      wTbase_(wTbase),
      joints_(joints),
      points_(points) {
  for (size_t i = 0; i < joints_.size(); ++i) {
    jointColumn_[joints_[i]->id()] = i;
  }
}

/* ************************************************************************* */
void RobotQueryPoints::forwardKinematics(
    const gtsam::Vector &q, std::map<uint8_t, gtsam::Pose3> *wTl,
    std::map<uint8_t, gtsam::Matrix> *linkJacobians) const {
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

/* ************************************************************************* */
void RobotQueryPoints::queryPoints(
    const gtsam::Vector &q, std::vector<gtsam::Point3> *wPts,
    std::vector<gtsam::Matrix> *ptJacobians) const {
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

/* ************************************************************************* */
gtsam::Matrix RobotQueryPoints::worldPoints(const gtsam::Vector &q) const {
  std::vector<gtsam::Point3> wPts;
  queryPoints(q, &wPts);
  gtsam::Matrix pts(3, nrPoints());
  for (size_t i = 0; i < nrPoints(); ++i) pts.col(i) = wPts[i];
  return pts;
}

}  // namespace gtdynamics
