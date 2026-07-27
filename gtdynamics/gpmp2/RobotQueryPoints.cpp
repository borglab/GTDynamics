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

#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
RobotQueryPoints::RobotQueryPoints(const Robot &robot,
                                   const std::string &baseLinkName,
                                   const std::vector<JointSharedPtr> &joints,
                                   const PointOnLinks &points,
                                   const gtsam::Pose3 &wTbase)
    : wTbase_(wTbase), points_(points), dof_(joints.size()) {
  std::map<uint8_t, size_t> jointColumn;
  for (size_t i = 0; i < joints.size(); ++i) {
    if (!joints[i]) {
      throw std::invalid_argument(
          "RobotQueryPoints: joints must not be null.");
    }
    // A repeated joint would leave its earlier column of q unused.
    if (!jointColumn.emplace(joints[i]->id(), i).second) {
      throw std::invalid_argument(
          "RobotQueryPoints: joint " + joints[i]->name() +
          " appears twice in the joint list.");
    }
  }

  // Traverse the tree once, recording each step in topological order.
  const LinkSharedPtr base = robot.link(baseLinkName);
  std::map<uint8_t, size_t> linkSlot{{base->id(), 0}};
  std::queue<LinkSharedPtr> frontier;
  frontier.push(base);
  while (!frontier.empty()) {
    const LinkSharedPtr link = frontier.front();
    frontier.pop();
    for (auto &&joint : link->joints()) {
      auto it = jointColumn.find(joint->id());
      if (it == jointColumn.end()) continue;  // subtree excluded from q
      const LinkSharedPtr other = joint->otherLink(link);
      if (linkSlot.count(other->id())) continue;
      const size_t childSlot = linkSlot.size();
      linkSlot[other->id()] = childSlot;
      steps_.push_back(
          {joint, other, linkSlot.at(link->id()), childSlot, it->second});
      frontier.push(other);
    }
  }

  // Every listed joint must be traversed, or its column of q would be unused.
  if (steps_.size() != joints.size()) {
    for (const auto &joint : joints) {
      bool used = false;
      for (const auto &step : steps_) used = used || step.joint == joint;
      if (!used) {
        throw std::invalid_argument(
            "RobotQueryPoints: joint " + joint->name() +
            " does not connect to the base through the listed joints.");
      }
    }
  }

  pointSlots_.reserve(points_.size());
  for (const auto &point : points_) {
    if (!point.link) {
      throw std::invalid_argument(
          "RobotQueryPoints: a query point's link must not be null.");
    }
    auto it = linkSlot.find(point.link->id());
    if (it == linkSlot.end()) {
      throw std::invalid_argument(
          "RobotQueryPoints: query point on link " + point.link->name() +
          " which the joints do not reach from the base.");
    }
    pointSlots_.push_back(it->second);
  }

  // Size of the pose/Jacobian workspace each evaluation allocates locally.
  nrLinks_ = linkSlot.size();
}

/* ************************************************************************* */
RobotQueryPoints::RobotQueryPoints(
    const std::shared_ptr<const RobotQueryPoints> &model,
    const std::vector<size_t> &pointIndices) {
  if (!model) {
    throw std::invalid_argument(
        "RobotQueryPoints: parent model must not be null.");
  }
  wTbase_ = model->wTbase_;
  dof_ = model->dof_;

  points_.reserve(pointIndices.size());
  std::vector<size_t> globalSlots;
  globalSlots.reserve(pointIndices.size());
  for (size_t idx : pointIndices) {
    if (idx >= model->points_.size()) {
      throw std::invalid_argument(
          "RobotQueryPoints: point index out of range of the parent model.");
    }
    points_.push_back(model->points_[idx]);
    globalSlots.push_back(model->pointSlots_[idx]);
  }

  // A step is needed iff its child slot is needed; walking the parent's
  // topologically ordered steps backward propagates that to every ancestor
  // in a single pass.
  std::vector<bool> needed(model->nrLinks_, false);
  for (size_t slot : globalSlots) needed[slot] = true;
  std::vector<bool> keepStep(model->steps_.size(), false);
  for (size_t i = model->steps_.size(); i-- > 0;) {
    const auto &step = model->steps_[i];
    if (needed[step.childSlot]) {
      keepStep[i] = true;
      needed[step.parentSlot] = true;
    }
  }

  // Compact the retained slots into 0..k-1, with the true base always slot 0.
  // Forward order guarantees a step's parent is already assigned by the time
  // its turn comes, since the parent slot is either 0 or an earlier step's
  // child slot.
  std::vector<size_t> localOfGlobal(model->nrLinks_,
                                    std::numeric_limits<size_t>::max());
  localOfGlobal[0] = 0;
  size_t nextLocal = 1;
  for (size_t i = 0; i < model->steps_.size(); ++i) {
    if (!keepStep[i]) continue;
    const auto &step = model->steps_[i];
    if (localOfGlobal[step.childSlot] == std::numeric_limits<size_t>::max()) {
      localOfGlobal[step.childSlot] = nextLocal++;
    }
    steps_.push_back({step.joint, step.childLink,
                      localOfGlobal[step.parentSlot],
                      localOfGlobal[step.childSlot], step.qCol});
  }
  nrLinks_ = nextLocal;

  pointSlots_.reserve(globalSlots.size());
  for (size_t slot : globalSlots) pointSlots_.push_back(localOfGlobal[slot]);
}

/* ************************************************************************* */
void RobotQueryPoints::computeForwardKinematics(
    const gtsam::Vector &q, std::vector<gtsam::Pose3> *poses,
    gtsam::Matrix *linkJacobians) const {
  if (static_cast<size_t>(q.size()) != dof()) {
    throw std::invalid_argument(
        "RobotQueryPoints: q size must equal the number of joints.");
  }
  (*poses)[0] = wTbase_;
  for (const auto &step : steps_) {
    const gtsam::Pose3 &wTparent = (*poses)[step.parentSlot];
    if (linkJacobians) {
      gtsam::Matrix6 HparentPose;
      gtsam::Vector6 HjointAngle;
      (*poses)[step.childSlot] = step.joint->poseOf(
          step.childLink, wTparent, q(step.qCol), HparentPose, HjointAngle);
      auto child = linkJacobians->middleRows(6 * step.childSlot, 6);
      child.noalias() =
          HparentPose * linkJacobians->middleRows(6 * step.parentSlot, 6);
      child.col(step.qCol) += HjointAngle;
    } else {
      (*poses)[step.childSlot] =
          step.joint->poseOf(step.childLink, wTparent, q(step.qCol));
    }
  }
}

/* ************************************************************************* */
void RobotQueryPoints::queryPoints(
    const gtsam::Vector &q, std::vector<gtsam::Point3> *wPts,
    std::vector<gtsam::Matrix> *ptJacobians) const {
  std::vector<gtsam::Pose3> poses(nrLinks_);
  gtsam::Matrix linkJacobians;
  if (ptJacobians) linkJacobians = gtsam::Matrix::Zero(6 * nrLinks_, dof());
  computeForwardKinematics(q, &poses, ptJacobians ? &linkJacobians : nullptr);

  wPts->resize(nrPoints());
  if (ptJacobians) ptJacobians->resize(nrPoints());
  for (size_t i = 0; i < nrPoints(); ++i) {
    const gtsam::Pose3 &wTl = poses[pointSlots_[i]];
    if (ptJacobians) {
      gtsam::Matrix36 Hpose;
      (*wPts)[i] = wTl.transformFrom(points_[i].point, Hpose);
      (*ptJacobians)[i] =
          Hpose * linkJacobians.middleRows(6 * pointSlots_[i], 6);
    } else {
      (*wPts)[i] = wTl.transformFrom(points_[i].point);
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
