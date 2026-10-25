/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ForwardKinematics.cpp
 * @brief Forward kinematics for the universal robot.
 * @author: Varun Agrawal
 */

#include <gtdynamics/universal_robot/ForwardKinematics.h>
#include <gtdynamics/utils/values.h>

#include <queue>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

using gtsam::Pose3;
using gtsam::Vector6;

namespace gtdynamics {

namespace {

// Add zero default values for joint angles and joint velocities.
// if they do not yet exist
void InsertZeroDefaults(size_t j, size_t t, gtsam::Values *values) {
  for (const auto key : {JointAngleKey(j, t), JointVelKey(j, t)}) {
    if (!values->exists(key)) {
      values->insertDouble(key, 0.0);
    }
  }
}

// Insert a pose/twist into values, but if they already are present, just check
// if they are consistent. Throw exception otherwise.
// Returns true if values were inserted.
bool InsertWithCheck(size_t i, size_t t,
                     const std::pair<Pose3, Vector6> &poseTwist,
                     gtsam::Values *values) {
  Pose3 pose;
  Vector6 twist;
  std::tie(pose, twist) = poseTwist;
  auto pose_key = PoseKey(i, t);
  auto twist_key = TwistKey(i, t);
  const bool exists = values->exists(pose_key);
  if (!exists) {
    values->insert(pose_key, pose);
    values->insert<Vector6>(twist_key, twist);
  } else {
    // If already insert, check for consistency.
    if (!(pose.equals(values->at<Pose3>(pose_key), 1e-4) &&
          (twist - values->at<Vector6>(twist_key)).norm() < 1e-4)) {
      throw std::runtime_error(
          "Inconsistent joint angles detected in forward kinematics");
    }
  }
  return !exists;
}

// Compute pose/twist of the link on the other side of `joint`,
// insert them into values (or check consistency if already present)
// and return whether they were inserted.
bool Propagate(const JointSharedPtr &joint, const LinkSharedPtr &link1,
               const Pose3 &T_w1, const Vector6 &V_1, size_t t,
               gtsam::Values *values, std::pair<Pose3, Vector6> *poseTwist) {
  InsertZeroDefaults(joint->id(), t, values);
  *poseTwist = joint->otherPoseTwist(link1, T_w1, V_1,
                                     JointAngle(*values, joint->id(), t),
                                     JointVel(*values, joint->id(), t));
  return InsertWithCheck(joint->otherLink(link1)->id(), t, *poseTwist, values);
}

// Forward kinematics by BFS, discovering the traversal order on the fly.
void BreadthFirstKinematics(const LinkSharedPtr &root_link, size_t t,
                            gtsam::Values *values) {
  std::queue<LinkSharedPtr> q;
  q.push(root_link);
  std::pair<Pose3, Vector6> poseTwist;
  while (!q.empty()) {
    // Pop link from the queue and retrieve the pose and twist.
    const auto link1 = q.front();
    const Pose3 T_w1 = Pose(*values, link1->id(), t);
    const Vector6 V_1 = Twist(*values, link1->id(), t);
    q.pop();

    // Loop through all joints to find the pose and twist of child links.
    for (auto &&joint : link1->joints()) {
      if (Propagate(joint, link1, T_w1, V_1, t, values, &poseTwist)) {
        q.push(joint->otherLink(link1));
      }
    }
  }
}

}  // namespace

/* ************************************************************************* */
FKTraversal::FKTraversal(const LinkSharedPtr &root) {
  links_.push_back(root);
  std::unordered_map<const Link *, size_t> link_index{{root.get(), 0}};
  std::unordered_set<const Joint *> visited_joints;

  // BFS, visiting the joints of each link in the same order as they are stored.
  for (size_t i = 0; i < links_.size(); ++i) {
    const LinkSharedPtr link1 = links_[i];
    for (auto &&joint : link1->joints()) {
      if (!visited_joints.insert(joint.get()).second) continue;
      const auto link2 = joint->otherLink(link1);
      const auto it = link_index.find(link2.get());
      if (it == link_index.end()) {
        const size_t j = links_.size();
        link_index.emplace(link2.get(), j);
        links_.push_back(link2);
        tree_edges_.push_back({joint, i, j});
      } else {
        loop_edges_.push_back({joint, i, it->second});
      }
    }
  }
  for (auto &&link : links_) {
    versions_.push_back(link->topologyVersion());
  }
}

/* ************************************************************************* */
bool FKTraversal::isCurrent() const {
  for (size_t i = 0; i < links_.size(); ++i) {
    if (links_[i]->topologyVersion() != versions_[i]) return false;
  }
  return true;
}

/* ************************************************************************* */
void FKTraversal::run(size_t t, gtsam::Values *values) const {
  // Links with known poses are checked, but not expanded further. In graphs
  // with kinematic loops this changes which spanning tree BFS finds, so fall
  // back to discovering the traversal on the fly.
  if (!loop_edges_.empty()) {
    for (size_t i = 1; i < links_.size(); ++i) {
      if (values->exists(PoseKey(links_[i]->id(), t))) {
        BreadthFirstKinematics(links_[0], t, values);
        return;
      }
    }
  }

  std::vector<Pose3> poses(links_.size());
  std::vector<Vector6> twists(links_.size());
  std::vector<bool> expanded(links_.size(), false);
  poses[0] = Pose(*values, links_[0]->id(), t);
  twists[0] = Twist(*values, links_[0]->id(), t);
  expanded[0] = true;

  // Walk the spanning tree, parents before children.
  std::pair<Pose3, Vector6> poseTwist;
  for (const Edge &edge : tree_edges_) {
    if (!expanded[edge.from]) continue;
    if (Propagate(edge.joint, links_[edge.from], poses[edge.from],
                  twists[edge.from], t, values, &poseTwist)) {
      std::tie(poses[edge.to], twists[edge.to]) = poseTwist;
      expanded[edge.to] = true;
    }
  }

  // Check that loop closing joints are consistent, from both sides.
  for (const Edge &edge : loop_edges_) {
    for (const size_t i : {edge.from, edge.to}) {
      if (!expanded[i]) continue;
      Propagate(edge.joint, links_[i], poses[i], twists[i], t, values,
                &poseTwist);
    }
  }
}

/* ************************************************************************* */
FKTraversalCache::FKTraversalCache(const FKTraversalCache &other) {
  std::lock_guard<std::mutex> lock(other.mutex_);
  traversals_ = other.traversals_;
}

FKTraversalCache &FKTraversalCache::operator=(const FKTraversalCache &other) {
  if (this != &other) {
    std::scoped_lock lock(mutex_, other.mutex_);
    traversals_ = other.traversals_;
  }
  return *this;
}

FKTraversalCache::FKTraversalCache(FKTraversalCache &&other) {
  std::lock_guard<std::mutex> lock(other.mutex_);
  traversals_ = std::move(other.traversals_);
}

FKTraversalCache &FKTraversalCache::operator=(FKTraversalCache &&other) {
  if (this != &other) {
    std::scoped_lock lock(mutex_, other.mutex_);
    traversals_ = std::move(other.traversals_);
  }
  return *this;
}

void FKTraversalCache::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  traversals_.clear();
}

std::shared_ptr<const FKTraversal> FKTraversalCache::get(
    const LinkSharedPtr &root) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto &traversal = traversals_[root.get()];
  if (!traversal || !traversal->isCurrent()) {
    traversal = std::make_shared<const FKTraversal>(root);
  }
  return traversal;
}

}  // namespace gtdynamics
