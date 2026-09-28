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

using gtsam::Pose3;
using gtsam::Vector6;

namespace gtdynamics {

bool FKTraversal::isCurrent() const {
  for (size_t i = 0; i < links.size(); ++i) {
    if (links[i]->numJoints() != num_joints[i]) return false;
  }
  return true;
}

FKTraversal ComputeFKTraversal(const LinkSharedPtr &root) {
  FKTraversal traversal;
  traversal.links.push_back(root);
  std::unordered_map<const Link *, size_t> link_index{{root.get(), 0}};
  std::unordered_set<const Joint *> visited_joints;

  // BFS, visiting the joints of each link in the same order as they are stored.
  for (size_t i = 0; i < traversal.links.size(); ++i) {
    const LinkSharedPtr link1 = traversal.links[i];
    for (auto &&joint : link1->joints()) {
      if (!visited_joints.insert(joint.get()).second) continue;
      const auto link2 = joint->otherLink(link1);
      const auto it = link_index.find(link2.get());
      if (it == link_index.end()) {
        const size_t j = traversal.links.size();
        link_index.emplace(link2.get(), j);
        traversal.links.push_back(link2);
        traversal.tree_edges.push_back({joint, i, j});
      } else {
        traversal.loop_edges.push_back({joint, i, it->second});
      }
    }
  }
  for (auto &&link : traversal.links) {
    traversal.num_joints.push_back(link->numJoints());
  }
  return traversal;
}

std::shared_ptr<const FKTraversal> FKTraversalCache::get(
    const LinkSharedPtr &root) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto &traversal = traversals_[root.get()];
  if (!traversal || !traversal->isCurrent()) {
    traversal = std::make_shared<const FKTraversal>(ComputeFKTraversal(root));
  }
  return traversal;
}

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

bool Propagate(const JointSharedPtr &joint, const LinkSharedPtr &link1,
               const Pose3 &T_w1, const Vector6 &V_1, size_t t,
               gtsam::Values *values, std::pair<Pose3, Vector6> *poseTwist) {
  InsertZeroDefaults(joint->id(), t, values);
  *poseTwist = joint->otherPoseTwist(link1, T_w1, V_1,
                                     JointAngle(*values, joint->id(), t),
                                     JointVel(*values, joint->id(), t));
  return InsertWithCheck(joint->otherLink(link1)->id(), t, *poseTwist, values);
}

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

}  // namespace gtdynamics
