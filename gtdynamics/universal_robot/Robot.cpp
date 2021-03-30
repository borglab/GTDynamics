/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Robot.h
 * @brief Robot structure.
 * @author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include "gtdynamics/universal_robot/Robot.h"

#include <algorithm>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/RobotTypes.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/values.h"

using gtsam::Pose3;
using gtsam::Vector3;
using gtsam::Vector6;

namespace gtdynamics {

template <typename K, typename V> std::vector<V> getValues(std::map<K, V> m) {
  std::vector<V> vec;
  vec.reserve(m.size());
  std::transform(m.begin(), m.end(), back_inserter(vec),
                 [](std::pair<K, V> const &pair) { return pair.second; });
  return vec;
}

Robot::Robot(const LinkMap &links, const JointMap &joints)
    : name_to_link_(links), name_to_joint_(joints) {}

std::vector<LinkSharedPtr> Robot::links() const {
  return getValues<std::string, LinkSharedPtr>(name_to_link_);
}

std::vector<JointSharedPtr> Robot::joints() const {
  return getValues<std::string, JointSharedPtr>(name_to_joint_);
}

void Robot::removeLink(const LinkSharedPtr &link) {
  // remove all joints associated to the link
  auto joints = link->getJoints();
  for (JointSharedPtr joint : joints) {
    removeJoint(joint);
  }

  // remove link from name_to_link_
  name_to_link_.erase(link->name());
}

void Robot::removeJoint(JointSharedPtr joint) {
  // in all links connected to the joint, remove the joint
  for (auto link : joint->links()) {
    link->removeJoint(joint);
  }
  // Remove the joint from name_to_joint_
  name_to_joint_.erase(joint->name());
}

LinkSharedPtr Robot::link(const std::string &name) const {
  if (name_to_link_.find(name) == name_to_link_.end()) {
    throw std::runtime_error("no link named " + name);
  }
  return name_to_link_.at(name);
}

Robot Robot::fixLink(const std::string &name) {
  if (name_to_link_.find(name) == name_to_link_.end()) {
    throw std::runtime_error("no link named " + name);
  }
  name_to_link_.at(name)->fix();
  return *this;
}

JointSharedPtr Robot::joint(const std::string &name) const {
  if (name_to_joint_.find(name) == name_to_joint_.end()) {
    throw std::runtime_error("no joint named " + name);
  }
  return name_to_joint_.at(name);
}

int Robot::numLinks() const { return name_to_link_.size(); }

int Robot::numJoints() const { return name_to_joint_.size(); }

void Robot::print() const {
  auto sorted_links = links();
  std::sort(sorted_links.begin(), sorted_links.end(),
            [](LinkSharedPtr i, LinkSharedPtr j) { return i->id() < j->id(); });

  std::cout << "LINKS:" << std::endl;
  for (const auto &link : sorted_links) {
    std::cout << link->name() << ", id=" << size_t(link->id()) << ":\n";
    std::cout << "\tlink pose: " << link->wTl().rotation().rpy().transpose()
              << ", " << link->wTl().translation().transpose() << "\n";
    std::cout << "\tcom pose: " << link->wTcom().rotation().rpy().transpose()
              << ", " << link->wTcom().translation().transpose() << "\n";
    std::cout << "\tjoints: ";
    for (const auto &joint : link->getJoints()) {
      std::cout << joint->name() << " ";
    }
    std::cout << "\n";
  }

  // Print joints in sorted id order
  auto sorted_joints = joints();
  std::sort(
      sorted_joints.begin(), sorted_joints.end(),
      [](JointSharedPtr i, JointSharedPtr j) { return i->id() < j->id(); });

  std::cout << "JOINTS:" << std::endl;
  for (const auto &joint : sorted_joints) {
    std::cout << joint << std::endl;

    gtsam::Values joint_angles;
    InsertJointAngle(&joint_angles, joint->id(), 0.0);

    auto pTc = joint->parentTchild(joint_angles);
    std::cout << "\tpMc: " << pTc.rotation().rpy().transpose() << ", "
              << pTc.translation().transpose() << "\n";
  }
}

gtsam::Values Robot::forwardKinematics(
    const gtsam::Values &known_values, size_t t,
    const boost::optional<std::string> &prior_link_name) const {
  gtsam::Values values = known_values;

  // Set root link.
  LinkSharedPtr root_link;

  // Use prior_link if given.
  if (prior_link_name) {
    root_link = link(*prior_link_name);
    if (!values.exists(internal::PoseKey(root_link->id(), t)) ||
        !values.exists(internal::TwistKey(root_link->id(), t))) {
      throw std::invalid_argument(
          "forwardKinematics: known_values does not contain pose/twist.");
    }
  } else {
    // Check for fixed links, root link will be last fixed link if any.
    for (auto &&link : links()) {
      if (link->isFixed()) {
        root_link = link;
        InsertPose(&values, link->id(), t, link->getFixedPose());
        InsertTwist(&values, link->id(), t, Vector6::Zero());
      }
    }
    if (!root_link) {
      throw std::runtime_error("forwardKinematics: no prior link given and "
                               "cannot find a fixed link.");
    }
  }

  // BFS to update all poses downstream in the graph.
  std::queue<LinkSharedPtr> q;
  q.push(root_link);
  int loop_count = 0;
  while (!q.empty()) {
    // Pop link from the queue and retrieve the pose and twist.
    LinkSharedPtr link1 = q.front();
    const Pose3 T_w1 = Pose(values, link1->id(), t);
    const Vector6 V_1 = Twist(values, link1->id(), t);
    q.pop();

    // Loop through all joints to find the pose and twist of child links.
    for (auto &&joint : link1->getJoints()) {
      Pose3 T_w2;
      Vector6 V_2;
      std::tie(T_w2, V_2) =
          joint->otherPoseTwist(link1, T_w1, V_1, known_values, t);

      // Save pose and twist if link 2 has not been assigned yet.
      LinkSharedPtr link2 = joint->otherLink(link1);
      auto pose_key = internal::PoseKey(link2->id(), t);
      auto twist_key = internal::TwistKey(link2->id(), t);
      if (!values.exists(pose_key)) {
        values.insert(pose_key, T_w2);
        values.insert<Vector6>(twist_key, V_2);
        q.push(link2);
      } else {
        // If link 2 is already assigned, check for consistency.
        Pose3 T_w2_prev = values.at<Pose3>(pose_key);
        Vector6 V_2_prev = values.at<Vector6>(twist_key);
        if (!(T_w2.equals(T_w2_prev, 1e-4) && (V_2 - V_2_prev).norm() < 1e-4)) {
          throw std::runtime_error(
              "Inconsistent joint angles detected in forward kinematics");
        }
      }
    }
    if (loop_count++ > 100000) {
      throw std::runtime_error("infinite loop in bfs");
    }
  }
  return values;
}

} // namespace gtdynamics.
