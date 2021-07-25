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

template <typename K, typename V>
std::vector<V> getValues(std::map<K, V> m) {
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
  auto joints = link->joints();
  for (JointSharedPtr joint : joints) {
    removeJoint(joint);
  }

  // remove link from name_to_link_
  name_to_link_.erase(link->name());
}

void Robot::removeJoint(const JointSharedPtr &joint) {
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

void Robot::print(const std::string &s) const {
  using std::cout;
  using std::endl;

  cout << (s.empty() ? s : s + " ") << endl;

  // Sort joints by id.
  auto sorted_links = links();
  std::sort(sorted_links.begin(), sorted_links.end(),
            [](LinkSharedPtr i, LinkSharedPtr j) { return i->id() < j->id(); });

  // Print links in sorted id order.
  cout << "LINKS:" << endl;
  for (const auto &link : sorted_links) {
    std::string fixed = link->isFixed() ? " (fixed)" : "";
    cout << link->name() << ", id=" << size_t(link->id()) << fixed << ":\n";
    cout << "\tlink pose: " << link->wTl().rotation().rpy().transpose() << ", "
         << link->wTl().translation().transpose() << "\n";
    cout << "\tcom pose: " << link->wTcom().rotation().rpy().transpose() << ", "
         << link->wTcom().translation().transpose() << "\n";
    cout << "\tjoints: ";
    for (const auto &joint : link->joints()) {
      cout << joint->name() << " ";
    }
    cout << "\n";
  }

  // Sort joints by id.
  auto sorted_joints = joints();
  std::sort(
      sorted_joints.begin(), sorted_joints.end(),
      [](JointSharedPtr i, JointSharedPtr j) { return i->id() < j->id(); });

  // Print joints in sorted id order.
  cout << "JOINTS:" << endl;
  for (const auto &joint : sorted_joints) {
    cout << joint << endl;

    gtsam::Values joint_angles;
    InsertJointAngle(&joint_angles, joint->id(), 0.0);

    auto pTc = joint->parentTchild(joint_angles);
    cout << "\tpMc: " << pTc.rotation().rpy().transpose() << ", "
         << pTc.translation().transpose() << "\n";
  }
}

LinkSharedPtr Robot::findRootLink(
    const gtsam::Values &values,
    const boost::optional<std::string> &prior_link_name) const {
  LinkSharedPtr root_link;

  // Use prior_link if given.
  if (prior_link_name) {
    root_link = link(*prior_link_name);
  } else {
    auto links = this->links();
    auto links_iter =
        std::find_if(links.rbegin(), links.rend(),
                     [](const LinkSharedPtr &link) { return link->isFixed(); });
    // If valid link is found by find_if, assign root_link to the iterator.
    if (links_iter != links.rend()) {
      root_link = *links_iter;
    }
  }
  if (!root_link) {
    throw std::runtime_error(
        "forwardKinematics: no prior link given and "
        "cannot find a fixed link.");
  }

  return root_link;
}

// Insert fixed link poses into values
static void InsertFixedLinks(const std::vector<LinkSharedPtr> &links, size_t t,
                             gtsam::Values *values) {
  for (auto &&link : links) {
    if (link->isFixed()) {
      InsertPose(values, link->id(), t, link->getFixedPose());
      InsertTwist(values, link->id(), t, Vector6::Zero());
    }
  }
}

// Add zero default values for joint angles and joint velocities.
// if they do not yet exist
static void InsertZeroDefaults(size_t j, size_t t, gtsam::Values *values) {
  using namespace internal;
  for (const auto key : {JointAngleKey(j, t), JointVelKey(j, t)}) {
    if (!values->exists(key)) {
      values->insertDouble(key, 0.0);
    }
  }
}

// Insert a pose/twist into values, but if they already are present, just check
// if they are consistent. Throw exception otherwise.
// Returns true if values were inserted.
static bool InsertWithCheck(size_t i, size_t t,
                            const std::pair<Pose3, Vector6> &poseTwist,
                            gtsam::Values *values) {
  Pose3 pose;
  Vector6 twist;
  std::tie(pose, twist) = poseTwist;
  auto pose_key = internal::PoseKey(i, t);
  auto twist_key = internal::TwistKey(i, t);
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

gtsam::Values Robot::forwardKinematics(
    const gtsam::Values &known_values, size_t t,
    const boost::optional<std::string> &prior_link_name) const {
  gtsam::Values values = known_values;

  // Set root link.
  const auto root_link = findRootLink(values, prior_link_name);
  InsertFixedLinks(links(), t, &values);

  if (!values.exists(internal::PoseKey(root_link->id(), t))) {
    InsertPose(&values, root_link->id(), t, gtsam::Pose3());
  }
  if (!values.exists(internal::TwistKey(root_link->id(), t))) {
    InsertTwist(&values, root_link->id(), t, gtsam::Vector6::Zero());
  }

  // BFS to update all poses downstream in the graph.
  std::queue<LinkSharedPtr> q;
  q.push(root_link);
  int loop_count = 0;
  while (!q.empty()) {
    // Pop link from the queue and retrieve the pose and twist.
    const auto link1 = q.front();
    const Pose3 T_w1 = Pose(values, link1->id(), t);
    const Vector6 V_1 = Twist(values, link1->id(), t);
    q.pop();

    // Loop through all joints to find the pose and twist of child links.
    for (auto &&joint : link1->joints()) {
      InsertZeroDefaults(joint->id(), t, &values);
      const auto poseTwist = joint->otherPoseTwist(link1, T_w1, V_1, values, t);
      const auto link2 = joint->otherLink(link1);
      if (InsertWithCheck(link2->id(), t, poseTwist, &values)) {
        q.push(link2);
      }
    }
    if (loop_count++ > 100000) {
      throw std::runtime_error("infinite loop in bfs");
    }
  }
  return values;
}

}  // namespace gtdynamics.
