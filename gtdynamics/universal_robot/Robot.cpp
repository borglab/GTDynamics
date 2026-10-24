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

#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/RobotTypes.h>
#include <gtdynamics/utils/utils.h>
#include <gtdynamics/utils/values.h>

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>

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
  resetFKCache();
}

void Robot::removeJoint(const JointSharedPtr &joint) {
  // in all links connected to the joint, remove the joint
  for (auto link : joint->links()) {
    link->removeJoint(joint);
  }
  // Remove the joint from name_to_joint_
  name_to_joint_.erase(joint->name());
  resetFKCache();
}

LinkSharedPtr Robot::link(const std::string &name) const {
  if (name_to_link_.find(name) == name_to_link_.end()) {
    throw std::runtime_error("no link named " + name);
  }
  return name_to_link_.at(name);
}

Robot Robot::fixLink(const std::string &name) const {
  if (name_to_link_.find(name) == name_to_link_.end()) {
    throw std::runtime_error("no link named " + name);
  }

  Robot fixed_robot = Robot(*this);
  fixed_robot.name_to_link_.at(name)->fix();
  return fixed_robot;
}

Robot Robot::unfixLink(const std::string &name) const {
  if (name_to_link_.find(name) == name_to_link_.end()) {
    throw std::runtime_error("no link named " + name);
  }

  Robot unfixed_robot = Robot(*this);
  unfixed_robot.name_to_link_.at(name)->unfix();
  return unfixed_robot;
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
  for (auto &&link : sorted_links) {
    cout << *link;
    cout << "\tjoints: ";
    for (auto &&joint : link->joints()) {
      cout << joint->name() << " ";
    }
    cout << std::endl;
  }

  // Sort joints by id.
  auto sorted_joints = joints();
  std::sort(
      sorted_joints.begin(), sorted_joints.end(),
      [](JointSharedPtr i, JointSharedPtr j) { return i->id() < j->id(); });

  // Print joints in sorted id order.
  cout << "JOINTS:" << endl;
  for (auto &&joint : sorted_joints) {
    cout << joint << endl;

    auto pTc = joint->parentTchild(0.0);
    cout << "\tpMc: " << pTc.rotation().rpy().transpose() << ", "
         << pTc.translation().transpose() << "\n";
  }
}

LinkSharedPtr Robot::findRootLink(
    const gtsam::Values &values,
    const std::optional<std::string> &prior_link_name) const {
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

void Robot::resetFKCache() {
  fk_cache_.clear();
}

gtsam::Values Robot::forwardKinematics(
    const gtsam::Values &known_values, size_t t,
    const std::optional<std::string> &prior_link_name) const {
  gtsam::Values values = known_values;

  // Set root link.
  const auto root_link = findRootLink(values, prior_link_name);
  InsertFixedLinks(links(), t, &values);

  if (!values.exists(PoseKey(root_link->id(), t))) {
    InsertPose(&values, root_link->id(), t, gtsam::Pose3());
  }
  if (!values.exists(TwistKey(root_link->id(), t))) {
    InsertTwist(&values, root_link->id(), t, gtsam::Vector6::Zero());
  }

  // Propagate from the root using the traversal cached for it, which is
  // computed on first use or recomputed if the robot topology changed.
  fk_cache_.get(root_link)->run(t, &values);
  return values;
}

void Robot::renameLinks(const std::map<std::string, std::string> &name_map) {
  LinkMap new_links;
  for (const auto &it : name_to_link_) {
    const std::string &old_name = it.first;
    const std::string &new_name = name_map.at(old_name);
    it.second->rename(new_name);
    new_links.insert({new_name, it.second});
  }
  name_to_link_ = new_links;
}

void Robot::renameJoints(const std::map<std::string, std::string> &name_map) {
  JointMap new_joints;
  for (const auto &it : name_to_joint_) {
    const std::string &old_name = it.first;
    const std::string &new_name = name_map.at(old_name);
    it.second->rename(new_name);
    new_joints.insert({new_name, it.second});
  }
  name_to_joint_ = new_joints;
}

void Robot::reassignLinks(const std::vector<std::string> &ordered_link_names) {
  for (size_t i = 0; i < ordered_link_names.size(); i++) {
    name_to_link_.at(ordered_link_names[i])->reassign(i);
  }
}

void Robot::reassignJoints(
    const std::vector<std::string> &ordered_joint_names) {
  for (size_t i = 0; i < ordered_joint_names.size(); i++) {
    name_to_joint_.at(ordered_joint_names[i])->reassign(i);
  }
}

std::vector<LinkSharedPtr> Robot::orderedLinks() const {
  std::map<uint8_t, LinkSharedPtr> ordered_links;
  for (const auto &it : name_to_link_) {
    ordered_links.insert({it.second->id(), it.second});
  }
  return getValues<uint8_t, LinkSharedPtr>(ordered_links);
}

std::vector<JointSharedPtr> Robot::orderedJoints() const {
  std::map<uint8_t, JointSharedPtr> ordered_joints;
  for (const auto &it : name_to_joint_) {
    ordered_joints.insert({it.second->id(), it.second});
  }
  return getValues<uint8_t, JointSharedPtr>(ordered_joints);
}

}  // namespace gtdynamics.
