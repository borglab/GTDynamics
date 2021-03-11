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

using gtsam::Pose3, gtsam::Vector6, gtsam::Vector3;

namespace gtdynamics {

template <typename K, typename V>
std::vector<V> getValues(std::map<K, V> m) {
  std::vector<V> vec;
  vec.reserve(m.size());
  std::transform(m.begin(), m.end(), back_inserter(vec),
                 [](std::pair<K, V> const &pair) { return pair.second; });
  return vec;
}

Robot::Robot(LinkJointPair links_and_joints)
    : name_to_link_(links_and_joints.first),
      name_to_joint_(links_and_joints.second) {}

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
  for (const auto &link : links()) {
    std::cout << link->name() << ":\n";
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

  for (const auto &joint : joints()) {
    std::cout << joint << std::endl;
    // std::cout<<"\tMpc: " << joint->Mpc().rotation().rpy().transpose() << ", "
    // << joint->Mpc().translation() << "\n";
    LinkSharedPtr child_link = joint->child();
    std::cout << "\tpMc_com: "
              << joint->transformTo(child_link).rotation().rpy().transpose()
              << ", "
              << joint->transformTo(child_link).translation().transpose()
              << "\n";
  }
}

FKResults Robot::forwardKinematics(
    const JointValues &joint_angles, const JointValues &joint_vels,
    const boost::optional<std::string> prior_link_name,
    const boost::optional<Pose3> &prior_link_pose,
    const boost::optional<Vector6> &prior_link_twist) const {
  LinkPoses link_poses;
  LinkTwists link_twists;

  // link_poses["aa"] = Pose3();

  // set root link
  LinkSharedPtr root_link;
  // check fixed links
  for (auto &&link : links()) {
    if (link->isFixed()) {
      root_link = link;
      link_poses[link->name()] = link->getFixedPose();
      link_twists[link->name()] = Vector6::Zero();
    }
  }
  if (prior_link_name) {
    root_link = link(*prior_link_name);
    link_poses[*prior_link_name] =
        prior_link_pose ? *prior_link_pose : gtsam::Pose3();
    link_twists[*prior_link_name] =
        prior_link_twist ? *prior_link_twist : gtsam::Vector6::Zero();
  }
  if (link_poses.size() == 0) {
    throw std::runtime_error("cannot find a fixed link");
  }

  // bfs to set the pose
  std::queue<LinkSharedPtr> q;
  q.push(root_link);
  int loop_count = 0;
  while (!q.empty()) {
    auto link1 = q.front();
    const Pose3 T_w1 = link_poses.at(link1->name());
    const Vector6 V_1 = link_twists.at(link1->name());
    q.pop();
    for (JointSharedPtr joint : link1->getJoints()) {
      auto joint_ptr = boost::dynamic_pointer_cast<ScrewJointBase>(joint);
      LinkSharedPtr link2 = joint_ptr->otherLink(link1);
      // calculate the pose and twist of link2
      double joint_angle = joint_angles.at(joint_ptr->name());
      double joint_vel = joint_vels.at(joint_ptr->name());
      const Pose3 T_12 = joint_ptr->transformTo(link1, joint_angle);
      const Pose3 T_21 = joint_ptr->transformFrom(link1, joint_angle);
      const Pose3 T_w2 = T_w1 * T_12;
      const Vector6 V_2 =
          joint_ptr->transformTwistFrom(link1, joint_angle, joint_vel, V_1);

      // check if link 2 is already assigned
      if (link_poses.find(link2->name()) == link_poses.end()) {
        link_poses[link2->name()] = T_w2;
        link_twists[link2->name()] = V_2;
        q.push(link2);
      } else {  // link 2 is already assigned
        Pose3 T_w2_prev = link_poses.at(link2->name());
        Vector6 V_2_prev = link_twists.at(link2->name());
        if (!(T_w2.equals(T_w2_prev, 1e-4) && (V_2 - V_2_prev).norm() < 1e-4)) {
          throw std::runtime_error(
              "inconsistent joint angles detected in forward kinematics");
        }
      }
    }
    if (loop_count++ > 100000) {
      throw std::runtime_error("infinite loop in bfs");
    }
  }
  return FKResults(link_poses, link_twists);
}

}  // namespace gtdynamics.
