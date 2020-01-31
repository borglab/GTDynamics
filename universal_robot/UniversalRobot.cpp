/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file UniversalRobot.h
 * @brief Robot structure.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include "universal_robot/UniversalRobot.h"

#include <queue>
#include <memory>
#include <algorithm>

#include "RobotTypes.h"

// using namespace std;
// using namespace gtsam;

namespace robot {

RobotJointPair extract_structure_from_sdf(
    const sdf::Model sdf,
    const boost::optional<std::vector<robot::RobotJointParams>> joint_params) {
  std::map<std::string, robot::RobotLinkSharedPtr> name_to_link_body;
  std::map<std::string, robot::RobotJointSharedPtr> name_to_link_joint;

  // Loop through all links in the urdf interface and construct RobotLink
  // objects without parents or children.
  for (uint i = 0; i < sdf.LinkCount(); i++) {
    robot::RobotLinkSharedPtr l_i =
        std::make_shared<robot::RobotLink>(*sdf.LinkByIndex(i));
    name_to_link_body.insert(std::make_pair(l_i->name(), l_i));
  }

  robot::RobotJointParams default_params;

  // Create RobotJoint objects and update list of parent and child links/joints.
  for (uint i = 0; i < sdf.JointCount(); i++) {
    sdf::Joint j_i = *sdf.JointByIndex(i);

    // Get this joint's parent and child links.
    std::string parent_link_name = j_i.ParentLinkName();
    std::string child_link_name = j_i.ChildLinkName();

    robot::RobotLinkSharedPtr parent_link_strong, child_link_strong;
    robot::RobotLinkWeakPtr child_link_weak;

    if (parent_link_name == "world") {
      // This joint fixes the child link in the world frame.
      child_link_strong = name_to_link_body[child_link_name];
      gtsam::Pose3 fixed_pose = child_link_strong->Twcom();
      child_link_strong->fix(fixed_pose);
      continue;
    } else {
      parent_link_strong = name_to_link_body[parent_link_name];
      child_link_strong = name_to_link_body[child_link_name];
      child_link_weak = child_link_strong->getWeakPtr();
    }

    // Obtain joint params.
    robot::RobotJointParams jps;
    if (joint_params) {
      auto jparams =
          std::find_if(joint_params.get().begin(), joint_params.get().end(),
                       [=](const robot::RobotJointParams& jps) {
                         return (jps.name == j_i.Name());
                       });
      jps = jparams == joint_params.get().end() ? default_params : *jparams;
    } else {
      jps = default_params;
    }

    // Construct RobotJoint and insert into name_to_link_joint.
    robot::RobotJointSharedPtr joint_strong =
        std::make_shared<robot::RobotJoint>(robot::RobotJoint(
            j_i, jps.jointEffortType, jps.springCoefficient,
            jps.jointLimitThreshold, jps.velocityLimitThreshold,
            jps.accelerationLimit, jps.accelerationLimitThreshold,
            jps.torqueLimitThreshold, parent_link_strong, child_link_weak));

    name_to_link_joint.insert(std::make_pair(j_i.Name(), joint_strong));
    robot::RobotJointWeakPtr joint_weak = joint_strong->getWeakPtr();

    // Update list of parent and child links/joints for each RobotLink.
    parent_link_strong->addChildLink(child_link_weak);
    parent_link_strong->addChildJoint(joint_weak);
    child_link_strong->addParentLink(parent_link_strong);
    child_link_strong->addParentJoint(joint_strong);
  }

  std::vector<robot::RobotLinkSharedPtr> link_bodies;
  for (auto name_link_pair : name_to_link_body)
    link_bodies.push_back(name_link_pair.second);

  std::vector<robot::RobotJointSharedPtr> link_joints;
  for (auto name_joint_pair : name_to_link_joint)
    link_joints.push_back(name_joint_pair.second);

  return std::make_pair(link_bodies, link_joints);
}

RobotJointPair extract_structure_from_file(
    const std::string file_path, const std::string model_name,
    const boost::optional<std::vector<robot::RobotJointParams>> joint_params) {
  std::string file_ext = file_path.substr(file_path.find_last_of(".") + 1);
  std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::tolower);

  if (file_ext == "urdf")
    return extract_structure_from_sdf(get_sdf(file_path), joint_params);
  else if (file_ext == "sdf")
    return extract_structure_from_sdf(get_sdf(file_path, model_name),
                                      joint_params);

  throw std::runtime_error("Invalid file extension.");
}

UniversalRobot::UniversalRobot(RobotJointPair links_and_joints)
    : link_bodies_(links_and_joints.first),
      link_joints_(links_and_joints.second) {
  unsigned char curr_id = 1;

  for (auto&& link_body : link_bodies_) {
    name_to_link_body_.insert(std::make_pair(link_body->name(), link_body));
    link_body->setID(curr_id++);
  }

  curr_id = 1;
  for (auto&& link_joint : link_joints_) {
    name_to_link_joint_.insert(std::make_pair(link_joint->name(), link_joint));
    link_joint->setID(curr_id++);
  }
}

UniversalRobot::UniversalRobot(const std::string file_path,
                               std::string model_name)
    : UniversalRobot(extract_structure_from_file(file_path, model_name)) {}

std::vector<RobotLinkSharedPtr> UniversalRobot::links() const {
  return link_bodies_;
}

std::vector<RobotJointSharedPtr> UniversalRobot::joints() const {
  return link_joints_;
}

void UniversalRobot::removeLink(RobotLinkSharedPtr link) {
  // remove all joints associated to the link
  for (auto&& joint : link->getJoints()) {
    removeJoint(joint);
  }

  // remove link from link_bodies_ and name_to_link_body_
  auto it = std::find(link_bodies_.begin(), link_bodies_.end(), link);
  link_bodies_.erase(it);
  name_to_link_body_.erase(link->name());
}

void UniversalRobot::removeJoint(RobotJointSharedPtr joint) {
  auto parent_link = joint->parentLink();
  auto child_link = joint->childLink().lock();

  // In parent link, remove the joint and child link
  parent_link->removeChildJoint(joint, child_link);

  // In all child link, remove the joint and parent link
  child_link->removeParentJoint(joint, parent_link);

  // Remove the joint from link_joints_ and name_to_link_joint_
  auto it = std::find(link_joints_.begin(), link_joints_.end(), joint);
  link_joints_.erase(it);
  name_to_link_joint_.erase(joint->name());
}

RobotLinkSharedPtr UniversalRobot::getLinkByName(std::string name) {
  return name_to_link_body_[name];
}

RobotJointSharedPtr UniversalRobot::getJointByName(std::string name) {
  return name_to_link_joint_[name];
}

int UniversalRobot::numLinks() const { return link_bodies_.size(); }

int UniversalRobot::numJoints() const { return link_joints_.size(); }

void UniversalRobot::printRobot() const {
  for (const auto& link : link_bodies_) {
    std::cout << link->name() << ":\n";
    std::cout << "\tlink pose: " << link->Twl().rotation().rpy().transpose()
              << ", " << link->Twl().translation() << "\n";
    std::cout << "\tcom pose: " << link->Twcom().rotation().rpy().transpose()
              << ", " << link->Twcom().translation() << "\n";
    std::cout << "\tjoints: ";
    for (const auto& joint : link->getJoints()) {
      std::cout << joint->name() << " ";
    }
    std::cout << "\n";
  }

  for (const auto& joint : link_joints_) {
    std::cout << joint->name() << ":\n";
    std::cout << "\tparent: " << joint->parentLink()->name()
              << "\tchild: " << joint->childLink().lock()->name() << "\n";
    std::cout << "\tscrew axis: " << joint->screwAxis().transpose() << "\n";
    // std::cout<<"\tMpc: " << joint->Mpc().rotation().rpy().transpose() << ", "
    // << joint->Mpc().translation() << "\n";
    std::cout << "\tMpc_com: " << joint->MpcCom().rotation().rpy().transpose()
              << ", " << joint->MpcCom().translation() << "\n";
  }
}

}  // namespace robot.
