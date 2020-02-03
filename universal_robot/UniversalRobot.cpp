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

#include <gtsam/geometry/Pose3.h>
#include <utils.h>

#include <stdexcept>
#include <sstream>
#include <queue>
#include <memory>
#include <algorithm>

#include "RobotTypes.h"

namespace robot {

template <typename K, typename V>
std::vector<V> getValues(std::map<K, V> m) {
  std::vector<V> vec;
  vec.reserve(m.size());
  std::transform (m.begin(), m.end(),back_inserter(vec), [] (std::pair<K, V> const & pair)
																				{
																					return pair.second;
																				});
  return vec;
}

robot::RobotJointParams getJointParams(sdf::Joint &j_i, 
                                      const boost::optional<std::vector<robot::RobotJointParams>> joint_params) {
    robot::RobotJointParams default_params;
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
    return jps;
}


LinkJointPair extract_structure_from_sdf(
    const sdf::Model sdf,
    const boost::optional<std::vector<robot::RobotJointParams>> joint_params) {
  
  // Loop through all links in the urdf interface and construct RobotLink
  // objects without parents or children.
  LinkMap name_to_link;
  for (uint i = 0; i < sdf.LinkCount(); i++) {
    robot::RobotLinkSharedPtr link =
        std::make_shared<robot::RobotLink>(*sdf.LinkByIndex(i));
    link -> setID(i);
    name_to_link.insert(std::make_pair(link->name(), link));
  }

  // Create RobotJoint objects and update list of parent and child links/joints.
  JointMap name_to_joint;
  for (uint j = 0; j < sdf.JointCount(); j++) {
    sdf::Joint sdf_joint = *sdf.JointByIndex(j);

    // Get this joint's parent and child links.
    std::string parent_link_name = sdf_joint.ParentLinkName();
    std::string child_link_name = sdf_joint.ChildLinkName();
    if (parent_link_name == "world") {
      // This joint fixes the child link in the world frame.
      robot::RobotLinkSharedPtr child_link = name_to_link[child_link_name];
      gtsam::Pose3 fixed_pose = child_link->Twcom();
      child_link->fix(fixed_pose);
      continue;
    }
    robot::RobotLinkSharedPtr parent_link = name_to_link[parent_link_name];
    robot::RobotLinkSharedPtr child_link = name_to_link[child_link_name];

    // Obtain joint params.
    robot::RobotJointParams jps = getJointParams(sdf_joint, joint_params);

    // Construct RobotJoint and insert into name_to_joint.
    robot::RobotJointSharedPtr joint =
        std::make_shared<robot::RobotJoint>(robot::RobotJoint(
            sdf_joint, jps.jointEffortType, jps.springCoefficient,
            jps.jointLimitThreshold, jps.velocityLimitThreshold,
            jps.accelerationLimit, jps.accelerationLimitThreshold,
            jps.torqueLimitThreshold, parent_link, child_link));
    name_to_joint.insert(std::make_pair(sdf_joint.Name(), joint));
    joint -> setID(j);

    // Update list of parent and child links/joints for each RobotLink.
    parent_link -> addJoint(joint);
    child_link -> addJoint(joint);
  }

  return std::make_pair(name_to_link, name_to_joint);
}

LinkJointPair extract_structure_from_file(
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

UniversalRobot::UniversalRobot(LinkJointPair links_and_joints)
    : name_to_link_(links_and_joints.first),
      name_to_joint_(links_and_joints.second) {}

UniversalRobot::UniversalRobot(const std::string file_path,
                               std::string model_name)
    : UniversalRobot(extract_structure_from_file(file_path, model_name)) {}

std::vector<RobotLinkSharedPtr> UniversalRobot::links() const {
  return getValues<std::string, robot::RobotLinkSharedPtr>(name_to_link_);
}

std::vector<RobotJointSharedPtr> UniversalRobot::joints() const {
  return getValues<std::string, robot::RobotJointSharedPtr>(name_to_joint_);
}

void UniversalRobot::removeLink(RobotLinkSharedPtr link) {
  // remove all joints associated to the link
  auto joints = link->getJoints();
  for (robot::RobotJointWeakPtr joint : joints) {
    removeJoint(joint.lock());
  }

  // remove link from name_to_link_
  name_to_link_.erase(link->name());
}

void UniversalRobot::removeJoint(RobotJointSharedPtr joint) {
  // in all links connected to the joint, remove the joint
  for (auto link: joint->links())
  {
    link.lock()->removeJoint(joint);
  }
  // Remove the joint from name_to_joint_
  name_to_joint_.erase(joint->name());
}

RobotLinkSharedPtr UniversalRobot::getLinkByName(std::string name) const {
  if (name_to_link_.find(name) == name_to_link_.end()) {
    throw std::runtime_error("no link named " + name);
  }
  return name_to_link_.at(name);
}

RobotJointSharedPtr UniversalRobot::getJointByName(std::string name) const {
  if (name_to_joint_.find(name) == name_to_joint_.end()) {
    throw std::runtime_error("no joint named " + name);
  }
  return name_to_joint_.at(name);
}

int UniversalRobot::numLinks() const { return name_to_link_.size(); }

int UniversalRobot::numJoints() const { return name_to_joint_.size(); }

void UniversalRobot::printRobot() const {
  for (const auto& link : links()) {
    std::cout << link->name() << ":\n";
    std::cout << "\tlink pose: " << link->Twl().rotation().rpy().transpose()
              << ", " << link->Twl().translation() << "\n";
    std::cout << "\tcom pose: " << link->Twcom().rotation().rpy().transpose()
              << ", " << link->Twcom().translation() << "\n";
    std::cout << "\tjoints: ";
    for (const auto& joint : link->getJoints()) {
      std::cout << joint.lock()->name() << " ";
    }
    std::cout << "\n";
  }

  for (const auto& joint : joints()) {
    std::cout << joint->name() << ":\n";
    robot::RobotLinkSharedPtr parent_link = joint->parentLink().lock();
    robot::RobotLinkSharedPtr child_link = joint->childLink().lock();
    std::cout << "\tparent: " << parent_link->name()
              << "\tchild: " << child_link->name() << "\n";
    std::cout << "\tscrew axis: " << joint->screwAxis(child_link).transpose() << "\n";
    // std::cout<<"\tMpc: " << joint->Mpc().rotation().rpy().transpose() << ", "
    // << joint->Mpc().translation() << "\n";
    std::cout << "\tMpc_com: " << joint->transformTo(child_link).rotation().rpy().transpose()
              << ", " << joint->transformTo(child_link).translation() << "\n";
  }
}

UniversalRobot::FKResults UniversalRobot::forwardKinematics(const JointValues &joint_angles, const JointValues &joint_vels,
                              const boost::optional<std::string> prior_link_name, 
                              const boost::optional<gtsam::Pose3> &prior_link_pose, 
                              const boost::optional<gtsam::Vector6>  &prior_link_twist) const
{
  LinkPoses link_poses;
  LinkTwists link_twists;

  // link_poses["aa"] = gtsam::Pose3();

  //// set root link
  robot::RobotLinkSharedPtr root_link;
  // check fixed links
  for (robot::RobotLinkSharedPtr link: links())
  {
    if (link->isFixed())
    {
      root_link = link;
      link_poses[link->name()] = link->getFixedPose();
      link_twists[link->name()] = gtsam::Vector6::Zero();
    }
  }
  if (prior_link_name) {
    root_link = getLinkByName(*prior_link_name);
    link_poses[*prior_link_name] = *prior_link_pose;
    link_twists[*prior_link_name] = *prior_link_twist;
  }
  if (link_poses.size() == 0)
  {
    throw std::runtime_error("cannot find a fixed link");
  }

  //// bfs to set the pose
  std::queue<RobotLinkSharedPtr> q;
  q.push(root_link);
  int loop_count = 0;
  while (!q.empty()) {
    auto link1 = q.front();
    gtsam::Pose3 T_w1 = link_poses.at(link1->name());
    gtsam::Vector6 V_1 = link_twists.at(link1->name());
    q.pop();
    for (robot::RobotJointWeakPtr joint : link1->getJoints()) {
      robot::RobotJointSharedPtr joint_ptr = joint.lock();
      // get the other link
      RobotLinkSharedPtr link2 = joint_ptr->otherLink(link1).lock();
      // calculate the pose and twist of link2
      double joint_angle = joint_angles.at(joint_ptr->name());
      double joint_vel = joint_vels.at(joint_ptr->name());
      gtsam::Pose3 T_12 = joint_ptr->transformTo(link1, joint_angle);
      gtsam::Pose3 T_21 = joint_ptr->transformFrom(link1, joint_angle);
      gtsam::Pose3 T_w2 = T_w1 * T_12;
      gtsam::Vector6 S_2 = joint_ptr->screwAxis(link2);
      gtsam::Vector6 V_2 = T_21.AdjointMap() * V_1 + S_2 * joint_vel;

      // check if link 2 is already assigned
      if (link_poses.find(link2->name())==link_poses.end()) {
        // add to link_poses & link_twists
        link_poses[link2->name()] = T_w2;
        link_twists[link2->name()] = V_2;
        // push link2 to queue
        q.push(link2);
      }
      else { // link 2 is already assigned
        gtsam::assert_equal(T_w2, link_poses.at(link2->name()));
        gtsam::assert_equal(V_2, link_twists.at(link2->name()));
      }
    }
    if (loop_count++ > 100000)
    {
      throw std::runtime_error("infinite loop in bfs");
    }
  }
  return FKResults(link_poses, link_twists);
}


}  // namespace robot.
