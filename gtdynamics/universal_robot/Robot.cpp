/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Robot.h
 * @brief Robot structure.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include "gtdynamics/universal_robot/Robot.h"

#include <gtsam/geometry/Pose3.h>

#include <algorithm>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>

#include "gtdynamics/universal_robot/RobotTypes.h"
#include "gtdynamics/utils/Utils.h"

namespace gtdynamics {

template <typename K, typename V>
std::vector<V> getValues(std::map<K, V> m) {
  std::vector<V> vec;
  vec.reserve(m.size());
  std::transform(m.begin(), m.end(), back_inserter(vec),
                 [](std::pair<K, V> const &pair) { return pair.second; });
  return vec;
}

gtdynamics::JointParams getJointParams(
    const sdf::Joint &joint_i,
    const boost::optional<std::vector<gtdynamics::JointParams>> joint_params) {
  gtdynamics::JointParams default_params;
  gtdynamics::JointParams jps;
  if (joint_params) {
    auto jparams =
        std::find_if(joint_params.get().begin(), joint_params.get().end(),
                     [=](const gtdynamics::JointParams &jps) {
                       return (jps.name == joint_i.Name());
                     });
    jps = jparams == joint_params.get().end() ? default_params : *jparams;
  } else {
    jps = default_params;
  }
  return jps;
}

LinkJointPair extract_structure_from_sdf(
    const sdf::Model sdf,
    const boost::optional<std::vector<gtdynamics::JointParams>> joint_params) {
  // Loop through all links in the urdf interface and construct Link
  // objects without parents or children.
  LinkMap name_to_link;
  for (uint i = 0; i < sdf.LinkCount(); i++) {
    gtdynamics::LinkSharedPtr link =
        std::make_shared<gtdynamics::Link>(*sdf.LinkByIndex(i));
    link->setID(i);
    name_to_link.insert(std::make_pair(link->name(), link));
  }

  // Create Joint objects and update list of parent and child links/joints.
  JointMap name_to_joint;
  for (uint j = 0; j < sdf.JointCount(); j++) {
    sdf::Joint sdf_joint = *sdf.JointByIndex(j);

    // Get this joint's parent and child links.
    std::string parent_link_name = sdf_joint.ParentLinkName();
    std::string child_link_name = sdf_joint.ChildLinkName();
    if (parent_link_name == "world") {
      // This joint fixes the child link in the world frame.
      gtdynamics::LinkSharedPtr child_link = name_to_link[child_link_name];
      gtsam::Pose3 fixed_pose = child_link->wTcom();
      child_link->fix(fixed_pose);
      continue;
    }
    gtdynamics::LinkSharedPtr parent_link = name_to_link[parent_link_name];
    gtdynamics::LinkSharedPtr child_link = name_to_link[child_link_name];

    // Obtain joint params.
    gtdynamics::JointParams jps = getJointParams(sdf_joint, joint_params);

    // Construct Joint and insert into name_to_joint.
    gtdynamics::JointSharedPtr joint = std::make_shared<gtdynamics::Joint>(
        gtdynamics::Joint(sdf_joint, jps.jointEffortType, jps.springCoefficient,
                          jps.jointLimitThreshold, jps.velocityLimitThreshold,
                          jps.accelerationLimit, jps.accelerationLimitThreshold,
                          jps.torqueLimitThreshold, parent_link, child_link));
    name_to_joint.insert(std::make_pair(sdf_joint.Name(), joint));
    joint->setID(j);

    // Update list of parent and child links/joints for each Link.
    parent_link->addJoint(joint);
    child_link->addJoint(joint);
  }

  return std::make_pair(name_to_link, name_to_joint);
}

LinkJointPair extract_structure_from_file(
    const std::string file_path, const std::string model_name,
    const boost::optional<std::vector<gtdynamics::JointParams>> joint_params) {
  std::string file_ext = file_path.substr(file_path.find_last_of(".") + 1);
  std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::tolower);

  if (file_ext == "urdf")
    return extract_structure_from_sdf(get_sdf(file_path), joint_params);
  else if (file_ext == "sdf")
    return extract_structure_from_sdf(get_sdf(file_path, model_name),
                                      joint_params);

  throw std::runtime_error("Invalid file extension.");
}

Robot::Robot(LinkJointPair links_and_joints)
    : name_to_link_(links_and_joints.first),
      name_to_joint_(links_and_joints.second) {}

Robot::Robot(const std::string file_path, std::string model_name)
    : Robot(extract_structure_from_file(file_path, model_name)) {}

std::vector<LinkSharedPtr> Robot::links() const {
  return getValues<std::string, gtdynamics::LinkSharedPtr>(name_to_link_);
}

std::vector<JointSharedPtr> Robot::joints() const {
  return getValues<std::string, gtdynamics::JointSharedPtr>(name_to_joint_);
}

void Robot::removeLink(LinkSharedPtr link) {
  // remove all joints associated to the link
  auto joints = link->getJoints();
  for (gtdynamics::JointSharedPtr joint : joints) {
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

LinkSharedPtr Robot::getLinkByName(std::string name) const {
  if (name_to_link_.find(name) == name_to_link_.end()) {
    throw std::runtime_error("no link named " + name);
  }
  return name_to_link_.at(name);
}

JointSharedPtr Robot::getJointByName(std::string name) const {
  if (name_to_joint_.find(name) == name_to_joint_.end()) {
    throw std::runtime_error("no joint named " + name);
  }
  return name_to_joint_.at(name);
}

int Robot::numLinks() const { return name_to_link_.size(); }

int Robot::numJoints() const { return name_to_joint_.size(); }

void Robot::printRobot() const {
  for (const auto &link : links()) {
    std::cout << link->name() << ":\n";
    std::cout << "\tlink pose: " << link->wTl().rotation().rpy().transpose()
              << ", " << link->wTl().translation() << "\n";
    std::cout << "\tcom pose: " << link->wTcom().rotation().rpy().transpose()
              << ", " << link->wTcom().translation() << "\n";
    std::cout << "\tjoints: ";
    for (const auto &joint : link->getJoints()) {
      std::cout << joint->name() << " ";
    }
    std::cout << "\n";
  }

  for (const auto &joint : joints()) {
    std::cout << joint->name() << ":\n";
    gtdynamics::LinkSharedPtr parent_link = joint->parentLink();
    gtdynamics::LinkSharedPtr child_link = joint->childLink();
    std::cout << "\tparent: " << parent_link->name()
              << "\tchild: " << child_link->name() << "\n";
    std::cout << "\tscrew axis: " << joint->screwAxis(child_link).transpose()
              << "\n";
    // std::cout<<"\tMpc: " << joint->Mpc().rotation().rpy().transpose() << ", "
    // << joint->Mpc().translation() << "\n";
    std::cout << "\tpMc_com: "
              << joint->transformTo(child_link).rotation().rpy().transpose()
              << ", " << joint->transformTo(child_link).translation() << "\n";
  }
}

Robot::FKResults Robot::forwardKinematics(
    const JointValues &joint_angles, const JointValues &joint_vels,
    const boost::optional<std::string> prior_link_name,
    const boost::optional<gtsam::Pose3> &prior_link_pose,
    const boost::optional<gtsam::Vector6> &prior_link_twist) const {
  LinkPoses link_poses;
  LinkTwists link_twists;

  // link_poses["aa"] = gtsam::Pose3();

  //// set root link
  gtdynamics::LinkSharedPtr root_link;
  // check fixed links
  for (gtdynamics::LinkSharedPtr link : links()) {
    if (link->isFixed()) {
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
  if (link_poses.size() == 0) {
    throw std::runtime_error("cannot find a fixed link");
  }

  //// bfs to set the pose
  std::queue<LinkSharedPtr> q;
  q.push(root_link);
  int loop_count = 0;
  while (!q.empty()) {
    auto link1 = q.front();
    gtsam::Pose3 T_w1 = link_poses.at(link1->name());
    gtsam::Vector6 V_1 = link_twists.at(link1->name());
    q.pop();
    for (gtdynamics::JointSharedPtr joint : link1->getJoints()) {
      gtdynamics::JointSharedPtr joint_ptr = joint;
      LinkSharedPtr link2 = joint_ptr->otherLink(link1);
      // calculate the pose and twist of link2
      double joint_angle = joint_angles.at(joint_ptr->name());
      double joint_vel = joint_vels.at(joint_ptr->name());
      gtsam::Pose3 T_12 = joint_ptr->transformTo(link1, joint_angle);
      gtsam::Pose3 T_21 = joint_ptr->transformFrom(link1, joint_angle);
      gtsam::Pose3 T_w2 = T_w1 * T_12;
      gtsam::Vector6 S_2 = joint_ptr->screwAxis(link2);
      gtsam::Vector6 V_2 = T_21.AdjointMap() * V_1 + S_2 * joint_vel;

      // check if link 2 is already assigned
      if (link_poses.find(link2->name()) == link_poses.end()) {
        link_poses[link2->name()] = T_w2;
        link_twists[link2->name()] = V_2;
        q.push(link2);
      } else {  // link 2 is already assigned
        gtsam::Pose3 T_w2_prev = link_poses.at(link2->name());
        gtsam::Vector6 V_2_prev = link_twists.at(link2->name());
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

gtsam::NonlinearFactorGraph Robot::qFactors(const int &t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&link : links())
    graph.add(link->qFactors(t, opt));
  for (auto &&joint : joints())
    graph.add(joint->qFactors(t, opt));
  return graph;
}

gtsam::NonlinearFactorGraph Robot::vFactors(const int &t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&link : links())
    graph.add(link->vFactors(t, opt));
  for (auto &&joint : joints())
    graph.add(joint->vFactors(t, opt));
  return graph;
}

gtsam::NonlinearFactorGraph Robot::aFactors(const int &t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&link : links())
    graph.add(link->aFactors(t, opt));
  for (auto &&joint : joints())
    graph.add(joint->aFactors(t, opt));
  return graph;
}

gtsam::NonlinearFactorGraph Robot::dynamicsFactors(const int &t, const OptimizerSetting &opt,
    const boost::optional<gtsam::Vector3> &planar_axis) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&joint : joints())
    graph.add(joint->dynamicsFactors(t, opt, planar_axis));
  return graph;
}

gtsam::NonlinearFactorGraph Robot::jointLimitFactors(const int &t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto &&joint : joints())
    graph.add(joint->jointLimitFactors(t, opt));
  return graph;
}

}  // namespace gtdynamics.
