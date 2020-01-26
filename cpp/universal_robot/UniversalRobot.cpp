/**
 * @file UniversalRobot.h
 * @brief Robot structure.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include "RobotTypes.h"
#include <UniversalRobot.h>
#include <queue>

// using namespace robot;
using namespace std;
using namespace gtsam;

namespace robot {

RobotJointPair extract_structure_from_urdf(
    const urdf::ModelInterfaceSharedPtr urdf_ptr,
    const boost::optional<std::vector<robot::RobotJointParams>> joint_params) {
  
  std::map<std::string, robot::RobotLinkSharedPtr> name_to_link_body;
  std::map<std::string, robot::RobotJointSharedPtr> name_to_link_joint;

  // Loop through all links in the urdf interface and construct RobotLink objects
  // without parents or children.
  for (auto&& link : urdf_ptr->links_)
    name_to_link_body.insert(std::make_pair(
      link.first,
      std::make_shared<robot::RobotLink>(robot::RobotLink(std::get<1>(link)))
    ));

  robot::RobotJointParams default_params;

  // Create RobotJoint objects and update list of parent and child links/joints.
  for (auto&& joint : urdf_ptr->joints_) {

    robot::RobotLinkSharedPtr parent_link_strong = name_to_link_body[(joint.second)->parent_link_name];
    robot::RobotLinkSharedPtr child_link_strong = name_to_link_body[(joint.second)->child_link_name];
    robot::RobotLinkWeakPtr child_link_weak = name_to_link_body[(joint.second)->child_link_name]->getWeakPtr();

    // Obtain joint params.
    robot::RobotJointParams jps;
    if (joint_params) {
      auto jparams = std::find_if(
        joint_params.get().begin(), joint_params.get().end(),
        [=] (const robot::RobotJointParams & jps) {
          return (jps.name == joint.first);
      });
      jps = jparams == joint_params.get().end() ? default_params : *jparams;
    } else {
      jps = default_params;
    }

    // Construct RobotJoint and insert into name_to_link_joint.
    robot::RobotJointSharedPtr link_joint_strong = std::make_shared<robot::RobotJoint>(
      robot::RobotJoint(
        joint.second, jps.jointEffortType, jps.springCoefficient,
        jps.jointLimitThreshold, jps.velocityLimitThreshold, jps.accelerationLimit,
        jps.accelerationLimitThreshold, jps.torqueLimitThreshold, parent_link_strong,
        child_link_weak));

    name_to_link_joint.insert(std::make_pair(joint.first, link_joint_strong));
    robot::RobotJointWeakPtr link_joint_weak = link_joint_strong->getWeakPtr();

    // Update list of parent and child links/joints for each RobotLink.
    parent_link_strong->addChildLink(child_link_weak);
    parent_link_strong->addChildJoint(link_joint_weak);
    child_link_strong->addParentLink(parent_link_strong);
    child_link_strong->addParentJoint(link_joint_strong);
  }

  std::vector<robot::RobotLinkSharedPtr> link_bodies;
  for (auto name_link_pair : name_to_link_body)
    link_bodies.push_back(name_link_pair.second);

  std::vector<robot::RobotJointSharedPtr> link_joints;
  for (auto name_joint_pair : name_to_link_joint)
    link_joints.push_back(name_joint_pair.second);

  return std::make_pair(link_bodies, link_joints);
}

RobotJointPair extract_structure_from_sdf(
    const sdf::Model sdf,
    const boost::optional<std::vector<robot::RobotJointParams>> joint_params) {
  
  std::map<std::string, robot::RobotLinkSharedPtr> name_to_link_body;
  std::map<std::string, robot::RobotJointSharedPtr> name_to_link_joint;

  // // Loop through all links in the urdf interface and construct RobotLink objects
  // // without parents or children.
  // for (auto&& link : urdf_ptr->links_)
  //   name_to_link_body.insert(std::make_pair(
  //     link.first,
  //     std::make_shared<robot::RobotLink>(robot::RobotLink(std::get<1>(link)))
  //   ));

  // robot::RobotJointParams default_params;

  // // Create RobotJoint objects and update list of parent and child links/joints.
  // for (auto&& joint : urdf_ptr->joints_) {

  //   robot::RobotLinkSharedPtr parent_link_strong = name_to_link_body[(joint.second)->parent_link_name];
  //   robot::RobotLinkSharedPtr child_link_strong = name_to_link_body[(joint.second)->child_link_name];
  //   robot::RobotLinkWeakPtr child_link_weak = name_to_link_body[(joint.second)->child_link_name]->getWeakPtr();

  //   // Obtain joint params.
  //   robot::RobotJointParams jps;
  //   if (joint_params) {
  //     auto jparams = std::find_if(
  //       joint_params.get().begin(), joint_params.get().end(),
  //       [=] (const robot::RobotJointParams & jps) {
  //         return (jps.name == joint.first);
  //     });
  //     jps = jparams == joint_params.get().end() ? default_params : *jparams;
  //   } else {
  //     jps = default_params;
  //   }

  //   // Construct RobotJoint and insert into name_to_link_joint.
  //   robot::RobotJointSharedPtr link_joint_strong = std::make_shared<robot::RobotJoint>(
  //     robot::RobotJoint(
  //       joint.second, jps.jointEffortType, jps.springCoefficient,
  //       jps.jointLimitThreshold, jps.velocityLimitThreshold, jps.accelerationLimit,
  //       jps.accelerationLimitThreshold, jps.torqueLimitThreshold, parent_link_strong,
  //       child_link_weak));

  //   name_to_link_joint.insert(std::make_pair(joint.first, link_joint_strong));
  //   robot::RobotJointWeakPtr link_joint_weak = link_joint_strong->getWeakPtr();

  //   // Update list of parent and child links/joints for each RobotLink.
  //   parent_link_strong->addChildLink(child_link_weak);
  //   parent_link_strong->addChildJoint(link_joint_weak);
  //   child_link_strong->addParentLink(parent_link_strong);
  //   child_link_strong->addParentJoint(link_joint_strong);
  // }

  std::vector<robot::RobotLinkSharedPtr> link_bodies;
  for (auto name_link_pair : name_to_link_body)
    link_bodies.push_back(name_link_pair.second);

  std::vector<robot::RobotJointSharedPtr> link_joints;
  for (auto name_joint_pair : name_to_link_joint)
    link_joints.push_back(name_joint_pair.second);

  return std::make_pair(link_bodies, link_joints);
}

RobotJointPair extract_structure_from_file(
    const std::string file_path,
    const boost::optional<std::vector<robot::RobotJointParams>> joint_params) {


  std::string file_ext = file_path.substr(file_path.find_last_of(".") + 1);
  std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::tolower);

  if (file_ext == "urdf")
    return extract_structure_from_urdf(get_urdf(load_file_into_string(file_path)));
  // else if (file_ext == "sdf")
  //   return extract_structure_from_sdf()

  throw std::runtime_error("Invalid file extension.");

}

UniversalRobot::UniversalRobot(RobotJointPair links_and_joints)
                               : link_bodies_(links_and_joints.first),
                                 link_joints_(links_and_joints.second) {

  unsigned char curr_id = 1;
  
  for (auto&& link_body : link_bodies_) {
    name_to_link_body_.insert(std::make_pair(
      link_body->name(), link_body));
    link_body->setID(curr_id++);
  }
    
  curr_id = 1;
  for (auto&& link_joint : link_joints_)
  {
    name_to_link_joint_.insert(std::make_pair(
      link_joint->name(), link_joint));
    link_joint->setID(curr_id++);
  }

  // calculate the com pose for all links
  // find the link with no parent
  RobotLinkSharedPtr root_link;
  for (auto&& link_body : link_bodies_) {
    if (link_body->getParentJoints().size()==0) {
      root_link = link_body;
    }
  }
  root_link -> setPose(Pose3::identity());

  // bfs to set the pose
  std::queue<RobotLinkSharedPtr> q;
  q.push(root_link);
  while (!q.empty()) {
    auto parent_link = q.front();
    q.pop();
    for (auto&& joint : parent_link->getChildJoints()) {
      auto joint_ptr = joint.lock();
      auto child_link = joint_ptr->childLink().lock();
      // check if is the first parent
      if (child_link->getParentJoints()[0]->name()==joint_ptr->name()) {
        if (child_link->isPoseSet()) {
          throw(std::runtime_error("repeat setting pose for Link" + child_link->name()));
        }
        child_link -> setPose(parent_link->getLinkPose() * joint_ptr->Mpj());
        q.push(child_link);
      }
    }
  }

  // set the transform for joints
  for (auto&& joint : link_joints_) {
    joint -> setTransform();
  }

}

UniversalRobot::UniversalRobot(const std::string file_path) : 
  UniversalRobot(extract_structure_from_file(file_path)) {}

std::vector<RobotLinkSharedPtr> UniversalRobot::links() const { return link_bodies_; }

std::vector<RobotJointSharedPtr> UniversalRobot::joints() const { return link_joints_; }

void UniversalRobot::removeLink(RobotLinkSharedPtr link) {
  // remove all joints associated to the link
  for (auto && joint: link->getJoints()) {
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

std::map<std::string, gtsam::Vector6> UniversalRobot::screwAxes() const {
  std::map<std::string, gtsam::Vector6> screw_axes;

  for (auto&& link_joint : link_joints_)
    screw_axes.insert(std::make_pair(
      link_joint->name(), link_joint->screwAxis()));
  
  return screw_axes;
}

std::map<std::string, double> UniversalRobot::jointLowerLimits() const {
  std::map<std::string, double> joint_lower_limits;

  for (auto&& link_joint : link_joints_)
    joint_lower_limits.insert(std::make_pair(
      link_joint->name(), link_joint->jointLowerLimit()));

  return joint_lower_limits;
}

std::map<std::string, double> UniversalRobot::jointUpperLimits() const {
  std::map<std::string, double> joint_upper_limits;

  for (auto&& link_joint : link_joints_)
    joint_upper_limits.insert(std::make_pair(
      link_joint->name(), link_joint->jointUpperLimit()));

  return joint_upper_limits;
}

std::map<std::string, double> UniversalRobot::jointLimitThresholds() const {
  std::map<std::string, double> joint_limit_threshold;

  for (auto&& link_joint : link_joints_)
    joint_limit_threshold.insert(std::make_pair(
      link_joint->name(), link_joint->jointLimitThreshold()));

  return joint_limit_threshold;
}

RobotJointSharedPtr UniversalRobot::getJointBetweenLinks(
    std::string l1, std::string l2) {

  for (auto&& link_joint : link_joints_) {
    if (link_joint->parentLink()->name() == l1) {
      if (link_joint->childLink().lock()->name() == l2) {
        return link_joint;
      }
    } else if (link_joint->childLink().lock()->name() == l1) {
      if (link_joint->parentLink()->name() == l2) {
        return link_joint;
      }
    }
  }

  // No connecting joint found. Throw error.
  std::ostringstream err_stream;
  err_stream << "Joint connecting ";
  err_stream << l1;
  err_stream << " to ";
  err_stream << l2;
  err_stream << " not found";
  throw std::runtime_error(err_stream.str());
}

void UniversalRobot::printRobot() const {
    for (const auto& link: link_bodies_) {
        std::cout<<link->name() << ":\n";
        std::cout<<"\tlink pose: " << link->getLinkPose().rotation().rpy().transpose() << ", " << link->getLinkPose().translation() << "\n";
        std::cout<<"\tcom pose: " << link->getComPose().rotation().rpy().transpose() << ", " << link->getComPose().translation() << "\n";
        std::cout << "\tjoints: ";
        for (const auto& joint: link->getJoints()) {
          std::cout << joint->name() << " ";
        }
        std::cout << "\n";
    }

    for (const auto& joint: link_joints_) {
        std::cout << joint->name() << ":\n";
        std::cout<<"\tparent: " << joint->parentLink()->name() << "\tchild: " << joint->childLink().lock()->name() << "\n";
        std::cout<<"\tscrew axis: " << joint->screwAxis().transpose() << "\n";
        std::cout<<"\tMpc: " << joint->Mpc().rotation().rpy().transpose() << ", " << joint->Mpc().translation() << "\n";
        std::cout<<"\tMpc_com: " << joint->MpcCom().rotation().rpy().transpose() << ", " << joint->MpcCom().translation() << "\n";
    }
}

} // namespace robot.
