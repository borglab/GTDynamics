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

RobotRobotJointPair extract_structure_from_urdf(
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

UniversalRobot::UniversalRobot(RobotRobotJointPair urdf_links_and_joints)
                               : link_bodies_(urdf_links_and_joints.first),
                                 link_joints_(urdf_links_and_joints.second) {

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
  RobotLinkSharedPtr parent_link;
  for (auto&& link_body : link_bodies_) {
    if (link_body->getParentJoints().size()==0) {
      parent_link = link_body;
    }
  }
  parent_link -> setPose(Pose3::identity());

  // bfs to set the pose
  std::queue<RobotLinkSharedPtr> q;
  q.push(parent_link);
  while (!q.empty()) {
    auto this_link = q.front();
    q.pop();
    for (auto&& joint : this_link->getChildJoints()) {
      auto next_joint = joint.lock();
      auto next_link = next_joint->childLink().lock();
      // check if is the first parent
      if (next_link->getParentJoints()[0]->name()==next_joint->name()) {
        if (next_link->isPoseSet()) {
          throw(std::runtime_error("repeat setting pose for Link" + next_link->name()));
        }
        next_link -> setPose(next_joint->pMc() * this_link->getLinkPose());
        q.push(next_link);
      }
    }
  }

  // set the transform for joints
  for (auto&& joint : link_joints_) {
    joint -> setTransform();
  }

}

UniversalRobot::UniversalRobot(const std::string urdf_file_path) : 
  UniversalRobot(extract_structure_from_urdf(get_urdf(load_file_into_string(urdf_file_path)))) {
}

std::vector<RobotLinkSharedPtr> UniversalRobot::links() { return link_bodies_; }

std::vector<RobotJointSharedPtr> UniversalRobot::joints() { return link_joints_; }

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

std::map<std::string, std::map<std::string, gtsam::Pose3>> UniversalRobot::linkTransforms(
        boost::optional<std::map<std::string, double>> joint_name_to_angle
    ) const {

  std::map<std::string, std::map<std::string, gtsam::Pose3>> link_transforms;

  std::map<std::string, double> q_map;
  if (joint_name_to_angle)
    q_map = joint_name_to_angle.get();

  for (auto&& link_body : link_bodies_) {
    // Transform from parent link(s) to the link_body.
    std::map<std::string, gtsam::Pose3> parent_to_link_transforms;

    // Cycle through parent joints and compute transforms.
    for (auto&& link_parent_joint : link_body->getParentJoints()) {
      double q = 0.0;
      if (joint_name_to_angle) {
        if (q_map.find(link_parent_joint->name()) != q_map.end())
          q = q_map[link_parent_joint->name()];
      }

      parent_to_link_transforms.insert(std::make_pair(
        link_parent_joint->parentLink()->name(),
        link_parent_joint->pTc(q)
      ));
    }

    link_transforms.insert(std::make_pair(
      link_body->name(), parent_to_link_transforms));
  }
  return link_transforms;
}

gtsam::Pose3 UniversalRobot::cTpCOM(std::string name, boost::optional<double> q) {

  double q_ = q ? q.get() : 0.0;

  RobotJointSharedPtr link_joint = name_to_link_joint_[name];
  gtsam::Pose3 pTcom = link_joint->parentLink()->centerOfMass();
  gtsam::Pose3 cTcom = link_joint->childLink().lock()->centerOfMass();

  // Return relative pose between pTc_com and pTcom,
  // in pTc_com coordinate frame.
  gtsam::Pose3 pTc_com = link_joint->pTc(q_) * cTcom;
  return pTc_com.between(pTcom);
}

std::map<std::string, std::map<std::string, gtsam::Pose3>> UniversalRobot::cTpCOMs(
        boost::optional<std::map<std::string, double>> joint_name_to_angle) {

  std::map<std::string, std::map<std::string, gtsam::Pose3>> cTp_COMs;

  std::map<std::string, double> q_map;
  if (joint_name_to_angle)
    q_map = joint_name_to_angle.get();

  for (auto&& link_joint : link_joints_)
  {
    // Insert map to contain transform from parent link to child link if not already
    // present.
    if (cTp_COMs.find(link_joint->childLink().lock()->name()) == cTp_COMs.end()) {
      std::map<std::string, gtsam::Pose3> link_to_parent_transforms;
      cTp_COMs.insert(std::make_pair(
        link_joint->childLink().lock()->name(),
        link_to_parent_transforms));
    }

    double q = 0.0;
    if (joint_name_to_angle) {
      if (q_map.find(link_joint->name()) != q_map.end())
        q = q_map[link_joint->name()];
    }

    cTp_COMs[link_joint->childLink().lock()->name()].insert(std::make_pair(
      link_joint->parentLink()->name(),
      cTpCOM(link_joint->name(), q)
    ));
  }
  return cTp_COMs;
}

} // namespace robot.
