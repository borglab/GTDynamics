/**
 * @file UniversalRobot.h
 * @brief Robot structure.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include <UniversalRobot.h>

// using namespace robot;
using namespace std;
using namespace gtsam;

namespace robot {

LinkBodyJointPair extract_structure_from_urdf(
    const urdf::ModelInterfaceSharedPtr urdf_ptr,
    const boost::optional<std::vector<robot::LinkJointParams>> joint_params) {
  
  std::map<std::string, robot::LinkBodySharedPtr> name_to_link_body;
  std::map<std::string, robot::LinkJointSharedPtr> name_to_link_joint;

  // Loop through all links in the urdf interface and construct LinkBody objects
  // without parents or children.
  for (auto&& link : urdf_ptr->links_)
    name_to_link_body.insert(std::make_pair(
      link.first,
      std::make_shared<robot::LinkBody>(robot::LinkBody(std::get<1>(link)))
    ));

  robot::LinkJointParams default_params;

  // Create LinkJoint objects and update list of parent and child links/joints.
  for (auto&& joint : urdf_ptr->joints_) {

    robot::LinkBodySharedPtr parent_link_strong = name_to_link_body[(joint.second)->parent_link_name];
    robot::LinkBodySharedPtr child_link_strong = name_to_link_body[(joint.second)->child_link_name];
    robot::LinkBodyWeakPtr child_link_weak = name_to_link_body[(joint.second)->child_link_name]->getWeakPtr();

    // Obtain joint params.
    robot::LinkJointParams jps;
    if (joint_params) {
      auto jparams = std::find_if(
        joint_params.get().begin(), joint_params.get().end(),
        [=] (const robot::LinkJointParams & jps) {
          return (jps.name == joint.first);
      });
      jps = jparams == joint_params.get().end() ? default_params : *jparams;
    } else {
      jps = default_params;
    }

    // TODO(aescontrela): Make LinkJoint constructor that takes the 
    // urdf::LinkSharedPtr object, the parent link, the child link, and computes 
    // requires attributes.
    // Construct LinkJoint and insert into name_to_link_joint.
    robot::LinkJointSharedPtr link_joint_strong = std::make_shared<robot::LinkJoint>(
      robot::LinkJoint(
        joint.second, jps.jointEffortType, jps.springCoefficient,
        jps.jointLimitThreshold, jps.velocityLimitThreshold, jps.accelerationLimit,
        jps.accelerationLimitThreshold, jps.torqueLimitThreshold, parent_link_strong,
        child_link_weak));

    name_to_link_joint.insert(std::make_pair(joint.first, link_joint_strong));
    robot::LinkJointWeakPtr link_joint_weak = link_joint_strong->getWeakPtr();

    // Update list of parent and child links/joints for each LinkBody.
    parent_link_strong->addChildLink(child_link_weak);
    parent_link_strong->addChildJoint(link_joint_weak);
    child_link_strong->addParentLink(parent_link_strong);
    child_link_strong->addParentJoint(link_joint_strong);
  }

  std::vector<robot::LinkBodySharedPtr> link_bodies;
  for (auto name_link_pair : name_to_link_body)
    link_bodies.push_back(name_link_pair.second);

  std::vector<robot::LinkJointSharedPtr> link_joints;
  for (auto name_joint_pair : name_to_link_joint)
    link_joints.push_back(name_joint_pair.second);

  return std::make_pair(link_bodies, link_joints);
}

UniversalRobot::UniversalRobot(const urdf::ModelInterfaceSharedPtr robot_urdf,
                               const gtsam::Pose3 &base)
                               : robot_urdf_(robot_urdf), base_(base) {
    
}

const gtsam::Pose3& UniversalRobot::base() const {
    return base_;
}

} // namespace robot.
