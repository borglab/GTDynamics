#include "gtdynamics/universal_robot/sdf.h"

#include <sdf/parser_urdf.hh>

#include "gtdynamics/universal_robot/PrismaticJoint.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/ScrewJoint.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

using gtsam::Pose3;

static ScrewJointBase::Parameters ParametersFromSDF(
    const sdf::Joint &sdf_joint) {
  ScrewJointBase::Parameters parameters;
  // TODO (stephanie): make this work

  parameters.joint_lower_limit = sdf_joint.Axis()->Lower();
  parameters.joint_upper_limit = sdf_joint.Axis()->Upper();
  parameters.damping_coefficient = sdf_joint.Axis()->Damping();

  // parameters.spring_coefficient = sdf_joint.Axis()->SpringReference() or
  // SpringStiffness()? ; // spring_coeff_(spring_coefficient),

  parameters.velocity_limit = sdf_joint.Axis()->MaxVelocity();

  // No matching function? (/usr/include/sdformat-8.7/sdf)
  // acceleration_limit_(acceleration_limit),

  parameters.torque_limit = sdf_joint.Axis()->Effort();

  return parameters;
}

// TODO(dellaert): document !!!!!!!!!!
static ScrewJointBase::Parameters GetJointParameters(
    JointConstSharedPtr joint, const sdf::Joint &joint_i,
    const boost::optional<std::vector<ScrewJointBase::Parameters>>
        joint_parameters) {
  ScrewJointBase::Parameters default_parameters;
  ScrewJointBase::Parameters jps;
  if (joint_parameters) {
    auto jparameters = std::find_if(joint_parameters.get().begin(),
                                    joint_parameters.get().end(),
                                    [=](const ScrewJointBase::Parameters &jps) {
                                      return (joint->name() == joint_i.Name());
                                    });
    jps = jparameters == joint_parameters.get().end() ? default_parameters
                                                      : *jparameters;
  } else {
    jps = default_parameters;
  }
  return jps;
}

Pose3 GetJointFrame(const sdf::Joint &sdf_joint,
                           const LinkSharedPtr &parent_link,
                           const LinkSharedPtr &child_link) {
  auto frame = sdf_joint.PoseFrame();
  if (frame == "" || frame == child_link->name()) {
    if (sdf_joint.Pose() == ignition::math::Pose3d())
      return child_link->wTl();
    else
      return child_link->wTl() * parse_ignition_pose(sdf_joint.Pose());
  } else if (frame == parent_link->name()) {
    if (sdf_joint.Pose() == ignition::math::Pose3d())
      return parent_link->wTl();
    else
      return parent_link->wTl() * parse_ignition_pose(sdf_joint.Pose());
  } else if (frame == "world") {
    return parse_ignition_pose(sdf_joint.Pose());
  } else {
    // TODO(gchen328): get pose frame from name. Need sdf::Model to do that
    throw std::runtime_error(
        "joint pose frames other than world, parent, or "
        "child not yet supported");
  }
}

static gtsam::Vector3 GetSdfAxis(const sdf::Joint &sdf_joint) {
  auto axis = sdf_joint.Axis()->Xyz();
  return gtsam::Vector3(axis[0], axis[1], axis[2]);
}

/** @fn Construct all Link and Joint objects from an input sdf::ElementPtr.
 * @param sdf_ptr a shared pointer to a sdf::ElementPtr containing the model.
 * @param joint_parameters a vector containing optional parameters for joints.
 * @return LinkMap and JointMap as a pair
 */
LinkJointPair ExtractRobotFromSdf(
    const sdf::Model sdf,
    const boost::optional<std::vector<ScrewJointBase::Parameters>>
        joint_parameters) {
  // Loop through all links in the urdf interface and construct Link
  // objects without parents or children.
  LinkMap name_to_link;
  for (uint i = 0; i < sdf.LinkCount(); i++) {
    LinkSharedPtr link = std::make_shared<Link>(*sdf.LinkByIndex(i));
    link->setID(i);
    name_to_link.emplace(link->name(), link);
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
      LinkSharedPtr child_link = name_to_link[child_link_name];
      Pose3 fixed_pose = child_link->wTcom();
      child_link->fix(fixed_pose);
      continue;
    }
    LinkSharedPtr parent_link = name_to_link[parent_link_name];
    LinkSharedPtr child_link = name_to_link[child_link_name];

    // Construct Joint and insert into name_to_joint.
    JointSharedPtr joint;

    // Obtain joint parameters.
    ScrewJointBase::Parameters parameters =
        GetJointParameters(joint, sdf_joint, joint_parameters);

    std::string name(sdf_joint.Name());
    Pose3 wTj = GetJointFrame(sdf_joint, parent_link, child_link);

    const gtsam::Vector3 axis = GetSdfAxis(sdf_joint);
    switch (sdf_joint.Type()) {
      case sdf::JointType::PRISMATIC:
        joint = std::make_shared<PrismaticJoint>(name, wTj, parent_link,
                                                 child_link, parameters, axis);
        break;
      case sdf::JointType::REVOLUTE:
        joint = std::make_shared<RevoluteJoint>(name, wTj, parent_link,
                                                child_link, parameters, axis);
        break;
      case sdf::JointType::SCREW:
        joint = std::make_shared<ScrewJoint>(name, wTj, parent_link, child_link,
                                             parameters, axis,
                                             sdf_joint.ThreadPitch());
        break;
      default:
        throw std::runtime_error("Joint type for [" + name +
                                 "] not yet supported");
    }

    name_to_joint.emplace(name, joint);
    joint->setID(j);

    // Update list of parent and child links/joints for each Link.
    parent_link->addJoint(joint);
    child_link->addJoint(joint);
  }

  return std::make_pair(name_to_link, name_to_joint);
}

/** @fn Construct all Link and Joint objects from an input urdf or sdf file.
 * @param[in] file_path absolute path to the urdf or sdf file containing the
 * robot description.
 * @param[in] model_name name of the robot we care about. Must be specified
 in
 * case sdf_file_path points to a world file.
 * @param[in] joint_parameters a vector contanining optional parameters for
 * joints.
 * @return LinkMap and JointMap as a pair
 */
LinkJointPair ExtractRobotFromFile(
    const std::string file_path, const std::string model_name,
    const boost::optional<std::vector<ScrewJointBase::Parameters>>
        joint_parameters = boost::none) {
  std::string file_ext = file_path.substr(file_path.find_last_of(".") + 1);
  std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::tolower);

  if (file_ext == "urdf")
    return ExtractRobotFromSdf(get_sdf(file_path), joint_parameters);
  else if (file_ext == "sdf")
    return ExtractRobotFromSdf(get_sdf(file_path, model_name),
                               joint_parameters);

  throw std::runtime_error("Invalid file extension.");
}

Robot CreateRobotFromFile(const std::string file_path, std::string model_name) {
  return Robot(ExtractRobotFromFile(file_path, model_name));
}

}  // namespace gtdynamics