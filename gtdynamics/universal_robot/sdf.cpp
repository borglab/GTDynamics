/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file sdf.cpp
 * @brief Implementation of SDF related utilities.
 * @author Frank Dellaert, Alejandro Escontrela, Stephanie McCormick
 */

#include "gtdynamics/universal_robot/sdf.h"

#include <fstream>
#include <sdf/parser.hh>
#include <sdf/sdf.hh>

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/PrismaticJoint.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/ScrewJoint.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"
#include "gtdynamics/universal_robot/sdf_internal.h"

namespace gtdynamics {

using gtsam::Pose3;

sdf::Model GetSdf(const std::string &sdf_file_path,
                  const std::string &model_name) {
  auto sdf = sdf::readFile(sdf_file_path);

  sdf::Model model = sdf::Model();
  model.Load(sdf->Root()->GetElement("model"));

  // Check whether this is a world file, in which case we have to first
  // access the world element then check whether one of its child models
  // corresponds to model_name.
  if (model.Name() != "__default__") return model;

  // Load the world element.
  sdf::World world = sdf::World();
  world.Load(sdf->Root()->GetElement("world"));

  for (uint i = 0; i < world.ModelCount(); i++) {
    sdf::Model curr_model = *world.ModelByIndex(i);
    if (curr_model.Name() == model_name) return curr_model;
  }

  throw std::runtime_error("Model not found in: " + sdf_file_path);
}

gtsam::Pose3 Pose3FromIgnition(const ignition::math::Pose3d &ignition_pose) {
  const auto &rot = ignition_pose.Rot();
  const auto &pos = ignition_pose.Pos();
  return gtsam::Pose3(
      gtsam::Rot3(gtsam::Quaternion(rot.W(), rot.X(), rot.Y(), rot.Z())),
      gtsam::Point3(pos[0], pos[1], pos[2]));
}

JointParams ParametersFromSdfJoint(const sdf::Joint &sdf_joint) {
  JointParams parameters;

  parameters.scalar_limits.value_lower_limit = sdf_joint.Axis()->Lower();
  parameters.scalar_limits.value_upper_limit = sdf_joint.Axis()->Upper();
  parameters.velocity_limit = sdf_joint.Axis()->MaxVelocity();
  parameters.torque_limit = sdf_joint.Axis()->Effort();
  parameters.damping_coefficient = sdf_joint.Axis()->Damping();

  return parameters;
}

Pose3 GetJointFrame(const sdf::Joint &sdf_joint,
                    const LinkSharedPtr &parent_link,
                    const LinkSharedPtr &child_link) {
  // Name of the coordinate frame the joint's pose is relative to.
  // Specified by `relative_to` in the SDF file.
  std::string frame_name = sdf_joint.PoseRelativeTo();

  // Get the pose of the joint in the parent or child link's frame depending on
  // the value of `frame_name`.
  Pose3 lTj = Pose3FromIgnition(sdf_joint.RawPose());

  if (frame_name.empty() || frame_name == child_link->name()) {
    // If `frame_name` is empty or has the same name as the child_link, it means
    // the joint frame is relative to the child link. So to get the joint pose
    // in the world frame, we pre-multiply by the child link's frame.
    return child_link->wTl() * lTj;

  } else if (frame_name == parent_link->name()) {
    // Else the joint pose is in the frame of the parent link.
    return parent_link->wTl() * lTj;

  } else if (frame_name == "world") {
    // If `frame_name` is "world", the joint pose is already in the world frame.
    return lTj;

  } else {
    // TODO(gchen328): get pose frame from name. Need sdf::Model to do that
    throw std::runtime_error(
        "joint pose frames other than world, parent, or "
        "child not yet supported");
  }
}

gtsam::Vector3 GetSdfAxis(const sdf::Joint &sdf_joint) {
  auto axis = sdf_joint.Axis()->Xyz();
  return gtsam::Vector3(axis[0], axis[1], axis[2]);
}

LinkSharedPtr LinkFromSdf(uint8_t id, const sdf::Link &sdf_link) {
  gtsam::Matrix3 inertia;
  const auto &I = sdf_link.Inertial().Moi();
  inertia << I(0, 0), I(0, 1), I(0, 2), I(1, 0), I(1, 1), I(1, 2), I(2, 0),
      I(2, 1), I(2, 2);

  /// Call SemanticPose::Resolve so the pose is resolved to the correct frame
  /// http://sdformat.org/tutorials?tut=pose_frame_semantics&ver=1.7&cat=specification&
  // Get non-const pose of link in the frame of the joint it is connect to
  // (http://wiki.ros.org/urdf/XML/link).
  auto raw_pose = sdf_link.RawPose();

  // Update from joint frame to base frame in-place.
  // Base frame is denoted by "".
  auto errors = sdf_link.SemanticPose().Resolve(raw_pose, "");
  // If any errors in the resolution, throw an exception.
  if (errors.size() > 0) {
    throw std::runtime_error(errors[0].Message());
  }
  // Pose is updated from joint frame to base frame.
  const auto bTl = Pose3FromIgnition(raw_pose);
  const auto lTcom = Pose3FromIgnition(sdf_link.Inertial().Pose());

  return boost::make_shared<Link>(id, sdf_link.Name(),
                                  sdf_link.Inertial().MassMatrix().Mass(),
                                  inertia, bTl, lTcom);
}

LinkSharedPtr LinkFromSdf(uint8_t id, const std::string &link_name,
                          const std::string &sdf_file_path,
                          const std::string &model_name) {
  auto model = GetSdf(sdf_file_path, model_name);
  return LinkFromSdf(id, *model.LinkByName(link_name));
}

JointSharedPtr JointFromSdf(uint8_t id, const LinkSharedPtr &parent_link,
                            const LinkSharedPtr &child_link,
                            const sdf::Joint &sdf_joint) {
  JointSharedPtr joint;

  // Generate a joint parameters struct with values from the SDF.
  JointParams parameters = ParametersFromSdfJoint(sdf_joint);

  std::string name(sdf_joint.Name());
  Pose3 wTj = GetJointFrame(sdf_joint, parent_link, child_link);

  const gtsam::Vector3 axis = GetSdfAxis(sdf_joint);
  switch (sdf_joint.Type()) {
    case sdf::JointType::PRISMATIC:
      joint = boost::make_shared<PrismaticJoint>(
          id, name, parent_link, child_link, axis, parameters, wTj);
      break;
    case sdf::JointType::REVOLUTE:
      joint = boost::make_shared<RevoluteJoint>(
          id, name, parent_link, child_link, axis, parameters, wTj);
      break;
    case sdf::JointType::SCREW:
      joint = boost::make_shared<ScrewJoint>(id, name, parent_link, child_link,
                                             axis, sdf_joint.ThreadPitch(),
                                             parameters, wTj);
      break;
    default:
      throw std::runtime_error("Joint type for [" + name +
                               "] not yet supported");
  }
  return joint;
}

/**
 * @fn Construct all Link and Joint objects from an input sdf::ElementPtr.
 * @param sdf_ptr a shared pointer to a sdf::ElementPtr containing the model.
 * @return LinkMap and JointMap as a pair
 */
static LinkJointPair ExtractRobotFromSdf(const sdf::Model &sdf) {
  // Loop through all links in the sdf interface and construct Link
  // objects without parents or children.
  LinkMap name_to_link;
  for (uint i = 0; i < sdf.LinkCount(); i++) {
    LinkSharedPtr link = LinkFromSdf(i, *sdf.LinkByIndex(i));
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
    JointSharedPtr joint = JointFromSdf(j, parent_link, child_link, sdf_joint);
    name_to_joint.emplace(joint->name(), joint);

    // Update list of parent and child links/joints for each Link.
    parent_link->addJoint(joint);
    child_link->addJoint(joint);
  }

  return std::make_pair(name_to_link, name_to_joint);
}

/**
 * @fn Construct all Link and Joint objects from an input urdf or sdf file.
 * @param[in] file_path absolute path to the urdf or sdf file containing the
 * robot description.
 * @param[in] model_name name of the robot we care about. Must be specified in
 * case sdf_file_path points to a world file.
 * @return LinkMap and JointMap as a pair
 */
static LinkJointPair ExtractRobotFromFile(const std::string &file_path,
                                          const std::string &model_name) {
  std::ifstream is(file_path);
  if (!is.good())
    throw std::runtime_error("ExtractRobotFromFile: no file found at " +
                             file_path);
  is.close();

  std::string file_ext = file_path.substr(file_path.find_last_of(".") + 1);
  std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::tolower);

  if (file_ext == "urdf")
    return ExtractRobotFromSdf(GetSdf(file_path));
  else if (file_ext == "sdf")
    return ExtractRobotFromSdf(GetSdf(file_path, model_name));

  throw std::runtime_error("Invalid file extension.");
}

Robot CreateRobotFromFile(const std::string &file_path,
                          const std::string &model_name) {
  auto links_joints_pair = ExtractRobotFromFile(file_path, model_name);
  return Robot(links_joints_pair.first, links_joints_pair.second);
}

}  // namespace gtdynamics
