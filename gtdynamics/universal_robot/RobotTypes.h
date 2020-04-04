/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RobotTypes.h
 * @brief Robot smart pointer types.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_ROBOTTYPES_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_ROBOTTYPES_H_

#include <memory>
#include <string>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#define LINK_TYPEDEF_CLASS_POINTER(Class)                     \
  class Class;                                                \
  typedef std::shared_ptr<Class> Class##SharedPtr;            \
  typedef std::shared_ptr<const Class> Class##ConstSharedPtr; \
  typedef std::weak_ptr<Class> Class##WeakPtr

namespace gtdynamics {

LINK_TYPEDEF_CLASS_POINTER(Link);
template<typename JointAngleType>
class Joint;
template<typename JointAngleType>
using JointSharedPtr = std::shared_ptr<Joint<JointAngleType>>;
template<typename JointAngleType>
using JointConstSharedPtr = std::shared_ptr<const Joint<JointAngleType>>;
template<typename JointAngleType>
using JointWeakPtr = std::weak_ptr<Joint<JointAngleType>>;
// LINK_TYPEDEF_CLASS_POINTER(Joint);
LINK_TYPEDEF_CLASS_POINTER(PrismaticJoint);
LINK_TYPEDEF_CLASS_POINTER(RevoluteJoint);
LINK_TYPEDEF_CLASS_POINTER(SphericalJoint);
struct JointParams;

// map from joint Key to joint angle/vel/accel/torque
typedef gtsam::Values JointValues;

// map from link name to link pose
typedef std::map<std::string, gtsam::Pose3> LinkPoses;

// map from link name to link twist
typedef std::map<std::string, gtsam::Vector6> LinkTwists;

}  // namespace gtdynamics.

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_ROBOTTYPES_H_
