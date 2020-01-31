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

#ifndef UNIVERSAL_ROBOT_ROBOTTYPES_H_
#define UNIVERSAL_ROBOT_ROBOTTYPES_H_

#include <memory>

#define LINK_TYPEDEF_CLASS_POINTER(Class)                     \
  class Class;                                                \
  typedef std::shared_ptr<Class> Class##SharedPtr;            \
  typedef std::shared_ptr<const Class> Class##ConstSharedPtr; \
  typedef std::weak_ptr<Class> Class##WeakPtr

namespace robot {

LINK_TYPEDEF_CLASS_POINTER(RobotLink);
LINK_TYPEDEF_CLASS_POINTER(RobotJoint);
struct RobotJointParams;

}  // namespace robot.

#endif  // UNIVERSAL_ROBOT_ROBOTTYPES_H_
