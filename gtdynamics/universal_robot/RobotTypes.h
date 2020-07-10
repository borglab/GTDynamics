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

#define LINK_TYPEDEF_CLASS_POINTER(Class)                     \
  class Class;                                                \
  typedef std::shared_ptr<Class> Class##SharedPtr;            \
  typedef std::shared_ptr<const Class> Class##ConstSharedPtr; \
  typedef std::weak_ptr<Class> Class##WeakPtr

namespace gtdynamics {

LINK_TYPEDEF_CLASS_POINTER(Link);
LINK_TYPEDEF_CLASS_POINTER(Joint);
LINK_TYPEDEF_CLASS_POINTER(ScrewJoint);
LINK_TYPEDEF_CLASS_POINTER(PrismaticJoint);
LINK_TYPEDEF_CLASS_POINTER(RevoluteJoint);

}  // namespace gtdynamics.

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_ROBOTTYPES_H_
