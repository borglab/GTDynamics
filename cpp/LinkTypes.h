/**
 * @file  LinkTypes.h
 * @brief Link smart pointer types.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#ifndef LINK_MODEL_TYPES_H
#define LINK_MODEL_TYPES_H

#include <memory>

#define LINK_TYPEDEF_CLASS_POINTER(Class) \
class Class; \
typedef std::shared_ptr<Class> Class##SharedPtr; \
typedef std::shared_ptr<const Class> Class##ConstSharedPtr; \
typedef std::weak_ptr<Class> Class##WeakPtr

namespace robot {

LINK_TYPEDEF_CLASS_POINTER(LinkBody);
LINK_TYPEDEF_CLASS_POINTER(LinkJoint);
struct LinkJointParams;

} // namespace robot.

#endif