/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RobotTypes.h
 * @brief Robot smart pointer types.
 * @author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <boost/shared_ptr.hpp>

#define LINK_TYPEDEF_CLASS_POINTER(Class)                       \
  class Class;                                                  \
  typedef boost::shared_ptr<Class> Class##SharedPtr;            \
  typedef boost::shared_ptr<const Class> Class##ConstSharedPtr; \
  typedef boost::weak_ptr<Class> Class##WeakPtr
