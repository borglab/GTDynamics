/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file JointValue.h
 * @brief JointValue structure.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela, Gerry Chen, Stephanie McCormick
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_JOINTVALUE_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_JOINTVALUE_H_

#include <sdf/parser_urdf.hh>

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <boost/optional.hpp>

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/PrismaticJoint.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/RobotTypes.h"

namespace gtdynamics {

/**
 * Robot is used to create a representation of a robot's
 * inertial/dynamic properties from a URDF/SDF file. The resulting object
 * provides getters for the robot's various joints and links, which can then
 * be fed into an optimization pipeline.
 */
class Robot {
 private:
  char jointType_;
  typedef union jointAngle_ {
      double            q;
      gtsam::Unit3      q2;
      gtsam::Rot3       q3;
  } JointAngle_t;

 public:
  /** Default Constructor */
  JointValue(double q):jointAngle_(q), jointType() {}

  <template tytpe>
  JointValue(type )





#endif