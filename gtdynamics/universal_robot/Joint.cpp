/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Joint.cpp
 * @brief Absract representation of a robot joint.
 */

#include "gtdynamics/universal_robot/Joint.h"

#include <iostream>

namespace gtdynamics {

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const Joint &j) {
  os << j.name() << "\n\tparent link: " << j.parent()->name()
     << "\n\t child link: " << j.child()->name();
  return os;
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const JointSharedPtr &j) {
  os << j->name() << "\n\tparent link: " << j->parent()->name()
     << "\n\t child link: " << j->child()->name();
  return os;
}

}  // namespace gtdynamics