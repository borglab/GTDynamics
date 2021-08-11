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

#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

/* ************************************************************************* */
Joint::Joint(uint8_t id, const std::string &name, const Pose3 &wTj,
             const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link,
             const JointParams &parameters)
    : id_(id),
      name_(name),
      wTj_(wTj),
      parent_link_(parent_link),
      child_link_(child_link),
      parameters_(parameters) {
  jMp_ = wTj_.inverse() * parent_link_->wTcom();
  jMc_ = wTj_.inverse() * child_link_->wTcom();
}

/* ************************************************************************* */
bool Joint::isChildLink(const LinkSharedPtr &link) const {
  if (link != child_link_ && link != parent_link_)
    throw std::runtime_error("link " + link->name() +
                             " is not connected to this joint " + name_);
  return link == child_link_;
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const Joint &j) {
  os << j.name() << "\n\tid=" << size_t(j.id())
     << "\n\tparent link: " << j.parent()->name()
     << "\n\t child link: " << j.child()->name();
  return os;
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const JointSharedPtr &j) {
  os << *j;
  return os;
}

}  // namespace gtdynamics