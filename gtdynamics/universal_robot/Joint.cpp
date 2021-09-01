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
      parent_link_(parent_link),
      child_link_(child_link),
      parameters_(parameters) {
  jMp_ = wTj.inverse() * parent_link_->wTcom();
  jMc_ = wTj.inverse() * child_link_->wTcom();
}

/* ************************************************************************* */
bool Joint::isChildLink(const LinkSharedPtr &link) const {
  if (link != child_link_ && link != parent_link_)
    throw std::runtime_error("link " + link->name() +
                             " is not connected to this joint " + name_);
  return link == child_link_;
}

std::ostream &Joint::to_stream(std::ostream &os) const {
  os << name_ << "\n\tid=" << size_t(id_)
     << "\n\tparent link: " << parent()->name()
     << "\n\tchild link: " << child()->name();
  return os;
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const Joint &j) {
  // Delegate printing responsibility to member function so we can override.
  return j.to_stream(os);
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const JointSharedPtr &j) {
  return j->to_stream(os);
}

}  // namespace gtdynamics