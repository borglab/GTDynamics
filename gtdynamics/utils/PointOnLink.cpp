/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file PointOnLink.h
 * @brief A point on a link that can be in contact with something.
 * @author Yetong Zhang, Alejandro Escontrela, Frank Dellaert
 */

#include <gtdynamics/utils/PointOnLink.h>

namespace gtdynamics {

std::ostream &operator<<(std::ostream &os, const PointOnLink &cp) {
  os << "{" << cp.link->name() << ", [" << cp.point.transpose() << "]}";
  return os;
}

void PointOnLink::print(const std::string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this;
}

}  // namespace gtdynamics
