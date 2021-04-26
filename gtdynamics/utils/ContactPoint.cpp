/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ContactPoint.h
 * @brief A point on a link that can be in contact with something.
 * @author Yetong Zhang, Alejandro Escontrela, Frank Dellaert
 */

#include <gtdynamics/utils/ContactPoint.h>

namespace gtdynamics {

std::ostream &operator<<(std::ostream &os, const ContactPoint &cp) {
  os << "{[" << cp.point.transpose() << "], " << cp.id << "}";
  return os;
}

void ContactPoint::print(const std::string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

}  // namespace gtdynamics
