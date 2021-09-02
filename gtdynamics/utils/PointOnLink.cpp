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

gtsam::Point3 PointOnLink::predict(const gtsam::Values &values,
                                   size_t k) const {
  const gtsam::Pose3 wTcom = Pose(values, link->id(), k);
  return wTcom.transformFrom(point);
}

std::ostream &operator<<(std::ostream &os, const PointOnLink &cp) {
  os << "{" << cp.link->name() << ", [" << cp.point.transpose() << "]}";
  return os;
}

void PointOnLink::print(const std::string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

bool PointOnLink::equals(const PointOnLink &other, double tol) const {
  return other.link->name() == link->name() &&
         gtsam::equal<gtsam::Point3>(other.point, point, tol);
}

}  // namespace gtdynamics
