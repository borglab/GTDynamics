/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  cableUtils.h
 * @brief Utility methods.
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/OptionalJacobian.h>

#include <boost/optional.hpp>

namespace gtdynamics {
namespace cablerobot {

// distance
double distance(const gtsam::Point2 &p1, const gtsam::Point2 &p2,
                gtsam::OptionalJacobian<1, 2> H1 = boost::none,
                gtsam::OptionalJacobian<1, 2> H2 = boost::none) {
  return gtsam::distance2(p1, p2, H1, H2);
}

double distance(const gtsam::Point3 &p1, const gtsam::Point3 &p2,
                gtsam::OptionalJacobian<1, 3> H1 = boost::none,
                gtsam::OptionalJacobian<1, 3> H2 = boost::none) {
  return gtsam::distance3(p1, p2, H1, H2);
}

// normalize
gtsam::Point2 normalize(const gtsam::Point2 &p,
                        gtsam::OptionalJacobian<2, 2> H = boost::none) {
  gtsam::Point2 normalized = p / p.norm();
  if (H) {
    // 3*3 Derivative
    double x2 = p.x() * p.x(), y2 = p.y() * p.y();
    double xy = p.x() * p.y();
    *H << y2, -xy, /**/ -xy, x2;
    *H /= pow(x2 + y2, 1.5);
  }
  return normalized;
}
// normalize
gtsam::Point3 normalize(const gtsam::Point3 &p,
                        gtsam::OptionalJacobian<3, 3> H = boost::none) {
  return gtsam::normalize(p, H);
}

// dot
double dot(const gtsam::Point2 &p, const gtsam::Point2 &q,
           gtsam::OptionalJacobian<1, 2> H1 = boost::none,
           gtsam::OptionalJacobian<1, 2> H2 = boost::none) {
  if (H1) *H1 << q.x(), q.y();
  if (H2) *H2 << p.x(), p.y();
  return p.x() * q.x() + p.y() * q.y();
}
double dot(const gtsam::Point3 &p, const gtsam::Point3 &q,
           gtsam::OptionalJacobian<1, 3> H1 = boost::none,
           gtsam::OptionalJacobian<1, 3> H2 = boost::none) {
  return gtsam::dot(p, q, H1, H2);
}

}  // namespace cablerobot
}  // namespace gtdynamics

