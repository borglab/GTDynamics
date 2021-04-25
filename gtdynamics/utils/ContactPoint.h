/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ContactPoint.h
 * @brief A point on a link that can be in contact with something.
 * @author Yetong Zhang, Alejandro Escontrela, Frank dellaert
 */

#pragma once

#include <gtsam/geometry/Point3.h>

#include <map>
#include <string>

namespace gtdynamics {

/**
 * ContactPoint defines a single contact point at a link.
 *
 * @param point The location of the contact point relative to the link COM.
 * @param id Each link's contact points must have a unique contact id.
 */
struct ContactPoint {
  gtsam::Point3 point;
  int id;

  ContactPoint() {}
  ContactPoint(const gtsam::Point3 &point, int id) : point(point), id(id) {}

  bool operator==(const ContactPoint &other) {
    return (point == other.point && id == other.id);
  }
  bool operator!=(const ContactPoint &other) { return !(*this == other); }

  /// Print to stream.
  friend std::ostream &operator<<(std::ostream &os, const ContactPoint &cp);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string &s) const;
};

///< Map of link name to ContactPoint
using ContactPoints = std::map<std::string, ContactPoint>;

}  // namespace gtdynamics
