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

#pragma once

#include <gtdynamics/universal_robot/Link.h>
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

/**
 * PointOnLink is a potential contact point on a particular link.
 *
 * @param link  The link on which this contact point lies.
 * @param point The location of the contact point relative to the link COM.
 */
struct PointOnLink {
  LinkSharedPtr link;
  gtsam::Point3 point;

  PointOnLink() {}
  PointOnLink(const LinkSharedPtr &link, const gtsam::Point3 &point)
      : link(link), point(point) {}

  bool operator==(const PointOnLink &other) {
    return (point == other.point && link == other.link);
  }
  bool operator!=(const PointOnLink &other) { return !(*this == other); }

  /**
   * @fn For given values, predict where point on link is in world frame.
   * @param values a GTSAM Values instance that should contain link pose.
   * @param k time step to check (default 0).
   */
  gtsam::Point3 predict(const gtsam::Values& values, size_t k = 0) const {
    const gtsam::Pose3 wTcom = Pose(values, link->id(), k);
    return wTcom.transformFrom(point);
  }

  /// Print to stream.
  friend std::ostream &operator<<(std::ostream &os, const PointOnLink &cp);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string &s) const;
};

///< Map of link name to PointOnLink
using PointOnLinks = std::vector<PointOnLink>;

}  // namespace gtdynamics
