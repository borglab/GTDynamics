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

#pragma once

#include <gtdynamics/universal_robot/Link.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Point3.h>

#include <map>
#include <string>

namespace gtdynamics {

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
  PointOnLink(const LinkSharedPtr& link, const gtsam::Point3& point)
      : link(link), point(point) {}

  bool operator==(const PointOnLink& other) const {
    return (point == other.point && link == other.link);
  }
  bool operator!=(const PointOnLink& other) const { return !(*this == other); }

  /**
   * @fn For given values, predict where point on link is in world frame.
   * @param values a GTSAM Values instance that should contain link pose.
   * @param k time step to check (default 0).
   */
  gtsam::Point3 predict(const gtsam::Values& values, size_t k = 0) const;

  /// Print to stream.
  friend std::ostream& operator<<(std::ostream& os, const PointOnLink& cp);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string& s = "") const;

  /// GTSAM-style equal, allows for using assert_equal.
  bool equals(const PointOnLink& other, double tol = 1e-9) const;
};

///< Vector of `PointOnLink`s
using PointOnLinks = std::vector<PointOnLink>;

}  // namespace gtdynamics

namespace gtsam {
template <>
struct traits<gtdynamics::PointOnLink>
    : public Testable<gtdynamics::PointOnLink> {};

}  // namespace gtsam
