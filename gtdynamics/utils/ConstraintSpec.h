/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstraintSpec.h
 * @brief Base class for constraint specification
 * @author: Dan Barladeanu, Frank Dellaert
 */

#pragma once

#include <gtdynamics/utils/PointOnLink.h>

#include <string>

namespace gtdynamics {

/** Specification of a phase's constraints and active contact points.
 * Contact points may belong to any link. A contact-free phase returns an empty
 * collection; the ground role is explicit on each PointOnLink.
 */
class ConstraintSpec {
 public:
  /// All active contacts, including contacts away from the ground plane.
  virtual const PointOnLinks& contactPoints() const = 0;

  /// Active contacts selected for ground-height objectives.
  PointOnLinks groundContactPoints() const {
    PointOnLinks ground;
    for (const auto& contact : contactPoints()) {
      if (contact.isGround) ground.push_back(contact);
    }
    return ground;
  }

  /// GTSAM-style print.
  virtual void print(const std::string& s) const = 0;
  virtual ~ConstraintSpec() = default;
};

}  // namespace gtdynamics
