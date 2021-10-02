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

namespace gtdynamics {

/**
 * @class ConstraintSpec is a base class for constraint specification on phases.
 * Derived Constraints could be walking, jumping, rocketry etc..
 * Derived Class currently implemented: FootContactConstraintSpec - specific for walking.
 */

class ConstraintSpec {
  public:
    /// GTSAM-style print, pure virtual here
    virtual void print(const std::string &s) const = 0;
};



}  // namespace gtdynamics