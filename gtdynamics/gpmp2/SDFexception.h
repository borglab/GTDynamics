/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SDFexception.h
 * @brief Custom exceptions for signed distance fields.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <stdexcept>

namespace gtdynamics {

/// Thrown when a query point falls outside the signed distance field grid.
class SDFQueryOutOfRange : public std::runtime_error {
 public:
  SDFQueryOutOfRange() : std::runtime_error("Querying SDF out of range") {}
};

}  // namespace gtdynamics
