/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  format.h
 * @brief Formatting utilities.
 * @author Gerry Chen, Varun Agrawal
 */

#include <gtsam/nonlinear/utilities.h>  // for RedirectCout

namespace gtdynamics {

/**
 * Convenience function for wrapping.
 * Wraps GTSAM objects to print using DynamicsSymbol formatting.
 */
template <typename T>
std::string GtdFormat(const T &t, const std::string &s = "") {
  gtsam::RedirectCout redirect;
  t.print(s, GTDKeyFormatter);
  return redirect.str();
}

}  // namespace gtdynamics
