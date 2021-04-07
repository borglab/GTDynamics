/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  DynamicsSymbol-wrap.h
 * @brief This file provides convenience stuff for wrapping.  It wraps some
 * gtsam objects so that printing will use DynamicsSymbol key formatting.
 * @author Gerry Chen
 */

#pragma once

#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

namespace gtdynamics {

template <typename T>
std::string str(const T& t, const std::string& s = "") {
  gtsam::RedirectCout redirect;
  t.print(s, GTDKeyFormatter);
  return redirect.str();
}

}  // namespace gtdynamics
