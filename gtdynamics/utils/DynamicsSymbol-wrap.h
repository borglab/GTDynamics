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

#include <gtdynamics/utils/DynamicsSymbol.h>

#include <gtsam/nonlinear/utilities.h>  // for RedirectCout.

namespace gtdynamics {

typedef std::function<std::string(gtsam::Key)> StdKeyFormatter;

/// Adapts the GTDKeyFormatter from a boost::function to a std::function
StdKeyFormatter KeyFormatter() {
  return [](gtsam::Key key) { return GTDKeyFormatter(key); };
}


template <typename T>
std::string str(const T& t, const std::string& s = "") {
  gtsam::RedirectCout redirect;
  t.print(s, GTDKeyFormatter);
  return redirect.str();
}

}  // namespace gtdynamics
