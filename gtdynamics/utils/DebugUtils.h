/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  utils.h
 * @brief Utility methods.
 * @author Frank Dellaert, Mandy Xie, Alejandro Escontrela
 */

#pragma once

#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace gtsam {

void PrintGraphWithError(
    const NonlinearFactorGraph &graph, const Values &values,
    double error_threshold = 1e-3,
    const KeyFormatter &key_formatter = gtdynamics::GTDKeyFormatter);

template <typename CONTAINER>
inline gtsam::Values SubValues(const gtsam::Values &values, const CONTAINER &keys) {
  gtsam::Values sub_values;
  for (const gtsam::Key &key : keys) {
    sub_values.insert(key, values.at(key));
  }
  return sub_values;
}

} // namespace gtsam
