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

template <typename CONTAINER, typename VALUES>
inline VALUES SubValues(const VALUES &values, const CONTAINER &keys) {
  VALUES sub_values;
  for (const gtsam::Key &key : keys) {
    sub_values.insert(key, values.at(key));
  }
  return sub_values;
}

template <typename CONTAINER>
inline Values PickValues(const CONTAINER &keys, const Values &priority_values,
                  const Values &supplementary_values) {
  Values values;
  for (const Key &key : keys) {
    if (priority_values.exists(key)) {
      values.insert(key, priority_values.at(key));
    } else if (supplementary_values.exists(key)) {
      values.insert(key, supplementary_values.at(key));
    } else {
      throw std::runtime_error("key " + gtdynamics::GTDKeyFormatter(key) + " does not exist in both values.");
    }
  }
  return values;
}

inline void CheckFeasible(const NonlinearFactorGraph &graph,
                          const Values &values,
                          const double feasible_threshold = 1e-3) {

  if (graph.error(values) > feasible_threshold) {
    std::cout << "fail: " << graph.error(values) << "\n";
    PrintGraphWithError(graph, values, feasible_threshold);
  }
}

} // namespace gtsam
