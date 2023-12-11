/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  utils.h
 * @brief Utility methods.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace gtsam {

/** Print the factors in the graph that has error larger than the specified
 * threshold. */
void PrintGraphWithError(
    const NonlinearFactorGraph &graph, const Values &values,
    double error_threshold = 1e-3,
    const KeyFormatter &key_formatter = gtdynamics::GTDKeyFormatter);

/** Check if the values is feasible to the constraints corresponding to the
 * factor graph. */
bool CheckFeasible(
    const NonlinearFactorGraph &graph, const Values &values,
    const std::string s = "", const double feasible_threshold = 1e-3,
    bool print_details = false,
    const KeyFormatter &key_formatter = gtdynamics::GTDKeyFormatter);

/// Solve gaussian factor graph using specified parameters.
VectorValues SolveLinear(const GaussianFactorGraph &gfg,
                         const NonlinearOptimizerParams &params);

VectorValues SqrtHessianDiagonal(const GaussianFactorGraph &graph,
                                 const LevenbergMarquardtParams &params);

/** Return a subset of the values of variables specified by keys. */
template <typename CONTAINER, typename VALUES>
inline VALUES SubValues(const VALUES &values, const CONTAINER &keys) {
  VALUES sub_values;
  for (const gtsam::Key &key : keys) {
    sub_values.insert(key, values.at(key));
  }
  return sub_values;
}

/** Create values by picking variables specified by the keys. Will try to pick
 * from the priority_values first, if key does not exist in priority_values,
 * then pick from supplementary_values. */
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
      throw std::runtime_error("key " + gtdynamics::GTDKeyFormatter(key) +
                               " does not exist in both values.");
    }
  }
  return values;
}

struct LMCachedModel {
  LMCachedModel() {} // default int makes zero-size matrices
  LMCachedModel(int dim, double sigma)
      : A(Matrix::Identity(dim, dim)), b(Vector::Zero(dim)),
        model(noiseModel::Isotropic::Sigma(dim, sigma)) {}
  LMCachedModel(int dim, double sigma, const Vector &diagonal)
      : A(Eigen::DiagonalMatrix<double, Eigen::Dynamic>(diagonal)),
        b(Vector::Zero(dim)), model(noiseModel::Isotropic::Sigma(dim, sigma)) {}
  Matrix A;
  Vector b;
  SharedDiagonal model;
};

inline size_t GraphDim(const NonlinearFactorGraph& graph) {
  size_t dim = 0;
  for (const auto& factor: graph) {
    dim += factor->dim();
  }
  return dim;
}

/// Compute the error norm in standard units given graph error
double ComputeErrorNorm(const double& graph_error, const double& sigma);

void ExportValuesToFile(const Values& values, const std::string& file_path);

Values LoadValuesFromFile(const std::string& file_path);

} // namespace gtsam
