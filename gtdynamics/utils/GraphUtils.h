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

/**
 * @brief Compute the diagonal of the square root Hessian.
 *
 * @param graph The Gaussian factor graph.
 * @param params The Levenberg-Marquardt parameters.
 * @return VectorValues The diagonal of the square root Hessian.
 */
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

/**
 * @struct LMCachedModel
 * @brief Cached matrices and noise model for Levenberg-Marquardt optimization.
 */
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

/**
 * @brief Calculate the total dimension of the graph factors.
 *
 * @param graph The nonlinear factor graph.
 * @return size_t The total dimension.
 */
inline size_t GraphDim(const NonlinearFactorGraph &graph) {
  size_t dim = 0;
  for (const auto &factor : graph) {
    dim += factor->dim();
  }
  return dim;
}

/// Compute the error norm in standard units given graph error
double ComputeErrorNorm(const double &graph_error, const double &sigma);

/**
 * @brief Export values to a file.
 *
 * @param values The values to export.
 * @param file_path The path to the file.
 */
void ExportValuesToFile(const Values &values, const std::string &file_path);

#if GTSAM_ENABLE_BOOST_SERIALIZATION
/**
 * @brief Load values from a file.
 *
 * @param file_path The path to the file.
 * @return Values The loaded values.
 */
Values LoadValuesFromFile(const std::string &file_path);

/**
 * @class IndexSet
 * @brief A set of indices.
 */
#endif
class IndexSet : public std::set<size_t> {
public:
  using base = std::set<size_t>;
  using base::base;

  bool exists(const size_t idx) const { return find(idx) != end(); }

  void print(const std::string &s = "") const {
    std::cout << s;
    for (const auto &val : *this) {
      std::cout << val << "\t";
    }
    std::cout << std::endl;
  }
};

/**
 * @class IndexSetMap
 * @brief A map from Key to IndexSet.
 */
class IndexSetMap : public std::map<Key, IndexSet> {
public:
  bool exists(const Key &key) const { return find(key) != end(); }

  void addIndex(const Key &key, const size_t &idx) {
    IndexSet idxset;
    idxset.insert(idx);
    addIndices(key, idxset);
  }

  void addIndices(const Key &key, const IndexSet &index_set) {
    if (!exists(key)) {
      insert({key, index_set});
    } else {
      IndexSet &current_indices = at(key);
      current_indices.insert(index_set.begin(), index_set.end());
    }
  }

  void mergeWith(const IndexSetMap &new_map) {
    for (const auto &it : new_map) {
      addIndices(it.first, it.second);
    }
  }
};

/**
 * @class IndexSetMapTranslator
 * @brief Translates between global index and (Key, index_in_key) pair.
 */
class IndexSetMapTranslator {
public:
  std::map<std::pair<Key, size_t>, size_t> encoder;
  std::map<size_t, std::pair<Key, size_t>> decoder;

  IndexSetMapTranslator() {}

  void insert(size_t index, Key key, size_t index_in_key);

  IndexSet encodeIndices(const IndexSetMap &index_set_map) const;

  IndexSetMap decodeIndices(const IndexSet &indices) const;
};


/// Return factors representing mu*\|Ax-b*b_scale\|^2.
GaussianFactorGraph ScaledBiasedFactors(const GaussianFactorGraph &graph,
                                        double mu, double b_scale);

/// Return factor by setting b to zero.
JacobianFactor::shared_ptr ZerobFactor(const JacobianFactor::shared_ptr factor);

} // namespace gtsam
