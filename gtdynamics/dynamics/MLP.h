/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MLP.h
 * @brief Linear multi-layer perceptron with analytic Jacobians.
 * @author Karthik Shaji
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <map>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * A linear multi-layer perceptron with an arbitrary number of hidden layers,
 * loaded from an ASCII weight file or built from explicit weights, evaluated
 * with an analytic Jacobian. Immutable after construction.
 */
class GTSAM_EXPORT MLP {
 public:
  enum class Activation { kRelu, kTanh, kLeakyRelu };

 private:
  std::vector<gtsam::Matrix> weights_;  ///< one [out x in] matrix per layer
  std::vector<gtsam::Vector> biases_;   ///< one [out] vector per layer
  Activation activation_;
  double leakySlope_ = 0.01;

  /// Optional affine normalization, applied iff the file provided it.
  bool normalizeInput_ = false, denormalizeOutput_ = false;
  gtsam::Vector inputScale_, inputShift_;  ///< x' = scale .* x + shift
  gtsam::Vector outputStd_, outputMean_;   ///< y = std .* y' + mean

  std::map<std::string, std::string> metadata_;  ///< unrecognized header lines

  /// Reject empty or dimension-inconsistent layers.
  void validate() const;

 public:
  /**
   * Load a network from an ASCII weight file: '#' comments, "key value..."
   * header lines, then per-layer "layerK_weight OUT IN" / "layerK_bias N"
   * blocks of row-major values.
   * @param filename path of the weight file
   * @throw std::runtime_error on a missing file or malformed or inconsistent
   *        header or layer blocks
   */
  explicit MLP(const std::string &filename);

  /**
   * Construct from explicit weights.
   * @param weights weight matrix of each layer, [out x in]
   * @param biases bias vector of each layer
   * @param activation activation applied after every layer but the last
   * @param leakySlope negative-side slope for kLeakyRelu
   * @throw std::invalid_argument on empty or dimension-inconsistent layers
   */
  MLP(const std::vector<gtsam::Matrix> &weights,
      const std::vector<gtsam::Vector> &biases, Activation activation,
      double leakySlope = 0.01);

  /// Return the input dimension.
  size_t inputDim() const { return weights_.front().cols(); }

  /// Return the output dimension.
  size_t outputDim() const { return weights_.back().rows(); }

  /// Return the number of layers.
  size_t nrLayers() const { return weights_.size(); }

  /// Return the activation applied after every layer but the last.
  Activation activation() const { return activation_; }

  /// Return unrecognized header lines of the weight file, key to remainder.
  const std::map<std::string, std::string> &metadata() const {
    return metadata_;
  }

  /**
   * Evaluate the network.
   * @param x input vector, size inputDim
   * @param H if non-null, filled with the outputDim x inputDim Jacobian
   * @return the output vector
   */
  gtsam::Vector forward(const gtsam::Vector &x,
                        gtsam::Matrix *H = nullptr) const;
};  // \class MLP

}  // namespace gtdynamics
