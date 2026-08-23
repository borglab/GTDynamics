/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MLP.cpp
 * @brief Linear multi-layer perceptron with analytic Jacobians.
 * @author Karthik Shaji
 */

#include <gtdynamics/dynamics/MLP.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
/// Activation value and derivative at z.
static double activate(MLP::Activation activation, double leakySlope, double z,
                       double *derivative) {
  switch (activation) {
    case MLP::Activation::kRelu:
      *derivative = z > 0.0 ? 1.0 : 0.0;
      return z > 0.0 ? z : 0.0;
    case MLP::Activation::kLeakyRelu:
      *derivative = z > 0.0 ? 1.0 : leakySlope;
      return z > 0.0 ? z : leakySlope * z;
    case MLP::Activation::kTanh:
    default: {
      const double t = std::tanh(z);
      *derivative = 1.0 - t * t;
      return t;
    }
  }
}

/* ************************************************************************* */
void MLP::validate() const {
  if (weights_.empty() || weights_.size() != biases_.size()) {
    throw std::invalid_argument(
        "MLP: weights and biases must be non-empty and the same length.");
  }
  for (size_t k = 0; k < weights_.size(); ++k) {
    if (weights_[k].rows() == 0 || weights_[k].cols() == 0) {
      throw std::invalid_argument(
          "MLP: weight matrices must have positive dimensions.");
    }
    if (biases_[k].size() != weights_[k].rows()) {
      throw std::invalid_argument(
          "MLP: a bias size does not match its weight's rows.");
    }
    if (k > 0 && weights_[k].cols() != weights_[k - 1].rows()) {
      throw std::invalid_argument(
          "MLP: consecutive layer dimensions do not chain.");
    }
  }
}

/* ************************************************************************* */
MLP::MLP(const std::vector<gtsam::Matrix> &weights,
         const std::vector<gtsam::Vector> &biases, Activation activation,
         double leakySlope)
    : weights_(weights),
      biases_(biases),
      activation_(activation),
      leakySlope_(leakySlope) {
  validate();
}

/* ************************************************************************* */
/// n doubles from stream, or throw naming what was being read.
static gtsam::Vector readValues(std::istream &stream, size_t n,
                                const std::string &what) {
  gtsam::Vector values(n);
  for (size_t i = 0; i < n; ++i) {
    if (!(stream >> values(i))) {
      throw std::runtime_error("MLP: failed to read values of " + what + ".");
    }
  }
  return values;
}

/* ************************************************************************* */
MLP::MLP(const std::string &filename) {
  std::ifstream file(filename);
  if (!file) {
    throw std::runtime_error("MLP: cannot open " + filename + ".");
  }

  // Header phase: "key value..." lines up to and including "layers K".
  size_t inputDim = 0, outputDim = 0, nrLayers = 0;
  std::vector<size_t> hiddenDims;
  bool sawActivation = false;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream tokens(line);
    std::string key;
    if (!(tokens >> key) || key[0] == '#') continue;
    std::string rest;
    std::getline(tokens, rest);
    if (key == "layers") {
      std::istringstream(rest) >> nrLayers;
      break;
    } else if (key == "input_dim") {
      std::istringstream(rest) >> inputDim;
    } else if (key == "output_dim") {
      std::istringstream(rest) >> outputDim;
    } else if (key == "hidden_dims") {
      std::istringstream dims(rest);
      size_t dim;
      while (dims >> dim) hiddenDims.push_back(dim);
    } else if (key == "activation") {
      std::string name;
      std::istringstream(rest) >> name;
      if (name == "relu") activation_ = Activation::kRelu;
      else if (name == "tanh") activation_ = Activation::kTanh;
      else if (name == "leaky_relu") activation_ = Activation::kLeakyRelu;
      else throw std::runtime_error("MLP: unknown activation " + name + ".");
      sawActivation = true;
    } else if (key == "leaky_slope") {
      std::istringstream(rest) >> leakySlope_;
    } else {
      // Trim both ends so CRLF endings do not pollute the stored value.
      const size_t start = rest.find_first_not_of(" \t\r");
      const size_t end = rest.find_last_not_of(" \t\r");
      metadata_[key] =
          start == std::string::npos ? "" : rest.substr(start, end - start + 1);
    }
  }
  if (nrLayers == 0) {
    throw std::runtime_error("MLP: missing or zero layers header in " +
                             filename + ".");
  }
  if (!sawActivation) {
    throw std::runtime_error("MLP: missing activation header in " + filename +
                             ".");
  }

  // Layer phase: "layerK_weight OUT IN" then values, "layerK_bias N" then
  // values, whitespace-agnostic.
  for (size_t k = 0; k < nrLayers; ++k) {
    std::string label;
    size_t rows, cols;
    if (!(file >> label >> rows >> cols) ||
        label != "layer" + std::to_string(k) + "_weight") {
      throw std::runtime_error("MLP: expected layer" + std::to_string(k) +
                               "_weight block in " + filename + ".");
    }
    // Values are row-major, matching PyTorch Linear.weight [out, in].
    const gtsam::Vector flat = readValues(file, rows * cols, label);
    weights_.push_back(Eigen::Map<const Eigen::Matrix<
        double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        flat.data(), rows, cols));

    size_t biasSize;
    if (!(file >> label >> biasSize) ||
        label != "layer" + std::to_string(k) + "_bias") {
      throw std::runtime_error("MLP: expected layer" + std::to_string(k) +
                               "_bias block in " + filename + ".");
    }
    biases_.push_back(readValues(file, biasSize, label));
  }

  try {
    validate();
  } catch (const std::invalid_argument &e) {
    throw std::runtime_error(std::string(e.what()) + " (" + filename + ")");
  }
  if (inputDim != 0 && inputDim != this->inputDim()) {
    throw std::runtime_error("MLP: input_dim header does not match layer 0.");
  }
  if (outputDim != 0 && outputDim != this->outputDim()) {
    throw std::runtime_error(
        "MLP: output_dim header does not match the last layer.");
  }
  for (size_t k = 0; k < hiddenDims.size(); ++k) {
    if (k + 1 >= weights_.size() ||
        hiddenDims[k] != static_cast<size_t>(weights_[k].rows())) {
      throw std::runtime_error(
          "MLP: hidden_dims header does not match the layers.");
    }
  }
}

/* ************************************************************************* */
gtsam::Vector MLP::forward(const gtsam::Vector &x, gtsam::Matrix *H) const {
  if (static_cast<size_t>(x.size()) != inputDim()) {
    throw std::invalid_argument("MLP: input has the wrong size.");
  }

  gtsam::Vector h = x;
  gtsam::Matrix J;
  if (H) J = gtsam::Matrix::Identity(inputDim(), inputDim());

  const size_t nrLayers = weights_.size();
  for (size_t k = 0; k < nrLayers; ++k) {
    h = weights_[k] * h + biases_[k];
    if (H) J = weights_[k] * J;
    if (k + 1 < nrLayers) {
      gtsam::Vector derivative(h.size());
      for (Eigen::Index i = 0; i < h.size(); ++i) {
        h(i) = activate(activation_, leakySlope_, h(i), &derivative(i));
      }
      if (H) J = derivative.asDiagonal() * J;
    }
  }

  if (H) *H = J;
  return h;
}

}  // namespace gtdynamics
