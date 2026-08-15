/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testMLP.cpp
 * @brief test the linear MLP forward pass, Jacobian, and file loader.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/MLP.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Vector;
using gtsam::Vector2;

// A fixed 2 -> 2 -> 1 network, small enough to check by hand.
static std::vector<Matrix> smallWeights() {
  Matrix W0(2, 2), W1(1, 2);
  W0 << 1.0, 2.0, -1.0, 0.5;
  W1 << 1.0, -1.0;
  return {W0, W1};
}
static std::vector<Vector> smallBiases() {
  return {Vector2(0.1, -0.2), Vector::Constant(1, 0.05)};
}

/* ************************* forward pass ******************************** */

// Hand arithmetic for each activation, at an input where both hidden
// pre-activations are negative so relu and leaky relu differ.
TEST(MLP, forwardKnownValues) {
  const Vector x = Vector2(1.0, -1.0);  // z0 = (-0.9, -1.7), both negative

  MLP relu(smallWeights(), smallBiases(), MLP::Activation::kRelu);
  EXPECT_DOUBLES_EQUAL(0.05, relu.forward(x)(0), 1e-12);

  MLP leaky(smallWeights(), smallBiases(), MLP::Activation::kLeakyRelu, 0.01);
  EXPECT_DOUBLES_EQUAL(-0.009 - (-0.017) + 0.05, leaky.forward(x)(0), 1e-12);

  MLP tanhNet(smallWeights(), smallBiases(), MLP::Activation::kTanh);
  EXPECT_DOUBLES_EQUAL(std::tanh(-0.9) - std::tanh(-1.7) + 0.05,
                       tanhNet.forward(x)(0), 1e-12);

  EXPECT_LONGS_EQUAL(2, relu.inputDim());
  EXPECT_LONGS_EQUAL(1, relu.outputDim());
  EXPECT_LONGS_EQUAL(2, relu.nrLayers());
}

/* ************************* Jacobian ************************************ */

// The analytic Jacobian must match the numerical one; relu is checked away
// from its kinks, tanh at several points.
TEST(MLP, jacobianAgainstNumerical) {
  MLP relu(smallWeights(), smallBiases(), MLP::Activation::kRelu);
  MLP tanhNet(smallWeights(), smallBiases(), MLP::Activation::kTanh);

  auto check = [](const MLP &net, const Vector &x) {
    Matrix H;
    net.forward(x, &H);
    std::function<Vector(const Vector &)> f = [&](const Vector &v) {
      return net.forward(v);
    };
    EXPECT(assert_equal(Matrix(gtsam::numericalDerivative11<Vector, Vector>(
                            f, x)),
                        H, 1e-6));
  };

  check(relu, Vector2(1.0, 1.0));    // z0 = (3.1, -0.7), away from kinks
  check(relu, Vector2(-2.0, 0.3));   // mixed signs, still away from kinks
  check(tanhNet, Vector2(0.4, -0.7));
  check(tanhNet, Vector2(-1.2, 2.0));
}

/* ************************* file loader ********************************* */

static std::string tempFile(const std::string &name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

// A 3-layer weight file with comments and unknown keys, as the cable model
// files have.
static std::string threeLayerFileText() {
  return
      "# synthetic test model, row major\n"
      "input_dim 2\n"
      "hidden_dims 3 2\n"
      "activation leaky_relu\n"
      "leaky_slope 0.02\n"
      "cheb_nodes 16\n"
      "endpoint_links link_3 link_6\n"
      "layers 3\n"
      "layer0_weight 3 2\n"
      "0.5\n-0.3\n1.0\n0.2\n-0.7\n0.4\n"
      "layer0_bias 3\n"
      "0.1\n-0.1\n0.05\n"
      "layer1_weight 2 3\n"
      "1.0\n0.5\n-0.5\n-1.0\n0.25\n0.75\n"
      "layer1_bias 2\n"
      "0.0\n0.2\n"
      "layer2_weight 1 2\n"
      "2.0\n-1.5\n"
      "layer2_bias 1\n"
      "-0.05\n";
}

// The same network built with the explicit constructor.
static MLP threeLayerTwin() {
  Matrix W0(3, 2), W1(2, 3), W2(1, 2);
  W0 << 0.5, -0.3, 1.0, 0.2, -0.7, 0.4;
  W1 << 1.0, 0.5, -0.5, -1.0, 0.25, 0.75;
  W2 << 2.0, -1.5;
  Vector b0(3), b1(2), b2(1);
  b0 << 0.1, -0.1, 0.05;
  b1 << 0.0, 0.2;
  b2 << -0.05;
  return MLP({W0, W1, W2}, {b0, b1, b2}, MLP::Activation::kLeakyRelu, 0.02);
}

// Loading must reproduce the explicit twin, keep unknown keys as metadata,
// and reject a truncated file.
TEST(MLP, loadRoundTrip) {
  const std::string path = tempFile("gtd_testMLP_weights.txt");
  std::ofstream(path) << threeLayerFileText();

  MLP loaded(path);
  const MLP twin = threeLayerTwin();
  EXPECT_LONGS_EQUAL(2, loaded.inputDim());
  EXPECT_LONGS_EQUAL(1, loaded.outputDim());
  EXPECT_LONGS_EQUAL(3, loaded.nrLayers());
  EXPECT(loaded.activation() == MLP::Activation::kLeakyRelu);
  EXPECT(loaded.metadata().at("cheb_nodes") == "16");
  EXPECT(loaded.metadata().at("endpoint_links") == "link_3 link_6");

  for (const Vector &x : {Vector2(0.3, -0.8), Vector2(-1.1, 0.6)}) {
    Matrix Hloaded, Htwin;
    EXPECT(assert_equal(twin.forward(x, &Htwin),
                        loaded.forward(x, &Hloaded), 1e-12));
    EXPECT(assert_equal(Htwin, Hloaded, 1e-12));
  }

  // Truncated values must be rejected, not silently zero-padded.
  const std::string bad = tempFile("gtd_testMLP_truncated.txt");
  const std::string text = threeLayerFileText();
  std::ofstream(bad) << text.substr(0, text.size() - 20);
  CHECK_EXCEPTION(MLP{bad}, std::runtime_error);

  std::filesystem::remove(path);
  std::filesystem::remove(bad);
}

// Normalization headers must map inputs to [-1, 1] over the bounds and
// de-standardize the outputs, in value and in Jacobian.
TEST(MLP, optionalNormalization) {
  const std::string path = tempFile("gtd_testMLP_normalized.txt");
  std::string text = threeLayerFileText();
  text.insert(text.find("layers 3"),
              "input_lower -1.0 0.0\n"
              "input_upper 3.0 2.0\n"
              "output_mean 1.5\n"
              "output_std 0.5\n");
  std::ofstream(path) << text;

  MLP loaded(path);
  const MLP twin = threeLayerTwin();
  const Vector x = Vector2(0.5, 1.7);
  // x' maps (-1,3) -> (-1,1) and (0,2) -> (-1,1).
  const Vector xNorm = Vector2((0.5 - 1.0) / 2.0, (1.7 - 1.0) / 1.0);
  EXPECT(assert_equal(Vector(0.5 * twin.forward(xNorm).array() + 1.5),
                      loaded.forward(x), 1e-12));

  Matrix H;
  loaded.forward(x, &H);
  std::function<Vector(const Vector &)> f = [&](const Vector &v) {
    return loaded.forward(v);
  };
  EXPECT(assert_equal(
      Matrix(gtsam::numericalDerivative11<Vector, Vector>(f, x)), H, 1e-6));

  std::filesystem::remove(path);
}

/* ************************* validation ********************************** */

// Dimension-inconsistent layer stacks are rejected at construction.
TEST(MLP, rejectsInconsistentDims) {
  auto W = smallWeights();
  auto b = smallBiases();

  CHECK_EXCEPTION(MLP({}, {}, MLP::Activation::kRelu), std::invalid_argument);
  CHECK_EXCEPTION(MLP(W, {b[0]}, MLP::Activation::kRelu),
                  std::invalid_argument);

  auto badBias = b;
  badBias[0] = Vector::Zero(3);  // layer 0 has 2 outputs
  CHECK_EXCEPTION(MLP(W, badBias, MLP::Activation::kRelu),
                  std::invalid_argument);

  auto badChain = W;
  badChain[1] = Matrix::Zero(1, 3);  // layer 0 outputs 2, not 3
  CHECK_EXCEPTION(MLP(badChain, b, MLP::Activation::kRelu),
                  std::invalid_argument);

  MLP net(W, b, MLP::Activation::kRelu);
  CHECK_EXCEPTION(net.forward(Vector::Zero(3)), std::invalid_argument);
}

/* ************************* real model ********************************** */

// Loads the trained cable model when GTD_CABLE_MODEL_FILE points at it, so
// the real-file path is exercised without committing the weights.
TEST(MLP, loadRealModelIfPresent) {
  const char *path = std::getenv("GTD_CABLE_MODEL_FILE");
  if (!path) return;

  MLP model{std::string(path)};
  EXPECT_LONGS_EQUAL(5, model.inputDim());
  EXPECT_LONGS_EQUAL(42, model.outputDim());
  const Vector y = model.forward(Vector::Zero(5));
  EXPECT(y.allFinite());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
