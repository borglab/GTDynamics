/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testNNCableFactor.cpp
 * @brief test the NN cable spline and its obstacle factor on bar_lab.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/config.h>
#include <gtdynamics/dynamics/MLP.h>
#include <gtdynamics/factors/NNCableFactor.h>
#include <gtdynamics/factors/NNCableFactorGP.h>
#include <gtdynamics/gpmp2/NNCableSpline.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "barLabFixtures.h"
#include "makeSphereSDF.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::noiseModel::Isotropic;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

static const Robot &kRobot = barLabRobot();

// The network reads robot1's five distal arm joints, as the trained cable
// model does; q is the 9-dof robot1 configuration (gantry then joints 1-6).
static const std::vector<size_t> kInputIndices = {4, 5, 6, 7, 8};

static PointOnLink attachment0() {
  return PointOnLink(kRobot.link("robot1_link_3"), Point3(0.1, 0.0, 0.05));
}
static PointOnLink attachment1() {
  return PointOnLink(kRobot.link("robot1_link_6"), Point3(0.0, 0.0, 0.1));
}

// A 5 -> 4 -> 3*(N-2) network with all-zero weights and the given last bias.
static std::shared_ptr<const MLP> biasOnlyMLP(size_t numChebNodes,
                                              const Vector &lastBias) {
  const size_t nOut = 3 * (numChebNodes - 2);
  return std::make_shared<const MLP>(
      std::vector<Matrix>{Matrix::Zero(4, 5), Matrix::Zero(nOut, 4)},
      std::vector<Vector>{Vector::Zero(4), lastBias}, MLP::Activation::kRelu);
}

// A smooth deterministic 5 -> 8 -> 3*(N-2) tanh network with small outputs,
// so numerical differentiation through it is clean.
static std::shared_ptr<const MLP> smoothMLP(size_t numChebNodes) {
  const size_t nOut = 3 * (numChebNodes - 2);
  Matrix W0(8, 5), W1(nOut, 8);
  Vector b0(8), b1(nOut);
  for (Eigen::Index i = 0; i < W0.rows(); ++i)
    for (Eigen::Index j = 0; j < W0.cols(); ++j)
      W0(i, j) = 0.4 * std::sin(1.3 * i + 2.1 * j + 0.5);
  for (Eigen::Index i = 0; i < W1.rows(); ++i)
    for (Eigen::Index j = 0; j < W1.cols(); ++j)
      W1(i, j) = 0.05 * std::sin(0.7 * i + 1.9 * j + 1.1);
  for (Eigen::Index i = 0; i < 8; ++i) b0(i) = 0.3 * std::sin(2.3 * i);
  for (Eigen::Index i = 0; i < b1.size(); ++i) b1(i) = 0.02 * std::sin(1.7 * i);
  return std::make_shared<const MLP>(std::vector<Matrix>{W0, W1},
                                     std::vector<Vector>{b0, b1},
                                     MLP::Activation::kTanh);
}

static std::shared_ptr<const NNCableSpline> makeCable(
    const std::shared_ptr<const MLP> &mlp, size_t numChebNodes,
    size_t numSamples) {
  return std::make_shared<const NNCableSpline>(
      kRobot, "columns", robot1Joints(), attachment0(), attachment1(),
      kRobot.link("robot1_link_1"), mlp, kInputIndices, numChebNodes,
      numSamples);
}

// A sphere centred just off the middle cable sample at q, its grid offset by
// half a cell so samples avoid the trilinear gradient's node discontinuities.
static std::shared_ptr<const SignedDistanceField> midCableSphereSDF(
    const NNCableSpline &cable, const Vector &q) {
  const Matrix pts = cable.worldPoints(q);
  const Point3 center = Point3(pts.col(cable.numSamples() / 2)) +
                        Point3(0.3 * kCell, 0.2 * kCell, 0.0);
  const Point3 origin = center - Point3::Constant(1.0 - kHalfCell);
  return std::make_shared<const SignedDistanceField>(
      makeSphereSDF(center, kRadius, origin, kCell, 41, 41, 41));
}

/* ************************ spline reconstruction ************************ */

// With zero residuals every sample lies on the chord, whose endpoints are
// exactly the FK attachment points.
TEST(NNCableSpline, chordWhenResidualZero) {
  const size_t N = 6, M = 5;
  const auto cable = makeCable(biasOnlyMLP(N, Vector::Zero(3 * (N - 2))), N, M);
  const Vector q = startConfig();

  // Independent endpoints from a plain query point model.
  const auto endpointModel = std::make_shared<const RobotQueryPoints>(
      kRobot, "columns", robot1Joints(),
      PointOnLinks{attachment0(), attachment1()});
  std::vector<Point3> ends;
  endpointModel->queryPoints(q, &ends);

  std::vector<Point3> wPts;
  cable->samplePoints(q, &wPts);
  EXPECT_LONGS_EQUAL(M, wPts.size());
  for (size_t m = 0; m < M; ++m) {
    const double s = static_cast<double>(m) / (M - 1);
    EXPECT(assert_equal(Point3((1.0 - s) * ends[0] + s * ends[1]), wPts[m],
                        1e-9));
  }
}

// A constant nodal residual must be interpolated with the Chebyshev weights
// and rotated by the reference link's world rotation. Catches both the
// row-major reshape and the residual frame.
TEST(NNCableSpline, constantResidualInReferenceFrame) {
  const size_t N = 6, M = 7;
  Vector bias(3 * (N - 2));
  for (Eigen::Index i = 0; i < bias.size(); ++i)
    bias(i) = 0.05 * std::sin(1.1 * i + 0.3);
  const auto cable = makeCable(biasOnlyMLP(N, bias), N, M);

  Vector q = startConfig();
  q(3) = 0.7;  // rotate joint 1, so the reference frame is not trivial

  // Expected pieces, each from independent public machinery.
  const auto endpointModel = std::make_shared<const RobotQueryPoints>(
      kRobot, "columns", robot1Joints(),
      PointOnLinks{attachment0(), attachment1(),
                   PointOnLink(kRobot.link("robot1_link_1"),
                               Point3(0.0, 0.0, 0.0))});
  std::vector<Point3> ends;
  endpointModel->queryPoints(q, &ends);
  std::vector<Pose3> wTls;
  endpointModel->queryPoses(q, &wTls);
  const Matrix R = wTls[2].rotation().matrix();

  std::vector<Point3> wPts;
  cable->samplePoints(q, &wPts);
  for (size_t m = 0; m < M; ++m) {
    const double s = static_cast<double>(m) / (M - 1);
    const Matrix weights = gtsam::Chebyshev2::CalculateWeights(N, s, 0.0, 1.0);
    Vector3 v = Vector3::Zero();
    for (size_t j = 0; j < N - 2; ++j) {
      v += weights(0, j + 1) * Vector3(bias.segment(3 * j, 3));
    }
    const Point3 expected =
        (1.0 - s) * ends[0] + s * ends[1] + Point3(R * v);
    EXPECT(assert_equal(expected, wPts[m], 1e-9));
  }
}

// The analytic sample Jacobians must match the numerical ones, at a config
// where the residual and the reference rotation are both non-trivial.
TEST(NNCableSpline, sampleJacobiansAgainstNumerical) {
  const size_t N = 8, M = 5;
  const auto cable = makeCable(smoothMLP(N), N, M);

  Vector q = startConfig();
  q(3) = 0.6;
  q(5) = -0.8;

  std::vector<Point3> wPts;
  std::vector<Matrix> ptJacobians;
  cable->samplePoints(q, &wPts, &ptJacobians);

  for (size_t m = 0; m < M; ++m) {
    std::function<Point3(const Vector &)> f = [&](const Vector &v) {
      std::vector<Point3> pts;
      cable->samplePoints(v, &pts);
      return pts[m];
    };
    EXPECT(assert_equal(
        Matrix(gtsam::numericalDerivative11<Point3, Vector, 9>(f, q)),
        ptJacobians[m], 1e-5));
  }
}

/* ************************ factor *************************************** */

// A sphere in the cable's path: the factor error and Jacobians must match
// the numerical ones on the active branch.
TEST(NNCableFactor, jacobianWhenActive) {
  const size_t N = 8, M = 9;
  const auto cable = makeCable(smoothMLP(N), N, M);
  const Vector q = startConfig();
  const auto sdf = midCableSphereSDF(*cable, q);

  NNCableFactor factor(X(0), cable, sdf, 0.01, kEpsilon, 0.02);
  const Vector err = factor.evaluateError(q);
  EXPECT(err(M / 2) > 0.0);  // active branch

  Values values;
  values.insert(X(0), q);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Only samples within the sphere's inflated band are active; with the
// obstacle far from the cable the whole error and Jacobian vanish.
TEST(NNCableFactor, activeAndInactiveSamples) {
  const size_t N = 8, M = 9;
  const auto cable = makeCable(smoothMLP(N), N, M);
  const Vector q = startConfig();
  const Matrix pts = cable->worldPoints(q);

  const auto sdf = midCableSphereSDF(*cable, q);
  NNCableFactor factor(X(0), cable, sdf, 0.01, kEpsilon, 0.02);
  const Vector err = factor.evaluateError(q);
  EXPECT(err(M / 2) > 0.0);
  EXPECT_DOUBLES_EQUAL(0.0, err(0), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, err(M - 1), 1e-9);

  // A grid that covers the cable with the sphere far away inside it: every
  // sample is in free space, so the error and Jacobian are identically zero.
  Vector3 lo = pts.rowwise().minCoeff(), hi = pts.rowwise().maxCoeff();
  const double pad = 2.0, cell = 0.1;
  const Point3 farOrigin(lo.x() - pad, lo.y() - pad, lo.z() - pad);
  const Vector3 extent = (hi - lo) + Vector3::Constant(2.0 * pad);
  const Point3 farCenter = Point3(hi) + Point3(1.2, 1.2, 1.2);
  auto farSdf = std::make_shared<const SignedDistanceField>(makeSphereSDF(
      farCenter, 0.1, farOrigin, cell,
      static_cast<size_t>(std::ceil(extent.x() / cell)) + 1,
      static_cast<size_t>(std::ceil(extent.y() / cell)) + 1,
      static_cast<size_t>(std::ceil(extent.z() / cell)) + 1));
  NNCableFactor farFactor(X(0), cable, farSdf, 0.01, kEpsilon, 0.02);
  Matrix H;
  const Vector farErr = farFactor.evaluateError(q, &H);
  EXPECT_DOUBLES_EQUAL(0.0, farErr.norm(), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, H.norm(), 1e-9);
}

// At tau = 0 the GP factor must reproduce the unary error and Jacobian at q1.
TEST(NNCableFactorGP, agreesWithUnaryFactorAtTauZero) {
  const size_t N = 8, M = 9;
  const auto cable = makeCable(smoothMLP(N), N, M);
  const Vector q1 = startConfig();
  Vector q2 = startConfig();
  q2(0) += 1.0;  // the bridge has moved along the rail
  const Vector v1 = Vector::Zero(9), v2 = Vector::Zero(9);

  const auto sdf = midCableSphereSDF(*cable, q1);

  const double deltaT = 0.5, costSigma = 0.01;
  NNCableFactor unary(X(0), cable, sdf, costSigma, kEpsilon, 0.02);
  NNCableFactorGP interpolated(X(0), V(0), X(1), V(1), cable, sdf, costSigma,
                               kEpsilon, 0.02, Isotropic::Sigma(9, 1.0),
                               deltaT, 0.0);

  // q1 passes through with unit weight, so H1 is the unary Jacobian and the
  // other support states get zero.
  Matrix Hu, H1, H2, H3, H4;
  EXPECT(assert_equal(unary.evaluateError(q1, &Hu),
                      interpolated.evaluateError(q1, v1, q2, v2, &H1, &H2, &H3,
                                                 &H4),
                      1e-9));
  EXPECT(assert_equal(Hu, H1, 1e-9));
  const Matrix zero = Matrix::Zero(Hu.rows(), Hu.cols());
  EXPECT(assert_equal(zero, H2, 1e-9));
  EXPECT(assert_equal(zero, H3, 1e-9));
  EXPECT(assert_equal(zero, H4, 1e-9));

  // At an interior tau the analytic Jacobians must match the numerical ones.
  Values values;
  values.insert(X(0), q1);
  values.insert(V(0), v1);
  values.insert(X(1), q2);
  values.insert(V(1), v2);
  NNCableFactorGP atTau(X(0), V(0), X(1), V(1), cable, sdf, costSigma,
                        kEpsilon, 0.02, Isotropic::Sigma(9, 1.0), deltaT,
                        0.1 * deltaT);
  EXPECT(atTau.evaluateError(q1, v1, q2, v2).norm() > 0.0);  // active branch
  EXPECT_CORRECT_FACTOR_JACOBIANS(atTau, values, 1e-7, 1e-5);
}

/* ************************ validation *********************************** */

// Inconsistent construction inputs are rejected with clear errors.
TEST(NNCableSpline, rejectsBadInputs) {
  const size_t N = 6, M = 5;
  const auto mlp = biasOnlyMLP(N, Vector::Zero(3 * (N - 2)));

  CHECK_EXCEPTION(makeCable(nullptr, N, M), std::invalid_argument);
  CHECK_EXCEPTION(
      NNCableSpline(kRobot, "columns", robot1Joints(), attachment0(),
                    attachment1(), LinkSharedPtr(), mlp, kInputIndices, N, M),
      std::invalid_argument);
  CHECK_EXCEPTION(
      NNCableSpline(kRobot, "columns", robot1Joints(), attachment0(),
                    attachment1(), kRobot.link("robot1_link_1"), mlp,
                    {4, 5, 6, 7, 9}, N, M),  // 9 is out of range of q
      std::invalid_argument);
  CHECK_EXCEPTION(
      NNCableSpline(kRobot, "columns", robot1Joints(), attachment0(),
                    attachment1(), kRobot.link("robot1_link_1"), mlp,
                    {4, 5, 6, 7, 7}, N, M),  // duplicated index
      std::invalid_argument);
  CHECK_EXCEPTION(
      NNCableSpline(kRobot, "columns", robot1Joints(), attachment0(),
                    attachment1(), kRobot.link("robot1_link_1"), mlp,
                    {4, 5, 6, 7}, N, M),  // one index too few
      std::invalid_argument);
  CHECK_EXCEPTION(makeCable(mlp, 8, M), std::invalid_argument);  // 3(N-2) off
  CHECK_EXCEPTION(makeCable(mlp, N, 1), std::invalid_argument);
  CHECK_EXCEPTION(makeCable(biasOnlyMLP(3, Vector::Zero(3)), 2, M),
                  std::invalid_argument);
}

// Inconsistent factor arguments are rejected with clear errors.
TEST(NNCableFactor, rejectsBadArgs) {
  const size_t N = 6, M = 5;
  const auto cable = makeCable(biasOnlyMLP(N, Vector::Zero(3 * (N - 2))), N, M);
  auto sdf = std::make_shared<const SignedDistanceField>(
      makeSphereSDF(Point3(5, 5, 5), 0.1, Point3(4, 4, 4), kCell, 5, 5, 5));

  CHECK_EXCEPTION(NNCableFactor(X(0), nullptr, sdf, 0.01, kEpsilon),
                  std::invalid_argument);
  CHECK_EXCEPTION(NNCableFactor(X(0), cable, nullptr, 0.01, kEpsilon),
                  std::invalid_argument);
  CHECK_EXCEPTION(NNCableFactor(X(0), cable, sdf, 0.01, -0.1),
                  std::invalid_argument);
  CHECK_EXCEPTION(NNCableFactor(X(0), cable, sdf, 0.01, kEpsilon,
                                Vector::Zero(M - 1)),
                  std::invalid_argument);
  CHECK_EXCEPTION(NNCableFactor(X(0), cable, sdf, 0.01, kEpsilon,
                                Vector::Constant(M, -0.01)),
                  std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
