/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testObstacleFactors.cpp
 * @brief test the signed distance field, the obstacle cost, and trajectory
 *        planning around a spherical obstacle in the bar_lab workspace.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/config.h>
#include <gtdynamics/factors/GPLinearPriorFactor.h>
#include <gtdynamics/factors/ObstacleSDFFactor.h>
#include <gtdynamics/factors/ObstacleSDFFactorGP.h>
#include <gtdynamics/gpmp2/ObstacleCost.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::noiseModel::Isotropic;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

static const double kCell = 0.05;
static const double kRadius = 0.15;
static const double kEpsilon = 0.10;

// Sample the exact signed distance to a sphere onto a grid. The layer for z
// index k is indexed as (row = y, col = x), matching SignedDistanceField.
static SignedDistanceField makeSphereSDF(const Point3 &center, double radius,
                                         const Point3 &origin, double cell,
                                         size_t nx, size_t ny, size_t nz) {
  std::vector<Matrix> data(nz);
  for (size_t k = 0; k < nz; ++k) {
    Matrix layer(ny, nx);
    for (size_t i = 0; i < ny; ++i) {
      for (size_t j = 0; j < nx; ++j) {
        const Point3 p =
            origin + Point3(j * cell, i * cell, k * cell);
        layer(i, j) = (p - center).norm() - radius;
      }
    }
    data[k] = layer;
  }
  return SignedDistanceField(origin, cell, data);
}

/* ********************** signed distance field ************************** */

// Trilinear interpolation of an exact sphere field must recover the distance,
// and its gradient must point radially outward. Probing along each axis in turn
// is what catches a transposed (row = x, col = y) data layout: a swapped grid
// still returns plausible distances but reports the gradient of the wrong axis.
TEST(SignedDistanceField, sphereDistanceAndGradient) {
  const Point3 center(1.0, 1.0, 1.0);
  const Point3 origin(0.0, 0.0, 0.0);
  const size_t n = 41;  // spans 0 .. 2.0 m at 5 cm
  const SignedDistanceField sdf =
      makeSphereSDF(center, kRadius, origin, kCell, n, n, n);

  // Offset by half a cell, since the interpolant is not differentiable exactly
  // on a grid point.
  const double h = 0.5 * kCell;
  struct Probe {
    Vector3 offset;
    Vector3 expected_gradient;
  };
  const std::vector<Probe> probes = {
      {Vector3(0.5 + h, 0.0, 0.0), Vector3(1.0, 0.0, 0.0)},
      {Vector3(0.0, 0.5 + h, 0.0), Vector3(0.0, 1.0, 0.0)},
      {Vector3(0.0, 0.0, 0.5 + h), Vector3(0.0, 0.0, 1.0)},
      {Vector3(-0.4 - h, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0)},
      {Vector3(0.0, -0.4 - h, 0.0), Vector3(0.0, -1.0, 0.0)},
  };

  for (auto &&probe : probes) {
    const Point3 p = center + Point3(probe.offset);
    Vector3 gradient;
    const double d = sdf.getSignedDistance(p, gradient);
    EXPECT_DOUBLES_EQUAL(probe.offset.norm() - kRadius, d, 1e-2);
    EXPECT(assert_equal(probe.expected_gradient, gradient, 1e-2));
  }
}

// A query on the far face of the grid must not read past the data.
TEST(SignedDistanceField, queryOnFarFace) {
  const Point3 origin(0.0, 0.0, 0.0);
  const SignedDistanceField sdf =
      makeSphereSDF(Point3(0.5, 0.5, 0.5), kRadius, origin, kCell, 11, 11, 11);
  const double far = 10 * kCell;
  EXPECT_DOUBLES_EQUAL((Point3(far, far, far) - Point3(0.5, 0.5, 0.5)).norm() -
                           kRadius,
                       sdf.getSignedDistance(Point3(far, far, far)), 1e-9);
  CHECK_EXCEPTION(sdf.getSignedDistance(Point3(far + kCell, 0.0, 0.0)),
                  SDFQueryOutOfRange);
}

/* ***************************** obstacle cost *************************** */

TEST(ObstacleCost, hingeLossAndJacobian) {
  const Point3 center(1.0, 1.0, 1.0);
  const SignedDistanceField sdf =
      makeSphereSDF(center, kRadius, Point3(0, 0, 0), kCell, 41, 41, 41);

  // Beyond epsilon of the surface there is no cost and no gradient.
  const Point3 far = center + Point3(0.8, 0.0, 0.0);
  Matrix13 H_far;
  EXPECT_DOUBLES_EQUAL(0.0, hingeLossObstacleCost(far, sdf, kEpsilon, H_far),
                       1e-9);
  EXPECT(assert_equal(Matrix13::Zero(), H_far, 1e-9));

  // Outside the sphere but within epsilon, the cost is epsilon - d.
  const double offset = kRadius + 0.5 * kEpsilon;
  const Point3 near = center + Point3(offset, 0.0, 0.0);
  Matrix13 H_near;
  const double cost = hingeLossObstacleCost(near, sdf, kEpsilon, H_near);
  EXPECT_DOUBLES_EQUAL(kEpsilon - 0.5 * kEpsilon, cost, 1e-2);

  std::function<double(const Point3 &)> f = [&](const Point3 &p) {
    return hingeLossObstacleCost(p, sdf, kEpsilon);
  };
  EXPECT(assert_equal(gtsam::numericalDerivative11<double, Point3>(f, near),
                      Matrix(H_near), 1e-5));

  // A point outside the grid is treated as free space, so the field fails open.
  Matrix13 H_out;
  EXPECT_DOUBLES_EQUAL(
      0.0, hingeLossObstacleCost(Point3(-1.0, 0.0, 0.0), sdf, kEpsilon, H_out),
      1e-9);
  EXPECT(assert_equal(Matrix13::Zero(), H_out, 1e-9));
}

// The frame attached overload reads the same field through a moving frame, so
// it must agree with the world frame overload on the transformed point.
TEST(ObstacleCost, frameAttachedOverload) {
  const Point3 center(1.0, 1.0, 1.0);
  const SignedDistanceField sdf =
      makeSphereSDF(center, kRadius, Point3(0, 0, 0), kCell, 41, 41, 41);

  const Pose3 wTs(Rot3::RzRyRx(0.2, -0.1, 0.35), Point3(0.4, -0.3, 0.25));
  const Point3 wP(1.3, 1.05, 0.9);

  const double expected =
      hingeLossObstacleCost(wTs.transformTo(wP), sdf, kEpsilon);
  Matrix16 H_pose;
  Matrix13 H_point;
  const double actual =
      hingeLossObstacleCost(wTs, wP, sdf, kEpsilon, H_pose, H_point);
  EXPECT_DOUBLES_EQUAL(expected, actual, 1e-9);

  // With the identity pose it degenerates to the world frame overload.
  EXPECT_DOUBLES_EQUAL(hingeLossObstacleCost(wP, sdf, kEpsilon),
                       hingeLossObstacleCost(Pose3(), wP, sdf, kEpsilon), 1e-9);

  std::function<double(const Pose3 &, const Point3 &)> f =
      [&](const Pose3 &T, const Point3 &p) {
        return hingeLossObstacleCost(T, p, sdf, kEpsilon);
      };
  EXPECT(assert_equal(
      gtsam::numericalDerivative21<double, Pose3, Point3>(f, wTs, wP),
      Matrix(H_pose), 1e-5));
  EXPECT(assert_equal(
      gtsam::numericalDerivative22<double, Pose3, Point3>(f, wTs, wP),
      Matrix(H_point), 1e-5));
}

/* ******************** bar_lab robot query points *********************** */

static const Robot kRobot =
    CreateRobotFromFile(kUrdfPath + std::string("bar_lab.urdf"));

// The nine movable joints of robot1, in the order q indexes them: the three
// gantry prismatic joints, then the six arm revolute joints.
static std::vector<JointSharedPtr> robot1Joints() {
  const std::vector<std::string> names = {
      "bridge1_joint_EA_X", "robot1_joint_EA_Y", "robot1_joint_EA_Z",
      "robot1_joint_1",     "robot1_joint_2",    "robot1_joint_3",
      "robot1_joint_4",     "robot1_joint_5",    "robot1_joint_6"};
  std::vector<JointSharedPtr> joints;
  for (auto &&name : names) joints.push_back(kRobot.joint(name));
  return joints;
}

// Two query points on the wrist: the link CoM and a point out along the tool.
static std::vector<PointOnLink> wristPoints() {
  const LinkSharedPtr link = kRobot.link("robot1_link_6");
  return {PointOnLink(link, Point3(0.0, 0.0, 0.0)),
          PointOnLink(link, Point3(0.0, 0.0, 0.1))};
}

static Vector startConfig() {
  return (Vector(9) << 2.0, 2.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.5, 0.0).finished();
}
static Vector goalConfig() {
  return (Vector(9) << 3.0, 2.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.5, 0.0).finished();
}

// Only robot1's joints are given to the model, so the bridge2 subtree is never
// traversed and robot2 is absent from the query point model entirely.
TEST(RobotQueryPoints, jacobiansAgainstNumerical) {
  RobotQueryPoints model(kRobot, "columns", robot1Joints(), wristPoints());
  EXPECT_LONGS_EQUAL(9, model.dof());
  EXPECT_LONGS_EQUAL(2, model.nrPoints());

  const Vector q = startConfig();
  std::vector<Point3> wPs;
  std::vector<Matrix> Js;
  model.queryPoints(q, &wPs, &Js);

  for (size_t i = 0; i < model.nrPoints(); ++i) {
    std::function<Point3(const Vector &)> f = [&](const Vector &qq) {
      std::vector<Point3> ps;
      model.queryPoints(qq, &ps);
      return ps[i];
    };
    EXPECT(assert_equal(gtsam::numericalDerivative11<Point3, Vector>(f, q),
                        Js[i], 1e-5));
  }
}

/* ******************** trajectory around a sphere *********************** */

// Plan a trajectory for robot1 that slides one metre along the gantry rail,
// with a spherical obstacle planted exactly on the straight line path of the
// wrist. The endpoints are pinned, so the trajectory has to bow around it.
TEST(ObstacleSDFFactor, planAroundSphere) {
  RobotQueryPoints model(kRobot, "columns", robot1Joints(), wristPoints());
  const size_t dof = model.dof(), m = model.nrPoints();
  const size_t N = 5;  // support states
  const double total_time = 2.0, delta_t = total_time / (N - 1);

  const Vector q_start = startConfig(), q_goal = goalConfig();
  const Vector v_avg = (q_goal - q_start) / total_time;

  // Straight line initialisation, and the wrist positions it sweeps through.
  std::vector<Vector> line(N);
  std::vector<Point3> swept;
  for (size_t k = 0; k < N; ++k) {
    line[k] = q_start + (static_cast<double>(k) / (N - 1)) * (q_goal - q_start);
    std::vector<Point3> ps;
    model.queryPoints(line[k], &ps);
    swept.insert(swept.end(), ps.begin(), ps.end());
  }

  // Plant the sphere on the wrist CoM at the midpoint of the straight line, so
  // the initialisation is deep inside it.
  std::vector<Point3> mid_points;
  model.queryPoints(line[N / 2], &mid_points);
  const Point3 center = mid_points[0];

  // Size the grid to contain the whole swept path with room to detour. The
  // padding matters: a point that leaves the grid reads as free space.
  const double pad = 0.7;
  Vector3 lo = swept[0], hi = swept[0];
  for (auto &&p : swept) {
    lo = lo.cwiseMin(Vector3(p));
    hi = hi.cwiseMax(Vector3(p));
  }
  const Point3 origin(lo.x() - pad, lo.y() - pad, lo.z() - pad);
  const Vector3 extent = (hi - lo) + Vector3::Constant(2 * pad);
  const size_t nx = static_cast<size_t>(std::ceil(extent.x() / kCell)) + 1;
  const size_t ny = static_cast<size_t>(std::ceil(extent.y() / kCell)) + 1;
  const size_t nz = static_cast<size_t>(std::ceil(extent.z() / kCell)) + 1;
  auto sdf = std::make_shared<const SignedDistanceField>(
      makeSphereSDF(center, kRadius, origin, kCell, nx, ny, nz));

  const double cost_sigma = 0.01;
  ObstacleSDFFactor probe(X(0), model, sdf, cost_sigma, kEpsilon);

  // The problem is only feasible if the pinned endpoints are already clear, and
  // only meaningful if the straight line is not. Assert both before planning.
  EXPECT(probe.evaluateError(line[N / 2]).maxCoeff() > 0.5 * kEpsilon);
  EXPECT_DOUBLES_EQUAL(0.0, probe.evaluateError(q_start).maxCoeff(), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, probe.evaluateError(q_goal).maxCoeff(), 1e-9);

  gtsam::NonlinearFactorGraph graph;
  auto Qc_model = Isotropic::Sigma(dof, 1.0);
  auto endpoint_model = Isotropic::Sigma(dof, 1e-4);

  graph.addPrior<Vector>(X(0), q_start, endpoint_model);
  graph.addPrior<Vector>(V(0), Vector::Zero(dof), endpoint_model);
  graph.addPrior<Vector>(X(N - 1), q_goal, endpoint_model);
  graph.addPrior<Vector>(V(N - 1), Vector::Zero(dof), endpoint_model);

  for (size_t k = 0; k < N; ++k) {
    graph.emplace_shared<ObstacleSDFFactor>(X(k), model, sdf, cost_sigma,
                                            kEpsilon);
  }
  // The interpolated obstacle cost is only a Gaussian process posterior mean
  // because this prior, with the same Qc_model and delta_t, joins the same
  // support states.
  for (size_t k = 0; k < N - 1; ++k) {
    graph.emplace_shared<GPLinearPrior>(X(k), V(k), X(k + 1), V(k + 1), delta_t,
                                        Qc_model);
  }

  Values init;
  for (size_t k = 0; k < N; ++k) {
    init.insert(X(k), line[k]);
    init.insert(V(k), Vector(v_avg));
  }

  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(100);
  const Values result =
      gtsam::LevenbergMarquardtOptimizer(graph, init, params).optimize();

  // The endpoints are held, and every query point at every support state has
  // been pushed clear of the sphere by at least epsilon. This is checked against
  // the analytic sphere rather than the sampled field, so a point that escaped
  // the grid cannot pass by reading as free space.
  EXPECT(assert_equal(q_start, result.at<Vector>(X(0)), 1e-3));
  EXPECT(assert_equal(q_goal, result.at<Vector>(X(N - 1)), 1e-3));

  for (size_t k = 0; k < N; ++k) {
    std::vector<Point3> ps;
    model.queryPoints(result.at<Vector>(X(k)), &ps);
    for (size_t i = 0; i < m; ++i) {
      EXPECT((ps[i] - center).norm() > kRadius + kEpsilon - 0.02);
    }
  }
}

// The interpolated obstacle factor must agree with the unary factor when tau is
// zero, where the interpolation reproduces the first support state exactly.
TEST(ObstacleSDFFactorGP, agreesWithUnaryFactorAtTauZero) {
  RobotQueryPoints model(kRobot, "columns", robot1Joints(), wristPoints());
  const size_t dof = model.dof();

  const Vector q1 = startConfig(), q2 = goalConfig();
  const Vector v1 = Vector::Zero(dof), v2 = Vector::Zero(dof);

  std::vector<Point3> mid_points;
  model.queryPoints(q1, &mid_points);
  const Point3 center = mid_points[0] + Point3(kRadius + 0.5 * kEpsilon, 0, 0);
  auto sdf = std::make_shared<const SignedDistanceField>(makeSphereSDF(
      center, kRadius, center - Point3(1.0, 1.0, 1.0), kCell, 41, 41, 41));

  const double delta_t = 0.5, cost_sigma = 0.01;
  ObstacleSDFFactor unary(X(0), model, sdf, cost_sigma, kEpsilon);
  ObstacleSDFFactorGP interpolated(X(0), V(0), X(1), V(1), model, sdf,
                                   cost_sigma, kEpsilon, Isotropic::Sigma(dof, 1.0),
                                   delta_t, 0.0);

  EXPECT(assert_equal(unary.evaluateError(q1),
                      interpolated.evaluateError(q1, v1, q2, v2), 1e-9));

  Values values;
  values.insert(X(0), q1);
  values.insert(V(0), v1);
  values.insert(X(1), q2);
  values.insert(V(1), v2);
  ObstacleSDFFactorGP at_tau(X(0), V(0), X(1), V(1), model, sdf, cost_sigma,
                             kEpsilon, Isotropic::Sigma(dof, 1.0), delta_t,
                             0.2 * delta_t);
  EXPECT_CORRECT_FACTOR_JACOBIANS(at_tau, values, 1e-7, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
