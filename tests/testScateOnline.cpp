/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testScateOnline.cpp
 * @author Harrison Jenkins
 *
 * Tests for scate_online.h with a toy robot, so they need no robot model:
 * a 3-DOF "point robot" whose joint vector q is the position of one collision
 * sphere. Its obstacle factor uses the same Hermite interpolation as the
 * GTDynamics GP factors, and its sensor pose has a q-dependent rotation so the
 * Pose3 Jacobian convention is exercised.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Rot3.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <random>

#include "build_scate_graph.h"
#include "scate_online.h"
#include "scate_replay.h"
#include "trajectory_init.h"

using namespace scate;
using gtsam::Point3;

namespace {

// ------------------------------------------------------------
// Toy robot
// ------------------------------------------------------------

/**
 * Hinge cost max(0, epsilon - d) at the Hermite-interpolated position, where d
 * is the gap between the robot sphere and the obstacle sphere.
 */
class ToySphereObstacleGP : public gtsam::NoiseModelFactorN<Vector, Vector, Vector, Vector> {
  using Base = gtsam::NoiseModelFactorN<Vector, Vector, Vector, Vector>;
  Point3 center_;
  double radius_sum_, epsilon_;
  HermiteCoefficients coefficients_;

 public:
  ToySphereObstacleGP(gtsam::Key q1, gtsam::Key v1, gtsam::Key q2, gtsam::Key v2,
                      const Point3& center, double radius_sum, double epsilon, double sigma,
                      double dt, double tau)
      : Base(gtsam::noiseModel::Isotropic::Sigma(1, sigma), q1, v1, q2, v2),
        center_(center),
        radius_sum_(radius_sum),
        epsilon_(epsilon),
        coefficients_(hermitePositionCoefficients(dt, tau)) {}

  gtsam::Vector evaluateError(const Vector& q1, const Vector& v1, const Vector& q2,
                              const Vector& v2, gtsam::OptionalMatrixType H1 = nullptr,
                              gtsam::OptionalMatrixType H2 = nullptr,
                              gtsam::OptionalMatrixType H3 = nullptr,
                              gtsam::OptionalMatrixType H4 = nullptr) const override {
    const Vector position = coefficients_.a * q1 + coefficients_.b * v1 +
                            coefficients_.c * q2 + coefficients_.d * v2;
    const Vector offset = position - Vector(center_);
    const double distance = offset.norm();
    const double gap = distance - radius_sum_;
    gtsam::Matrix gradient = gtsam::Matrix::Zero(1, 3);
    double error = 0.0;
    if (gap < epsilon_) {
      error = epsilon_ - gap;
      gradient = -(offset / distance).transpose();
    }
    if (H1) *H1 = coefficients_.a * gradient;
    if (H2) *H2 = coefficients_.b * gradient;
    if (H3) *H3 = coefficients_.c * gradient;
    if (H4) *H4 = coefficients_.d * gradient;
    return (gtsam::Vector(1) << error).finished();
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new ToySphereObstacleGP(*this));
  }
};

class ToyPointRobot : public CollisionModel {
 public:
  double robot_radius = 0.05, dt = 1.0, sigma_obstacle = 0.01, epsilon = 0.05;
  Point3 center = Point3::Zero();
  double radius = 0.0;

  size_t dof() const override { return 3; }

  // Sensor at the robot's position, rotated by Exp(0.5 q).
  gtsam::Pose3 sensorPose(const Vector& q, gtsam::Matrix* H = nullptr) const override {
    gtsam::Matrix3 H_exp;
    const gtsam::Rot3 rotation = gtsam::Rot3::Expmap(gtsam::Vector3(0.5 * q), H_exp);
    if (H) {
      *H = gtsam::Matrix(6, 3);
      H->topRows(3) = 0.5 * H_exp;
      H->bottomRows(3) = rotation.matrix().transpose();
    }
    return gtsam::Pose3(rotation, Point3(q));
  }

  void setObstacle(const Point3& obstacle_center, double obstacle_radius) override {
    center = obstacle_center;
    radius = obstacle_radius;
  }

  gtsam::NonlinearFactor::shared_ptr obstacleFactor(int i, double tau) const override {
    return gtsam::NonlinearFactor::shared_ptr(new ToySphereObstacleGP(
        QKey(i), VKey(i), QKey(i + 1), VKey(i + 1), center, radius + robot_radius, epsilon,
        sigma_obstacle, dt, tau));
  }

  double obstacleClearance(const Vector& q, const Point3& obstacle_center,
                           double obstacle_radius) const override {
    return (q - Vector(obstacle_center)).norm() - obstacle_radius - robot_radius;
  }

  // Drawing hooks: one sphere, which is also the tool.
  gtsam::Matrix sphereCenters(const Vector& q) const override { return gtsam::Matrix(q); }
  Vector sphereRadii() const override { return Vector::Constant(1, robot_radius); }
  int toolSphere() const override { return 0; }
};

// ------------------------------------------------------------
// Scenario: cross a 1.2 m line with a sphere in the middle
// ------------------------------------------------------------

GraphSpec toySpec() {
  GraphSpec spec;
  spec.q_start = (Vector(3) << -0.6, 0.0, 0.4).finished();
  spec.q_goal = (Vector(3) << 0.6, 0.0, 0.4).finished();
  spec.q_lower = Vector::Constant(3, -2.0);
  spec.q_upper = Vector::Constant(3, 2.0);
  spec.u_limit = Vector::Constant(3, 1.0);
  spec.v_limit = Vector::Constant(3, 1.0);
  spec.num_poses = 21;
  spec.dt = 1.0;
  return spec;
}

gtsam::Values toyInitialValues(const GraphSpec& spec) {
  TrajectoryConfig config;
  config.num_poses = spec.num_poses;
  config.dt = spec.dt;
  const JointTrajectory trajectory =
      makeInitialTrajectory(spec.q_start, spec.q_goal, config);
  gtsam::Values values;
  for (int i = 0; i < spec.num_poses; ++i) {
    values.insert(QKey(i), trajectory.q[i]);
    values.insert(VKey(i), trajectory.v[i]);
  }
  for (int i = 0; i + 1 < spec.num_poses; ++i) values.insert(UKey(i), trajectory.u[i]);
  return values;
}

SimulationConfig toySimulation(unsigned seed) {
  SimulationConfig simulation;
  // On the straight path from start to goal, so the blind robot must hit it.
  simulation.obstacle_center = Point3(0.0, 0.0, 0.4);
  simulation.obstacle_radius = 0.08;
  simulation.sensor_range = 0.4;
  simulation.seed = seed;
  return simulation;
}

OnlineConfig toyConfig() {
  OnlineConfig config;
  config.obstacle_radius = 0.08;
  config.self_collision = false;
  return config;
}

// ------------------------------------------------------------
// Numerical Jacobian check for any NoiseModelFactor
// ------------------------------------------------------------

double maxJacobianError(const gtsam::NoiseModelFactor& factor, const gtsam::Values& x,
                        double h = 1e-6) {
  std::vector<gtsam::Matrix> H(factor.size());
  factor.unwhitenedError(x, &H);
  double worst = 0.0;
  for (size_t k = 0; k < factor.size(); ++k) {
    const gtsam::Key key = factor.keys()[k];
    const size_t dimension = x.at(key).dim();
    for (size_t j = 0; j < dimension; ++j) {
      gtsam::VectorValues delta_plus = x.zeroVectors(), delta_minus = x.zeroVectors();
      delta_plus.at(key)(j) = h;
      delta_minus.at(key)(j) = -h;
      const gtsam::Vector column = (factor.unwhitenedError(x.retract(delta_plus)) -
                                    factor.unwhitenedError(x.retract(delta_minus))) /
                                   (2 * h);
      worst = std::max(worst, (column - H[k].col(j)).cwiseAbs().maxCoeff());
    }
  }
  return worst;
}

Vector randomVector(std::mt19937& generator, long n, double scale) {
  std::uniform_real_distribution<double> uniform(-scale, scale);
  Vector x(n);
  for (long j = 0; j < n; ++j) x(j) = uniform(generator);
  return x;
}

}  // namespace

// ============================================================
// Interpolation
// ============================================================

// Open question 4 from the handoff: on a constant-acceleration segment (what
// DoubleIntegratorFactor enforces) the GP interpolation is exact.
TEST(Interpolation, hermiteReproducesConstantAcceleration) {
  std::mt19937 generator(1);
  for (int trial = 0; trial < 20; ++trial) {
    const double dt = 0.3 + trial * 0.1;
    const Vector q1 = randomVector(generator, 6, 2.0);
    const Vector v1 = randomVector(generator, 6, 1.0);
    const Vector u = randomVector(generator, 6, 1.0);
    const Vector q2 = q1 + dt * v1 + 0.5 * dt * dt * u;
    const Vector v2 = v1 + dt * u;
    for (int s = 0; s <= 10; ++s) {
      const double tau = dt * s / 10.0;
      const Vector exact = q1 + tau * v1 + 0.5 * tau * tau * u;
      EXPECT((interpolatePosition(q1, v1, q2, v2, dt, tau) - exact).cwiseAbs().maxCoeff() < 1e-12);
    }
  }
}

TEST(Interpolation, endpoints) {
  const HermiteCoefficients h0 = hermitePositionCoefficients(0.7, 0.0);
  const HermiteCoefficients h1 = hermitePositionCoefficients(0.7, 0.7);
  EXPECT_DOUBLES_EQUAL(1.0, h0.a, 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, h0.b, 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, h0.c, 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, h0.d, 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, h1.a, 1e-15);
  EXPECT_DOUBLES_EQUAL(1.0, h1.c, 1e-15);
  EXPECT_DOUBLES_EQUAL(0.0, h1.d, 1e-15);
}

// ============================================================
// Factors
// ============================================================

TEST(ObstacleMeasurementFactor, jacobians) {
  auto robot = std::make_shared<ToyPointRobot>();
  std::mt19937 generator(2);
  double worst = 0.0;
  for (int trial = 0; trial < 20; ++trial) {
    gtsam::Values x;
    x.insert(QKey(3), randomVector(generator, 3, 1.5));
    x.insert(LKey(), Point3(randomVector(generator, 3, 1.0)));
    const ObstacleMeasurementFactor factor(QKey(3), LKey(), Point3(0.1, -0.2, 0.3), robot,
                                      gtsam::noiseModel::Isotropic::Sigma(3, 0.01));
    worst = std::max(worst, maxJacobianError(factor, x));
  }
  std::cout << "  [diag] measurement factor max Jacobian error " << worst << "\n";
  EXPECT(worst < 1e-7);
}

TEST(ObstacleMeasurementFactor, zeroErrorAtTruth) {
  auto robot = std::make_shared<ToyPointRobot>();
  const Vector q = (Vector(3) << 0.3, -0.4, 0.5).finished();
  const Point3 l(0.2, 0.1, -0.3);
  const Point3 z = robot->sensorPose(q).transformTo(l);
  const ObstacleMeasurementFactor factor(QKey(0), LKey(), z, robot,
                                    gtsam::noiseModel::Isotropic::Sigma(3, 0.01));
  gtsam::Values x;
  x.insert(QKey(0), q);
  x.insert(LKey(), l);
  EXPECT(factor.unwhitenedError(x).norm() < 1e-12);
}

TEST(ToySphereObstacleGP, jacobiansWithActiveHinge) {
  ToyPointRobot robot;
  robot.setObstacle(Point3(0.0, 0.0, 0.0), 0.3);
  std::mt19937 generator(3);
  double worst = 0.0;
  int active = 0;
  for (int trial = 0; trial < 40; ++trial) {
    gtsam::Values x;
    x.insert(QKey(0), randomVector(generator, 3, 0.4));
    x.insert(VKey(0), randomVector(generator, 3, 0.2));
    x.insert(QKey(1), randomVector(generator, 3, 0.4));
    x.insert(VKey(1), randomVector(generator, 3, 0.2));
    const auto factor = robot.obstacleFactor(0, 0.4);
    const auto& noise_factor = dynamic_cast<const gtsam::NoiseModelFactor&>(*factor);
    if (noise_factor.unwhitenedError(x)(0) > 1e-3) ++active;
    worst = std::max(worst, maxJacobianError(noise_factor, x));
  }
  EXPECT(active > 10);  // the check means something only if the hinge is on
  EXPECT(worst < 1e-7);
}

// ============================================================
// Planner bookkeeping
// ============================================================

// The planner's named base factors must be the same cost as buildScateGraph.
TEST(OnlinePlanner, baseFactorsMatchBuildScateGraph) {
  const GraphSpec spec = toySpec();
  OnlineConfig config = toyConfig();
  config.sigma_effort = 0.0;
  OnlinePlanner planner(spec, config, std::make_shared<ToyPointRobot>(), toyInitialValues(spec));
  const gtsam::NonlinearFactorGraph reference = buildScateGraph(spec);
  EXPECT(planner.graph().size() == reference.size());
  std::mt19937 generator(4);
  for (int trial = 0; trial < 5; ++trial) {
    gtsam::Values x;
    for (int i = 0; i < spec.num_poses; ++i) {
      x.insert(QKey(i), randomVector(generator, 3, 2.5));
      x.insert(VKey(i), randomVector(generator, 3, 1.0));
    }
    for (int i = 0; i + 1 < spec.num_poses; ++i) x.insert(UKey(i), randomVector(generator, 3, 1.5));
    const double a = planner.graph().error(x), b = reference.error(x);
    EXPECT(std::fabs(a - b) <= 1e-9 * std::max(1.0, std::fabs(b)));
  }
}

TEST(OnlinePlanner, offlinePlanPassesEvaluation) {
  const GraphSpec spec = toySpec();
  OnlinePlanner planner(spec, toyConfig(), std::make_shared<ToyPointRobot>(),
                        toyInitialValues(spec));
  const StepInfo info = planner.planOffline();
  EXPECT(!info.have_obstacle);
  EXPECT(planner.count("obstacle") == 0);
  EXPECT(evaluateSolution(planner.values(), spec, false).passed);
}

// After each update only future intervals carry obstacle factors, executed
// controls lose their limits, and every measurement stays in the graph.
TEST(OnlinePlanner, outdatedFactorsAreRemoved) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyPointRobot>();
  OnlinePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  planner.planOffline();
  const Point3 obstacle(0.0, 0.0, 0.4);
  int detections = 0;
  for (int k = 1; k <= 8; ++k) {
    const Vector q = planner.values().at<Vector>(QKey(k));
    const Vector v = planner.values().at<Vector>(VKey(k));
    std::optional<Point3> z;
    if (k >= 4) {
      z = robot->sensorPose(q).transformTo(obstacle);
      ++detections;
    }
    planner.update(k, q, v, z);
    EXPECT(planner.count("obstacle", k) == 0);  // nothing on past intervals
    EXPECT(!planner.has("control_limit", k - 1));  // executed control
    EXPECT(planner.has("control_limit", k));       // next control still limited
    EXPECT(planner.count("measurement_q") == static_cast<size_t>(k));
    EXPECT(planner.count("measurement_l") == static_cast<size_t>(detections));
    if (k >= 4) {
      // checks per future interval, minus tau = 0 on the current one (the
      // state is measured), plus tau = dt on the last one (the final state).
      const size_t checks = toyConfig().tau_fractions.size();
      const size_t expected = checks * static_cast<size_t>(planner.numIntervals() - k);
      EXPECT(planner.count("obstacle") == expected);
      EXPECT((planner.obstacleEstimate() - obstacle).norm() < 1e-6);
    } else {
      EXPECT(planner.count("obstacle") == 0);
    }
  }
}

// Every state, executed or ahead, keeps its velocity limit.
TEST(OnlinePlanner, everyStateHasAVelocityLimit) {
  const GraphSpec spec = toySpec();
  OnlinePlanner planner(spec, toyConfig(), std::make_shared<ToyPointRobot>(),
                        toyInitialValues(spec));
  EXPECT(planner.count("velocity_limit") == static_cast<size_t>(spec.num_poses));
  for (int i = 0; i < spec.num_poses; ++i) EXPECT(planner.has("velocity_limit", i));
}

TEST(OnlinePlanner, rejectsEffortSigmasOfTheWrongSize) {
  const GraphSpec spec = toySpec();
  OnlineConfig config = toyConfig();
  config.effort_sigmas = Vector::Constant(2, 0.5);  // the toy robot has 3 joints
  bool threw = false;
  try {
    OnlinePlanner planner(spec, config, std::make_shared<ToyPointRobot>(), toyInitialValues(spec));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  EXPECT(threw);
}

// An obstacle placed on the goal is reported: no plan can avoid it.
TEST(OnlinePlanner, reportsAGoalInsideTheObstacle) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyPointRobot>();
  OnlinePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  const StepInfo offline = planner.planOffline();
  EXPECT(!offline.goal_clearance.has_value());  // no obstacle yet
  const Vector q = planner.values().at<Vector>(QKey(1));
  const Vector v = planner.values().at<Vector>(VKey(1));
  const Point3 on_goal(spec.q_goal);
  const StepInfo step = planner.update(1, q, v, robot->sensorPose(q).transformTo(on_goal));
  EXPECT(step.goal_clearance.has_value());
  EXPECT(*step.goal_clearance < 0.0);
}

TEST(OnlinePlanner, rejectsOutOfOrderUpdate) {
  const GraphSpec spec = toySpec();
  OnlinePlanner planner(spec, toyConfig(), std::make_shared<ToyPointRobot>(),
                        toyInitialValues(spec));
  planner.planOffline();
  bool threw = false;
  try {
    planner.update(2, spec.q_start, Vector::Zero(3), std::nullopt);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  EXPECT(threw);
}

// ============================================================
// Noise
// ============================================================

// The same seed gives the same samples on every platform (unlike
// std::normal_distribution). Reference values from an independent Python
// implementation on numpy's MT19937, which matches std::mt19937.
TEST(GaussianNoise, matchesReferenceValuesOnEveryPlatform) {
  GaussianNoise noise(0);
  EXPECT_DOUBLES_EQUAL(-0.914274096437681, noise.standardNormal(), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.457098685713743, noise.standardNormal(), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.631322460741121, noise.standardNormal(), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.632239184594120, noise.standardNormal(), 1e-12);
}

TEST(GaussianNoise, hasUnitVariance) {
  GaussianNoise noise(7);
  const Vector samples = noise.sample(20000, 2.0);
  const double mean = samples.mean();
  const double std_dev = std::sqrt((samples.array() - mean).square().mean());
  EXPECT_DOUBLES_EQUAL(0.0, mean, 0.05);
  EXPECT_DOUBLES_EQUAL(2.0, std_dev, 0.05);
}

// ============================================================
// Closed loop (paper Algorithm 1 end to end)
// ============================================================

TEST(ClosedLoop, avoidsObstacleDiscoveredOnline) {
  const GraphSpec spec = toySpec();
  double worst_clearance = 1e9, worst_goal = 0.0, worst_speed = 0.0, worst_l = 0.0, worst_ms = 0.0;
  int earliest = 99, latest = -1;
  for (unsigned seed = 0; seed < 10; ++seed) {
    auto robot = std::make_shared<ToyPointRobot>();
    OnlinePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
    const SimulationConfig simulation = toySimulation(seed);
    const ClosedLoopResult result = runClosedLoop(planner, *robot, simulation);
    EXPECT(!result.collided());
    worst_clearance = std::min(worst_clearance, result.min_obstacle_clearance);
    worst_goal = std::max(worst_goal, result.goal_error);
    worst_speed = std::max(worst_speed, result.final_speed);
    worst_l = std::max(worst_l, (planner.obstacleEstimate() - simulation.obstacle_center).norm());
    worst_ms = std::max(worst_ms, result.max_solve_ms);
    earliest = std::min(earliest, result.first_detection);
    latest = std::max(latest, result.first_detection);
  }
  std::cout << "  [diag] 10 seeds: min clearance " << worst_clearance << " m (target 0.05),"
            << " goal error <= " << worst_goal << ", final speed <= " << worst_speed
            << ", l error <= " << worst_l << " m, first seen at step " << earliest << "-"
            << latest << ", slowest step " << worst_ms << " ms\n";
  EXPECT(earliest >= 3);                 // discovered online, not known at the start
  EXPECT(worst_clearance > 0.05 - 0.01);  // standoff kept to within 1 cm
  // The last executed control's noise (0.01 rad/s^2) cannot be corrected
  // afterwards, and one control cannot fix both q_N and v_N, so the final
  // state misses by a few sigma of that noise (a blind robot misses equally).
  EXPECT(worst_goal < 0.05);
  EXPECT(worst_speed < 0.05);
  EXPECT(worst_l < 0.01);
}

// Without process noise the loop must land on the goal at rest.
TEST(ClosedLoop, reachesGoalWithoutProcessNoise) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyPointRobot>();
  OnlinePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  SimulationConfig simulation = toySimulation(0);
  simulation.sigma_process_u = 0.0;
  const ClosedLoopResult result = runClosedLoop(planner, *robot, simulation);
  std::cout << "  [diag] noiseless: goal error " << result.goal_error << ", final speed "
            << result.final_speed << ", min clearance " << result.min_obstacle_clearance << "\n";
  EXPECT(!result.collided());
  EXPECT(result.goal_error < 5e-3);
  EXPECT(result.final_speed < 5e-3);
}

// Negative control: a blind robot executing the same loop must hit the
// obstacle, or the test above proves nothing about the online updates.
TEST(ClosedLoop, blindRobotCollides) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyPointRobot>();
  OnlinePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  SimulationConfig simulation = toySimulation(0);
  simulation.sensor_range = 0.0;
  const ClosedLoopResult result = runClosedLoop(planner, *robot, simulation);
  std::cout << "  [diag] blind robot min clearance " << result.min_obstacle_clearance << " m\n";
  EXPECT(result.first_detection < 0);
  EXPECT(result.collided());
}

// ============================================================
// Replay file
// ============================================================

namespace {
size_t countOf(const std::string& text, const std::string& pattern) {
  size_t count = 0;
  for (size_t position = text.find(pattern); position != std::string::npos;
       position = text.find(pattern, position + 1)) {
    ++count;
  }
  return count;
}
}  // namespace

TEST(Replay, recordsPlansAndWritesBothRuns) {
  const GraphSpec spec = toySpec();
  OnlineConfig config = toyConfig();
  config.compute_l_covariance = true;
  auto robot = std::make_shared<ToyPointRobot>();
  OnlinePlanner online_planner(spec, config, robot, toyInitialValues(spec));
  const ClosedLoopResult online = runClosedLoop(online_planner, *robot, toySimulation(0));
  OnlinePlanner blind_planner(spec, config, robot, toyInitialValues(spec));
  SimulationConfig blind_simulation = toySimulation(0);
  blind_simulation.sensor_range = 0.0;
  const ClosedLoopResult blind = runClosedLoop(blind_planner, *robot, blind_simulation);

  const int N = spec.num_poses - 1;
  EXPECT(online.u.size() == static_cast<size_t>(N));
  EXPECT(online.steps.size() == static_cast<size_t>(N + 1));
  for (const StepInfo& step : online.steps) {
    EXPECT(step.plan_q.size() == static_cast<size_t>(N - step.k + 1));  // states k..N
    EXPECT(step.l_covariance.has_value() == step.have_obstacle);
  }
  // Each step's plan starts at the measured state it was solved from.
  EXPECT((online.steps[3].plan_q.front() - online.q[3]).cwiseAbs().maxCoeff() < 1e-2);
  // The covariance of l shrinks as measurements accumulate.
  const auto& last = online.steps.back();
  const StepInfo* first = nullptr;
  for (const auto& step : online.steps) {
    if (step.l_covariance) {
      first = &step;
      break;
    }
  }
  EXPECT(first && last.l_covariance && last.l_covariance->trace() < first->l_covariance->trace());

  ReplayScenario scenario;
  scenario.source = "testScateOnline \"toy\"";
  scenario.obstacle_center = toySimulation(0).obstacle_center;
  scenario.obstacle_radius = 0.08;
  scenario.obstacle_standoff = 0.05;
  scenario.substeps = 4;
  const std::string path = "test_replay.json";
  writeReplayJson(path, scenario, spec, config, *robot,
                  {{"online", "Online", &online, 0.4}, {"blind", "Blind", &blind, 0.0}});
  std::ifstream in(path);
  const std::string json((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT(json.rfind("{\"format\":\"scate-replay\"", 0) == 0);
  EXPECT(json.find("\\\"toy\\\"") != std::string::npos);  // quotes escaped
  EXPECT(countOf(json, "\"t\":") == 2 * static_cast<size_t>(N * scenario.substeps + 1));
  EXPECT(countOf(json, "\"plan_tool\":") == 2 * static_cast<size_t>(N + 1));
  EXPECT(countOf(json, "{") == countOf(json, "}"));
  EXPECT(countOf(json, "[") == countOf(json, "]"));
  EXPECT(json.find("nan") == std::string::npos && json.find("inf") == std::string::npos);
  std::remove(path.c_str());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
