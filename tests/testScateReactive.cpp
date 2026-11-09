/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testScateReactive.cpp
 * @author Harrison Jenkins
 *
 * Tests of scate_reactive.h (reactive SCATE with motion-capture sensing) on a
 * toy robot: a sphere whose joint angles are its 3D position, with one
 * motion-capture marker at its center, among moving spherical obstacles.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "scate_reactive.h"
#include "trajectory_init.h"

using namespace scate;
using gtsam::Point3;
using gtsam::assert_equal;

namespace {

/// Hinge max(0, epsilon - gap) to each obstacle, at the Hermite-interpolated position.
class ToySpheresObstacleGP : public gtsam::NoiseModelFactorN<Vector, Vector, Vector, Vector> {
  using Base = gtsam::NoiseModelFactorN<Vector, Vector, Vector, Vector>;
  std::vector<SphereObstacle> obstacles_;
  double robot_radius_, epsilon_;
  HermiteCoefficients coefficients_;

 public:
  ToySpheresObstacleGP(gtsam::Key q1, gtsam::Key v1, gtsam::Key q2, gtsam::Key v2,
                       const std::vector<SphereObstacle>& obstacles, double robot_radius,
                       double epsilon, double sigma, double dt, double tau)
      : Base(gtsam::noiseModel::Isotropic::Sigma(obstacles.size(), sigma), q1, v1, q2, v2),
        obstacles_(obstacles),
        robot_radius_(robot_radius),
        epsilon_(epsilon),
        coefficients_(hermitePositionCoefficients(dt, tau)) {}

  gtsam::Vector evaluateError(const Vector& q1, const Vector& v1, const Vector& q2,
                              const Vector& v2, gtsam::OptionalMatrixType H1 = nullptr,
                              gtsam::OptionalMatrixType H2 = nullptr,
                              gtsam::OptionalMatrixType H3 = nullptr,
                              gtsam::OptionalMatrixType H4 = nullptr) const override {
    const Vector position = coefficients_.a * q1 + coefficients_.b * v1 +
                            coefficients_.c * q2 + coefficients_.d * v2;
    const size_t count = obstacles_.size();
    gtsam::Vector error = gtsam::Vector::Zero(count);
    gtsam::Matrix gradient = gtsam::Matrix::Zero(count, 3);
    for (size_t j = 0; j < count; ++j) {
      const Vector offset = position - Vector(obstacles_[j].center);
      const double distance = offset.norm();
      const double gap = distance - obstacles_[j].radius - robot_radius_;
      if (gap < epsilon_) {
        error(j) = epsilon_ - gap;
        gradient.row(j) = -(offset / distance).transpose();
      }
    }
    if (H1) *H1 = coefficients_.a * gradient;
    if (H2) *H2 = coefficients_.b * gradient;
    if (H3) *H3 = coefficients_.c * gradient;
    if (H4) *H4 = coefficients_.d * gradient;
    return error;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new ToySpheresObstacleGP(*this));
  }
};

/// A sphere at q (3D), with a motion-capture marker at its center.
class ToyMarkerRobot : public CollisionModel {
 public:
  double robot_radius = 0.05, dt = 1.0, sigma_obstacle = 0.01, epsilon = 0.05;
  std::vector<SphereObstacle> installed;  ///< what the planner last installed
  int install_count = 0;

  size_t dof() const override { return 3; }
  gtsam::Pose3 sensorPose(const Vector& q, gtsam::Matrix* H = nullptr) const override {
    if (H) *H = gtsam::Matrix::Zero(6, 3);
    return gtsam::Pose3(gtsam::Rot3(), Point3(q));
  }
  void setObstacle(const Point3& center, double radius) override {
    installed = {SphereObstacle{center, radius}};
    ++install_count;
  }
  void setObstacles(const std::vector<SphereObstacle>& obstacles) override {
    installed = obstacles;
    ++install_count;
  }
  gtsam::NonlinearFactor::shared_ptr obstacleFactor(int i, double tau) const override {
    return gtsam::NonlinearFactor::shared_ptr(
        new ToySpheresObstacleGP(QKey(i), VKey(i), QKey(i + 1), VKey(i + 1), installed,
                                 robot_radius, epsilon, sigma_obstacle, dt, tau));
  }
  double obstacleClearance(const Vector& q, const Point3& center, double radius) const override {
    return (q - Vector(center)).norm() - radius - robot_radius;
  }
  size_t numMarkers() const override { return 1; }
  Vector markerPositions(const Vector& q, gtsam::Matrix* H = nullptr) const override {
    if (H) *H = gtsam::Matrix::Identity(3, 3);
    return q;
  }
  gtsam::Matrix sphereCenters(const Vector& q) const override { return gtsam::Matrix(q); }
  Vector sphereRadii() const override { return Vector::Constant(1, robot_radius); }
  int toolSphere() const override { return 0; }
};

/// Cross 1.2 m along x in 20 s (as in testScateOnline).
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
  const JointTrajectory trajectory = makeInitialTrajectory(spec.q_start, spec.q_goal, config);
  gtsam::Values values;
  for (int i = 0; i < spec.num_poses; ++i) {
    values.insert(QKey(i), trajectory.q[i]);
    values.insert(VKey(i), trajectory.v[i]);
  }
  for (int i = 0; i + 1 < spec.num_poses; ++i) values.insert(UKey(i), trajectory.u[i]);
  return values;
}

/**
 * An obstacle crossing the straight path: it starts 0.3 m to the side and
 * reaches the path's midpoint (0, 0, 0.4) at t = 10 s, when the straight-line
 * plan is there too.
 */
MovingObstacle crossingObstacle() {
  MovingObstacle obstacle;
  obstacle.start = Point3(0.0, -0.3, 0.4);
  obstacle.velocity = gtsam::Vector3(0.0, 0.03, 0.0);
  obstacle.radius = 0.08;
  return obstacle;
}

ReactiveConfig toyConfig(size_t obstacles = 1) {
  ReactiveConfig config;
  config.obstacle_radii.assign(obstacles, 0.08);
  config.sigma_executed_control = 2e-3;  // the toy's process noise
  config.self_collision = false;
  return config;
}

/**
 * The toy's process noise is 5x smaller than the default: with 1 cm/s^2 the
 * sphere drifts about 10 cm over the run, as much as the obstacle's margin,
 * and the negative controls could miss by luck.
 */
ViconSimulationConfig toySimulation(ObstacleSensing sensing, unsigned seed = 0) {
  ViconSimulationConfig simulation;
  simulation.obstacles = {crossingObstacle()};
  simulation.sigma_process_u = 2e-3;
  simulation.sensing = sensing;
  simulation.seed = seed;
  return simulation;
}

ClosedLoopResult runToy(ObstacleSensing sensing, unsigned seed = 0) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyMarkerRobot>();
  ReactivePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  return runReactiveClosedLoop(planner, *robot, toySimulation(sensing, seed));
}

/// A reading of the toy robot at q with the given obstacle centers.
ViconMeasurement reading(const Vector& q, const std::vector<std::optional<Point3>>& obstacles) {
  ViconMeasurement measurement;
  measurement.markers = q;
  measurement.obstacles = obstacles;
  return measurement;
}

double maxJacobianError(const gtsam::NoiseModelFactor& factor, const gtsam::Values& x,
                        double h = 1e-6) {
  std::vector<gtsam::Matrix> H(factor.size());
  factor.unwhitenedError(x, &H);
  double worst = 0.0;
  for (size_t k = 0; k < factor.size(); ++k) {
    const gtsam::Key key = factor.keys()[k];
    for (size_t j = 0; j < x.at(key).dim(); ++j) {
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

}  // namespace

// ============================================================
// Keys and factors
// ============================================================

TEST(ObstacleKey, isUniquePerObstacleAndStep) {
  EXPECT(ObstacleKey(0, 3) != ObstacleKey(1, 3));
  EXPECT(ObstacleKey(0, 3) != ObstacleKey(0, 4));
  EXPECT(ObstacleKey(1, 0) != QKey(0) && ObstacleKey(1, 0) != LKey());
  THROWS_EXCEPTION(ObstacleKey(26, 0));
}

TEST(MarkerMeasurementFactor, errorAndJacobian) {
  auto robot = std::make_shared<ToyMarkerRobot>();
  const Vector measured = (Vector(3) << 0.1, 0.2, 0.3).finished();
  const MarkerMeasurementFactor factor(QKey(2), measured, robot,
                                       gtsam::noiseModel::Isotropic::Sigma(3, 1e-3));
  gtsam::Values x;
  x.insert(QKey(2), Vector((Vector(3) << 0.15, 0.1, 0.3).finished()));
  EXPECT(assert_equal(Vector((Vector(3) << 0.05, -0.1, 0.0).finished()),
                      factor.unwhitenedError(x), 1e-12));
  EXPECT(maxJacobianError(factor, x) < 1e-8);
  THROWS_EXCEPTION(MarkerMeasurementFactor(QKey(0), Vector::Zero(4), robot,
                                           gtsam::noiseModel::Isotropic::Sigma(4, 1.0)));
}

TEST(SphereObstacles, defaultModelAcceptsExactlyOne) {
  // CollisionModel's default setObstacles forwards a single obstacle only.
  class OneObstacle : public ToyMarkerRobot {
   public:
    void setObstacles(const std::vector<SphereObstacle>& obstacles) override {
      CollisionModel::setObstacles(obstacles);
    }
  } robot;
  robot.setObstacles({SphereObstacle{Point3(1, 2, 3), 0.1}});
  EXPECT(robot.installed.size() == 1);
  THROWS_EXCEPTION(robot.setObstacles({}));
}

// ============================================================
// Planner bookkeeping (paper Algorithm 1)
// ============================================================

TEST(ReactivePlanner, initialPlanHasNoObstacleFactors) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyMarkerRobot>();
  ReactivePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  planner.planOffline();
  EXPECT_LONGS_EQUAL(0, planner.count("obstacle"));
  EXPECT(planner.has("prior_q", 0));
  EXPECT(!planner.obstacleEstimate(0));
}

TEST(ReactivePlanner, measuredStartReplacesTheStartPriorOnQOnly) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyMarkerRobot>();
  ReactivePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  planner.planOffline();
  planner.update(0, reading(spec.q_start, {Point3(0.0, -0.3, 0.4)}));
  EXPECT(!planner.has("prior_q", 0));
  EXPECT(planner.has("prior_v", 0));  // the arm is known to start at rest
  EXPECT(planner.has("measurement_markers", 0));
  // With the obstacle known at k = 0, every interval gets the default 3 checks,
  // minus the measured state's own, plus the final state.
  EXPECT_LONGS_EQUAL(3 * 20 - 1 + 1, planner.count("obstacle"));
}

TEST(ReactivePlanner, executedControlIsTiedToTheCommandedOne) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyMarkerRobot>();
  ReactivePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  planner.planOffline();
  planner.update(0, reading(spec.q_start, {std::nullopt}));
  const Vector commanded = planner.control(0);
  planner.update(1, reading(spec.q_start, {std::nullopt}));
  EXPECT(!planner.has("control_limit", 0));
  EXPECT(!planner.has("effort", 0));
  EXPECT(planner.has("executed_control", 0));
  EXPECT(planner.has("control_limit", 1));
  // The prior holds the value that was sent, not the re-solved one.
  const auto graph = planner.graph();
  bool found = false;
  for (const auto& factor : graph) {
    const auto prior = std::dynamic_pointer_cast<gtsam::PriorFactor<Vector>>(factor);
    if (prior && prior->key() == UKey(0) &&
        prior->noiseModel()->sigmas()(0) == toyConfig().sigma_executed_control) {
      found = true;
      EXPECT(assert_equal(commanded, prior->prior(), 1e-12));
    }
  }
  EXPECT(found);
}

// The reactive assumption: future obstacle factors use the latest reading.
TEST(ReactivePlanner, obstacleFactorsFollowTheLatestReading) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyMarkerRobot>();
  ReactivePlanner planner(spec, toyConfig(2), robot, toyInitialValues(spec));
  planner.planOffline();
  planner.update(0, reading(spec.q_start, {Point3(0, -0.3, 0.4), Point3(0.3, 0.3, 0.4)}));
  planner.update(1, reading(spec.q_start, {Point3(0, -0.27, 0.4), std::nullopt}));
  CHECK(robot->installed.size() == 2);
  EXPECT(assert_equal(Point3(0, -0.27, 0.4), robot->installed[0].center, 1e-6));
  // Not seen at step 1: the planner keeps planning against its last reading.
  EXPECT(assert_equal(Point3(0.3, 0.3, 0.4), robot->installed[1].center, 1e-6));
  EXPECT(assert_equal(Point3(0, -0.27, 0.4), *planner.obstacleEstimate(0), 1e-6));
  // Past readings stay in the graph as history: two of obstacle 0, one of 1.
  EXPECT_LONGS_EQUAL(3, planner.count("measurement_l"));
  // Obstacle factors only on the intervals still ahead.
  EXPECT_LONGS_EQUAL(0, planner.count("obstacle", 1));
}

// Paper Algorithm 1, "if i = N remove phi^goal": at the end the reading, not
// the goal prior, says where the arm is.
TEST(ReactivePlanner, finalReadingReplacesTheGoalPriors) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyMarkerRobot>();
  ReactivePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  planner.planOffline();
  const int last = planner.numIntervals();
  // Readings along the plan (perfect execution) until the last state.
  for (int k = 0; k < last; ++k) {
    planner.update(k, reading(planner.values().at<Vector>(QKey(k)), {std::nullopt}));
  }
  EXPECT(planner.has("goal_q", last));
  // The arm ends 5 mm short of the goal: the estimate must side with the reading.
  const Vector short_of_goal = spec.q_goal - (Vector(3) << 0.005, 0.0, 0.0).finished();
  const StepInfo info = planner.update(last, reading(short_of_goal, {std::nullopt}));
  EXPECT(!planner.has("goal_q", last) && !planner.has("goal_v", last));
  const double to_reading = (info.plan_q[0] - short_of_goal).norm();
  const double to_goal = (info.plan_q[0] - spec.q_goal).norm();
  std::cout << "  [diag] final estimate: " << to_reading << " m from the reading, " << to_goal
            << " m from the goal\n";
  EXPECT(to_reading < to_goal);
}

TEST(ReactivePlanner, rejectsOutOfOrderOrMismatchedReadings) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyMarkerRobot>();
  ReactivePlanner planner(spec, toyConfig(), robot, toyInitialValues(spec));
  THROWS_EXCEPTION(planner.update(0, reading(spec.q_start, {std::nullopt})));  // before planOffline
  planner.planOffline();
  THROWS_EXCEPTION(planner.update(1, reading(spec.q_start, {std::nullopt})));  // skips k = 0
  THROWS_EXCEPTION(planner.update(0, reading(spec.q_start, {})));  // wrong obstacle count
}

// ============================================================
// Closed loop
// ============================================================

TEST(MovingObstacle, movesAtConstantVelocity) {
  const MovingObstacle obstacle = crossingObstacle();
  EXPECT(assert_equal(Point3(0, 0, 0.4), obstacle.center(10.0), 1e-12));
}

// Markers give positions only; with the executed-control prior and exact
// executed dynamics the velocity is still estimated to the noise level.
TEST(ReactiveClosedLoop, estimatesVelocityFromMarkersAlone) {
  const ClosedLoopResult result = runToy(ObstacleSensing::kEveryStep);
  double sum_squares = 0.0;
  for (size_t k = 1; k < result.steps.size(); ++k) {
    sum_squares += (result.steps[k].plan_v[0] - result.v[k]).squaredNorm();
  }
  const double rms = std::sqrt(sum_squares / (3.0 * (result.steps.size() - 1)));
  std::cout << "  [diag] velocity estimate RMS error " << rms << " m/s\n";
  EXPECT(rms < 2e-3);
}

// The same at the default process noise (1 cm/s^2), which is where exact
// executed dynamics matter: 3.3 mm/s RMS here, 5.7 mm/s with the planning
// sigma (1 cm) left on executed intervals.
TEST(ReactiveClosedLoop, estimatesVelocityAtTheDefaultProcessNoise) {
  const GraphSpec spec = toySpec();
  auto robot = std::make_shared<ToyMarkerRobot>();
  ReactiveConfig config = toyConfig();
  config.sigma_executed_control = 1e-2;
  ReactivePlanner planner(spec, config, robot, toyInitialValues(spec));
  ViconSimulationConfig simulation = toySimulation(ObstacleSensing::kNone);
  simulation.sigma_process_u = 1e-2;
  const ClosedLoopResult result = runReactiveClosedLoop(planner, *robot, simulation);
  double sum_squares = 0.0;
  for (size_t k = 1; k < result.steps.size(); ++k) {
    sum_squares += (result.steps[k].plan_v[0] - result.v[k]).squaredNorm();
  }
  const double rms = std::sqrt(sum_squares / (3.0 * (result.steps.size() - 1)));
  std::cout << "  [diag] velocity estimate RMS error at 1 cm/s^2 process noise " << rms << " m/s\n";
  EXPECT(rms < 4.5e-3);
}

TEST(ReactiveClosedLoop, avoidsAMovingObstacle) {
  for (unsigned seed : {0u, 1u, 2u}) {
    const ClosedLoopResult result = runToy(ObstacleSensing::kEveryStep, seed);
    std::cout << "  [diag] seed " << seed << ": reactive min clearance "
              << result.min_obstacle_clearance << " m, goal error " << result.goal_error << "\n";
    EXPECT(!result.collided());
    EXPECT(result.goal_error < 0.02);
    EXPECT_LONGS_EQUAL(0, result.first_detection);
  }
}

// Negative controls: the same run, but the obstacle is only seen at the
// start ("frozen") or never ("blind"). Both hit it.
TEST(ReactiveClosedLoop, frozenAndBlindRobotsCollide) {
  const ClosedLoopResult frozen = runToy(ObstacleSensing::kFirstStepOnly);
  const ClosedLoopResult blind = runToy(ObstacleSensing::kNone);
  std::cout << "  [diag] frozen " << frozen.min_obstacle_clearance << " m, blind "
            << blind.min_obstacle_clearance << " m\n";
  EXPECT(frozen.min_obstacle_clearance < 0.0);
  EXPECT(blind.min_obstacle_clearance < 0.0);
  EXPECT_LONGS_EQUAL(-1, blind.first_detection);
}

TEST(ReactiveClosedLoop, recordsTrueObstacleCenters) {
  const ClosedLoopResult result = runToy(ObstacleSensing::kEveryStep);
  EXPECT_LONGS_EQUAL(21, result.obstacle_centers.size());
  EXPECT_LONGS_EQUAL(21, result.steps.size());
  EXPECT_LONGS_EQUAL(20, result.u.size());
  EXPECT(assert_equal(Point3(0, 0, 0.4), result.obstacle_centers[10][0], 1e-12));
  EXPECT_LONGS_EQUAL(1, result.min_clearance_per_obstacle.size());
  EXPECT_DOUBLES_EQUAL(result.min_obstacle_clearance, result.min_clearance_per_obstacle[0], 1e-12);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
