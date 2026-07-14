/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSTEAP.cpp
 * @brief test simultaneous trajectory estimation and planning: a GPMP2 plan
 *        solved and then updated with measurement factors through iSAM2.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/config.h>
#include <gtdynamics/factors/GPLinearPriorFactor.h>
#include <gtdynamics/factors/ObstacleSDFFactor.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Point3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::noiseModel::Isotropic;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

static const double kCell = 0.05;
static const double kRadius = 0.15;
static const double kEpsilon = 0.10;
static const double kCostSigma = 0.01;
static const size_t kNumStates = 5;
static const double kDeltaT = 0.5;

static const Robot kRobot =
    CreateRobotFromFile(kUrdfPath + std::string("bar_lab.urdf"));

// The nine movable joints of robot1, gantry prismatic then arm revolute.
static std::vector<JointSharedPtr> robot1Joints() {
  const std::vector<std::string> names = {
      "bridge1_joint_EA_X", "robot1_joint_EA_Y", "robot1_joint_EA_Z",
      "robot1_joint_1",     "robot1_joint_2",    "robot1_joint_3",
      "robot1_joint_4",     "robot1_joint_5",    "robot1_joint_6"};
  std::vector<JointSharedPtr> joints;
  for (auto &&name : names) joints.push_back(kRobot.joint(name));
  return joints;
}

static std::vector<PointOnLink> wristPoints() {
  const LinkSharedPtr link = kRobot.link("robot1_link_6");
  return {PointOnLink(link, Point3(0.0, 0.0, 0.0)),
          PointOnLink(link, Point3(0.0, 0.0, 0.1))};
}

static Vector startConfig() {
  return (Vector(9) << 2.0, 2.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.5, 0.0)
      .finished();
}
static Vector goalConfig() {
  return (Vector(9) << 3.0, 2.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.5, 0.0)
      .finished();
}

// Sample the exact signed distance to a sphere onto a grid, layer (row=y,col=x).
static SignedDistanceField makeSphereSDF(const Point3 &center, double radius,
                                         const Point3 &origin, double cell,
                                         size_t nx, size_t ny, size_t nz) {
  std::vector<Matrix> data(nz);
  for (size_t k = 0; k < nz; ++k) {
    Matrix layer(ny, nx);
    for (size_t i = 0; i < ny; ++i) {
      for (size_t j = 0; j < nx; ++j) {
        const Point3 p = origin + Point3(j * cell, i * cell, k * cell);
        layer(i, j) = (p - center).norm() - radius;
      }
    }
    data[k] = layer;
  }
  return SignedDistanceField(origin, cell, data);
}

// Holds the whole planning problem: the query point model, the obstacle field,
// the straight line initialisation, and the sphere centre.
struct SteapProblem {
  RobotQueryPoints model;
  std::shared_ptr<const SignedDistanceField> sdf;
  std::vector<Vector> line;
  Point3 center;
};

static SteapProblem makeProblem() {
  RobotQueryPoints model(kRobot, "columns", robot1Joints(), wristPoints());
  const Vector q_start = startConfig(), q_goal = goalConfig();

  std::vector<Vector> line(kNumStates);
  std::vector<Point3> swept;
  for (size_t k = 0; k < kNumStates; ++k) {
    line[k] = q_start +
              (static_cast<double>(k) / (kNumStates - 1)) * (q_goal - q_start);
    std::vector<Point3> ps;
    model.queryPoints(line[k], &ps);
    swept.insert(swept.end(), ps.begin(), ps.end());
  }

  std::vector<Point3> mid;
  model.queryPoints(line[kNumStates / 2], &mid);
  const Point3 center = mid[0];

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

  return SteapProblem{model, sdf, line, center};
}

// The GPMP2 planning graph: GP priors, obstacle factors, and a start and goal
// fix factor. The position of the goal pose factor is returned so it can later
// be swapped for a measurement, as STEAP does at the end of execution.
static gtsam::NonlinearFactorGraph buildPlanGraph(const SteapProblem &prob,
                                                  size_t *goal_factor_pos) {
  const size_t dof = prob.model.dof();
  gtsam::NonlinearFactorGraph graph;
  auto qc_model = Isotropic::Sigma(dof, 1.0);
  auto fix = Isotropic::Sigma(dof, 1e-4);

  graph.addPrior<Vector>(X(0), startConfig(), fix);
  graph.addPrior<Vector>(V(0), Vector::Zero(dof), fix);
  *goal_factor_pos = graph.size();
  graph.addPrior<Vector>(X(kNumStates - 1), goalConfig(), fix);
  graph.addPrior<Vector>(V(kNumStates - 1), Vector::Zero(dof), fix);

  for (size_t k = 0; k < kNumStates; ++k) {
    graph.emplace_shared<ObstacleSDFFactor>(X(k), prob.model, prob.sdf,
                                            kCostSigma, kEpsilon);
  }
  for (size_t k = 0; k < kNumStates - 1; ++k) {
    graph.emplace_shared<GPLinearPrior>(X(k), V(k), X(k + 1), V(k + 1), kDeltaT,
                                        qc_model);
  }
  return graph;
}

static Values straightLineInit(const SteapProblem &prob) {
  const Vector velocity =
      (goalConfig() - startConfig()) / (kNumStates * kDeltaT);
  Values init;
  for (size_t k = 0; k < kNumStates; ++k) {
    init.insert(X(k), prob.line[k]);
    init.insert(V(k), velocity);
  }
  return init;
}

// iSAM2 is non-linear, so a few updates are needed to reach the batch solution.
static gtsam::ISAM2Params steapParams() {
  gtsam::ISAM2Params params;
  params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  return params;
}

static Values iterate(gtsam::ISAM2 &isam, int iterations = 10) {
  for (int i = 0; i < iterations; ++i) isam.update();
  return isam.calculateEstimate();
}

/* ******************** the initial plan is GPMP2 *********************** */

// At the first step, before any measurement, STEAP is exactly the GPMP2 plan.
// Solved through iSAM2, the endpoints are held and every query point is pushed
// clear of the sphere, checked against the analytic sphere so a point that left
// the grid cannot pass by reading as free space.
TEST(STEAP, initialPlanIsCollisionFree) {
  const SteapProblem prob = makeProblem();
  size_t goal_pos;
  const gtsam::NonlinearFactorGraph graph = buildPlanGraph(prob, &goal_pos);

  gtsam::ISAM2 isam(steapParams());
  isam.update(graph, straightLineInit(prob));
  const Values result = iterate(isam);

  EXPECT(assert_equal(startConfig(), result.at<Vector>(X(0)), 1e-3));
  EXPECT(assert_equal(goalConfig(), result.at<Vector>(X(kNumStates - 1)), 1e-3));

  for (size_t k = 0; k < kNumStates; ++k) {
    std::vector<Point3> ps;
    prob.model.queryPoints(result.at<Vector>(X(k)), &ps);
    for (auto &&p : ps) {
      EXPECT((p - prob.center).norm() > kRadius + kEpsilon - 0.02);
    }
  }
}

/* ****************** a measurement updates the trajectory *************** */

// A measurement factor is a unary Gaussian prior on the state at the current
// step, with the sensor reading as its mean. Adding one at an interior state
// pulls that state toward the measurement, while the endpoints stay pinned by
// their fix factors. This is the estimation half of STEAP responding to data.
TEST(STEAP, measurementFactorUpdatesTrajectory) {
  const SteapProblem prob = makeProblem();
  const size_t dof = prob.model.dof();
  size_t goal_pos;
  const gtsam::NonlinearFactorGraph graph = buildPlanGraph(prob, &goal_pos);

  gtsam::ISAM2 isam(steapParams());
  isam.update(graph, straightLineInit(prob));
  const Values plan = iterate(isam);

  const size_t step = kNumStates / 2;
  // A noisy execution has landed the robot a third of a metre further along the
  // gantry rail than planned, a collision-free config the sensor reports.
  Vector measured = plan.at<Vector>(X(step));
  measured(0) += 0.35;

  gtsam::NonlinearFactorGraph measurement;
  measurement.addPrior<Vector>(X(step), measured, Isotropic::Sigma(dof, 1e-4));
  isam.update(measurement, Values());
  const Values updated = iterate(isam);

  // The tight measurement dominates, so the state snaps onto it.
  EXPECT(assert_equal(measured, updated.at<Vector>(X(step)), 3e-2));
  // The endpoints are untouched by an interior measurement.
  EXPECT(assert_equal(startConfig(), updated.at<Vector>(X(0)), 1e-3));
  EXPECT(
      assert_equal(goalConfig(), updated.at<Vector>(X(kNumStates - 1)), 1e-3));
}

/* *************** the goal factor is swapped at the end **************** */

// When the trajectory finishes, STEAP replaces the goal fix factor with a pose
// measurement, so the final posterior is a pure estimation solution. Removing
// the goal factor and adding a measurement at the last state should move it to
// wherever the robot actually ended up.
TEST(STEAP, goalFactorSwappedForMeasurement) {
  const SteapProblem prob = makeProblem();
  const size_t dof = prob.model.dof();
  size_t goal_pos;
  const gtsam::NonlinearFactorGraph graph = buildPlanGraph(prob, &goal_pos);

  gtsam::ISAM2 isam(steapParams());
  const gtsam::ISAM2Result first = isam.update(graph, straightLineInit(prob));
  iterate(isam);
  const gtsam::FactorIndex goal_index = first.newFactorsIndices.at(goal_pos);

  // The robot arrived a little short of the planned goal.
  Vector arrived = goalConfig();
  arrived(0) -= 0.2;

  gtsam::NonlinearFactorGraph measurement;
  measurement.addPrior<Vector>(X(kNumStates - 1), arrived,
                               Isotropic::Sigma(dof, 1e-4));
  gtsam::FactorIndices remove;
  remove.push_back(goal_index);
  isam.update(measurement, Values(), remove);
  const Values updated = iterate(isam);

  // With the goal factor gone, the final state follows the measurement.
  EXPECT(assert_equal(arrived, updated.at<Vector>(X(kNumStates - 1)), 3e-2));
  EXPECT(assert_equal(startConfig(), updated.at<Vector>(X(0)), 1e-3));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
