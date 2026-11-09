/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testBaseWrench.cpp
 * @author Harrison Jenkins
 *
 * Checks of base_wrench.h on the UR10e: against GTDynamics' own inverse
 * dynamics (the factor-graph solver DynamicsGraph::linearSolveID, with
 * gravity off), and against momentum conservation, which holds whatever
 * the inertias: over a rest-to-rest motion the time integral of the base
 * wrench is zero.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/TestableAssertions.h>

#include <cmath>
#include <iostream>
#include <random>

#include "base_wrench.h"
#include "scate_reactive_scenario.h"
#include "scate_scenario.h"
#include "ur10e_scenarios.h"
#include "trajectory_init.h"
#include "ur10e_collision.h"

using namespace scate;

namespace {

const gtdynamics::Robot& ur10e() {
  static const gtdynamics::Robot robot = loadRobot(ur10eUrdfPath());
  return robot;
}

Vector randomVector(std::mt19937* generator, double scale) {
  std::uniform_real_distribution<double> uniform(-scale, scale);
  Vector x(6);
  for (int i = 0; i < 6; ++i) x(i) = uniform(*generator);
  return x;
}

/**
 * The wrench on the base from GTDynamics' inverse dynamics, without gravity,
 * moved to the base frame like BaseWrenchModel::wrench.
 */
gtsam::Vector6 gtdynamicsBaseWrench(const gtdynamics::Robot& robot, const Vector& q,
                                    const Vector& v, const Vector& a) {
  const int t = 0;
  const auto order = jointIdsInChainOrder(robot);
  const auto base = robot.link(baseLinkName(robot));
  gtsam::Values values;
  for (auto&& joint : robot.joints()) {
    const size_t p = chainIndex(order, joint->id());
    gtdynamics::InsertJointAngle(&values, joint->id(), t, q(p));
    gtdynamics::InsertJointVel(&values, joint->id(), t, v(p));
  }
  // forwardKinematics inserts the fixed base's pose and zero twist itself.
  gtsam::Values known = robot.forwardKinematics(values, t, base->name());
  for (auto&& joint : robot.joints()) {
    gtdynamics::InsertJointAccel(&known, joint->id(), t, a(chainIndex(order, joint->id())));
  }
  gtdynamics::DynamicsGraph graph(gtsam::Vector3::Zero());
  const gtsam::Values result = graph.linearSolveID(robot, t, known);
  // The first joint along the chain connects the base to link 1.
  const gtsam::Vector6 on_base_com = gtdynamics::Wrench(result, base->id(), order.front(), t);
  return base->bMcom().inverse().AdjointMap().transpose() * on_base_com;
}

}  // namespace

TEST(BaseWrenchModel, matchesGTDynamicsInverseDynamics) {
  const BaseWrenchModel model(ur10e());
  EXPECT_LONGS_EQUAL(6, model.dof());
  std::mt19937 generator(42);
  for (int trial = 0; trial < 5; ++trial) {
    const Vector q = randomVector(&generator, 3.0);
    const Vector v = randomVector(&generator, 1.0);
    const Vector a = randomVector(&generator, 2.0);
    const gtsam::Vector6 ours = model.wrench(q, v, a);
    const gtsam::Vector6 reference = gtdynamicsBaseWrench(ur10e(), q, v, a);
    if (trial == 0) {
      std::cout << "  [diag] base wrench " << ours.transpose() << "\n"
                << "  [diag] GTDynamics  " << reference.transpose() << "\n";
    }
    EXPECT(gtsam::assert_equal(gtsam::Vector(reference), gtsam::Vector(ours),
                               1e-6 * std::max(1.0, reference.norm())));
  }
}

// Without gravity there is no wrench when nothing moves, in any pose.
TEST(BaseWrenchModel, isZeroAtRest) {
  const BaseWrenchModel model(ur10e());
  const Vector zero = Vector::Zero(6);
  EXPECT(model.wrench(startJoints(), zero, zero).norm() < 1e-12);
  EXPECT(model.wrench(sweepGoalJoints(), zero, zero).norm() < 1e-12);
}

// Spinning only the base joint at constant acceleration about the vertical
// axis: the base feels a pure torque about z opposite to the acceleration
// (at rest, so no centrifugal force), and only horizontal force.
TEST(BaseWrenchModel, shoulderPanAccelerationTwistsTheBaseBack) {
  const BaseWrenchModel model(ur10e());
  Vector a = Vector::Zero(6);
  a(0) = 1.0;
  const gtsam::Vector6 wrench = model.wrench(startJoints(), Vector::Zero(6), a);
  std::cout << "  [diag] pan at 1 rad/s^2 from home: " << wrench.transpose() << "\n";
  EXPECT(wrench(2) < -1.0);                  // reaction torque about +z
  EXPECT(std::fabs(wrench(5)) < 1e-9);       // no vertical force
}

// Momentum conservation: rest-to-rest, the integral of the wrench is zero.
// Linear momentum gives the force; angular momentum about the fixed base
// origin gives the torque. Independent of how inertias are expressed.
TEST(BaseWrenchModel, restToRestImpulseIsZero) {
  const BaseWrenchModel model(ur10e());
  TrajectoryConfig config;
  config.num_poses = 21;
  config.dt = 1.0;
  const JointTrajectory trajectory =
      makeInitialTrajectory(startJoints(), sweepGoalJoints(), config);
  // Integrate exactly piecewise-constant accelerations between rest states,
  // re-deriving each interval's start state so the motion is consistent.
  Vector q = startJoints(), v = Vector::Zero(6);
  const int samples = 200;
  gtsam::Vector6 impulse = gtsam::Vector6::Zero(), peak = gtsam::Vector6::Zero();
  for (int i = 0; i + 1 < config.num_poses; ++i) {
    const Vector u = trajectory.u[i];
    for (int s = 0; s < samples; ++s) {
      const double tau = (s + 0.5) / samples * config.dt;
      const gtsam::Vector6 wrench =
          model.wrench(q + tau * v + 0.5 * tau * tau * u, v + tau * u, u);
      impulse += wrench * config.dt / samples;
      peak = peak.cwiseMax(wrench.cwiseAbs());
    }
    q = q + config.dt * v + 0.5 * config.dt * config.dt * u;
    v = v + config.dt * u;
  }
  std::cout << "  [diag] final speed " << v.norm() << ", impulse " << impulse.transpose()
            << ", peak " << peak.transpose() << "\n";
  // The fitted controls bring v back to about 1e-3, not exactly zero, so
  // compare with the momentum such a residual speed can carry.
  for (int i = 0; i < 6; ++i) EXPECT(std::fabs(impulse(i)) < 0.05 * peak(i) * config.num_poses);
  EXPECT(impulse.norm() < 0.05);
}

TEST(BaseWrenchFactor, evaluatesTheWrenchInsideTheInterval) {
  auto model = std::make_shared<const BaseWrenchModel>(ur10e());
  const double tau = 0.4;
  const BaseWrenchFactor factor(QKey(3), VKey(3), UKey(3), model, tau, baseWrenchNoise(1.0, 1.0));
  std::mt19937 generator(7);
  const Vector q = randomVector(&generator, 2.0), v = randomVector(&generator, 0.5),
               u = randomVector(&generator, 0.5);
  gtsam::Values x;
  x.insert(QKey(3), q);
  x.insert(VKey(3), v);
  x.insert(UKey(3), u);
  EXPECT(gtsam::assert_equal(
      gtsam::Vector(model->wrench(q + tau * v + 0.5 * tau * tau * u, v + tau * u, u)),
      factor.unwhitenedError(x), 1e-12));
  // Jacobians: compare with a coarser central difference of the whole factor.
  std::vector<gtsam::Matrix> H(3);
  factor.unwhitenedError(x, &H);
  const double h = 1e-4;
  double worst = 0.0;
  for (size_t k = 0; k < 3; ++k) {
    for (int j = 0; j < 6; ++j) {
      gtsam::VectorValues plus = x.zeroVectors(), minus = x.zeroVectors();
      plus.at(factor.keys()[k])(j) = h;
      minus.at(factor.keys()[k])(j) = -h;
      const gtsam::Vector column =
          (factor.unwhitenedError(x.retract(plus)) - factor.unwhitenedError(x.retract(minus))) /
          (2 * h);
      worst = std::max(worst, (column - H[k].col(j)).cwiseAbs().maxCoeff());
    }
  }
  std::cout << "  [diag] base wrench factor Jacobian error " << worst << "\n";
  EXPECT(worst < 1e-4);
}

// The planner adds the cost on every interval and drops it once executed.
TEST(ReactivePlannerBaseWrench, costCoversOnlyTheIntervalsAhead) {
  const gtdynamics::Robot& robot = ur10e();
  const gtdynamics::Kinematics kinematics = makeKinematics();
  const InitialProblem problem = initializeProblem(kinematics, robot);
  const GraphSpec spec = makeGraphSpec(robot, problem);
  auto model = std::make_shared<SphereCollisionModel>(robot, ur10eSphereModel(),
                                                      ur10eCollisionParams());
  ReactiveConfig config;
  config.base_wrench = std::make_shared<const BaseWrenchModel>(robot);
  ReactivePlanner planner(spec, config, model, problem.values);
  EXPECT_LONGS_EQUAL(4 * 20, planner.count("base_wrench"));
  planner.planOffline();
  ViconMeasurement reading;
  reading.markers = model->markerPositions(startJoints());
  planner.update(0, reading);
  planner.update(1, reading);
  EXPECT(!planner.has("base_wrench", 0));
  EXPECT(planner.has("base_wrench", 1));
  EXPECT_LONGS_EQUAL(4 * 19, planner.count("base_wrench"));
}

// The point of the cost: dodging the moving obstacles shakes the base much
// less, without hitting them (UR10e scenario, seed 0).
TEST(ReactivePlannerBaseWrench, reducesTheBaseDisturbanceWhileDodging) {
  const std::vector<scate::BaseWrenchStudyRow> rows =
      runBaseWrenchStudy(ur10eUrdfPath(), 1, ur10eReactiveScenario(), ur10eSphereModel(),
                         ur10eCollisionParams(), {0.0, 3.0}, "/tmp/testBaseWrench_series.csv");
  const auto& off = rows[0];
  const auto& on = rows[1];
  std::cout << "  [diag] base torque rms " << off.mean.rms_torque << " -> " << on.mean.rms_torque
            << " N m, clearance " << off.worst_clearance << " -> " << on.worst_clearance << " m\n";
  EXPECT(on.mean.rms_torque < 0.5 * off.mean.rms_torque);
  EXPECT(on.mean.torque_impulse < 0.5 * off.mean.torque_impulse);
  EXPECT_LONGS_EQUAL(0, on.collisions);
  EXPECT(on.worst_goal_error < 0.1);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
