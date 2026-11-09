/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testUr10eCollision.cpp
 * @author Harrison Jenkins
 *
 * Checks of ur10e_collision.h against the UR10e model that ships with
 * GTDynamics. The reference positions were computed independently, by a
 * numpy forward kinematics written straight from the URDF joint origins and
 * axes (including the fixed base_link -> base_link_inertia rotation), so a
 * frame or CoM-offset mistake in the model, or in how GTDynamics reads this
 * URDF, shows up here.
 */

#include <CppUnitLite/TestHarness.h>

#include <cmath>
#include <iostream>

#include "scate_online.h"
#include "scate_reactive.h"
#include "scate_scenario.h"
#include "ur10e_collision.h"

using namespace scate;
using gtsam::Point3;

namespace {

const gtdynamics::Robot& ur10e() {
  static const gtdynamics::Robot robot = loadRobot(ur10eUrdfPath());
  return robot;
}

Vector homeJoints() { return startJoints(); }  // 0, -1.5708, 1.5708, -1.5708, -1.5708, 0

Vector otherJoints() {
  Vector q(6);
  q << 0.3, -1.0, 1.1, -1.4, -1.2, 0.5;
  return q;
}

/// Largest difference between the factor's Jacobians and central differences.
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

/// Whether a is within tolerance of b; prints both if not.
bool near(const Point3& a, const Point3& b, double tolerance, const char* what) {
  const double error = (a - b).norm();
  if (error > tolerance) {
    std::cout << "  " << what << ": got " << a.transpose() << ", expected " << b.transpose()
              << " (error " << error << ")\n";
  }
  return error <= tolerance;
}

}  // namespace

TEST(Ur10eCollisionModel, builds) {
  const Ur10eCollisionModel model(ur10e());
  EXPECT(model.dof() == 6);
  EXPECT(model.numSpheres() == 15);
  EXPECT(model.numPairs() == 23);
  EXPECT(model.sensorSphere() == kUr10eSensorSphere);
  EXPECT(model.toolSphere() == static_cast<int>(kUr10eToolSphere));
}

// Reference FK: sphere centers at two configurations, in the URDF world frame.
TEST(Ur10eCollisionModel, sphereCentersMatchReferenceFK) {
  const Ur10eCollisionModel model(ur10e());
  const double tolerance = 1e-5;
  const Vector home = homeJoints(), other = otherJoints();
  EXPECT(near(model.spherePosition(home, 0), Point3(0.0, 0.0, 0.180700), tolerance, "home s0"));
  EXPECT(near(model.spherePosition(home, 5), Point3(-0.000002, 0.176000, 0.793400), tolerance, "home s5"));
  EXPECT(near(model.spherePosition(home, 10), Point3(0.571548, 0.043000, 0.793400), tolerance, "home s10"));
  EXPECT(near(model.spherePosition(home, 13), Point3(0.691398, 0.174150, 0.726850), tolerance, "home s13"));
  EXPECT(near(model.spherePosition(home, 14), Point3(0.691398, 0.174150, 0.676850), tolerance, "home s14"));
  EXPECT(near(model.spherePosition(other, 5), Point3(0.264246, 0.265969, 0.696269), tolerance, "other s5"));
  EXPECT(near(model.spherePosition(other, 10), Point3(0.846845, 0.306970, 0.639209), tolerance, "other s10"));
  EXPECT(near(model.spherePosition(other, 13), Point3(0.895435, 0.484524, 0.547383), tolerance, "other s13"));
  EXPECT(near(model.spherePosition(other, 14), Point3(0.878171, 0.498149, 0.502479), tolerance, "other s14"));
}

// The sensor frame is wrist_3's CoM frame, 2.6 cm behind the flange.
TEST(Ur10eCollisionModel, sensorPoseMatchesReferenceFK) {
  const Ur10eCollisionModel model(ur10e());
  const gtsam::Pose3 wTs = model.sensorPose(otherJoints());
  EXPECT(near(wTs.translation(), Point3(0.887148, 0.491064, 0.525829), 1e-5, "sensor origin"));
  const gtsam::Matrix3 rotation = wTs.rotation().matrix();
  EXPECT(near(Point3(rotation.row(0).transpose()), Point3(-0.280869, -0.895490, -0.345268), 1e-5,
              "sensor rotation row 0"));
}

// Link frames for drawing meshes. The forearm's CoM is 0.33 m from its link
// origin, so this checks the CoM-to-link-frame conversion, not just FK.
TEST(Ur10eCollisionModel, linkFramesMatchReferenceFK) {
  const Ur10eCollisionModel model(ur10e());
  const auto frames = model.linkFrames(otherJoints());
  EXPECT_LONGS_EQUAL(7, frames.size());  // base and six moving links
  bool found_forearm = false, found_wrist2 = false;
  for (const auto& [name, pose] : frames) {
    const Point3 z(pose.rotation().matrix().col(2));
    if (name == "forearm_link") {
      found_forearm = true;
      EXPECT(near(pose.translation(), Point3(0.316258, 0.097830, 0.696269), 1e-5, "forearm origin"));
      EXPECT(near(z, Point3(-0.295520, 0.955336, 0.0), 1e-5, "forearm z axis"));
    }
    if (name == "wrist_2_link") {
      found_wrist2 = true;
      EXPECT(near(pose.translation(), Point3(0.918412, 0.466390, 0.607150), 1e-5, "wrist_2 origin"));
      EXPECT(near(z, Point3(0.920522, 0.284751, -0.267499), 1e-5, "wrist_2 z axis"));
    }
  }
  EXPECT(found_forearm);
  EXPECT(found_wrist2);
}

TEST(Ur10eCollisionModel, clearancesMatchReference) {
  const Ur10eCollisionModel model(ur10e());
  EXPECT_DOUBLES_EQUAL(0.079719, model.selfClearance(homeJoints()), 1e-5);
  EXPECT_DOUBLES_EQUAL(0.095715, model.selfClearance(otherJoints()), 1e-5);
  const Point3 center(0.8, 0.35, 0.70);
  EXPECT_DOUBLES_EQUAL(0.078419, model.obstacleClearance(homeJoints(), center, 0.08), 1e-5);
  EXPECT_DOUBLES_EQUAL(-0.052014, model.obstacleClearance(otherJoints(), center, 0.08), 1e-5);
}

// Every sphere of the scenario's start and goals lies well inside the grid.
TEST(Ur10eCollisionModel, gridCoversTheScenario) {
  const Ur10eCollisionModel model(ur10e());
  for (const Vector& q : {startJoints(), hiddenGoalJoints(), sweepGoalJoints(), otherJoints()}) {
    EXPECT(model.gridMargin(q) > 0.3);
  }
}

// The grid layout must not swap x and y, also on the larger UR10e grid.
TEST(Ur10eSphereObstacleSDF, matchesExactDistance) {
  const SdfGrid grid = ur10eSdfGrid();
  const Point3 center(0.91, -0.47, 0.82);
  const auto sdf = makeSphereObstacleSDF(center, 0.08, grid);
  double worst = 0.0;
  for (const Point3& point : {Point3(1.2, -0.4, 0.8), Point3(0.2, 0.3, 1.4), Point3(-1.3, -1.1, -0.3),
                              Point3(0.91, -0.70, 0.82), Point3(1.15, -0.47, 0.82)}) {
    const double exact = (point - center).norm() - 0.08;
    worst = std::max(worst, std::fabs(sdf->getSignedDistance(point) - exact));
  }
  std::cout << "  [diag] UR10e SDF max interpolation error " << worst << " m\n";
  EXPECT(worst < 2e-3);
}

// Several obstacles: the field is the distance to the nearest one.
TEST(Ur10eSphereObstacleSDF, unionOfSpheresIsTheNearest) {
  const SdfGrid grid = ur10eSdfGrid();
  const std::vector<SphereObstacle> obstacles = {{Point3(0.9, -0.4, 0.8), 0.08},
                                                 {Point3(-0.3, 0.6, 0.5), 0.12}};
  const auto sdf = makeSpheresObstacleSDF(obstacles, grid);
  double worst = 0.0;
  // At least 0.2 m from the centers, where trilinear interpolation of the
  // cone-shaped field is accurate to a millimeter.
  for (const Point3& point : {Point3(1.15, -0.4, 0.8), Point3(-0.3, 0.85, 0.5),
                              Point3(0.3, 0.1, 0.65), Point3(-1.2, 1.2, 1.4)}) {
    const double exact = std::min((point - obstacles[0].center).norm() - obstacles[0].radius,
                                  (point - obstacles[1].center).norm() - obstacles[1].radius);
    worst = std::max(worst, std::fabs(sdf->getSignedDistance(point) - exact));
  }
  EXPECT(worst < 2e-3);
  THROWS_EXCEPTION(makeSpheresObstacleSDF({}, grid));
}

// Our wiring of the sensor Jacobian (queryPoses) into the measurement factor.
TEST(Ur10eCollisionModel, measurementFactorJacobians) {
  auto model = std::make_shared<Ur10eCollisionModel>(ur10e());
  gtsam::Values x;
  x.insert(QKey(2), otherJoints());
  x.insert(LKey(), Point3(0.6, 0.4, 0.7));
  const ObstacleMeasurementFactor factor(QKey(2), LKey(), Point3(0.05, -0.1, 0.2), model,
                                         gtsam::noiseModel::Isotropic::Sigma(3, 0.01));
  const double error = maxJacobianError(factor, x);
  std::cout << "  [diag] UR10e measurement factor Jacobian error " << error << "\n";
  EXPECT(error < 1e-6);
}

// Our wiring of ObstacleSDFFactorGP (keys, radii, interpolation), hinge active.
TEST(Ur10eCollisionModel, obstacleFactorJacobiansAndActivation) {
  Ur10eCollisionModel model(ur10e());
  model.setObstacle(Point3(0.8, 0.35, 0.70), 0.08);
  gtsam::Values x;
  x.insert(QKey(4), otherJoints());
  x.insert(VKey(4), (Vector(6) << 0.02, 0.01, -0.01, 0.0, 0.01, 0.0).finished());
  x.insert(QKey(5), Vector(otherJoints().array() + 0.03));
  x.insert(VKey(5), (Vector(6) << 0.01, 0.0, 0.01, 0.0, 0.0, 0.01).finished());
  const auto factor = model.obstacleFactor(4, 0.4);
  const auto& noise_factor = dynamic_cast<const gtsam::NoiseModelFactor&>(*factor);
  EXPECT(noise_factor.unwhitenedError(x).maxCoeff() > 1e-3);  // in contact: hinge on
  const double error = maxJacobianError(noise_factor, x);
  std::cout << "  [diag] UR10e obstacle factor Jacobian error " << error << "\n";
  EXPECT(error < 1e-5);
}

TEST(Ur10eCollisionModel, selfCollisionFactorBuilds) {
  const Ur10eCollisionModel model(ur10e());
  gtsam::Values x;
  x.insert(QKey(0), homeJoints());
  x.insert(VKey(0), Vector(Vector::Zero(6)));
  x.insert(QKey(1), homeJoints());
  x.insert(VKey(1), Vector(Vector::Zero(6)));
  const auto factor = model.selfCollisionFactor(0, 0.5);
  const auto& noise_factor = dynamic_cast<const gtsam::NoiseModelFactor&>(*factor);
  EXPECT(noise_factor.dim() == 23);
  // Home is 8.0 cm from its closest pair, beyond the 2 cm standoff: no cost.
  EXPECT(noise_factor.unwhitenedError(x).norm() < 1e-12);
}

// Motion-capture markers (scate_reactive.h), against the numpy FK.
TEST(Ur10eCollisionModel, markersMatchReferenceFK) {
  const Ur10eCollisionModel model(ur10e());
  EXPECT_LONGS_EQUAL(8, model.numMarkers());
  const Vector markers = model.markerPositions(otherJoints());
  EXPECT_LONGS_EQUAL(24, markers.size());
  EXPECT(near(Point3(markers.segment<3>(0)), Point3(0.000590, 0.272338, 0.306921), 1e-5, "m0"));
  EXPECT(near(Point3(markers.segment<3>(9)), Point3(0.708986, 0.264326, 0.719015), 1e-5, "m3"));
  EXPECT(near(Point3(markers.segment<3>(12)), Point3(0.782532, 0.424357, 0.542854), 1e-5, "m4"));
  EXPECT(near(Point3(markers.segment<3>(21)), Point3(0.781717, 0.522672, 0.524732), 1e-5, "m7"));
}

TEST(Ur10eCollisionModel, markerFactorJacobian) {
  auto model = std::make_shared<Ur10eCollisionModel>(ur10e());
  gtsam::Values x;
  x.insert(QKey(3), otherJoints());
  const Vector measured = model->markerPositions(sweepGoalJoints());
  const MarkerMeasurementFactor factor(QKey(3), measured, model,
                                       gtsam::noiseModel::Isotropic::Sigma(24, 1e-3));
  const double error = maxJacobianError(factor, x);
  std::cout << "  [diag] UR10e marker factor Jacobian error " << error << "\n";
  EXPECT(error < 1e-6);
}

// Every joint angle is observable from one reading, to under 0.01 rad with
// 1 mm marker noise, along the scenario's motion.
TEST(Ur10eCollisionModel, everyJointIsObservableFromTheMarkers) {
  const Ur10eCollisionModel model(ur10e());
  for (const Vector& q : {startJoints(), sweepGoalJoints(), otherJoints()}) {
    gtsam::Matrix H;
    model.markerPositions(q, &H);
    const gtsam::Matrix covariance = (H.transpose() * H).inverse() * 1e-6;
    const double worst = covariance.diagonal().cwiseSqrt().maxCoeff();
    std::cout << "  [diag] worst joint std from markers " << worst << " rad\n";
    EXPECT(worst < 0.01);
  }
}

TEST(SphereCollisionModel, rejectsIndicesOfMissingSpheres) {
  SphereModel bad_pair = ur10eSphereModel();
  bad_pair.self_collision_pairs.push_back({3, 15});  // there are 15 spheres: 0..14
  THROWS_EXCEPTION(SphereCollisionModel(ur10e(), bad_pair, ur10eCollisionParams()));
  SphereModel bad_tool = ur10eSphereModel();
  bad_tool.tool_sphere = 15;
  THROWS_EXCEPTION(SphereCollisionModel(ur10e(), bad_tool, ur10eCollisionParams()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
