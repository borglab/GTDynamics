/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testUr5Collision.cpp
 * @author Harrison Jenkins
 *
 * Checks of ur5_collision.h against the UR5 model that ships with GTDynamics.
 * The reference positions were
 * computed independently, by a numpy forward kinematics written straight from
 * the URDF joint origins and axes, so a frame or CoM-offset mistake in the
 * model (or in how GTDynamics reads the URDF) shows up here.
 */

#include <CppUnitLite/TestHarness.h>
#include <cmath>
#include <iostream>

#include "scate_online.h"
#include "ur5_collision.h"

using namespace scate;
using gtsam::Point3;

namespace {

const gtdynamics::Robot& ur5() {
  static const gtdynamics::Robot robot = loadRobot(ur5UrdfPath());
  return robot;
}

Vector homeJoints() { return startJoints(); }  // 0, -1.5708, 1.5708, -1.5708, -1.5708, 0

Vector otherJoints() {
  Vector q(6);
  q << 0.3, -1.0, 1.1, -1.4, -1.2, 0.5;
  return q;
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

bool near(const Point3& a, const Point3& b, double tolerance, const char* what) {
  const double error = (a - b).norm();
  if (error > tolerance) {
    std::cout << "  " << what << ": got " << a.transpose() << ", expected " << b.transpose()
              << " (error " << error << ")\n";
  }
  return error <= tolerance;
}

}  // namespace

TEST(Ur5CollisionModel, builds) {
  const Ur5CollisionModel model(ur5());
  EXPECT(model.dof() == 6);
  EXPECT(model.numSpheres() == 15);
  EXPECT(model.numPairs() == 22);
}

// Reference FK: sphere centers at two configurations.
TEST(Ur5CollisionModel, sphereCentersMatchReferenceFK) {
  const Ur5CollisionModel model(ur5());
  const double tolerance = 1e-5;
  const Vector home = homeJoints(), other = otherJoints();
  EXPECT(near(model.spherePosition(home, 0), Point3(0.0, 0.0, 0.089159), tolerance, "home s0"));
  EXPECT(near(model.spherePosition(home, 5), Point3(-0.000002, 0.135850, 0.514159), tolerance, "home s5"));
  EXPECT(near(model.spherePosition(home, 10), Point3(0.391998, 0.016150, 0.514159), tolerance, "home s10"));
  EXPECT(near(model.spherePosition(home, 13), Point3(0.486898, 0.109150, 0.514159), tolerance, "home s13"));
  EXPECT(near(model.spherePosition(home, 14), Point3(0.486899, 0.109150, 0.431859), tolerance, "home s14"));
  EXPECT(near(model.spherePosition(other, 5), Point3(0.179226, 0.197642, 0.446784), tolerance, "other s5"));
  EXPECT(near(model.spherePosition(other, 10), Point3(0.587221, 0.198554, 0.407649), tolerance, "other s10"));
  EXPECT(near(model.spherePosition(other, 13), Point3(0.647103, 0.314425, 0.382306), tolerance, "other s13"));
  EXPECT(near(model.spherePosition(other, 14), Point3(0.618687, 0.336852, 0.308394), tolerance, "other s14"));
}

TEST(Ur5CollisionModel, sensorPoseMatchesReferenceFK) {
  const Ur5CollisionModel model(ur5());
  const gtsam::Pose3 wTs = model.sensorPose(otherJoints());
  EXPECT(near(wTs.translation(), Point3(0.647103, 0.314425, 0.382306), 1e-5, "sensor origin"));
  const gtsam::Matrix3 rotation = wTs.rotation().matrix();
  EXPECT(near(Point3(rotation.row(0).transpose()), Point3(-0.280869, -0.345268, 0.89549), 1e-5,
              "sensor rotation row 0"));
}

// Link frames for drawing meshes. The forearm's CoM is 0.25 m from its link
// origin, so this checks the CoM-to-link-frame conversion, not just FK.
TEST(Ur5CollisionModel, linkFramesMatchReferenceFK) {
  const Ur5CollisionModel model(ur5());
  const auto frames = model.linkFrames(otherJoints());
  bool found_forearm = false, found_wrist2 = false;
  for (const auto& [name, pose] : frames) {
    const Point3 z(pose.rotation().matrix().col(2));
    if (name == "forearm_link") {
      found_forearm = true;
      EXPECT(near(pose.translation(), Point3(0.214600, 0.083289, 0.446784), 1e-5, "forearm origin"));
      EXPECT(near(z, Point3(0.950564, 0.294044, -0.099833), 1e-5, "forearm z axis"));
    }
    if (name == "wrist_2_link") {
      found_wrist2 = true;
      EXPECT(near(pose.translation(), Point3(0.559975, 0.287474, 0.407625), 1e-5, "wrist_2 origin"));
      EXPECT(near(z, Point3(0.920522, 0.284751, -0.267499), 1e-5, "wrist_2 z axis"));
    }
  }
  EXPECT(found_forearm);
  EXPECT(found_wrist2);
}

TEST(Ur5CollisionModel, clearancesMatchReference) {
  const Ur5CollisionModel model(ur5());
  EXPECT_DOUBLES_EQUAL(0.071144, model.selfClearance(homeJoints()), 1e-5);
  EXPECT_DOUBLES_EQUAL(0.087997, model.selfClearance(otherJoints()), 1e-5);
  const Point3 center(0.3, 0.2, 0.5);
  EXPECT_DOUBLES_EQUAL(0.005246, model.obstacleClearance(homeJoints(), center, 0.08), 1e-5);
  EXPECT_DOUBLES_EQUAL(-0.016788, model.obstacleClearance(otherJoints(), center, 0.08), 1e-5);
  EXPECT(model.gridMargin(homeJoints()) > 0.3);
}

// The grid layout must not swap x and y (GTDynamics warns about exactly this).
TEST(SphereObstacleSDF, matchesExactDistance) {
  const SdfGrid grid;
  const Point3 center(0.31, -0.17, 0.42);
  const auto sdf = makeSphereObstacleSDF(center, 0.08, grid);
  double worst = 0.0;
  for (const Point3& point : {Point3(0.6, -0.1, 0.4), Point3(0.2, 0.3, 0.7), Point3(-0.4, -0.5, 0.1),
                          Point3(0.31, -0.40, 0.42), Point3(0.55, -0.17, 0.42)}) {
    const double exact = (point - center).norm() - 0.08;
    worst = std::max(worst, std::fabs(sdf->getSignedDistance(point) - exact));
  }
  std::cout << "  [diag] SDF max interpolation error " << worst << " m\n";
  EXPECT(worst < 2e-3);
}

// Our wiring of the sensor Jacobian (queryPoses) into the measurement factor.
TEST(Ur5CollisionModel, measurementFactorJacobians) {
  auto model = std::make_shared<Ur5CollisionModel>(ur5());
  gtsam::Values x;
  x.insert(QKey(2), otherJoints());
  x.insert(LKey(), Point3(0.4, 0.3, 0.5));
  const ObstacleMeasurementFactor factor(QKey(2), LKey(), Point3(0.05, -0.1, 0.2), model,
                                    gtsam::noiseModel::Isotropic::Sigma(3, 0.01));
  const double error = maxJacobianError(factor, x);
  std::cout << "  [diag] UR5 measurement factor Jacobian error " << error << "\n";
  EXPECT(error < 1e-6);
}

// Our wiring of ObstacleSDFFactorGP (keys, radii, interpolation), hinge active.
TEST(Ur5CollisionModel, obstacleFactorJacobiansAndActivation) {
  Ur5CollisionModel model(ur5());
  model.setObstacle(Point3(0.3, 0.2, 0.5), 0.08);
  gtsam::Values x;
  x.insert(QKey(4), otherJoints());
  x.insert(VKey(4), (Vector(6) << 0.02, 0.01, -0.01, 0.0, 0.01, 0.0).finished());
  x.insert(QKey(5), Vector(otherJoints().array() + 0.03));
  x.insert(VKey(5), (Vector(6) << 0.01, 0.0, 0.01, 0.0, 0.0, 0.01).finished());
  const auto factor = model.obstacleFactor(4, 0.4);
  const auto& noise_factor = dynamic_cast<const gtsam::NoiseModelFactor&>(*factor);
  EXPECT(noise_factor.unwhitenedError(x).maxCoeff() > 1e-3);  // in contact: hinge on
  const double error = maxJacobianError(noise_factor, x);
  std::cout << "  [diag] UR5 obstacle factor Jacobian error " << error << "\n";
  EXPECT(error < 1e-5);
}

TEST(Ur5CollisionModel, selfCollisionFactorBuilds) {
  const Ur5CollisionModel model(ur5());
  gtsam::Values x;
  x.insert(QKey(0), homeJoints());
  x.insert(VKey(0), Vector(Vector::Zero(6)));
  x.insert(QKey(1), homeJoints());
  x.insert(VKey(1), Vector(Vector::Zero(6)));
  const auto factor = model.selfCollisionFactor(0, 0.5);
  const auto& noise_factor = dynamic_cast<const gtsam::NoiseModelFactor&>(*factor);
  EXPECT(noise_factor.dim() == 22);
  // Home is 7.1 cm from its closest pair, beyond the 2 cm standoff: no cost.
  EXPECT(noise_factor.unwhitenedError(x).norm() < 1e-12);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
