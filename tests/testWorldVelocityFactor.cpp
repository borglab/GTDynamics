/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testWorldVelocityFactor.cpp
 * @brief Verify body-to-world velocity convention and Jacobians.
 * @author GTDynamics contributors
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/constrained/TangentCone.h>
#include <gtdynamics/dynamics/WorldVelocityFactor.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/utils/PoseContactConstraintSpec.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/PriorFactor.h>
using namespace gtsam;
using namespace gtdynamics;

/* ************************************************************************* */
namespace velocity_frames {
// Checks the right-local pose Jacobian and the body/world velocity convention
// under a nonidentity rotation, with nonzero angular and linear body velocity.
TEST(WorldVelocityFactor, WorldVelocityJacobians) {
  const Pose3 pose(Rot3::RzRyRx(.2, -.3, .5), Point3(2, 3, 4));
  const Vector6 twist{.2, .3, -.1, .8, -.4, .5};
  const Vector3 world = pose.rotation() * twist.tail<3>();
  const WorldVelocityFactor factor(1, 2, 3, noiseModel::Unit::Create(3));
  Matrix H1, H2, H3;
  EXPECT(assert_equal(Vector3::Zero(),
                      factor.evaluateError(pose, twist, world, &H1, &H2, &H3),
                      1e-12));
  std::function<Vector3(const Pose3&, const Vector6&, const Vector3&)> error =
      [&factor](const Pose3& p, const Vector6& v, const Vector3& w) {
        return Vector3(factor.evaluateError(p, v, w));
      };
  EXPECT(
      assert_equal(numericalDerivative31(error, pose, twist, world), H1, 1e-8));
  EXPECT(
      assert_equal(numericalDerivative32(error, pose, twist, world), H2, 1e-8));
  EXPECT(
      assert_equal(numericalDerivative33(error, pose, twist, world), H3, 1e-8));
}
}  // namespace velocity_frames
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
