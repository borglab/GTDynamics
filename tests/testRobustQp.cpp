/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/RobustQp.h>
#include <gtdynamics/utils/ContactWrench.h>
#include <gtsam/base/TestableAssertions.h>

#include <Eigen/Geometry>

using namespace gtsam;
using namespace gtdynamics;

/* ************************************************************************* */
namespace degenerate_faces {
// Redundant active rows have nonunique duals but a unique primal projection;
// the AL fallback preserves the free third coordinate and multirow constraints.
TEST(RobustQp, DependentMultirowWorkingSet) {
  QpProblem problem;
  problem.addCost(JacobianFactor(1, Matrix3::Identity(), Vector3(1, 1, 2)));
  Matrix A(3, 3);
  A << 1, 0, 0, 0, 1, 0, 1, 1, 0;
  problem.addConstraint(
      LinearConstraint::LessEqual(JacobianFactor(1, A, Vector3::Zero())));
  Values initial;
  initial.insert(1, Vector(Vector3::Zero()));
  ActiveSetSolver::State working;
  working.activeInequalityRows = {true, true, true};
  const auto result =
      gtdynamics::internal::robustQp(problem, initial, working, 1000);
  EXPECT(result.converged);
  EXPECT(assert_equal(Vector3(0, 0, 2), result.values.at<Vector>(1), 1e-7));
  EXPECT_LONGS_EQUAL(3, result.activeInequalityRows.size());
}

// A complete 17-face sole wrench cone can be projected from its apex without
// losing a nonzero feasible direction or treating dependent faces as failure.
TEST(RobustQp, FullWrenchConeApex) {
  const RectangularContactWrench patch(.102, .036, .6);
  const Matrix A = patch.matrix();
  for (const auto& target :
       {Vector6((Vector6() << .1, -.05, .02, .3, -.2, 0).finished()),
        Vector6((Vector6() << -.03, .02, .01, -.1, .2, .1).finished())}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(1, Matrix6::Identity(), target));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(1, A, Vector::Zero(A.rows()))));
    Values initial;
    initial.insert(1, Vector(Vector6::Zero()));
    ActiveSetSolver::State working;
    working.activeInequalityRows.assign(A.rows(), true);
    const auto result =
        gtdynamics::internal::robustQp(problem, initial, working, 1000);
    const Vector6 projected = result.values.at<Vector6>(1);
    EXPECT(result.converged);
    EXPECT((A * projected).maxCoeff() < 1e-7);
    EXPECT(projected.norm() > .01);
    EXPECT_DOUBLES_EQUAL(0, projected.dot(target - projected), 1e-5);
    // The residual belongs to the polar cone: every feasible corner force is
    // on the inward side of the projection's supporting hyperplane.
    const double mu = patch.pyramidCoefficient();
    for (double x : {-.102, .102})
      for (double y : {-.036, .036})
        for (double sx : {-1.0, 1.0})
          for (double sy : {-1.0, 1.0}) {
            const Vector3 force(100 * mu * sx, 100 * mu * sy, 100);
            Vector6 sample;
            sample << Vector3(x, y, 0).cross(force), force;
            EXPECT((target - projected).dot(sample - projected) < 1e-5);
          }
  }
}
}  // namespace degenerate_faces
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
