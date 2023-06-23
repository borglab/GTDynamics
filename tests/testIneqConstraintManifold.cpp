
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtdynamics/manifold/CartPoleWithFriction.h>
#include <gtdynamics/manifold/IneqConstraintManifold.h>

using namespace gtdynamics;
using namespace gtsam;

template <typename Container>
bool container_equal(const Container &c1, const Container &c2) {
  if (c1.size() != c2.size()) {
    return false;
  }
  return std::equal(c1.begin(), c1.end(), c2.begin());
}

/// Simple manifold with corner defined by x1>=0, x2>=0
// TEST(IneqConstraintManifold, SimpleLinearConstraints) {
//   Key x1_key = 1;
//   Key x2_key = 2;

//   InequalityConstraints constraints;
//   Double_ x1_expr(x1_key);
//   Double_ x2_expr(x2_key);
//   constraints.emplace_shared<DoubleExpressionInequality>(x1_expr, 1.0);
//   constraints.emplace_shared<DoubleExpressionInequality>(x2_expr, 1.0);

//   Values values;
//   values.insert(x1_key, 1.0);
//   values.insert(x2_key, 0.0);

//   std::set<size_t> active_indices{1};

//   // Test manifold construction
//   IneqConstraintManifold manifold(constraints, values, active_indices);

//   // Test blocking constraints
//   {
//     VectorValues v;
//     v.insert(x1_key, Vector1(1.0));
//     v.insert(x2_key, Vector1(1.0));
//     VectorValues proj_v;
//     std::set<size_t> blocking_indices;
//     std::tie(blocking_indices, proj_v) =
//         manifold.identifyBlockingConstraints(v);
//     VectorValues expected_proj_v;
//     expected_proj_v.insert(x1_key, Vector1(1.0));
//     expected_proj_v.insert(x2_key, Vector1(1.0));
//     EXPECT(assert_equal(expected_proj_v, proj_v));
//     std::set<size_t> expected_blocking_indices{};
//     EXPECT(container_equal(expected_blocking_indices, blocking_indices));
//   }
//   {
//     VectorValues v;
//     v.insert(x1_key, Vector1(1.0));
//     v.insert(x2_key, Vector1(-1.0));
//     VectorValues proj_v;
//     std::set<size_t> blocking_indices;
//     std::tie(blocking_indices, proj_v) =
//         manifold.identifyBlockingConstraints(v);
//     VectorValues expected_proj_v;
//     expected_proj_v.insert(x1_key, Vector1(1.0));
//     expected_proj_v.insert(x2_key, Vector1(0.0));
//     EXPECT(assert_equal(expected_proj_v, proj_v));
//     std::set<size_t> expected_blocking_indices{1};
//     EXPECT(container_equal(expected_blocking_indices, blocking_indices));
//   }
//   values = Values();
//   values.insert(x1_key, 0.0);
//   values.insert(x2_key, 0.0);
//   active_indices = std::set<size_t>{0, 1};
//   manifold = IneqConstraintManifold(constraints, values, active_indices);
//   {
//     VectorValues v;
//     v.insert(x1_key, Vector1(-1.0));
//     v.insert(x2_key, Vector1(-1.0));
//     VectorValues proj_v;
//     std::set<size_t> blocking_indices;
//     std::tie(blocking_indices, proj_v) =
//         manifold.identifyBlockingConstraints(v);
//     VectorValues expected_proj_v;
//     expected_proj_v.insert(x1_key, Vector1(0.0));
//     expected_proj_v.insert(x2_key, Vector1(0.0));
//     EXPECT(assert_equal(expected_proj_v, proj_v));
//     std::set<size_t> expected_blocking_indices{0, 1};
//     EXPECT(container_equal(expected_blocking_indices, blocking_indices));
//   }

//   // Test retraction
//   values = Values();
//   values.insert(x1_key, 1.0);
//   values.insert(x2_key, 0.0);
//   active_indices = std::set<size_t>{1};
//   manifold = IneqConstraintManifold(constraints, values, active_indices);
//   VectorValues delta;
//   delta.insert(x1_key, Vector1(-2.0));
//   delta.insert(x2_key, Vector1(0.0));
//   std::set<size_t> tight_indices{1};
//   auto new_manifold = manifold.retract(delta, tight_indices);
//   Values expected_values;
//   expected_values.insert(x1_key, 0.0);
//   expected_values.insert(x2_key, 0.0);
//   EXPECT(assert_equal(expected_values, new_manifold.values()));
//   std::set<size_t> expected_active_indices{0, 1};
//   EXPECT(
//       container_equal(expected_active_indices,
//       new_manifold.activeIndices()));
// }

// TEST(IneqConstraintManifold, CartPoleWithFrictionCone) {
//   CartPoleWithFriction cp;
//   Key q_key = gtsam::Symbol('q', 0);
//   Key v_key = gtsam::Symbol('v', 0);
//   Key a_key = gtsam::Symbol('a', 0);

//   Double_ q_expr(q_key), v_expr(v_key), a_expr(a_key);
//   Double_ torque_expr = cp.torqueExpr(q_expr, a_expr);
//   Double_ min_torque_expr = torque_expr - Double_(cp.tau_min);
//   Double_ max_torque_expr = Double_(cp.tau_max) - torque_expr;
//   auto fc_exprs = cp.fcExprs(q_expr, v_expr, a_expr);

//   InequalityConstraints constraints;
//   constraints.emplace_shared<DoubleExpressionInequality>(min_torque_expr, 1.0);
//   constraints.emplace_shared<DoubleExpressionInequality>(max_torque_expr, 1.0);
//   constraints.emplace_shared<DoubleExpressionInequality>(fc_exprs.first, 1.0);
//   constraints.emplace_shared<DoubleExpressionInequality>(fc_exprs.second, 1.0);

//   Values values;
//   values.insert(q_key, 0.0);
//   values.insert(v_key, 0.0);
//   values.insert(a_key, 0.0);
//   IndexSet active_indices{};

//   IneqConstraintManifold manifold(constraints, values, active_indices);
//   {
//     VectorValues delta;
//     delta.insert(q_key, Vector1(2.0));
//     delta.insert(v_key, Vector1(10.0));
//     delta.insert(a_key, Vector1(20.0));
//     auto new_manifold = manifold.retractLineSearch(delta, active_indices);
//     new_manifold->values().print();
//     new_manifold->activeIndices().print("active constraints:\n");
//     for (const auto& constraint: constraints) {
//       std::cout << "constriant evaluation: " <<
//       (*constraint)(new_manifold->values()) << "\n";
//     }
//   }
// }

TEST(IneqConstraintManifold, Dome) {
  Key point_key = gtsam::Symbol('p', 0);
  double radius = 1;
  Point3 point(1, 0, 0);
  DomeManifold manifold(point_key, radius, point);

  {
    VectorValues delta;
    delta.insert(point_key, Vector3(0, 1, 0));

    // check projected direction
    IndexSet tight_indices;
    VectorValues proj_delta;
    std::tie(tight_indices, proj_delta) =
        manifold.identifyBlockingConstraints(delta);
    EXPECT(tight_indices.size() == 0);
    EXPECT(assert_equal(delta, proj_delta));

    // check retraction
    Values expected_values;
    expected_values.insert(point_key, Point3(1 / sqrt(2), 1 / sqrt(2), 0));
    auto new_manifold = manifold.retract(delta, tight_indices);
    EXPECT(assert_equal(expected_values, new_manifold->values()));
  }

  {
    VectorValues delta;
    delta.insert(point_key, Vector3(1, 1, 0));

    // check projected direction
    IndexSet tight_indices;
    VectorValues proj_delta;
    std::tie(tight_indices, proj_delta) =
        manifold.identifyBlockingConstraints(delta);
    EXPECT(tight_indices.size() == 1);
    EXPECT(tight_indices.find(0) != tight_indices.end());
    VectorValues expected_proj_delta;
    expected_proj_delta.insert(point_key, Vector3(0, 1, 0));
    EXPECT(assert_equal(expected_proj_delta, proj_delta));

    // check retraction
    Values expected_values;
    expected_values.insert(point_key, Point3(1 / sqrt(2), 1 / sqrt(2), 0));
    auto new_manifold = manifold.retract(proj_delta, tight_indices);
    EXPECT(assert_equal(expected_values, new_manifold->values()));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
