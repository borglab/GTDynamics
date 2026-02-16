
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/kinematics/PoseFactor.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <gtdynamics/scenarios/IECartPoleWithFriction.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/IERetractor.h>

using namespace gtsam;
using namespace gtdynamics;

TEST(IECartPoleWithFrictionCone, BarrierRetractor) {
  IECartPoleWithFriction cp;
  int k = 0;

  auto e_constraints = std::make_shared<NonlinearEqualityConstraints>();
  auto i_constraints = std::make_shared<NonlinearInequalityConstraints>();
  i_constraints->add(cp.iConstraints(k));
  e_constraints->add(cp.eConstraints(k));

  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->ecm_params = std::make_shared<ConstraintManifold::Params>();
  params->retractor_creator = std::make_shared<BarrierRetractorCreator>();
  params->e_basis_creator = std::make_shared<OrthonormalBasisCreator>();

  Values values;
  double q = M_PI_2, v = 0, a = 0;
  values.insert(QKey(k), q);
  values.insert(VKey(k), v);
  values.insert(AKey(k), a);
  values.insert(TauKey(k), cp.computeTau(q, a));
  values.insert(FxKey(k), cp.computeFx(q, v, a));
  values.insert(FyKey(k), cp.computeFy(q, v, a));

  IEConstraintManifold manifold(params, e_constraints, i_constraints, values);

  VectorValues delta;
  double new_a = 20.0;
  delta.insert(QKey(k), Vector1(0.0));
  delta.insert(VKey(k), Vector1(0.0));
  delta.insert(AKey(k), Vector1(new_a));
  delta.insert(TauKey(k), Vector1(cp.computeTau(q, new_a)));
  delta.insert(FxKey(k),
               Vector1(cp.computeFx(q, v, new_a) - cp.computeFx(q, v, a)));
  delta.insert(FyKey(k),
               Vector1(cp.computeFy(q, v, new_a) - cp.computeFy(q, v, a)));

  auto new_manifold = manifold.retractor()->retract(&manifold, delta);
  // new_manifold.values().print();
  IndexSet expected_active_indices;
  expected_active_indices.insert(1);
  EXPECT(assert_container_equality(expected_active_indices,
                                   new_manifold.activeIndices()));
}

// TEST(IECartPoleWithFrictionCone, CPRetractor) {
//   IECartPoleWithFriction cp;
//   int k = 0;

//   EqualityConstraints e_constraints;
//   auto i_constraints = std::make_shared<InequalityConstraints>();
//   i_constraints->add(cp.iConstraints(k));
//   e_constraints.add(cp.eConstraints(k));
//   auto e_cc = std::make_shared<ConnectedComponent>(e_constraints);

//   auto params = std::make_shared<IEConstraintManifold::Params>();
//   params->ecm_params = std::make_shared<ConstraintManifold::Params>();
//   params->retractor_creator = std::make_shared<UniversalIERetractorCreator>(std::make_shared<BarrierRetractor>());
//   params->e_basis_creator =
//       std::make_shared<TspaceBasisCreator>(params->ecm_params->basis_params);

//   Values values;
//   double q = M_PI_2, v = 0, a = 0;
//   values.insert(QKey(k), q);
//   values.insert(VKey(k), v);
//   values.insert(AKey(k), a);
//   values.insert(TauKey(k), cp.computeTau(q, a));
//   values.insert(FxKey(k), cp.computeFx(q, v, a));
//   values.insert(FyKey(k), cp.computeFy(q, v, a));

//   IEConstraintManifold manifold(params, e_cc, i_constraints, values);

//   {
//     VectorValues delta;
//     double new_a = 20.0;
//     delta.insert(QKey(k), Vector1(0.0));
//     delta.insert(VKey(k), Vector1(0.0));
//     delta.insert(AKey(k), Vector1(new_a));
//     delta.insert(TauKey(k), Vector1(cp.computeTau(q, new_a)));
//     delta.insert(FxKey(k),
//                  Vector1(cp.computeFx(q, v, new_a) - cp.computeFx(q, v, a)));
//     delta.insert(FyKey(k),
//                  Vector1(cp.computeFy(q, v, new_a) - cp.computeFy(q, v, a)));

//     auto new_manifold = manifold.retractor()->retract(&manifold, delta);
//     Values expected_values;
//     new_a = cp.mu * (cp.M + cp.m) * cp.g;
//     expected_values.insert(QKey(k), q);
//     expected_values.insert(VKey(k), v);
//     expected_values.insert(AKey(k), new_a);
//     expected_values.insert(TauKey(k), cp.computeTau(q, new_a));
//     expected_values.insert(FxKey(k), cp.computeFx(q, v, new_a));
//     expected_values.insert(FyKey(k), cp.computeFy(q, v, new_a));
//     EXPECT(assert_equal(expected_values, new_manifold.values()));

//     IndexSet expected_active_indices;
//     expected_active_indices.insert(1);
//     EXPECT(assert_container_equality(expected_active_indices,
//                                      new_manifold.activeIndices()));
//   }

//   {
//     VectorValues delta;
//     double new_a = -30.0;
//     delta.insert(QKey(k), Vector1(0.0));
//     delta.insert(VKey(k), Vector1(0.0));
//     delta.insert(AKey(k), Vector1(new_a));
//     delta.insert(TauKey(k), Vector1(cp.computeTau(q, new_a)));
//     delta.insert(FxKey(k),
//                  Vector1(cp.computeFx(q, v, new_a) - cp.computeFx(q, v, a)));
//     delta.insert(FyKey(k),
//                  Vector1(cp.computeFy(q, v, new_a) - cp.computeFy(q, v, a)));

//     auto new_manifold = manifold.retractor()->retract(&manifold, delta);
//     Values expected_values;
//     new_a = -cp.mu * (cp.M + cp.m) * cp.g;
//     expected_values.insert(QKey(k), q);
//     expected_values.insert(VKey(k), v);
//     expected_values.insert(AKey(k), new_a);
//     expected_values.insert(TauKey(k), cp.computeTau(q, new_a));
//     expected_values.insert(FxKey(k), cp.computeFx(q, v, new_a));
//     expected_values.insert(FyKey(k), cp.computeFy(q, v, new_a));
//     EXPECT(assert_equal(expected_values, new_manifold.values()));

//     IndexSet expected_active_indices;
//     expected_active_indices.insert(0);
//     EXPECT(assert_container_equality(expected_active_indices,
//                                      new_manifold.activeIndices()));
//   }
// }



// TEST(IECartPoleWithFrictionCone, CPBarrierRetractor) {
//   IECartPoleWithFriction cp;
//   int k = 0;

//   EqualityConstraints e_constraints;
//   auto i_constraints = std::make_shared<InequalityConstraints>();
//   i_constraints->add(cp.iConstraints(k));
//   e_constraints.add(cp.eConstraints(k));
//   auto e_cc = std::make_shared<ConnectedComponent>(e_constraints);

//   auto params = std::make_shared<IEConstraintManifold::Params>();
//   params->ecm_params = std::make_shared<ConstraintManifold::Params>();
//   params->retractor_creator = std::make_shared<UniversalIERetractorCreator>(std::make_shared<BarrierRetractor>());
//   params->e_basis_creator =
//       std::make_shared<TspaceBasisCreator>(params->ecm_params->basis_params);

//   Values values;
//   double q = M_PI_2, v = 0, a = 0;
//   values.insert(QKey(k), q);
//   values.insert(VKey(k), v);
//   values.insert(AKey(k), a);
//   values.insert(TauKey(k), cp.computeTau(q, a));
//   values.insert(FxKey(k), cp.computeFx(q, v, a));
//   values.insert(FyKey(k), cp.computeFy(q, v, a));

//   IEConstraintManifold manifold(params, e_cc, i_constraints, values);

//   {
//     VectorValues delta;
//     double new_a = 20.0;
//     delta.insert(QKey(k), Vector1(0.0));
//     delta.insert(VKey(k), Vector1(0.0));
//     delta.insert(AKey(k), Vector1(new_a));
//     delta.insert(TauKey(k), Vector1(cp.computeTau(q, new_a)));
//     delta.insert(FxKey(k),
//                  Vector1(cp.computeFx(q, v, new_a) - cp.computeFx(q, v, a)));
//     delta.insert(FyKey(k),
//                  Vector1(cp.computeFy(q, v, new_a) - cp.computeFy(q, v, a)));

//     auto new_manifold = manifold.retractor()->retract(&manifold, delta);
//     Values expected_values;
//     new_a = cp.mu * (cp.M + cp.m) * cp.g;
//     expected_values.insert(QKey(k), q);
//     expected_values.insert(VKey(k), v);
//     expected_values.insert(AKey(k), new_a);
//     expected_values.insert(TauKey(k), cp.computeTau(q, new_a));
//     expected_values.insert(FxKey(k), cp.computeFx(q, v, new_a));
//     expected_values.insert(FyKey(k), cp.computeFy(q, v, new_a));
//     EXPECT(assert_equal(expected_values, new_manifold.values()));

//     IndexSet expected_active_indices;
//     expected_active_indices.insert(1);
//     EXPECT(assert_container_equality(expected_active_indices,
//                                      new_manifold.activeIndices()));
//   }

//   {
//     VectorValues delta;
//     double new_a = -30.0;
//     delta.insert(QKey(k), Vector1(0.0));
//     delta.insert(VKey(k), Vector1(0.0));
//     delta.insert(AKey(k), Vector1(new_a));
//     delta.insert(TauKey(k), Vector1(cp.computeTau(q, new_a)));
//     delta.insert(FxKey(k),
//                  Vector1(cp.computeFx(q, v, new_a) - cp.computeFx(q, v, a)));
//     delta.insert(FyKey(k),
//                  Vector1(cp.computeFy(q, v, new_a) - cp.computeFy(q, v, a)));

//     auto new_manifold = manifold.retractor()->retract(&manifold, delta);
//     Values expected_values;
//     new_a = -cp.mu * (cp.M + cp.m) * cp.g;
//     expected_values.insert(QKey(k), q);
//     expected_values.insert(VKey(k), v);
//     expected_values.insert(AKey(k), new_a);
//     expected_values.insert(TauKey(k), cp.computeTau(q, new_a));
//     expected_values.insert(FxKey(k), cp.computeFx(q, v, new_a));
//     expected_values.insert(FyKey(k), cp.computeFy(q, v, new_a));
//     EXPECT(assert_equal(expected_values, new_manifold.values()));

//     IndexSet expected_active_indices;
//     expected_active_indices.insert(0);
//     EXPECT(assert_container_equality(expected_active_indices,
//                                      new_manifold.activeIndices()));
//   }
// }



int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
