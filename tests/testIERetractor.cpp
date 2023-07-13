
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <gtdynamics/imanifold/IECartPoleWithFriction.h>
#include <gtdynamics/imanifold/IERetractor.h>
#include <gtdynamics/imanifold/IEConstraintManifold.h>

using namespace gtsam;
using namespace gtdynamics;

TEST(IECartPoleWithFrictionCone, jacobian) {
  IECartPoleWithFriction cp;
  int k = 0;

  EqualityConstraints e_constraints;
  auto i_constraints = std::make_shared<InequalityConstraints>();
  i_constraints->add(cp.iConstraints(k));
  e_constraints.add(cp.eConstraints(k));
  auto e_cc = std::make_shared<ConnectedComponent>(e_constraints);
  
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->ecm_params = std::make_shared<ConstraintManifold::Params>();
  params->ie_retract_type = IERetractType::Barrier;

  Values values;
  double q = M_PI_2, v=0, a=0;
  values.insert(QKey(k), q);
  values.insert(VKey(k), v);
  values.insert(AKey(k), a);
  values.insert(TauKey(k), cp.computeTau(a));
  values.insert(FxKey(k), cp.computeFx(q, v, a));
  values.insert(FyKey(k), cp.computeFy(q, v, a));

  IEConstraintManifold manifold(params, e_cc, i_constraints, values);

  auto retractor = IERetractor::create(IERetractType::Barrier);

  VectorValues delta;
  double new_a = 20.0;
  delta.insert(QKey(k), Vector1(0.0));
  delta.insert(VKey(k), Vector1(0.0));
  delta.insert(AKey(k), Vector1(new_a));
  delta.insert(TauKey(k), Vector1(cp.computeTau(new_a)));
  delta.insert(FxKey(k), Vector1(cp.computeFx(q, v, new_a) - cp.computeFx(q, v, a)));
  delta.insert(FyKey(k), Vector1(cp.computeFy(q, v, new_a) - cp.computeFy(q, v, a)));

  auto new_manifold = retractor->retract(&manifold, delta);
  new_manifold.values().print();
  IndexSet expected_active_indices;
  expected_active_indices.insert(1);
  EXPECT(assert_container_equality(expected_active_indices, new_manifold.activeIndices()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
