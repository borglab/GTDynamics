#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

namespace so2_scenario {

double dist_square_func(const double& x1, const double& x2,
                        gtsam::OptionalJacobian<1, 1> H1 = boost::none,
                        gtsam::OptionalJacobian<1, 1> H2 = boost::none) {
  if (H1) *H1 << 2 * x1;
  if (H2) *H2 << 2 * x2;
  return x1 * x1 + x2 * x2;
}

Key x1_key = 1;
Key x2_key = 2;
Double_ x1(x1_key);
Double_ x2(x2_key);
Double_ dist_square(dist_square_func, x1, x2);
Double_ dist_error = dist_square - Double_(1.0);

boost::shared_ptr<gtdynamics::EqualityConstraints> get_constraints() {
  auto constraints = boost::make_shared<gtdynamics::EqualityConstraints>();
  double tolerance = 1e-3;
  constraints->emplace_shared<gtdynamics::DoubleExpressionEquality>(dist_error,
                                                                    tolerance);
  return constraints;
}

boost::shared_ptr<NonlinearFactorGraph> get_graph(const double& x1_val,
                                                  const double& x2_val) {
  auto graph = boost::make_shared<NonlinearFactorGraph>();
  auto model = noiseModel::Isotropic::Sigma(1, 1.0);
  graph->addPrior(x1_key, x1_val, model);
  graph->addPrior(x2_key, x2_val, model);
  return graph;
}

}  // namespace so2_scenario

}  // namespace gtsam
