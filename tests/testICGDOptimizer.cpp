
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtdynamics/manifold/IneqConstraintManifold.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <fstream>
#include <gtdynamics/manifold/CartPoleWithFriction.h>
#include <gtdynamics/manifold/ICGradientDescentOptimizer.h>
#include <iostream>

using namespace gtdynamics;
using namespace gtsam;

// InequalityConstraints CartPoleConstraints(int k) {
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
//   return constraints;
// }

// Values CartPoleValues(int k, double q, double v, double a) {
//   Values values;
//   values.insert(QKey(k), q);
//   values.insert(VKey(k), v);
//   values.insert(AKey(k), a);
//   return values;
// }

// void SaveOptResult(const std::string &file_path, const GDResult &result) {
//   std::ofstream file;
//   file.open(file_path);
//   for (int i = 0; i < result.values_vec.size(); i++) {
//     const Values &values = result.values_vec.at(i);
//     const VectorValues &descent_dir = result.descent_dir_vec.at(i);
//     const VectorValues &proj_dir = result.proj_dir_vec.at(i);

//     double q = values.atDouble(QKey(0));
//     double v = values.atDouble(VKey(0));
//     double a = values.atDouble(AKey(0));
//     double q_descent = descent_dir.at(QKey(0))(0);
//     double v_descent = descent_dir.at(VKey(0))(0);
//     double a_descent = descent_dir.at(AKey(0))(0);
//     double q_proj = proj_dir.at(QKey(0))(0);
//     double v_proj = proj_dir.at(VKey(0))(0);
//     double a_proj = proj_dir.at(AKey(0))(0);

//     file << q << " " << v << " " << a << " " << q_descent << " " << v_descent
//          << " " << a_descent << " " << q_proj << " " << v_proj << " " << a_proj
//          << "\n";
//   }
//   file.close();
// }

// TEST(ICGDOptimizer, CartPoleWithFrictionCone_single) {
//   auto constraints = CartPoleConstraints(0);
//   Values values = CartPoleValues(0, 0.0, 0.0, 0.0);
//   IndexSet active_indices{};
//   IneqConstraintManifold manifold(constraints, values, active_indices);
//   NonlinearFactorGraph graph;
//   auto q_model = noiseModel::Isotropic::Sigma(1, 8.0);
//   auto v_model = noiseModel::Isotropic::Sigma(1, 10.0);
//   auto a_model = noiseModel::Isotropic::Sigma(1, 10.0);
//   graph.addPrior<double>(QKey(0), 4.4, q_model);
//   graph.addPrior<double>(VKey(0), 10.0, v_model);
//   graph.addPrior<double>(AKey(0), 10.0, a_model);

//   std::vector<IneqConstraintManifold> manifolds{manifold};
//   ICGradientDescentOptimizer optimizer;
//   auto result = optimizer.optimize(graph, manifolds);

//   std::string file_path = "../../data/gd.txt";
//   SaveOptResult(file_path, result);
// }

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
