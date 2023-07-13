

#include "gtdynamics/imanifold/IERetractor.h"
#include "gtdynamics/optimizer/ConstrainedOptimizer.h"
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point3.h>

#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>

using namespace gtdynamics;
using namespace gtsam;


TEST(IEGDOptimizer, HalfSphere) {
  Key point_key = gtsam::Symbol('p', 0);

  gtsam::Expression<Point3> point_expr(point_key);
  gtsam::Expression<double> sphere_expr(&norm3, point_expr);
  gtsam::Expression<double> z_expr(&point3_z, point_expr);

  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  e_constraints.emplace_shared<DoubleExpressionEquality>(sphere_expr, 1.0);
  i_constraints.emplace_shared<DoubleExpressionInequality>(z_expr, 1.0);
  
  Values initial_values;
  initial_values.insert(point_key, Point3(1/sqrt(3),1/sqrt(3),1/sqrt(3)));

  NonlinearFactorGraph graph;
  graph.addPrior<Point3>(point_key, Point3(-1, 0, -1), noiseModel::Unit::Create(3));

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->ie_retract_type = IERetractType::HalfSphere;
  IEGDOptimizer gd_optimizer;
  auto gd_result = gd_optimizer.optimize(graph, e_constraints, i_constraints, initial_values, iecm_params);
  EXPECT(assert_equal(Point3(-1, 0, 0), gd_result.at<Point3>(point_key)));

  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  IELMOptimizer lm_optimizer(params);
  ConstrainedOptResult lm_inter_result;
  auto lm_result = lm_optimizer.optimize(graph, e_constraints, i_constraints, initial_values, iecm_params, &lm_inter_result);
  EXPECT(assert_equal(Point3(-1, 0, 0), lm_result.at<Point3>(point_key)));
  for (int i=0; i<lm_inter_result.intermediate_values.size(); i++) {
    std::cout << "iter: " << i << "\n";
    lm_inter_result.intermediate_values.at(i).print();
  }
}

// TEST(IEGDOptimizer, HalfSphereTrajectory) {
//   size_t num_steps = 10;

//   Point3 attractor(0.0, 0.0, -2.0);

//   EqualityConstraints e_constraints;
//   InequalityConstraints i_constraints;
//   Values initial_values;
//   NonlinearFactorGraph graph;
//   Key first_point_key = gtsam::Symbol('p', 0);
//   Key last_point_key = gtsam::Symbol('p', num_steps);
//   graph.addPrior<Point3>(first_point_key, Point3(0.8, 0, 0.6), noiseModel::Isotropic::Sigma(3, 1e-3));
//   graph.addPrior<Point3>(last_point_key, Point3(0, 0.8, 0.6), noiseModel::Isotropic::Sigma(3, 1e-3));
//   for (size_t k=0; k<=num_steps; k++) {
//     Key point_key = gtsam::Symbol('p', k);
//     gtsam::Expression<Point3> point_expr(point_key);
//     gtsam::Expression<double> sphere_expr(&norm3, point_expr);
//     gtsam::Expression<double> z_expr(&point3_z, point_expr);
//     e_constraints.emplace_shared<DoubleExpressionEquality>(sphere_expr, 1.0);
//     i_constraints.emplace_shared<DoubleExpressionInequality>(z_expr, 1.0);

//     double theta = M_PI_2 * k / num_steps;
//     initial_values.insert(point_key, Point3(cos(theta),sin(theta),0.6));

//     graph.addPrior<Point3>(point_key, attractor, noiseModel::Isotropic::Sigma(3, 1e-2));
//     if (k>0) {
//       Key prev_point_key = gtsam::Symbol('p', k-1);
//       graph.emplace_shared<BetweenFactor<Point3>>(prev_point_key, point_key, Point3(0,0,0), noiseModel::Isotropic::Sigma(3, 1.0));
//     }
//   }

//   auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
//   IEGDOptimizer gd_optimizer;
//   auto gd_result = gd_optimizer.optimize(graph, e_constraints, i_constraints, initial_values, iecm_params);
//   gd_result.print();

//   IELMOptimizer lm_optimizer;
//   auto lm_result = lm_optimizer.optimize(graph, e_constraints, i_constraints, initial_values, iecm_params);
//   lm_result.print();

// } 

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
