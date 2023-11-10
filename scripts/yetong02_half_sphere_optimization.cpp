#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>

using namespace gtsam;
using namespace gtdynamics;

size_t k = 0;
Key point_key = PointKey(k);

int main(int argc, char **argv) {
  IEHalfSphere half_sphere;
  Point3 attractor_point(0, -2, -1);
  Point3 init_point(1 / sqrt(3), 1 / sqrt(3), 1 / sqrt(3));

  // graph
  NonlinearFactorGraph graph;
  auto prior_noise = noiseModel::Unit::Create(3);
  graph.addPrior<Point3>(point_key, attractor_point, prior_noise);

  // constraints
  EqualityConstraints e_constraints = half_sphere.eConstraints(k);
  InequalityConstraints i_constraints = half_sphere.iConstraints(k);

  // init values
  Values initial_values;
  initial_values.insert(point_key, init_point);

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor_creator =
      std::make_shared<UniversalIERetractorCreator>(
          make_shared<HalfSphereRetractor>(half_sphere));
  iecm_params->e_basis_creator = std::make_shared<TspaceBasisCreator>(
      iecm_params->ecm_params->basis_params);

  IEConsOptProblem problem(graph, e_constraints, i_constraints, initial_values);

  LevenbergMarquardtParams lm_params;
  auto soft_result = OptimizeSoftConstraints(problem, lm_params, 100);

  BarrierParameters barrier_params;
  barrier_params.num_iterations = 15;
  auto barrier_result = OptimizeBarrierMethod(problem, barrier_params);

  GDParams gd_params;
  auto gd_result = OptimizeIEGD(problem, gd_params, iecm_params);

  IELMParams ie_params;
  ie_params.lm_params.minModelFidelity = 0.5;
  // lm_params.setVerbosityLM("SUMMARY");
  auto lm_result = OptimizeIELM(problem, ie_params, iecm_params);

  soft_result.first.printLatex(std::cout);
  barrier_result.first.printLatex(std::cout);
  gd_result.first.printLatex(std::cout);
  lm_result.first.printLatex(std::cout);

  // // Run LM optimization
  // {
  //   LevenbergMarquardtParams params;
  //   params.setVerbosityLM("SUMMARY");
  //   params.minModelFidelity = 0.5;
  //   IELMOptimizer lm_optimizer(params);
  //   ConstrainedOptResult lm_inter_result;
  //   auto lm_result = lm_optimizer.optimize(graph, e_constraints,
  //   i_constraints,
  //                                          initial_values, iecm_params);

  //   const auto &details = lm_optimizer.details();
  //   for (const auto &iter_details : details) {
  //     IEOptimizer::PrintIterDetails(iter_details, 0, false,
  //                                   IEHalfSphere::PrintValues,
  //                                   IEHalfSphere::PrintDelta);
  //   }
  // }

  // // Run GD optimization
  // {
  //   GDParams params;
  //   params.maxIterations = 30;
  //   IEGDOptimizer gd_optimizer(params);
  //   auto gd_result = gd_optimizer.optimize(graph, e_constraints,
  //   i_constraints,
  //                                          initial_values, iecm_params);

  //   const auto &details = gd_optimizer.details();
  //   for (const auto &iter_details : details) {
  //     IEOptimizer::PrintIterDetails(iter_details, 0, false,
  //                                   IEHalfSphere::PrintValues,
  //                                   IEHalfSphere::PrintDelta);
  //   }
  // }

  return 0;
}
