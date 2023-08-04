
#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/imanifold/IEHalfSphere.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtdynamics/manifold/IneqConstraintManifold.h>
#include <gtdynamics/manifold/ICGradientDescentOptimizer.h>
#include <gtdynamics/manifold/ICLMOptimizer.h>
#include <gtdynamics/optimizer/BarrierOptimizer.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;
using namespace gtdynamics;

int main(int argc, char **argv) {
  int num_steps = 5;
  double radius = 1;
  IEHalfSphere half_sphere(radius);

  // graph
  NonlinearFactorGraph graph;
  auto prior_noise = noiseModel::Isotropic::Sigma(3, 0.1);
  auto between_noise = noiseModel::Isotropic::Sigma(3, 0.1);
  graph.addPrior<Point3>(PointKey(0), Point3(0, 0, 0), prior_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(0), PointKey(1), Point3(0.6, 0.3, 0), between_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(1), PointKey(2), Point3(0.6, -0.3, 0), between_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(2), PointKey(3), Point3(-1, 0.0, 0.5), between_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(3), PointKey(4), Point3(0.0, 0.0, 0.6), between_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(4), PointKey(5), Point3(0.0, 0.6, -0.2), between_noise);

  // constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    i_constraints.add(half_sphere.iDomeConstraints(k));
  }

  // values
  Values initial_values;
  initial_values.insert(PointKey(0), Point3(0, 0, 0));
  initial_values.insert(PointKey(1), Point3(0, 0, 0));
  initial_values.insert(PointKey(2), Point3(0, 0, 0));
  initial_values.insert(PointKey(3), Point3(0, 0, 0));
  initial_values.insert(PointKey(4), Point3(0, 0, 0));
  initial_values.insert(PointKey(5), Point3(0, 0, 0));


  IEConsOptProblem problem(graph, e_constraints, i_constraints, initial_values);

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor = std::make_shared<DomeRetractor>(half_sphere);

  LevenbergMarquardtParams lm_params;
  auto soft_result = OptimizeSoftConstraints(problem, lm_params, 100);

  BarrierParameters barrier_params;
  barrier_params.num_iterations = 15;
  auto barrier_result = OptimizeBarrierMethod(problem, barrier_params);

  GDParams gd_params;
  auto gd_result = OptimizeIEGD(problem, gd_params, iecm_params);

  IELMParams ie_params;
  auto lm_result = OptimizeIELM(problem, lm_params, ie_params, iecm_params);

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
  //   auto lm_result = lm_optimizer.optimize(graph, e_constraints, i_constraints,
  //                                          initial_values, iecm_params);

  //   const auto &details = lm_optimizer.details();
  //   for (const auto &iter_details : details) {
  //     IEOptimizer::PrintIterDetails(iter_details, num_steps, false,
  //                                   IEHalfSphere::PrintValues,
  //                                   IEHalfSphere::PrintDelta);
  //   }
  // }

  // // Run GD optimization
  // {
  //   GDParams params;
  //   params.maxIterations = 100;
  //   IEGDOptimizer gd_optimizer(params);
  //   auto gd_result = gd_optimizer.optimize(graph, e_constraints, i_constraints,
  //                                          initial_values, iecm_params);

  //   const auto &details = gd_optimizer.details();
  //   for (const auto &iter_details : details) {
  //     IEOptimizer::PrintIterDetails(iter_details, num_steps, false,
  //                                   IEHalfSphere::PrintValues,
  //                                   IEHalfSphere::PrintDelta);
  //   }
  // }


  return 0;
}
