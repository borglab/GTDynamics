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

#include <gtdynamics/scenarios/IECartPoleWithFriction.h>
#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtdynamics/optimizer/BarrierOptimizer.h>

#include <iomanip>
#include <string>

using namespace gtsam;
using namespace gtdynamics;

Values ComputeInitialValues(const IECartPoleWithFriction &cp,
                            const size_t num_steps, const double dt) {
  Values values;
  for (size_t k = 0; k <= num_steps; k++) {

    double q = M_PI_2 - M_PI_2 * k / num_steps;
    double v = 0;
    double a = 0;
    double tau = cp.computeTau(q, a);
    double fx = cp.computeFx(q, v, a);
    double fy = cp.computeFy(q, v, a);
    values.insert(QKey(k), q);
    values.insert(VKey(k), v);
    values.insert(AKey(k), a);
    values.insert(TauKey(k), tau);
    values.insert(FxKey(k), fx);
    values.insert(FyKey(k), fy);
  }
  return values;
}

int main(int argc, char **argv) {
  IECartPoleWithFriction cp;
  cp.include_torque_limits = true;
  size_t num_steps = 20;
  double dt = 0.05;

  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    auto e_constraints_k = cp.eConstraints(k);
    auto i_constraints_k = cp.iConstraints(k);
    e_constraints.add(e_constraints_k);
    i_constraints.add(i_constraints_k);
  }
  e_constraints.emplace_shared<DoubleExpressionEquality>(
      Double_(QKey(0)) - Double_(M_PI_2), 1.0);
  e_constraints.emplace_shared<DoubleExpressionEquality>(Double_(VKey(0)), 1.0);
  e_constraints.emplace_shared<DoubleExpressionEquality>(
      Double_(QKey(num_steps)), 1.0);
  e_constraints.emplace_shared<DoubleExpressionEquality>(
      Double_(VKey(num_steps)), 1.0);
  NonlinearFactorGraph graph;
  auto collo_model = noiseModel::Isotropic::Sigma(1, 1e-1);
  auto prior_model = noiseModel::Isotropic::Sigma(1, 1e-2);
  auto cost_model = noiseModel::Isotropic::Sigma(1, 1e0);
  // graph.addPrior<double>(QKey(0), M_PI_2, prior_model);
  // graph.addPrior<double>(VKey(0), 0.0, prior_model);
  // graph.addPrior<double>(QKey(num_steps), 0.0, prior_model);
  // graph.addPrior<double>(VKey(num_steps), 0.0, prior_model);
  for (size_t k = 0; k < num_steps; k++) {
    Double_ q0_expr(QKey(k));
    Double_ q1_expr(QKey(k + 1));
    Double_ v0_expr(VKey(k));
    Double_ v1_expr(VKey(k + 1));
    Double_ a0_expr(AKey(k));
    Double_ a1_expr(AKey(k + 1));
    graph.add(
        ExpressionFactor(collo_model, 0.0, q0_expr + dt * v0_expr - q1_expr));
    graph.add(
        ExpressionFactor(collo_model, 0.0, v0_expr + dt * a0_expr - v1_expr));
  }
  for (size_t k = 0; k <= num_steps; k++) {
    graph.addPrior<double>(QKey(k), 0.0, cost_model);
  }

  Values initial_values = ComputeInitialValues(cp, num_steps, dt);
  // double nominal_q = 0;
  // double nominal_v = 0;
  // for (size_t k=1; k<=num_steps; k++) {
  //   double nominal_a = k>num_steps/2 ? -1 : 1;
  //   nominal_q += nominal_v * dt;
  //   nominal_v += nominal_a * dt;
  // }
  // double coeff = -M_PI_2 / nominal_q;
  // double a = coeff;
  // double v = 0;
  // double q = M_PI_2;
  // double prev_a, prev_v, prev_q;
  // for (size_t k=0; k<=num_steps; k++) {
  //   if (k>0) {
  //     a = k>=num_steps/2 ? -coeff : coeff;
  //     v = prev_v + prev_a * dt;
  //     q = prev_q + prev_v * dt;
  //   }
  //   prev_a = a;
  //   prev_v = v;
  //   prev_q = q;
  //   // a= 0; v=0; q=0;
  //   double tau = cp.computeTau(a);
  //   double fx = cp.computeFx(q, v, a);
  //   double fy = cp.computeFy(q, v, a);
  //   initial_values.insert(QKey(k), q);
  //   initial_values.insert(VKey(k), v);
  //   initial_values.insert(AKey(k), a);
  //   initial_values.insert(TauKey(k), tau);
  //   initial_values.insert(FxKey(k), fx);
  //   initial_values.insert(FyKey(k), fy);
  // }

  // for (const auto& factor : graph) {
  //   auto factor_error = factor->error(initial_values);
  //   if (factor_error > 1e-5) {
  //     factor->print();
  //     std::cout << "factor error: " << factor->error(initial_values) << "\n";
  //   }
  // }

  IECartPoleWithFriction::PrintValues(initial_values, num_steps);

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor_creator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<CPBarrierRetractor>(cp));
  iecm_params->e_basis_creator = std::make_shared<OrthonormalBasisCreator>();

  IEConsOptProblem problem(graph, e_constraints, i_constraints, initial_values);

  LevenbergMarquardtParams lm_params;
  auto soft_result = OptimizeSoftConstraints(problem, lm_params, 100);

  auto barrier_params = std::make_shared<BarrierParameters>();
  barrier_params->num_iterations = 15;
  auto barrier_result = OptimizeBarrierMethod(problem, barrier_params);

  GDParams gd_params;
  auto gd_result = OptimizeIEGD(problem, gd_params, iecm_params);

  IELMParams ie_params;
  ie_params.lm_params = lm_params;
  auto lm_result = OptimizeIELM(problem, ie_params, iecm_params);

  soft_result.first.printLatex(std::cout);
  barrier_result.first.printLatex(std::cout);
  gd_result.first.printLatex(std::cout);
  lm_result.first.printLatex(std::cout);

  soft_result.first.exportFile("../../results/pole_soft/summary.txt");
  barrier_result.first.exportFile("../../results/pole_barrier/summary.txt");
  gd_result.first.exportFile("../../results/pole_gd/summary.txt");
  lm_result.first.exportFile("../../results/pole_lm/summary.txt");

  IECartPoleWithFriction::ExportValues(
      lm_result.second.back().state.baseValues(), num_steps,
      "../../results/pole_lm/values_final.txt");

  IECartPoleWithFriction::PrintValues(
      lm_result.second.back().state.baseValues(), num_steps);

  // // Run LM optimization
  // {
  //   LevenbergMarquardtParams params;
  //   params.setVerbosityLM("SUMMARY");
  //   params.setLinearSolverType("SEQUENTIAL_QR");
  //   // params.minModelFidelity = 0.5;
  //   IELMOptimizer lm_optimizer(params);
  //   ConstrainedOptResult lm_inter_result;
  //   std::cout << "initial error: " << graph.error(initial_values) << "\n";
  //   auto lm_result =
  //       lm_optimizer.optimize(graph, e_constraints, i_constraints,
  //                             initial_values, iecm_params, &lm_inter_result);
  //   // lm_result.print();
  //   // std::string folder_path = "../../results/half_sphere_traj_lm/";
  //   // SaveResult(lm_inter_result, initial_values, num_steps, folder_path);
  //   // initial_values = lm_result;
  //   std::cout << "error: " << graph.error(lm_result) << "\n";

  //   const auto &details = lm_optimizer.details();

  for (const auto &iter_details : lm_result.second) {
    IEOptimizer::PrintIterDetails(iter_details, num_steps, false,
                                  IECartPoleWithFriction::PrintValues,
                                  IECartPoleWithFriction::PrintDelta);
  }
  //   IECartPoleWithFriction::PrintValues(lm_result, num_steps);
  // }

  // // Run GD optimization
  // {
  //   GDParams params;
  //   params.maxIterations = 5;
  //   params.init_lambda = 100;
  //   IEGDOptimizer gd_optimizer(params);
  //   auto gd_result = gd_optimizer.optimize(graph, e_constraints,
  //   i_constraints,
  //                                          initial_values, iecm_params);
  //   std::cout << "error: " << graph.error(gd_result) << "\n";

  //   const auto &details = gd_optimizer.details();

  //   for (const auto &iter_details : details) {
  //     PrintIterDetails(iter_details, num_steps, true);
  //   }
  //   IECartPoleWithFriction::PrintValues(gd_result, num_steps);
  // }

  // // Run Barrier optimization
  // {
  //   BarrierParameters barrier_params;
  //   barrier_params.initial_mu = 1e1;
  //   barrier_params.num_iterations = 15;
  //   // barrier_params.mu_increase_rate = 2.0;
  //   // barrier_params.lm_parameters.setVerbosityLM("SUMMARY");
  //   BarrierOptimizer barrier_optimizer(barrier_params);
  //   auto barrier_result = barrier_optimizer.optimize(
  //       graph, e_constraints, i_constraints, initial_values);
  //   IECartPoleWithFriction::PrintValues(barrier_result.intermediate_values.back(),
  //   num_steps); std::cout << "error: " << std::setprecision(4)
  //             << graph.error(barrier_result.intermediate_values.back()) <<
  //             "\n";
  // }

  return 0;
}