#include "gtdynamics/imanifold/IEManifoldOptimizer.h"
#include "gtdynamics/imanifold/IERetractor.h"
#include "gtdynamics/manifold/IneqConstraintManifold.h"
#include "gtdynamics/optimizer/EqualityConstraint.h"
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

#include <gtdynamics/imanifold/IECartPoleWithFriction.h>
#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtdynamics/optimizer/BarrierOptimizer.h>

#include <iomanip>
#include <string>

using namespace gtsam;
using namespace gtdynamics;

void SaveResult(const ConstrainedOptResult &result,
                const Values &initial_values, const size_t &num_steps,
                const std::string &folder) {
  std::filesystem::create_directory(folder);
  {
    std::ofstream file;
    file.open(folder + "intermediate_values.txt");
    for (int i = 0; i < result.intermediate_values.size(); i++) {
      for (int k = 0; k <= num_steps; k++) {
        Key point_key = gtsam::Symbol('p', k);
        Point3 point = result.intermediate_values.at(i).at<Point3>(point_key);
        file << point.x() << " " << point.y() << " " << point.z() << " ";
      }
      file << "\n";
    }
    file.close();
  }
  {
    std::ofstream file;
    file.open(folder + "initial_values.txt");
    for (int k = 0; k <= num_steps; k++) {
      Key point_key = gtsam::Symbol('p', k);
      Point3 point = initial_values.at<Point3>(point_key);
      file << point.x() << " " << point.y() << " " << point.z() << " ";
    }
    file << "\n";
    file.close();
  }
  {
    std::ofstream file;
    file.open(folder + "tangent_vectors.txt");
    for (int i = 0; i < result.tangent_vectors.size(); i++) {
      for (int k = 0; k <= num_steps; k++) {
        Key point_key = gtsam::Symbol('p', k);
        Vector tv = result.tangent_vectors.at(i).at(point_key);
        file << tv(0) << " " << tv(1) << " " << tv(2) << " ";
      }
      file << "\n";
    }
    file.close();
  }
}

Values ComputeInitialValues(const IECartPoleWithFriction &cp,
                            const size_t num_steps, const double dt) {
  Values values;
  for (size_t k = 0; k <= num_steps; k++) {

    double q = M_PI_2 - M_PI_2 * k / num_steps;
    double v = 0;
    double a = 0;
    double tau = cp.computeTau(a);
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

std::string IndicesStr(const std::map<Key, IndexSet>& indices_map) {
  std::string str;
  for (const auto &it : indices_map) {
    if (it.second.size() > 0) {
      str += "(" + _defaultKeyFormatter(it.first) + ":";
      str += ")";
      for (const auto &idx : it.second) {
        str += " " + std::to_string((idx));
      }
      str += "\t";
    }
  }
  return str;
}

std::string IndicesStr(const IEManifoldValues& manifolds) {
  std::string str;
  for (const auto &it : manifolds) {
    if (it.second.activeIndices().size() > 0) {
      str += "(" + _defaultKeyFormatter(it.first) + ":";
      str += ")";
      for (const auto &idx : it.second.activeIndices()) {
        str += " " + std::to_string((idx));
      }
      str += "\t";
    }
  }
  return str;
}


void PrintIterDetails(const IELMNonlinearIterDetails& iter_detail, const size_t num_steps, bool is_successful) {
  std::string color = is_successful ? "1;31" : "1;34";
  std::cout << "\033["+color+"mlambda: " << iter_detail.lambda
            << "\terror: " << iter_detail.cost << " -> " << iter_detail.new_error
            << "\tfidelity: " << iter_detail.model_fidelity
            << "\tlinear: " << iter_detail.linear_cost_change
            << "\tnonlinear: " << iter_detail.nonlinear_cost_change
            << "\033[0m\n";
  
  auto current_str = IndicesStr(iter_detail.manifolds);
  auto blocking_str = IndicesStr(iter_detail.blocking_indices_map);
  auto new_str = IndicesStr(iter_detail.new_manifolds);
  auto forced_str = IndicesStr(iter_detail.forced_indices_map);

  if (current_str.size() > 0) {
    std::cout << "current: " << current_str << "\n";
  }
  if (blocking_str.size() > 0) {
    std::cout << "blocking: " << blocking_str << "\n";
  }
  if (forced_str.size() > 0) {
    std::cout << "forced: " << forced_str << "\n";
  }
  if (new_str.size() > 0) {
    std::cout << "new: " << new_str << "\n";
  }

  
  std::cout << "current values: \n";
  PrintValues(IEOptimizer::CollectManifoldValues(iter_detail.manifolds),
  num_steps);

  if (iter_detail.delta.size() > 0) {
    std::cout << "tangent vector: \n";
    PrintDelta(iter_detail.delta, num_steps);
  }


  std::cout << "new values: \n";
  PrintValues(IEOptimizer::CollectManifoldValues(iter_detail.new_manifolds),
  num_steps);

}



int main(int argc, char **argv) {
  IECartPoleWithFriction cp;
  size_t num_steps = 4;
  double dt = 0.2;

  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    auto e_constraints_k = cp.eConstraints(k);
    auto i_constraints_k = cp.iConstraints(k);
    e_constraints.add(e_constraints_k);
    i_constraints.add(i_constraints_k);
  }
  e_constraints.emplace_shared<DoubleExpressionEquality>(Double_(QKey(0)) - Double_(M_PI_2), 1.0);
  e_constraints.emplace_shared<DoubleExpressionEquality>(Double_(VKey(0)), 1.0);
  e_constraints.emplace_shared<DoubleExpressionEquality>(Double_(QKey(num_steps)), 1.0);
  e_constraints.emplace_shared<DoubleExpressionEquality>(Double_(VKey(num_steps)), 1.0);
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

  PrintValues(initial_values, num_steps);

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor = std::make_shared<CPBarrierRetractor>(cp);
  // Run LM optimization
  {
    LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    params.setLinearSolverType("SEQUENTIAL_QR");
    // params.minModelFidelity = 0.5;
    IELMOptimizer lm_optimizer(params);
    ConstrainedOptResult lm_inter_result;
    std::cout << "initial error: " << graph.error(initial_values) << "\n";
    auto lm_result =
        lm_optimizer.optimize(graph, e_constraints, i_constraints,
                              initial_values, iecm_params, &lm_inter_result);
    PrintValues(lm_result, num_steps);
    // lm_result.print();
    // std::string folder_path = "../../results/half_sphere_traj_lm/";
    // SaveResult(lm_inter_result, initial_values, num_steps, folder_path);
    // initial_values = lm_result;
    std::cout << "error: " << graph.error(lm_result) << "\n";

    const auto &details = lm_optimizer.details();

    for (const auto& iter_details : details) {
      PrintIterDetails(iter_details.initial, num_steps, true);
      for (const auto& trial : iter_details.trials) {
        PrintIterDetails(trial, num_steps, false);
      }



    // for (int iter_id = 0; iter_id < 6; iter_id ++ ) {
    //   const auto& iter_detail = details.at(iter_id);
    // }


      // std::cout << "current values: \n";
      // PrintValues(IEOptimizer::CollectManifoldValues(iter_detail.manifolds),
      // num_steps);

      // std::cout << "tangent vector: \n";
      // PrintDelta(iter_detail.delta, num_steps);

      // std::cout << "new values: \n";
      // PrintValues(IEOptimizer::CollectManifoldValues(iter_detail.new_manifolds),
      // num_steps);

      // Values new_values =
      // IEOptimizer::CollectManifoldValues(iter_detail.new_manifolds); for
      // (const auto& factor: graph) {
      //   double error = factor->error(new_values);
      //   if (error > 1e-1) {
      //     factor->print();
      //     std::cout << "error: " << error << "\n";
      //   }
      // }
    }
  }

  // // Run GD optimization
  // {
  //   IEGDOptimizer gd_optimizer;
  //   ConstrainedOptResult gd_inter_result;
  //   auto gd_result = gd_optimizer.optimize(graph, e_constraints,
  //   i_constraints, initial_values, iecm_params, &gd_inter_result); std::cout
  //   << "error: " << graph.error(gd_result) << "\n";
  //   // std::string folder_path = "../../results/half_sphere_traj_gd/";
  //   // SaveResult(gd_inter_result, initial_values, num_steps, folder_path);
  // }

  // Run Barrier optimization
  {
    BarrierParameters barrier_params;
    barrier_params.initial_mu = 1e3;
    // barrier_params.num_iterations = 15;
    // barrier_params.mu_increase_rate = 2.0;
    // barrier_params.lm_parameters.setVerbosityLM("SUMMARY");
    BarrierOptimizer barrier_optimizer(barrier_params);
    auto barrier_result = barrier_optimizer.optimize(
        graph, e_constraints, i_constraints, initial_values);
    PrintValues(barrier_result.intermediate_values.back(), num_steps);
    std::cout << "error: " << std::setprecision(4)
              << graph.error(barrier_result.intermediate_values.back()) << "\n";
  }

  return 0;
}