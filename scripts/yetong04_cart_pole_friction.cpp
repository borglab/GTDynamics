#include "gtdynamics/imanifold/IERetractor.h"
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
#include <gtdynamics/imanifold/IECartPoleWithFriction.h>
#include <gtdynamics/optimizer/BarrierOptimizer.h>

using namespace gtsam;
using namespace gtdynamics;


void SaveResult(const ConstrainedOptResult& result, const Values& initial_values, const size_t& num_steps, const std::string &folder) {
  std::filesystem::create_directory(folder);
  {
    std::ofstream file;
    file.open(folder + "intermediate_values.txt");
    for (int i=0; i<result.intermediate_values.size(); i++) {
      for (int k=0; k<=num_steps; k++) {
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
    for (int k=0; k<=num_steps; k++) {
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
    for (int i=0; i<result.tangent_vectors.size(); i++) {
      for (int k=0; k<=num_steps; k++) {
        Key point_key = gtsam::Symbol('p', k);
        Vector tv = result.tangent_vectors.at(i).at(point_key);
        file << tv(0) << " " << tv(1) << " " << tv(2) << " ";
      }
      file << "\n";
    }
    file.close();
  }
}


int main(int argc, char **argv) {
  IECartPoleWithFriction cp;
  size_t num_steps = 10;
  double dt = 0.1;

  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k=0; k<=num_steps; k++) {
    auto e_constraints_k = cp.eConstraints(k);
    auto i_constraints_k = cp.iConstraints(k);
    e_constraints.add(e_constraints_k);
    i_constraints.add(i_constraints_k);
  }

  NonlinearFactorGraph graph;
  auto collo_model = noiseModel::Isotropic::Sigma(1, 1e-2);
  auto prior_model = noiseModel::Isotropic::Sigma(1, 1e-2);
  auto cost_model = noiseModel::Isotropic::Sigma(1, 1e0);
  graph.addPrior<double>(QKey(0), M_PI_2, prior_model);
  graph.addPrior<double>(VKey(0), 0.0, prior_model);
  graph.addPrior<double>(QKey(num_steps), 0.0, prior_model);
  graph.addPrior<double>(VKey(num_steps), 0.0, prior_model);
  for (size_t k=0; k<num_steps; k++) {
    Double_ q0_expr(QKey(k));
    Double_ q1_expr(QKey(k+1));
    Double_ v0_expr(VKey(k));
    Double_ v1_expr(VKey(k+1));
    Double_ a0_expr(AKey(k));
    Double_ a1_expr(AKey(k+1));
    graph.add(ExpressionFactor(collo_model, 0.0, q0_expr + dt * v0_expr - q1_expr));
    graph.add(ExpressionFactor(collo_model, 0.0, v0_expr + dt * a0_expr - v1_expr));
  }
  for (size_t k=0; k<=num_steps; k++) {
    graph.addPrior<double>(QKey(k), 0.0, cost_model);
  }


  Values initial_values;
  double nominal_q = 0;
  double nominal_v = 0;
  for (size_t k=1; k<=num_steps; k++) {
    double nominal_a = k>num_steps/2 ? -1 : 1;
    std::cout << nominal_a << "\t" << nominal_v << "\t" << nominal_q << "\n";
    nominal_q += nominal_v * dt;
    nominal_v += nominal_a * dt;
  }
  std::cout << "nominal_q: " << nominal_q << "\n";
  double coeff = -M_PI_2 / nominal_q;
  double a = coeff;
  double v = 0;
  double q = M_PI_2;
  double prev_a, prev_v, prev_q;
  for (size_t k=0; k<=num_steps; k++) {
    // if (k>0) {
    //   a = k>=num_steps/2 ? -coeff : coeff;
    //   v = prev_v + prev_a * dt;
    //   q = prev_q + prev_v * dt;
    // }
    // prev_a = a;
    // prev_v = v;
    // prev_q = q;
    a= 0; v=0; q=0;
    double tau = cp.computeTau(a);
    double fx = cp.computeFx(q, v, a);
    double fy = cp.computeFy(q, v, a);
    initial_values.insert(QKey(k), q);
    initial_values.insert(VKey(k), v);
    initial_values.insert(AKey(k), a);
    initial_values.insert(TauKey(k), tau);
    initial_values.insert(FxKey(k), fx);
    initial_values.insert(FyKey(k), fy);
  }

  initial_values.print();

  for (const auto& factor : graph) {
    auto factor_error = factor->error(initial_values);
    if (factor_error > 1e-5) {
      factor->print();
      std::cout << "factor error: " << factor->error(initial_values) << "\n";
    }

  }


  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->ie_retract_type = IERetractType::Barrier;
  // Run LM optimization
  {
    LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    params.minModelFidelity = 0.5;
    IELMOptimizer lm_optimizer(params);
    ConstrainedOptResult lm_inter_result;
    std::cout << "initial error: " << graph.error(initial_values) << "\n";
    auto lm_result = lm_optimizer.optimize(graph, e_constraints, i_constraints, initial_values, iecm_params, &lm_inter_result);
    lm_result.print();
    // std::string folder_path = "../../results/half_sphere_traj_lm/";
    // SaveResult(lm_inter_result, initial_values, num_steps, folder_path);
    // initial_values = lm_result;
  }


  // // Run GD optimization
  // {
  //   IEGDOptimizer gd_optimizer;
  //   ConstrainedOptResult gd_inter_result;
  //   auto gd_result = gd_optimizer.optimize(graph, e_constraints, i_constraints, initial_values, iecm_params, &gd_inter_result);
  //   std::cout << "error: " << graph.error(gd_result) << "\n";
  //   // std::string folder_path = "../../results/half_sphere_traj_gd/";
  //   // SaveResult(gd_inter_result, initial_values, num_steps, folder_path);
  // }

  // Run Barrier optimization
  {
    BarrierParameters barrier_params;
    // barrier_params.initial_mu = 1e0;
    // barrier_params.num_iterations = 15;
    // barrier_params.mu_increase_rate = 2.0;
    // barrier_params.lm_parameters.setVerbosityLM("SUMMARY");
    BarrierOptimizer barrier_optimizer(barrier_params);
    auto barrier_result = barrier_optimizer.optimize(graph, e_constraints, i_constraints, initial_values);
    barrier_result.intermediate_values.back().print();
    std::cout << "error: " << graph.error(barrier_result.intermediate_values.back()) << "\n";
  }

  return 0;
} 