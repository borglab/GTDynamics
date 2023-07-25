
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

gtsam::Key PointKey(int k) {
  return Symbol('p', k);
}


void SaveResult(const Values& result, const size_t& num_steps, const std::string &folder) {
  std::filesystem::create_directory(folder);
  {
    std::ofstream file;
    file.open(folder + "result.txt");

    for (int k=0; k<=num_steps; k++) {
      Key point_key = gtsam::Symbol('p', k);
      Point3 point = result.at<Point3>(point_key);
      file << point.x() << " " << point.y() << " " << point.z() << "\n";
    }
    // file << "\n";
    file.close();
  }
}

int main(int argc, char **argv) {
  int num_steps = 5;
  double radius = 1;

  NonlinearFactorGraph graph;
  auto prior_noise = noiseModel::Isotropic::Sigma(3, 0.1);
  auto between_noise = noiseModel::Isotropic::Sigma(3, 0.1);

  graph.addPrior<Point3>(PointKey(0), Point3(0, 0, 0), prior_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(0), PointKey(1), Point3(0.6, 0.3, 0), between_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(1), PointKey(2), Point3(0.6, -0.3, 0), between_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(2), PointKey(3), Point3(-1, 0.0, 0.5), between_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(3), PointKey(4), Point3(0.0, 0.0, 0.6), between_noise);
  graph.emplace_shared<BetweenFactor<Point3>>(PointKey(4), PointKey(5), Point3(0.0, 0.6, -0.2), between_noise);

  Values init_values;
  init_values.insert(PointKey(0), Point3(0, 0, 0));
  init_values.insert(PointKey(1), Point3(0, 0, 0));
  init_values.insert(PointKey(2), Point3(0, 0, 0));
  init_values.insert(PointKey(3), Point3(0, 0, 0));
  init_values.insert(PointKey(4), Point3(0, 0, 0));
  init_values.insert(PointKey(5), Point3(0, 0, 0));

  std::vector<IneqConstraintManifold::shared_ptr> manifolds;
  for (int k=0; k<=num_steps; k++) {
    manifolds.push_back(std::make_shared<DomeManifold>(PointKey(k), radius, init_values.at<Point3>(PointKey(k))));
  }
  gtdynamics::InequalityConstraints constraints;
  for (const auto& manifold : manifolds) {
    constraints.insert(constraints.end(), manifold->constraints().begin(), manifold->constraints().end());
  }

  double manopt_gd_cost, manopt_gd_vio, manopt_gd_duration, manopt_gd_num_iters;
  double manopt_lm_cost, manopt_lm_vio, manopt_lm_duration, manopt_lm_num_iters;
  double barrier_cost, barrier_vio, barrier_duration, barrier_num_iters;
  {
    // Call IC gradient descent optimizer
    ICGradientDescentOptimizer icgd_optimizer;
    auto optimization_start = std::chrono::system_clock::now();
    auto icgd_result = icgd_optimizer.optimize(graph, manifolds);
    auto optimization_end = std::chrono::system_clock::now();
    auto optimization_time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                              optimization_start);
    const Values& icgd_values = icgd_result.values_vec.back();

    manopt_gd_cost = graph.error(icgd_values);
    manopt_gd_vio = EvaluateConstraintViolationL2Norm(constraints, icgd_values);
    manopt_gd_duration = optimization_time_ms.count() * 1e-3;
    manopt_gd_num_iters = icgd_result.values_vec.size();
  }
  {
    // Call ICLM optimizer
    LevenbergMarquardtParams lm_params;
    lm_params.setVerbosityLM("SUMMARY");
    ICLMOptimizer iclm_optimizer(lm_params);
    gtdynamics::ConstrainedOptResult intermediate_result;

    auto optimization_start = std::chrono::system_clock::now();
    auto iclm_result = iclm_optimizer.optimize(graph, manifolds, &intermediate_result);
    auto optimization_end = std::chrono::system_clock::now();
    auto optimization_time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                              optimization_start);
    manopt_lm_cost = graph.error(iclm_result);
    manopt_lm_vio = EvaluateConstraintViolationL2Norm(constraints, iclm_result);
    manopt_lm_duration = optimization_time_ms.count() * 1e-3;
    manopt_lm_num_iters = intermediate_result.num_iters[0];
    // iclm_result.print();
    SaveResult(iclm_result, num_steps, "../../results/dome_lm/");
  }
  {
  // Call barrier optimizer
    BarrierParameters barrier_params;
    barrier_params.num_iterations = 15;
    barrier_params.mu_increase_rate = 2.0;
    // barrier_params.lm_parameters.setVerbosityLM("SUMMARY");
    BarrierOptimizer barrier_optimizer(barrier_params);
    auto optimization_start = std::chrono::system_clock::now();
    auto barrier_result = barrier_optimizer.optimize(graph, constraints, init_values);
    auto optimization_end = std::chrono::system_clock::now();
    auto optimization_time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                              optimization_start);
    const Values& barrier_values = barrier_result.intermediate_values.back();

    barrier_cost = graph.error(barrier_values);
    barrier_vio = EvaluateConstraintViolationL2Norm(constraints, barrier_values);
    barrier_duration = optimization_time_ms.count() * 1e-3;
    barrier_num_iters = std::accumulate(barrier_result.num_iters.begin(),
                        barrier_result.num_iters.end(), 0);
  }
  std::cout << "barrier & " << std::setprecision(2) << barrier_duration << " & " << barrier_num_iters << " & " << barrier_cost << " & " << barrier_vio << "\n";
  std::cout << "manopt (1st order) & " << std::setprecision(2) << manopt_gd_duration << " & " << manopt_gd_num_iters << " & " << manopt_gd_cost << " & " << manopt_gd_vio << "\n";
  std::cout << "manopt (2nd order EQP) & " << std::setprecision(2) << manopt_lm_duration << " & " << manopt_lm_num_iters << " & " << manopt_lm_cost << " & " << manopt_lm_vio << "\n";

  return 0;
}
