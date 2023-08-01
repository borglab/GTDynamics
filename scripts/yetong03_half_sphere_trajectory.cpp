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
#include <gtdynamics/imanifold/IEHalfSphere.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>

using namespace gtsam;
using namespace gtdynamics;


// void SaveResult(const ConstrainedOptResult &result,
//                 const Values &initial_values, const size_t &num_steps,
//                 const std::string &folder) {
//   std::filesystem::create_directory(folder);
//   {
//     std::ofstream file;
//     file.open(folder + "intermediate_values.txt");
//     for (int i = 0; i < result.intermediate_values.size(); i++) {
//       for (int k = 0; k <= num_steps; k++) {
//         Key point_key = gtsam::Symbol('p', k);
//         Point3 point = result.intermediate_values.at(i).at<Point3>(point_key);
//         file << point.x() << " " << point.y() << " " << point.z() << " ";
//       }
//       file << "\n";
//     }
//     file.close();
//   }
//   {
//     std::ofstream file;
//     file.open(folder + "initial_values.txt");
//     for (int k = 0; k <= num_steps; k++) {
//       Key point_key = gtsam::Symbol('p', k);
//       Point3 point = initial_values.at<Point3>(point_key);
//       file << point.x() << " " << point.y() << " " << point.z() << " ";
//     }
//     file << "\n";
//     file.close();
//   }
//   {
//     std::ofstream file;
//     file.open(folder + "tangent_vectors.txt");
//     for (int i = 0; i < result.tangent_vectors.size(); i++) {
//       for (int k = 0; k <= num_steps; k++) {
//         Key point_key = gtsam::Symbol('p', k);
//         Vector tv = result.tangent_vectors.at(i).at(point_key);
//         file << tv(0) << " " << tv(1) << " " << tv(2) << " ";
//       }
//       file << "\n";
//     }
//     file.close();
//   }
// }

int main(int argc, char **argv) {
  IEHalfSphere half_sphere;
  size_t num_steps = 10;
  Point3 attractor_point(0.0, 0.0, -2.0);

  // graph
  NonlinearFactorGraph graph;
  auto prior_noise = noiseModel::Isotropic::Sigma(3, 1e-1);
  auto between_noise = noiseModel::Isotropic::Sigma(3, 1e-1);
  auto attractor_noise = noiseModel::Isotropic::Sigma(3, 5e-1);
  Key first_point_key = PointKey(0);
  Key last_point_key = PointKey(num_steps);
  graph.addPrior<Point3>(first_point_key, Point3(0.8, 0, 0.6), prior_noise);
  graph.addPrior<Point3>(last_point_key, Point3(0, 0.8, 0.6), prior_noise);
  for (size_t k = 0; k <= num_steps; k++) {
    Key point_key = PointKey(k);
    graph.addPrior<Point3>(point_key, attractor_point, attractor_noise);
    if (k > 0) {
      Key prev_point_key = PointKey(k - 1);
      graph.emplace_shared<BetweenFactor<Point3>>(
          prev_point_key, point_key, Point3(0, 0, 0), between_noise);
    }
  }

  // constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    e_constraints.add(half_sphere.eConstraints(k));
    i_constraints.add(half_sphere.iConstraints(k));
  }

  // init values
  Values initial_values;
  for (size_t k = 0; k <= num_steps; k++) {
    double theta = M_PI_2 * k / num_steps;
    initial_values.insert(PointKey(k),
                          Point3(0.8 * cos(theta), 0.8 * sin(theta), 0.6));
  }

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor = std::make_shared<HalfSphereRetractor>();

  // Run LM optimization
  {
    LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    params.minModelFidelity = 0.5;
    IELMOptimizer lm_optimizer(params);
    auto lm_result = lm_optimizer.optimize(graph, e_constraints, i_constraints,
                                           initial_values, iecm_params);

    const auto &details = lm_optimizer.details();
    for (const auto &iter_details : details) {
      IEOptimizer::PrintIterDetails(iter_details, num_steps, false,
                                    IEHalfSphere::PrintValues,
                                    IEHalfSphere::PrintDelta);
    }
  }

  // Run GD optimization
  {
    GDParams params;
    params.maxIterations = 30;
    IEGDOptimizer gd_optimizer(params);
    auto gd_result = gd_optimizer.optimize(graph, e_constraints, i_constraints,
                                           initial_values, iecm_params);

    const auto &details = gd_optimizer.details();
    for (const auto &iter_details : details) {
      IEOptimizer::PrintIterDetails(iter_details, num_steps, false,
                                    IEHalfSphere::PrintValues,
                                    IEHalfSphere::PrintDelta);
    }
  }

  return 0;
}