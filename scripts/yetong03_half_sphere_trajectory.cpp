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
  size_t num_steps = 10;

  Point3 attractor(0.0, 0.0, -2.0);

  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  Values initial_values;
  NonlinearFactorGraph graph;
  Key first_point_key = gtsam::Symbol('p', 0);
  Key last_point_key = gtsam::Symbol('p', num_steps);
  graph.addPrior<Point3>(first_point_key, Point3(0.8, 0, 0.6), noiseModel::Isotropic::Sigma(3, 1e-5));
  graph.addPrior<Point3>(last_point_key, Point3(0, 0.8, 0.6), noiseModel::Isotropic::Sigma(3, 1e-5));
  for (size_t k=0; k<=num_steps; k++) {
    Key point_key = gtsam::Symbol('p', k);
    gtsam::Expression<Point3> point_expr(point_key);
    gtsam::Expression<double> sphere_expr(&norm3, point_expr);
    gtsam::Expression<double> z_expr(&point3_z, point_expr);
    e_constraints.emplace_shared<DoubleExpressionEquality>(sphere_expr, 1.0);
    i_constraints.emplace_shared<DoubleExpressionInequality>(z_expr, 1.0);

    double theta = M_PI_2 * k / num_steps;
    initial_values.insert(point_key, Point3(0.8*cos(theta),0.8*sin(theta),0.6));

    graph.addPrior<Point3>(point_key, attractor, noiseModel::Isotropic::Sigma(3, 5e-1));
    if (k>0) {
      Key prev_point_key = gtsam::Symbol('p', k-1);
      graph.emplace_shared<BetweenFactor<Point3>>(prev_point_key, point_key, Point3(0,0,0), noiseModel::Isotropic::Sigma(3, 1e-1));
    }
  }

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->ie_retract_type = IERetractType::HalfSphere;
  
  // Run LM optimization
  {
    LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    params.minModelFidelity = 0.5;
    IELMOptimizer lm_optimizer(params);
    ConstrainedOptResult lm_inter_result;
    auto lm_result = lm_optimizer.optimize(graph, e_constraints, i_constraints, initial_values, iecm_params, &lm_inter_result);

    std::string folder_path = "../../results/half_sphere_traj_lm/";
    SaveResult(lm_inter_result, initial_values, num_steps, folder_path);
  }


  // Run GD optimization
  {
    IEGDOptimizer gd_optimizer;
    ConstrainedOptResult gd_inter_result;
    auto gd_result = gd_optimizer.optimize(graph, e_constraints, i_constraints, initial_values, iecm_params, &gd_inter_result);

    std::string folder_path = "../../results/half_sphere_traj_gd/";
    SaveResult(gd_inter_result, initial_values, num_steps, folder_path);
  }

  return 0;
} 