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

Key point_key = gtsam::Symbol('p', 0);

void SaveResult(const ConstrainedOptResult& result, const Values& initial_values, const std::string &folder) {
  std::filesystem::create_directory(folder);
  {
    std::ofstream file;
    file.open(folder + "intermediate_values.txt");
    for (int i=0; i<result.intermediate_values.size(); i++) {
      Point3 point = result.intermediate_values.at(i).at<Point3>(point_key);
      file << point.x() << " " << point.y() << " " << point.z() << "\n";
    }
    file.close();
  }
  {
    std::ofstream file;
    file.open(folder + "initial_values.txt");
    Point3 point = initial_values.at<Point3>(point_key);
    file << point.x() << " " << point.y() << " " << point.z() << "\n";
    file.close();
  }
  {
    std::ofstream file;
    file.open(folder + "tangent_vectors.txt");
    for (int i=0; i<result.tangent_vectors.size(); i++) {
      Vector tv = result.tangent_vectors.at(i).at(point_key);
      file << tv(0) << " " << tv(1) << " " << tv(2) << "\n";
    }
    file.close();
  }
}


int main(int argc, char **argv) {
  

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
  graph.addPrior<Point3>(point_key, Point3(0, -2, -1), noiseModel::Unit::Create(3));

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

    std::string folder_path = "../../results/half_sphere_lm_opt/";
    SaveResult(lm_inter_result, initial_values, folder_path);
  }


  // Run GD optimization
  {
    IEGDOptimizer gd_optimizer;
    ConstrainedOptResult gd_inter_result;
    auto gd_result = gd_optimizer.optimize(graph, e_constraints, i_constraints, initial_values, iecm_params, &gd_inter_result);

    std::string folder_path = "../../results/half_sphere_gd_opt/";
    SaveResult(gd_inter_result, initial_values, folder_path);
  }

  

  return 0;
}
