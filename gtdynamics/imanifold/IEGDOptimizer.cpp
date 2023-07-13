/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEManifoldOptimizer.cpp
 * @brief Tagent space basis implementations.
 * @author: Yetong Zhang
 */

#include "imanifold/IEManifoldOptimizer.h"
#include <gtdynamics/imanifold/IEGDOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
IEManifoldValues
IEGDOptimizer::lineSearch(const NonlinearFactorGraph &graph,
                          const IEManifoldValues &manifolds,
                          const gtsam::VectorValues &proj_dir,
                          const gtsam::VectorValues &descent_dir,
                          gtsam::VectorValues& delta) const {
  double alpha = 0.2;
  double beta = 0.5;
  double t = 1;

  Values values = CollectManifoldValues(manifolds);
  double eval = graph.error(values);

  while (true) {
    // count ++;
    delta = t * proj_dir;
    // delta.print("delta");

    IEManifoldValues new_manifolds = RetractManifolds(manifolds, delta);
    Values new_values = CollectManifoldValues(new_manifolds);
    double new_eval = graph.error(new_values);

    double nonlinear_error_decrease = eval - new_eval;
    double linear_error_decrease = descent_dir.dot(delta);

    std::cout << "t: " << t << "\tnonlinear: " << nonlinear_error_decrease
              << "\tlinear: " << linear_error_decrease << std::endl;
    if (linear_error_decrease < 1e-10) {
      return manifolds;
    }

    if (nonlinear_error_decrease > linear_error_decrease * alpha) {
      return new_manifolds;
    }
    t *= beta;
  }
  std::cout << "bad\n";
  return manifolds;
}

/* ************************************************************************* */
Values IEGDOptimizer::optimizeManifolds(
    const NonlinearFactorGraph &graph, const IEManifoldValues &manifolds,
    gtdynamics::ConstrainedOptResult *intermediate_result) const {
  // construct equivalent factors on e-manifolds
  // std::cout << "in optimize\n";
  std::map<Key, Key> keymap_var2manifold = Var2ManifoldKeyMap(manifolds);
  NonlinearFactorGraph manifold_graph =
      ManifoldOptimizer::ManifoldGraph(graph, keymap_var2manifold);
  // std::cout << "graph done\n";

  IEManifoldValues current_manifolds = manifolds;
  double current_error = graph.error(CollectManifoldValues(manifolds));
  for (int i = 0; i < 100; i++) {
    // construct e-manifolds
    Values e_manifolds = EManifolds(current_manifolds);
    // std::cout << "e manifold done\n";

    // compute gradient descent direction
    auto linear_graph = manifold_graph.linearize(e_manifolds);
    VectorValues descent_dir = -1 * linear_graph->gradientAtZero();
    // std::cout << "gradient descent direction done\n";

    // project gradient into tangent cone
    VectorValues proj_descent_dir =
        ProjectTangentCone(current_manifolds, descent_dir).second;
    // std::cout << "proj gradient descent direction done\n";

    // line search for the update step
    VectorValues delta;
    IEManifoldValues new_manifolds =
        lineSearch(graph, current_manifolds, proj_descent_dir, descent_dir, delta);
    // std::cout << "line search done\n";

    if (intermediate_result) {
      intermediate_result->intermediate_values.emplace_back(CollectManifoldValues(new_manifolds));
      intermediate_result->tangent_vectors.emplace_back(ComputeTangentVector(current_manifolds, delta));
    }

    // update the manifold values and check convergence
    Values new_values = CollectManifoldValues(new_manifolds);
    double new_error = graph.error(new_values);
    current_manifolds = new_manifolds;
    if (current_error - new_error < 1e-4) {
      break;
    }
    current_error = new_error;
  }
  return CollectManifoldValues(current_manifolds);
}

} // namespace gtsam
