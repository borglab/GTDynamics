/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ICGradientDescentOptimizer.h
 * @brief Optimizer that treat equality-constrained components as manifolds.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/manifold/IneqConstraintManifold.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

namespace gtsam {

class GDResult {
 public:
  std::vector<Values> values_vec;
  std::vector<VectorValues> proj_dir_vec;
  std::vector<VectorValues> descent_dir_vec;
};


class ICGradientDescentOptimizer {

public:
  ICGradientDescentOptimizer() {}

  std::vector<IneqConstraintManifold::shared_ptr>
  lineSearch(const NonlinearFactorGraph &graph,
             const std::vector<IneqConstraintManifold::shared_ptr> &ic_manifolds,
             const std::vector<IndexSet> &tight_indices_vec,
             const gtsam::VectorValues &proj_dir,
             const gtsam::VectorValues &descent_dir) const {
    double alpha = 0.2;
    double beta = 0.5;
    double t = 1;

    Values values = CollectManifoldValues(ic_manifolds);
    double eval = graph.error(values);

    while (true) {
      // count ++;
      VectorValues delta = t * proj_dir;
      // delta.print("delta");

      std::vector<IneqConstraintManifold::shared_ptr> new_manifolds;
      for (int i = 0; i < ic_manifolds.size(); i++) {
        const auto &manifold = ic_manifolds.at(i);
        VectorValues delta_m =
            pickVectorValues(delta, manifold->values().keys());
        new_manifolds.push_back(
            manifold->retract(delta_m, tight_indices_vec[i]));
      }
      Values new_values = CollectManifoldValues(new_manifolds);
      double new_eval = graph.error(new_values);

      double nonlinear_error_decrease = eval - new_eval;
      double linear_error_decrease = descent_dir.dot(delta);

      std::cout << "t: " << t << "\tnonlinear: " << nonlinear_error_decrease <<
      "\tlinear: " << linear_error_decrease << std::endl;
      if (linear_error_decrease < 1e-10) {
        return ic_manifolds;
      }

      if (nonlinear_error_decrease > linear_error_decrease * alpha) {
        return new_manifolds;
      }
      t *= beta;
    }
    std::cout << "bad\n";
    return ic_manifolds;
  }

  VectorValues pickVectorValues(const VectorValues &vvalues,
                                const KeyVector &keys) const {
    VectorValues new_vvalues;
    for (const auto &key : keys) {
      new_vvalues.insert(key, vvalues.at(key));
    }
    return new_vvalues;
  }

  GDResult
  optimize(const NonlinearFactorGraph &graph,
           const std::vector<IneqConstraintManifold::shared_ptr> &ic_manifolds) const {

    GDResult result;
    auto manifolds = ic_manifolds;
    double prev_error = 1e10;

    for (int iter = 0; iter < 100; iter++) {
      Values values = CollectManifoldValues(manifolds);
      // values.print("values");

      // compute gradient by disregarding constraints
      auto linear_graph = graph.linearize(values);
      auto descent_dir = -1 * linear_graph->gradientAtZero();
      // descent_dir.print("descent_dir");

      // project gradient into tspace of each manifold
      VectorValues proj_dir;
      std::vector<IndexSet> tight_indices_vec;
      for (const auto &manifold : manifolds) {
        VectorValues direction_m;
        for (const Key &key : manifold->values().keys()) {
          direction_m.insert(key, descent_dir.at(key));
        }
        auto result = manifold->identifyBlockingConstraints(direction_m);
        tight_indices_vec.push_back(result.first);
        result.first.print("tight indices: ");
        proj_dir.insert(result.second);
      }
      // proj_dir.print("proj_dir");

      // perform line search
      manifolds = lineSearch(graph, manifolds, tight_indices_vec, proj_dir, descent_dir);

      // record result
      Values new_values = CollectManifoldValues(manifolds);
      double new_error = graph.error(new_values);
      result.values_vec.push_back(new_values);
      result.proj_dir_vec.push_back(proj_dir);
      result.descent_dir_vec.push_back(descent_dir);
      if (prev_error - new_error < 1e-4) {
        break;
      }
      prev_error = new_error;
    }
    return result;
  }
};

} // namespace gtsam
