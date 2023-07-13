/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEGradientDescentOptimizer.h
 * @brief First order optimization on manifolds with boundaries/corners.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEManifoldOptimizer.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

namespace gtsam {

class IEGDOptimizer : public IEOptimizer {
public:
  IEGDOptimizer() : IEOptimizer() {}

  IEManifoldValues lineSearch(const NonlinearFactorGraph &graph,
                              const IEManifoldValues &manifolds,
                              const gtsam::VectorValues &proj_dir,
                              const gtsam::VectorValues &descent_dir,
                              gtsam::VectorValues &delta) const;

  virtual Values
  optimizeManifolds(const NonlinearFactorGraph &graph,
                    const IEManifoldValues &manifolds,
                    gtdynamics::ConstrainedOptResult *intermediate_result =
                        nullptr) const override;
};

} // namespace gtsam
