/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  OptimizationBenchmark.h
 * @brief Helper functions for benchmarking constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include "gtdynamics/manifold/ManifoldOptimizer.h"
#include <gtdynamics/optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/manifold/ManifoldOptimizerType1.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>
#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <iostream>
#include <numeric>
#include <opt/homebrew/Cellar/boost/1.79.0/include/boost/optional/optional.hpp>
#include <opt/homebrew/Cellar/boost/1.79.0/include/boost/smart_ptr/make_shared_object.hpp>
#include <ostream>

using gtsam::Values;
using gtsam::LevenbergMarquardtParams;

namespace gtdynamics {

/** Run optimization using soft constraints, e.g., treating constraints as
 * costs.
 */
Values OptimizeSoftConstraints(const EqConsOptProblem &problem,
                                 std::ostream &latex_os, 
                                 LevenbergMarquardtParams lm_params = LevenbergMarquardtParams(),
                                 double mu = 100);

/// Default parameters for manifold optimization.
gtsam::ManifoldOptimizerParameters DefaultMoptParams();

/// Default parameters for manifold optimization, with basis constructed by
/// specifying variables.
gtsam::ManifoldOptimizerParameters DefaultMoptParamsSV();

/** Run optimization using constraint manifold. */
Values OptimizeConstraintManifold(
    const EqConsOptProblem &problem, std::ostream &latex_os,
    gtsam::ManifoldOptimizerParameters mopt_params = DefaultMoptParams(),
    LevenbergMarquardtParams lm_params = LevenbergMarquardtParams(),
    std::string exp_name = "Constraint Manifold");

/** Run constrained optimization using the penalty method. */
Values OptimizePenaltyMethod(const EqConsOptProblem &problem,
                               std::ostream &latex_os,
                               PenaltyMethodParameters params = PenaltyMethodParameters());

/** Run constrained optimization using the Augmented Lagrangian method. */
Values OptimizeAugmentedLagrangian(const EqConsOptProblem &problem,
                                     std::ostream &latex_os,
                                     AugmentedLagrangianParameters params = AugmentedLagrangianParameters());

/** Functor version of JointLimitFactor, for creating expressions. Compute error
 * for joint limit error, to reproduce joint limit factor in expressions. */
class JointLimitFunctor {
protected:
  double low_, high_;

public:
  JointLimitFunctor(const double &low, const double &high)
      : low_(low), high_(high) {}

  double operator()(const double &q,
                    gtsam::OptionalJacobian<1, 1> H_q = boost::none) const {
    if (q < low_) {
      if (H_q)
        *H_q = -gtsam::I_1x1;
      return low_ - q;
    } else if (q <= high_) {
      if (H_q)
        *H_q = gtsam::Z_1x1;
      return 0.0;
    } else {
      if (H_q)
        *H_q = gtsam::I_1x1;
      return q - high_;
    }
  }
};

} // namespace gtdynamics
