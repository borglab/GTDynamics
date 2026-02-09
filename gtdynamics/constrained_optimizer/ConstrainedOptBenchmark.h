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

#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtdynamics/cmopt/NonlinearMOptimizer.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Values.h>

#include <iostream>
#include <ostream>

namespace gtdynamics {

using gtsam::LevenbergMarquardtParams;
using gtsam::Values;

/// Default parameters for manifold optimization.
ManifoldOptimizerParameters DefaultMoptParams();

/// Default parameters for manifold optimization, with basis constructed by
/// specifying variables.
ManifoldOptimizerParameters
DefaultMoptParamsSV(const BasisKeyFunc &basis_key_func);

/** Run optimization using soft constraints, e.g., treating constraints as
 * costs.
 */
Values OptimizeE_SoftConstraints(
    const EConsOptProblem &problem, std::ostream &latex_os,
    LevenbergMarquardtParams lm_params = LevenbergMarquardtParams(),
    double mu = 100, double constraint_unit_scale = 1.0);

/** Run optimization using constraint manifold. */
Values
OptimizeE_CMOpt(const EConsOptProblem &problem, std::ostream &latex_os,
                ManifoldOptimizerParameters mopt_params = DefaultMoptParams(),
                LevenbergMarquardtParams lm_params = LevenbergMarquardtParams(),
                std::string exp_name = "Constraint Manifold",
                double constraint_unit_scale = 1.0);

/** Run constrained optimization using the penalty method. */
Values OptimizeE_Penalty(const EConsOptProblem &problem, std::ostream &latex_os,
                         gtsam::PenaltyOptimizerParams::shared_ptr params =
                             std::make_shared<gtsam::PenaltyOptimizerParams>(),
                         double constraint_unit_scale = 1.0);

/** Run constrained optimization using the Augmented Lagrangian method. */
Values OptimizeE_AugmentedLagrangian(
    const EConsOptProblem &problem, std::ostream &latex_os,
    gtsam::AugmentedLagrangianParams::shared_ptr params =
        std::make_shared<gtsam::AugmentedLagrangianParams>(),
    double constraint_unit_scale = 1.0);

} // namespace gtdynamics
