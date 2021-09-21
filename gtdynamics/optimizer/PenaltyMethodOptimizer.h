/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PenaltyMethodOptimizer.h
 * @brief Penalty method optimizer for constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "gtdynamics/optimizer/Optimizer.h"

namespace gtdynamics {

/// Parameters for penalty method
struct PenaltyMethodParameters :  public OptimizationParameters {
  size_t num_iterations;

  PenaltyMethodParameters() : num_iterations(12) {}
};


/// Penalty method only considering equality constraints.
class PenaltyMethodOptimizer : public Optimizer  {
 protected:
  boost::shared_ptr<const PenaltyMethodParameters> p_;

 public:
  /**
   * Constructor from parameters.
   */
  PenaltyMethodOptimizer(const boost::shared_ptr<const PenaltyMethodParameters>& parameters =
                 boost::make_shared<const PenaltyMethodParameters>())
      : Optimizer(parameters), p_(parameters) {}

  /// Run optimization with penalty method.
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                         const gtsam::Values& initial_values) const {
    gtsam::Values values = initial_values;
    double mu = 1.0;

    // increase the penalty
    for (int i = 0; i < p_->num_iterations; i++) {
      // converting constrained factors to unconstrained factors
      gtsam::NonlinearFactorGraph merit_graph;
      for (const auto& factor : graph) {
        // check if is noisemodel factor
        if (auto noise_factor =
                boost::dynamic_pointer_cast<gtsam::NoiseModelFactor,
                                            gtsam::NonlinearFactor>(factor)) {
          auto basic_noise = noise_factor->noiseModel();
          if (basic_noise->isConstrained()) {
            auto constrained_noise = boost::dynamic_pointer_cast<gtsam::noiseModel::Constrained, gtsam::noiseModel::Base>(basic_noise);
            auto tolerance = constrained_noise->mu();
            auto original_sigmas = constrained_noise->sigmas();
            auto sigmas = tolerance;
            for (size_t i=0; i<constrained_noise->dim(); i++) {
              if (constrained_noise->constrained(i)) {
                sigmas[i] = tolerance[i]/mu;
              }
              else {
                sigmas[i] = original_sigmas[i];
              }
            }
            auto modified_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
            merit_graph.add(noise_factor->cloneWithNewNoiseModel(modified_noise));
          } 
          else {
            merit_graph.add(factor);
          }
        } 
        else {
          merit_graph.add(factor);
        }
      }

      // run optimization
      gtsam::LevenbergMarquardtOptimizer optimizer(merit_graph, values, p_->lm_parameters);
      auto result = optimizer.optimize();

      // save results and update parameters
      values = result;
      mu *= 2.0;
    }
    return values;
  }
};

}  // namespace gtdynamics