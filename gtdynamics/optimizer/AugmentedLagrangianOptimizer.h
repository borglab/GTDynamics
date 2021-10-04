/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  AugmentedLagrangianOptimizer.h
 * @brief Augmented Lagrangian method for constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "gtdynamics/optimizer/NonlinearBiasFactor.h"
#include "gtdynamics/optimizer/EqualityConstraintFactor.h"
#include "gtdynamics/optimizer/Optimizer.h"

namespace gtdynamics {


/// Parameters for Augmented Lagrangian method
struct AugmentedLagrangianParameters :  public OptimizationParameters {
  size_t num_iterations;

  AugmentedLagrangianParameters() : num_iterations(12) {}
};

/// Augmented Lagrangian method only considering equality constraints.
class AugmentedLagrangianOptimizer : public Optimizer {
 protected:
  boost::shared_ptr<const AugmentedLagrangianParameters> p_;

 public:
  /* Constructor. */
  AugmentedLagrangianOptimizer(const boost::shared_ptr<const AugmentedLagrangianParameters>& parameters =
                 boost::make_shared<const AugmentedLagrangianParameters>())
      : Optimizer(parameters), p_(parameters) {}

  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                         const gtsam::Values& initial_values) const {
    gtsam::Values values = initial_values;

    std::map<gtsam::FactorIndex, double> mu;        // penalty parameter
    std::map<gtsam::FactorIndex, gtsam::Vector> z;  // Lagrangian multiplier

    for (size_t factor_index = 0; factor_index < graph.size();
         factor_index++) {
      auto factor = graph.at(factor_index);
      if (auto noise_factor =
              boost::dynamic_pointer_cast<gtsam::NoiseModelFactor,
                                          gtsam::NonlinearFactor>(factor)) {
        auto basic_noise = noise_factor->noiseModel();
        if (basic_noise->isConstrained()) {
          z[factor_index] = gtsam::Vector::Zero(basic_noise->dim());
          mu[factor_index] = 1;
        }
      }
    }

    // increase the penalty
    for (int i = 0; i < p_->num_iterations; i++) {
      // std::cout << "------------------ " << i << " ------------------\n";

      // converting constrained factors to unconstrained factors
      gtsam::NonlinearFactorGraph merit_graph;
      for (size_t factor_index = 0; factor_index < graph.size();
           factor_index++) {
        auto factor = graph.at(factor_index);
        // check if is noisemodel factor
        if (auto noise_factor =
                boost::dynamic_pointer_cast<gtsam::NoiseModelFactor,
                                            gtsam::NonlinearFactor>(factor)) {
          auto basic_noise = noise_factor->noiseModel();
          if (basic_noise->isConstrained()) {
            auto constrained_noise =
                boost::dynamic_pointer_cast<gtsam::noiseModel::Constrained,
                                            gtsam::noiseModel::Base>(
                    basic_noise);
            auto original_sigmas = constrained_noise->sigmas();
            auto tolerance = constrained_noise->mu();
            auto sigmas = tolerance;
            for (size_t i = 0; i < constrained_noise->dim(); i++) {
              if (constrained_noise->constrained(i)) {
                sigmas[i] = tolerance[i]/mu[factor_index];
              }
              else {
                sigmas[i] = original_sigmas[i];
              }
            }
            auto modified_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

            gtsam::Vector bias = z[factor_index] / (2 * mu[factor_index]);
            gtsam::Vector unwhitened_bias = modified_noise->unwhiten(bias);
            auto bias_factor =
                gtsam::NonlinearBiasFactor(noise_factor, unwhitened_bias);
            merit_graph.add(bias_factor.cloneWithNewNoiseModel(modified_noise));
          } else {
            merit_graph.add(factor);
          }
        } else {
          merit_graph.add(factor);
        }
      }

      gtsam::LevenbergMarquardtOptimizer optimizer(merit_graph, values,
                                                   p_->lm_parameters);
      auto result = optimizer.optimize();

      // merit_graph.print();
      // std::cout << "mu\t" << mu.begin()->second << "\n";
      // result.print();

      // save results and update parameters
      for (const auto it : mu) {
        auto factor_index = it.first;
        auto factor = graph.at(factor_index);

        auto noise_factor =
            boost::dynamic_pointer_cast<gtsam::NoiseModelFactor,
                                        gtsam::NonlinearFactor>(factor);
        auto basic_noise = noise_factor->noiseModel();
        auto constrained_noise =
            boost::dynamic_pointer_cast<gtsam::noiseModel::Constrained,
                                        gtsam::noiseModel::Base>(basic_noise);
        auto sigmas = constrained_noise->sigmas();
        auto modified_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
        auto unconstrained_factor =
            noise_factor->cloneWithNewNoiseModel(modified_noise);

        // update multipliers
        auto whitened_error = unconstrained_factor->whitenedError(result);
        z[factor_index] += 2 * mu[factor_index] * whitened_error;

        // update penalty parameter
        double previous_error = sqrt(2 * unconstrained_factor->error(values));
        double current_error = sqrt(2 * unconstrained_factor->error(result));
        if (current_error >= 0.25 * previous_error) {
          mu[factor_index] *= 2;
        }
      }

      values = result;
    }
    return values;
  }







  /// Run optimization with EqualityConstraintFactor.
  gtsam::Values optimize1(const gtsam::NonlinearFactorGraph& graph,
                         const gtsam::Values& initial_values) const {
    gtsam::Values values = initial_values;

    std::map<gtsam::FactorIndex, double> mu;        // penalty parameter
    std::map<gtsam::FactorIndex, gtsam::Vector> z;  // Lagrangian multiplier

    for (size_t factor_index = 0; factor_index < graph.size();
         factor_index++) {
      auto factor = graph.at(factor_index);
      if (auto constraint_factor =
              boost::dynamic_pointer_cast<gtsam::EqualityConstraintFactor<double>>(factor)) {
          z[factor_index] = gtsam::Vector::Zero(constraint_factor->dim());
          mu[factor_index] = 1;
      }
    }

    gtsam::NonlinearFactorGraph merit_graph = graph;
    // increase the penalty
    for (int i = 0; i < p_->num_iterations; i++) {
      // std::cout << "------------------ " << i << " ------------------\n";      
      for (size_t factor_index = 0; factor_index < graph.size();
           factor_index++) {
        auto factor = graph.at(factor_index);
        // check if is noisemodel factor
        if (auto constraint_factor =
                boost::dynamic_pointer_cast<gtsam::EqualityConstraintFactor<double>>(factor)) {
          constraint_factor->update_mu(mu[factor_index]);
          gtsam::Vector bias = z[factor_index] / (2 * mu[factor_index]);
          gtsam::Vector unwhitened_bias = constraint_factor->noiseModel()->unwhiten(bias);
          constraint_factor->update_bias(unwhitened_bias);
        }
      }
      gtsam::LevenbergMarquardtOptimizer optimizer(merit_graph, values,
                                                   p_->lm_parameters);
      auto result = optimizer.optimize();

      // merit_graph.print();
      // std::cout << "mu\t" << mu.begin()->second << "\n";
      // result.print();

      // save results and update parameters
      for (const auto it : mu) {
        auto factor_index = it.first;
        auto factor = graph.at(factor_index);

        auto constraint_factor =
            boost::dynamic_pointer_cast<gtsam::EqualityConstraintFactor<double>>(factor);

        // update multipliers
        auto whitened_error = constraint_factor->tolerance_scaled_error(result);
        z[factor_index] += 2 * mu[factor_index] * whitened_error;

        // update penalty parameter
        double previous_error = constraint_factor->original_error(values).norm();
        double current_error = constraint_factor->original_error(result).norm();
        if (current_error >= 0.25 * previous_error) {
          mu[factor_index] *= 2;
        }
      }

      values = result;
    }
    return values;
  }

};

}  // namespace gtdynamics