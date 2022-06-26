/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TspaceBasis.cpp
 * @brief Tagent space basis implementations.
 * @author: Yetong Zhang
 */

#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtdynamics/optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>
#include <gtdynamics/optimizer/Retractor.h>

namespace gtsam {

/* ************************************************************************* */
UoptRetractor::UoptRetractor(const ConnectedComponent::shared_ptr& cc,
                             const LevenbergMarquardtParams& lm_params)
    : optimizer_(cc->merit_graph_, lm_params) {}

/* ************************************************************************* */
Values UoptRetractor::retract(const Values& values) {
  // gtsam::Values init_values_cc;
  // for (const Key& key : cc->keys_) {
  //   init_values_cc.insert(key, values.at(key));
  // }

  optimizer_.setValues(values);
  return optimizer_.optimize();
}

/* ************************************************************************* */
ProjRetractor::ProjRetractor(const ConnectedComponent::shared_ptr& cc,
                             const LevenbergMarquardtParams& lm_params)
    : optimizer_(gtdynamics::PenaltyMethodParameters(lm_params)), cc_(cc) {}


/* ************************************************************************* */
Values ProjRetractor::retract(const gtsam::Values& values) {
  NonlinearFactorGraph prior_graph;
  Values init_values_cc;
  for (const Key& key : cc_->keys_) {
    init_values_cc.insert(key, values.at(key));
    size_t dim = values.at(key).dim();
    auto linear_factor = boost::make_shared<JacobianFactor>(
        key, Matrix::Identity(dim, dim), Vector::Zero(dim),
        noiseModel::Unit::Create(dim));
    // TODO(yetong): replace Unit with a tunable parameter
    Values linearization_point;
    linearization_point.insert(key, values.at(key));
    prior_graph.emplace_shared<LinearContainerFactor>(linear_factor,
                                                      linearization_point);
  }
  return optimizer_.optimize(prior_graph, cc_->constraints_, init_values_cc);
}

/* ************************************************************************* */
BasisRetractor::BasisRetractor(const ConnectedComponent::shared_ptr& cc, const KeyVector& basis_keys,
                             const LevenbergMarquardtParams& lm_params)
    : optimizer_(lm_params), basis_keys_(basis_keys) {
  NonlinearFactorGraph graph;
  KeySet fixed_keys(basis_keys.begin(), basis_keys.end());
  for (const auto& factor: cc->merit_graph_) {
    // check if the factor contains fixed keys
    bool contain_fixed_keys = false;
    for (const Key& key: factor->keys()) {
      if (fixed_keys.exists(key)) {
        contain_fixed_keys = true;
        break;
      }
    }
    // update the factor if it constains fixed keys
    if (contain_fixed_keys) {
      NoiseModelFactor::shared_ptr noise_factor =
          boost::dynamic_pointer_cast<NoiseModelFactor>(factor);
      auto const_var_factor = boost::make_shared<ConstVarFactor>(noise_factor, fixed_keys);
      if (const_var_factor->checkActive()) {
        factors_with_fixed_vars_.push_back(const_var_factor);
        graph.add(const_var_factor);
      }
    }
    else {
      graph.add(factor);
    }
  }
  // set the graph for optimizer
  optimizer_.setGraph(graph);
}


/* ************************************************************************* */
Values BasisRetractor::retract(const gtsam::Values& values) {
  // set fixed values for the const variable factors
  for (auto& factor: factors_with_fixed_vars_) {
    factor->setFixedValues(values);
  }
  // set initial values for optimizer
  Values opt_values;
  for (const Key& key: optimizer_.graph().keys()) {
    opt_values.insert(key, values.at(key));
  }
  optimizer_.setValues(opt_values);
  // optimize
  Values result = optimizer_.optimize();
  // add fixed varaibles to the result
  for (const Key& key: basis_keys_) {
    result.insert(key, values.at(key));
  }
  return result;
}

}  // namespace gtsam
