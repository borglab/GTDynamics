/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstraintManifold.cpp
 * @brief Constraint manifold implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/optimizer/ConstraintManifold.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace gtsam {

/* ************************************************************************* */
void ConstraintManifold::initializeValues(const gtsam::Values& values) {
  values_ = gtsam::Values();
  embedding_dim_ = 0;
  for (const gtsam::Key& key : cc_->keys_) {
    const auto& value = values.at(key);
    embedding_dim_ += value.dim();
    values_.insert(key, value);
  }
  constraint_dim_ = 0;
  for (const auto& constraint : cc_->constraints_) {
    constraint_dim_ += constraint->dim();
  }
  if (embedding_dim_ > constraint_dim_) {
    dim_ = embedding_dim_ - constraint_dim_;
  } else {
    dim_ = 0;
  }
}

/* ************************************************************************* */
Retractor::shared_ptr ConstraintManifold::constructRetractor(
    const Params::shared_ptr& params,
    const BasisParams::shared_ptr& basis_params,
    const ConnectedComponent::shared_ptr& cc) {
  if (params->retract_type == Params::RetractType::UOPT) {
    return boost::make_shared<UoptRetractor>(cc, params->lm_params);
  } else if (params->retract_type == Params::RetractType::PROJ) {
    return boost::make_shared<ProjRetractor>(cc, params->lm_params);
  } else if (params->retract_type == Params::RetractType::PARTIAL_PROJ) {
    return boost::make_shared<BasisRetractor>(cc, basis_params->basis_keys_,
                                              params->lm_params);
  }
  else {
    // Default
    return boost::make_shared<UoptRetractor>(cc, params->lm_params);
  }
}

/* ************************************************************************* */
Values ConstraintManifold::retractConstraints(const Values& values) const {
  // if (params_->retract_type == Params::RetractType::UOPT) {
  //   return retractUopt(values);
  // } else if (params_->retract_type == Params::RetractType::PROJ) {
  //   return retractProj(values);
  // } else if (params_->retract_type == Params::RetractType::PARTIAL_PROJ) {
  //   return retractPProj(values);
  // }
  return retractor_->retract(values);
}

/* ************************************************************************* */
void ConstraintManifold::computeBasis() {
  if (params_->basis_type == Params::BasisType::KERNEL) {
    basis_ = boost::make_shared<MatrixBasis>(cc_, values_);
  } else if (params_->basis_type == Params::BasisType::SPECIFY_VARIABLES) {
    basis_ = boost::make_shared<EliminationBasis>(cc_, values_,
                                                  basis_params_->basis_keys_);
  }
  // Check the total dimension of basis variables should be the same as the
  // dimension of the manifold.
  if (basis_->dim() != dim()) {
    throw std::runtime_error(
        "specified basis has wrong dimensions. manifold dim: " +
        std::to_string(dim()) +
        "\tbasis dim: " + std::to_string(basis_->dim()));
  }
}

/* ************************************************************************* */
const gtsam::Value& ConstraintManifold::recover(const gtsam::Key key,
                                                ChartJacobian H) const {
  if (H) {
    // choose the corresponding rows in basis
    *H = basis_->recoverJacobian(key);
  }
  return values_.at(key);
}

/* ************************************************************************* */
ConstraintManifold ConstraintManifold::retract(const gtsam::Vector& xi,
                                               ChartJacobian H1,
                                               ChartJacobian H2) const {
  // Compute delta for each variable and perform update.
  Values new_values = basis_->retractBaseVariables(values_, xi);

  // Set jacobian as 0 since they are not used for optimization.
  if (H1)
    throw std::runtime_error(
        "ConstraintManifold retract jacobian not implemented.");
  if (H2)
    throw std::runtime_error(
        "ConstraintManifold retract jacobian not implemented.");

  // Satisfy the constraints in the connected component.
  return createWithNewValues(new_values, true);
}

/* ************************************************************************* */
gtsam::Vector ConstraintManifold::localCoordinates(const ConstraintManifold& g,
                                                   ChartJacobian H1,
                                                   ChartJacobian H2) const {
  Vector xi = basis_->localCoordinates(values_, g.values_);

  // Set jacobian as 0 since they are not used for optimization.
  if (H1)
    throw std::runtime_error(
        "ConstraintManifold localCoordinates jacobian not implemented.");
  if (H2)
    throw std::runtime_error(
        "ConstraintManifold localCoordinates jacobian not implemented.");

  return xi;
}

/* ************************************************************************* */
void ConstraintManifold::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + " ") << "ConstraintManifold" << std::endl;
  values_.print();
}

/* ************************************************************************* */
bool ConstraintManifold::equals(const ConstraintManifold& other,
                                double tol) const {
  return values_.equals(other.values_, tol);
}

// /* *************************************************************************
// */ gtsam::Values ConstraintManifold::retractUopt(
//     const gtsam::Values& values) const {
//   // return values;
//   gtsam::Values init_values_cc;
//   for (const Key& key : cc_->keys_) {
//     init_values_cc.insert(key, values.at(key));
//   }

//   // TODO: avoid copy-paste of graph, avoid constructing optimizer everytime
//   gtsam::LevenbergMarquardtOptimizer optimizer(
//       cc_->merit_graph_, init_values_cc, params_->lm_params);
//   return optimizer.optimize();
// }

// /* *************************************************************************
// */ gtsam::Values ConstraintManifold::retractProj(
//     const gtsam::Values& values) const {
//   NonlinearFactorGraph prior_graph;
//   Values init_values_cc;
//   for (const Key& key : cc_->keys_) {
//     init_values_cc.insert(key, values.at(key));
//     size_t dim = values.at(key).dim();
//     auto linear_factor = boost::make_shared<JacobianFactor>(
//         key, Matrix::Identity(dim, dim), Vector::Zero(dim),
//         noiseModel::Unit::Create(dim));
//     // TODO(yetong): replace Unit with a tunable parameter
//     Values linearization_point;
//     linearization_point.insert(key, values.at(key));
//     prior_graph.emplace_shared<LinearContainerFactor>(linear_factor,
//                                                       linearization_point);
//   }
//   gtdynamics::PenaltyMethodParameters al_params(params_->lm_params);
//   gtdynamics::PenaltyMethodOptimizer optimizer(al_params);

//   return optimizer.optimize(prior_graph, cc_->constraints_, init_values_cc);
// }

// /* *************************************************************************
// */ gtsam::Values ConstraintManifold::retractPProj(
//     const gtsam::Values& values) const {
//   Values init_values_cc;
//   for (const Key& key : cc_->keys_) {
//     init_values_cc.insert(key, values.at(key));
//   }
//   NonlinearFactorGraph graph = cc_->merit_graph_;
//   for (const Key& key : basis_params_->basis_keys_) {
//     size_t dim = values.at(key).dim();
//     // TODO: make it a tunable parameter.
//     auto linear_factor = boost::make_shared<JacobianFactor>(
//         key, Matrix::Identity(dim, dim) * 1e6, Vector::Zero(dim),
//         noiseModel::Unit::Create(dim));
//     Values linearization_point;
//     linearization_point.insert(key, values.at(key));
//     graph.emplace_shared<LinearContainerFactor>(linear_factor,
//                                                 linearization_point);
//   }
//   gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values_cc,
//                                                params_->lm_params);
//   return optimizer.optimize();
// }

}  // namespace gtsam
