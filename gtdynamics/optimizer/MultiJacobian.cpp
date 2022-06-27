/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MultiJacobian.cpp
 * @brief multi-jacobian implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/optimizer/MultiJacobian.h>

namespace gtsam {

/* ************************************************************************* */
MultiJacobian::MultiJacobian(const Key& key, const Matrix& matrix) {
  emplace(key, matrix);
}

/* ************************************************************************* */
MultiJacobian MultiJacobian::Identity(const Key& key, const size_t& dim) {
  MultiJacobian jacobian;
  jacobian[key] = Matrix::Identity(dim, dim);
  return jacobian;
}

/* ************************************************************************* */
void MultiJacobian::addJacobian(const Key& key, const Matrix& matrix) {
  if (find(key) == end()) {
    emplace(key, matrix);
  } else {
    at(key) = at(key) + matrix;
  }
}

/* ************************************************************************* */
void MultiJacobian::addJacobianChainRule(const Matrix& H_relative,
                                         const MultiJacobian parent_jacobian) {
  for (const auto& it : parent_jacobian) {
    addJacobian(it.first, H_relative * it.second);
  }
}

/* ************************************************************************* */
void MultiJacobian::print(const std::string& s,
                          const KeyFormatter& keyFormatter) const {
  std::cout << s;
  for (const auto& it : (*this)) {
    std::cout << keyFormatter(it.first) << ":\n";
    std::cout << it.second << "\n";
  }
}

/* ************************************************************************* */
bool MultiJacobian::equals(const MultiJacobian& other, double tol) const {
  for (const auto& it : *this) {
    const Key& key = it.first;
    if (other.find(key) == other.end()) {
      if (!assert_equal(it.second,
                        Matrix::Zero(it.second.rows(), it.second.cols()),
                        tol)) {
        return false;
      }
    } else if (!assert_equal(it.second, other.at(key), tol)) {
      return false;
    }
  }
  for (const auto& it : other) {
    if (find(it.first) == end()) {
      if (!assert_equal(it.second,
                        Matrix::Zero(it.second.rows(), it.second.cols()),
                        tol)) {
        return false;
      }
    }
  }
  return true;
}

/* ************************************************************************* */
void ComputeBayesNetJacobian(const GaussianBayesNet& bn,
                             const KeyVector& basis_keys,
                             const std::map<Key, size_t>& var_dim,
                             MultiJacobians& jacobians) {
  // set jacobian of basis variables to identity
  for (const Key& key : basis_keys) {
    jacobians.emplace(key, MultiJacobian::Identity(key, var_dim.at(key)));
  }

  // iteratively set jacobian of other variables
  for (auto cg : boost::adaptors::reverse(bn)) {
    const auto S_mat = -cg->R().triangularView<Eigen::Upper>().solve(cg->S());
    DenseIndex frontal_position = 0;
    for (auto frontal = cg->beginFrontals(); frontal != cg->endFrontals();
         ++frontal) {
      const Key frontal_key = *frontal;
      const auto frontal_dim = cg->getDim(frontal);
      // initialize jacobian for frontal variable
      jacobians[frontal_key] = MultiJacobian();
      // add the jacobian component from each parent
      DenseIndex parent_position = 0;
      for (auto parent = cg->beginParents(); parent != cg->endParents();
           ++parent) {
        const Key parent_key = *parent;
        const auto parent_dim = cg->getDim(parent);
        if (frontal_position < 0) {
          throw std::runtime_error("frontal_position<0: " +
                                   std::to_string(frontal_position));
        }
        if (parent_position < 0) {
          throw std::runtime_error("parent_position<0: " +
                                   std::to_string(parent_position));
        }
        if (frontal_position + frontal_dim > S_mat.rows()) {
          throw std::runtime_error(
              "frontal_position + frontal_dim > S_mat.rows(): " +
              std::to_string(frontal_position) + ", " +
              std::to_string(frontal_dim) + ", " +
              std::to_string(S_mat.rows()));
        }
        if (parent_position + parent_dim > S_mat.cols()) {
          throw std::runtime_error(
              "parent_position + parent_dim > S_mat.cols(): " +
              std::to_string(parent_position) + ", " +
              std::to_string(parent_dim) + ", " + std::to_string(S_mat.cols()) +
              "\tR: " + std::to_string(cg->R().cols()) + ", " +
              std::to_string(cg->R().rows()) +
              "\tS: " + std::to_string(cg->S().cols()) + ", " +
              std::to_string(cg->S().rows()));
        }
        Matrix S_parent = S_mat.block(frontal_position, parent_position,
                                      frontal_dim, parent_dim);
        jacobians[frontal_key].addJacobianChainRule(S_parent,
                                                    jacobians[parent_key]);
        parent_position += parent_dim;
      }
      frontal_position += frontal_dim;
    }
  }
}

}  // namespace gtsam
