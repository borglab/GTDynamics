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

#include <gtdynamics/cmopt/MultiJacobian.h>

namespace gtsam {

/* ************************************************************************* */
MultiJacobian::MultiJacobian(const Key &key, const Matrix &matrix)
    : Base(), dim_(matrix.rows()) {
  emplace(key, matrix);
}

/* ************************************************************************* */
MultiJacobian MultiJacobian::Identity(const Key &key, const size_t &dim) {
  MultiJacobian jacobian;
  jacobian[key] = Matrix::Identity(dim, dim);
  jacobian.dim_ = dim;
  return jacobian;
}

/* ************************************************************************* */
MultiJacobian MultiJacobian::VerticalStack(const MultiJacobian &jac1,
                                           const MultiJacobian &jac2) {
  size_t dim1 = jac1.numRows();
  size_t dim2 = jac2.numRows();
  size_t dim = jac1.numRows() + jac2.numRows();

  MultiJacobian jac;
  for (const auto &[key, matrix] : jac1) {
    Matrix expaned_mat(dim, matrix.cols());
    expaned_mat << matrix, Matrix::Zero(dim2, matrix.cols());
    jac.addJacobian(key, expaned_mat);
  }
  for (const auto &[key, matrix] : jac2) {
    Matrix expaned_mat(dim, matrix.cols());
    expaned_mat << Matrix::Zero(dim1, matrix.cols()), matrix;
    jac.addJacobian(key, expaned_mat);
  }
  return jac;
}

/* ************************************************************************* */
size_t MultiJacobian::numRows() const {
  if (dim_ == -1) {
    throw std::runtime_error("no data yet");
  }
  return dim_;
}

/* ************************************************************************* */
void MultiJacobian::addJacobian(const Key &key, const Matrix &matrix) {
  if (dim_ == -1) {
    dim_ = matrix.rows();
  }
  if (find(key) == end()) {
    if (!matrix.isZero(1e-8)) {
      emplace(key, matrix);
    }
  } else {
    at(key) = at(key) + matrix;
    if (at(key).isZero(1e-8)) {
      erase(key);
    }
  }
}

/* ************************************************************************* */
void MultiJacobian::addJacobianChainRule(const Matrix &H_relative,
                                         const MultiJacobian parent_jacobian) {
  for (const auto &it : parent_jacobian) {
    addJacobian(it.first, H_relative * it.second);
  }
}

/* ************************************************************************* */
VectorValues MultiJacobian::row(const size_t i) const {
  VectorValues jac_i;
  for (const auto &[key, mat] : *this) {
    jac_i.insert(key, mat.row(i).transpose());
  }
  return jac_i;
}

/* ************************************************************************* */
void MultiJacobian::print(const std::string &s,
                          const KeyFormatter &keyFormatter) const {
  std::cout << s;
  for (const auto &it : (*this)) {
    std::cout << keyFormatter(it.first) << ":\n";
    std::cout << it.second << "\n";
  }
}

/* ************************************************************************* */
bool MultiJacobian::equals(const MultiJacobian &other, double tol) const {
  for (const auto &it : *this) {
    const Key &key = it.first;
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
  for (const auto &it : other) {
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

KeySet MultiJacobian::keys() const {
  KeySet jac_keys;
  for (const auto &it : *this) {
    jac_keys.insert(it.first);
  }
  return jac_keys;
}

/* ************************************************************************* */
void ComputeBayesNetJacobian(const GaussianBayesNet &bn,
                             const KeyVector &basis_keys,
                             const std::map<Key, size_t> &var_dim,
                             MultiJacobians &jacobians) {
  // set jacobian of basis variables to identity
  for (const Key &key : basis_keys) {
    jacobians.emplace(key, MultiJacobian::Identity(key, var_dim.at(key)));
  }

  GaussianBayesNet reversed_bn = bn;
  std::reverse(reversed_bn.begin(), reversed_bn.end());

  // iteratively set jacobian of other variables
  for (auto cg : reversed_bn) {
    const Matrix S_mat = -cg->R().triangularView<Eigen::Upper>().solve(cg->S());
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
        if (frontal_position + frontal_dim > S_mat.rows() ||
            parent_position + parent_dim > S_mat.cols()) {
          throw std::runtime_error(
              "frontal: " + std::to_string(frontal_position) + ", " +
              std::to_string(frontal_dim) +
              "\tparent: " + std::to_string(parent_position) + ", " +
              std::to_string(parent_dim) +
              "\tS_mat: " + std::to_string(S_mat.rows()) + ", " +
              std::to_string(S_mat.cols()) +
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

void MultiJacobian::operator+=(const MultiJacobian &other) {
  for (const auto &it : other) {
    addJacobian(it.first, it.second);
  }
}

MultiJacobian operator*(const Matrix &m, const MultiJacobian &jac) {
  MultiJacobian new_jac;
  for (const auto &it : jac) {
    new_jac.insert({it.first, m * it.second});
  }
  return new_jac;
}

MultiJacobian operator+(const MultiJacobian &jac1, const MultiJacobian &jac2) {
  MultiJacobian sum_jac = jac1;
  for (const auto &it : jac2) {
    sum_jac.addJacobian(it.first, it.second);
  }
  return sum_jac;
}

MultiJacobians JacobiansMultiply(const MultiJacobians &jacs1,
                                 const MultiJacobians &jacs2) {
  MultiJacobians result_jacs;
  for (const auto &it : jacs1) {
    MultiJacobian jac;
    const MultiJacobian &jac1 = it.second;
    for (const auto &it1 : jac1) {
      const Key &key = it1.first;
      const Matrix &m = it1.second;
      jac += m * jacs2.at(key);
    }
    result_jacs.insert({it.first, jac});
  }
  return result_jacs;
}

} // namespace gtsam
